"""Unitree Go2W Ethernet control using the official Unitree SDK.

The script is intended to be copied to the Go2W onboard computer and run
locally while the robot is tethered to a host over Ethernet (interface
``eth0`` by default). Running on-robot keeps the latency predictable and
ensures the SportClient API can talk to the actuator boards without any
wireless drops.

Requirements
-----------
* Unitree SDK installed at ``/home/unitree/unitree_sdk2_python``
* Direct Ethernet connection to the robot control network
* Python dependencies listed in ``environment.yml`` (``numpy`` in this file)

Usage
-----
python3 go2_gait.py eth0
"""

import sys
import os
import time
import math
import select
import termios
import tty
import threading
import numpy as np

sys.path.append('/home/unitree/unitree_sdk2_python')

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    _SDK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Official Unitree SDK not available: {e}")
    _SDK_AVAILABLE = False

# Default network configuration used when no CLI argument is provided.
NETWORK_INTERFACE = "eth0"
ROBOT_IP = "192.168.123.18"

class Go2EthernetControl:
    """Thin wrapper around SportClient for Ethernet-based Go2W control."""
    
    def __init__(self, network_interface="eth0"):
        if not _SDK_AVAILABLE:
            raise RuntimeError("Official Unitree SDK not available. Please install it first.")
        
        # Initialize the channel factory with network interface
        print(f"Initializing Ethernet connection via {network_interface}...")
        ChannelFactoryInitialize(0, network_interface)
        
        # Initialize sport client
        self.client = SportClient()
        self.client.SetTimeout(10.0) 
        self.client.Init()
        
        # Maintain a rough pose estimate for console output only
        self.position = [0.0, 0.0, 0.35]
        self.yaw = 0.0
        self.is_standing = False
        
        # Maintain smoothed velocities for human-readable status updates
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        print(f"Ethernet SDK connection initialized via {network_interface}")
        print("Using official SportClient API")

    def stand_up(self):
        """Command the robot to stand up via the SportClient API."""
        print("Standing up...")
        code = self.client.StandUp()
        if code == 0:
            print("Stand up successful")
            balance_code = self.client.BalanceStand()
            if balance_code == 0:
                print("Balance stand enabled")
                self.is_standing = True
                return True
            else:
                print(f"Balance stand failed: {balance_code}")
                return False
        else:
            print(f"Stand up failed: {code}")
            return False
    
    def sit_down(self):
        """Command the robot to sit safely."""
        print("Sitting down...")
        code = self.client.StandDown()
        if code == 0:
            print("Sit down successful")
            self.is_standing = False
            return True
        else:
            print(f"StandDown failed ({code}), trying Damp mode...")
            damp_code = self.client.Damp()
            if damp_code == 0:
                print("Damp mode activated (robot should relax)")
                self.is_standing = False
                return True
            else:
                print(f"Both StandDown and Damp failed: StandDown={code}, Damp={damp_code}")
                return False
        
    def set_velocity(self, vx, vy, omega):
        """Send a velocity command using SportClient.Move.
        
        Note on Go2W speed limits:
        - The Go2W (wheeled version) may have different speed limits than standard Go2
        - SportClient.Move() may internally limit speeds regardless of what we send
        - Typical Go2W forward speed limit: ~1.5-2.0 m/s (varies by firmware)
        """
        # Update smoothed values only to keep console output readable.
        if abs(vx) < 0.1 and abs(vy) < 0.1:
            smoothing = 0.3
        else:
            smoothing = 0.2
            
        self.current_vx += smoothing * (vx - self.current_vx)
        self.current_vy += smoothing * (vy - self.current_vy)
        self.current_omega += smoothing * (omega - self.current_omega)
        
        # Clamp to SDK limits (the robot may further limit these internally)
        # Go2W wheeled mode may have lower actual limits than these
        vx_clamped = np.clip(vx, -3.0, 3.0)  # Forward/backward limits
        vy_clamped = np.clip(vy, -0.50, 0.50)  # Lateral limits
        omega_clamped = np.clip(omega, -1.5, 1.5)  # Rotation limits
        
        # Debug when lateral speeds are being limited
        if abs(vy) > 0.50 and abs(vy_clamped) <= 0.50:
            print(f"\n[LATERAL_LIMIT] Requested vy={vy:.2f} clamped to {vy_clamped:.2f}")
        
        # Send stop command if all velocities are zero
        if abs(vx_clamped) < 0.01 and abs(vy_clamped) < 0.01 and abs(omega_clamped) < 0.01:
            code = self.client.StopMove()
        else:
            # Send move command via official SDK
            code = self.client.Move(vx_clamped, vy_clamped, omega_clamped)
        
        # Integrate velocity to keep a coarse distance estimate for the console
        dt = 0.02
        old_pos = self.position.copy()
        self.position[0] += vx_clamped * dt
        self.position[1] += vy_clamped * dt
        self.yaw += omega_clamped * dt
        
        # Debug output (simplified - removed position spam)
        if abs(vx_clamped) > 0.01 or abs(vy_clamped) > 0.01 or abs(omega_clamped) > 0.01:
            print(f"\rMove cmd: vx={vx_clamped:.2f}, vy={vy_clamped:.2f}, omega={omega_clamped:.2f}, result={code}", end='', flush=True)
        elif code == 0:
            print(f"\rStop cmd sent, result={code}", end='', flush=True)
        
        return code == 0
        
    def stop(self):
        """Stop all commanded motion."""
        code = self.client.StopMove()
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        return code == 0
        
    def get_state(self):
        """Return the latest robot state from SportClient.GetState."""
        keys = ["pose", "velocity", "imu"]
        code, data = self.client.GetState(keys)
        if code == 0:
            return data
        return None
        
    def close(self):
        """Stop motion and sit down the robot before exiting."""
        print("Safely shutting down...")
        self.stop()
        time.sleep(1.0)  
        self.sit_down()
        time.sleep(2.0)  

class SimplePathController:
    """Path controller with multiple speed profiles"""
    def __init__(self):
        self.path_length = 16.0
        self.braking_distance = 2.5  # Increased braking distance so robot reaches full speed earlier
        
        # Speed category: 'profile' for variable speed profiles, 'fixed' for constant speeds
        self.speed_category = 'profile'
        
        # Variable speed profiles (original modes)
        self.speed_modes = [
            "0.5-3.0_gradual",  # 0.5m/s to 3.0m/s gradual increase 
            "3.0-0.5_gradual",  # 3.0m/s to 0.5m/s gradual decrease  
            "1.0_stop_1.0",     # 1.0m/s, stop at 8m, then 1.0m/s
            "3.0_stop_3.0"      # 3.0m/s, stop at 8m, then 3.0m/s
        ]
        self.current_speed_mode = 0
        
        # Fixed speed modes (linear forward motion at constant speed)
        # Note: Go2W wheeled mode may have actual max speed ~1.5 m/s
        # These are the commanded speeds - robot may internally limit higher values
        self.fixed_speeds = [0.5, 0.75, 1.0, 1.25, 1.5]  # m/s (realistic range for Go2W)
        self.current_fixed_speed_index = 0
        
        self.gaze_enabled = False
        self.was_stopped = False
        self.stop_start_time = None
        self.last_speed = 0.0
        self.speed_smoothing = 0.65  # Less smoothing for more responsive speed changes
        
        # Different max speeds for different movement modes
        self.max_speed_forward = 3.0  # Full speed for forward movement
        self.max_speed_lateral = 0.50  # Allow testing up to 0.5m/s for lateral movement
        
    def get_speed(self, distance_traveled, current_time=None, movement_mode='forward'):
        """Calculate current speed based on selected profile and movement mode"""
        mode = self.speed_modes[self.current_speed_mode]
        
        # Get the appropriate max speed for this movement mode
        max_speed = self.max_speed_forward if movement_mode == 'forward' else self.max_speed_lateral
        
        if mode == "0.5-3.0_gradual":
            if movement_mode == 'leftward':
                # For leftward, use steeper acceleration curve to reach max speed by 10m
                acceleration_distance = 10.0  # Reach max speed by 10m instead of 15m
                progress = min(1.0, distance_traveled / acceleration_distance)
                min_start_speed = 0.15  # Start higher to ensure proper crabwalk engagement
                base_speed = min_start_speed + progress * (max_speed - min_start_speed)
            else:
                # For forward mode, keep original logic: 0.5 to max_speed over full path
                progress = min(1.0, distance_traveled / self.path_length)
                base_speed = 0.5 + progress * (max_speed - 0.5)
            
        elif mode == "3.0-0.5_gradual":
            progress = min(1.0, distance_traveled / self.path_length)
            if movement_mode == 'leftward':
                # For leftward, start at max_speed and gradually decrease to reasonable minimum
                min_end_speed = 0.15  # End at reasonable speed to maintain crabwalk
                base_speed = max(min_end_speed, max_speed - progress * (max_speed - min_end_speed))
            else:
                # For forward mode, keep original logic: max_speed down to 0.5
                base_speed = max(0.5, max_speed - progress * (max_speed - 0.5))
            
        elif mode == "1.0_stop_1.0":
            # Scale 1.0 to appropriate speed for movement mode
            target_speed = min(1.0, max_speed)
            if distance_traveled < 8.0:
                base_speed = target_speed
            elif distance_traveled >= 8.0 and distance_traveled < 10.0:  # Stop only in 8-10m range
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                
                if current_time and (current_time - self.stop_start_time) < 2.0:
                    base_speed = 0.0
                    if not self.was_stopped:
                        print(f"\nRobot STOPPED at {distance_traveled:.1f}m")
                        self.was_stopped = True
                else:
                    if self.was_stopped:
                        print(f"\nRobot RESUMED at {distance_traveled:.1f}m")
                        self.was_stopped = False
                    base_speed = target_speed
            else:  # distance_traveled >= 10.0 - continue without stopping again
                base_speed = target_speed
                
        elif mode == "3.0_stop_3.0":
            # Use max speed appropriate for movement mode
            target_speed = max_speed
            if distance_traveled < 8.0:
                base_speed = target_speed
            elif distance_traveled >= 8.0 and distance_traveled < 10.0:  # Stop only in 8-10m range
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                
                if current_time and (current_time - self.stop_start_time) < 2.0:
                    base_speed = 0.0
                    if not self.was_stopped:
                        print(f"\nRobot STOPPED at {distance_traveled:.1f}m")
                        self.was_stopped = True
                else:
                    if self.was_stopped:
                        print(f"\nRobot RESUMED at {distance_traveled:.1f}m")
                        self.was_stopped = False
                    base_speed = target_speed
            else:  # distance_traveled >= 10.0 - continue without stopping again
                base_speed = target_speed
        else:
            base_speed = min(1.0, max_speed)
        
        # Check if in programmed stop
        is_in_programmed_stop = (base_speed == 0.0 and 'stop' in mode and distance_traveled >= 8.0)
        
        # Apply braking only if not in programmed stop
        if not is_in_programmed_stop:
            distance_to_goal = self.path_length - distance_traveled
            if distance_to_goal <= self.braking_distance and distance_to_goal > 0:
                brake_factor = distance_to_goal / self.braking_distance
                base_speed = base_speed * brake_factor
            elif distance_to_goal <= 0:
                base_speed = 0.0
        
        # Smooth speed transitions
        smoothed_speed = self.speed_smoothing * self.last_speed + (1 - self.speed_smoothing) * base_speed
        self.last_speed = smoothed_speed
        
        return smoothed_speed
    
    def get_fixed_speed(self, distance_traveled):
        """Get speed for fixed speed mode (constant speed with braking at end)"""
        base_speed = self.fixed_speeds[self.current_fixed_speed_index]
        
        # Apply braking near the end
        distance_to_goal = self.path_length - distance_traveled
        if distance_to_goal <= self.braking_distance and distance_to_goal > 0:
            brake_factor = distance_to_goal / self.braking_distance
            base_speed = base_speed * brake_factor
        elif distance_to_goal <= 0:
            base_speed = 0.0
        
        # For fixed speed modes, use minimal smoothing to reach target quickly
        # Only smooth during acceleration from stop, not during constant speed
        if self.last_speed < 0.1:
            # Starting from stop - ramp up quickly
            smoothed_speed = 0.3 * self.last_speed + 0.7 * base_speed
        else:
            # Already moving - use very light smoothing (almost no delay)
            smoothed_speed = 0.1 * self.last_speed + 0.9 * base_speed
        
        self.last_speed = smoothed_speed
        
        return smoothed_speed
    
    def get_current_speed_description(self):
        """Get a human-readable description of current speed setting"""
        if self.speed_category == 'fixed':
            return f"Fixed {self.fixed_speeds[self.current_fixed_speed_index]:.2f} m/s"
        else:
            return f"Profile: {self.speed_modes[self.current_speed_mode]}"

class KeyboardInput:
    """Non-blocking keyboard input handler"""
    def __init__(self):
        self.keys_pressed = set()
        self.key_events = {}
        self.running = True
        self.old_settings = None
        
    def start(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno()) 
        self.thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
    def _keyboard_listener(self):
        while self.running:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    key = sys.stdin.read(1)
                    if key:
                        key_code = ord(key.lower()) if len(key) == 1 else 0
                        if key == ' ':
                            key_code = ord(' ')
                        elif key == '\x03':  # Ctrl+C
                            self.running = False
                            raise KeyboardInterrupt()
                        elif key == '\x1b':  # ESC key
                            self.key_events[27] = 3  # ESC keycode
                        self.key_events[key_code] = 3  
                except KeyboardInterrupt:
                    raise
                except:
                    continue
                    
    def get_keys(self):
        keys = self.key_events.copy()
        self.key_events.clear()
        return keys

def main():
    if len(sys.argv) < 2:
        script = os.path.basename(sys.argv[0])
        print(f"Try: python3 {script} eth0")
        sys.exit(-1)

    print("\n" + "="*70)
    print("     Go2W Ethernet Control - Official Unitree SDK")
    print("="*70)
    
    network_interface = sys.argv[1]
    
    # Initialize components
    keyboard = KeyboardInput()
    robot = None
    path_controller = SimplePathController()
    
    try:
        print(f"Initializing robot connection via {network_interface}...")
        robot = Go2EthernetControl(network_interface=network_interface)
        
        # Stand up robot
        robot.stand_up()
        
        # Start keyboard input
        keyboard.start()
        
        # State variables
        path_modes = ['forward', 'leftward', 'leftward_zigzag']
        path_mode_index = 0
        path_mode = path_modes[path_mode_index]
        is_walking = False
        start_time = None
        initial_pos = None
        last_status_time = time.time()
        
        print("\n" + "="*70)
        print("READY FOR CONTROL")
        print("="*70)
        
        print("\nControls:")
        print("   SPACE  : Start/Stop Movement")
        print("   P      : Cycle Path Mode (forward/leftward/leftward_zigzag)")
        print("   S      : Cycle Speed (within current category)")
        print("   C      : Toggle Speed Category (Profile/Fixed)")
        print("   R      : Reset Position")
        print("   ESC    : Emergency Stop")
        print("   Ctrl+C : Force Exit")
        
        print("\nSpeed Categories:")
        print("   Profile : Variable speed profiles (gradual, stop modes)")
        print("   Fixed   : Constant speeds (0.5, 0.75, 1.0, 1.25, 1.5 m/s)")
        print("   Note    : Go2W max speed in wheeled mode may be ~1.5 m/s")
        
        print("\nPath Settings:")
        print(f"   Length         : {path_controller.path_length}m")
        print(f"   Braking Distance: {path_controller.braking_distance}m")
        print(f"   Current Speed  : {path_controller.get_current_speed_description()}")
        
        print("\nPress SPACE to start movement")
        print("="*70 + "\n")
        
        # Main control loop
        while True:
            current_time = time.time()
            
            # Process keyboard input
            keys = keyboard.get_keys()
            
            if ord(' ') in keys:
                is_walking = not is_walking
                if is_walking:
                    start_time = current_time
                    if initial_pos is None:  # Only set initial position on first start
                        initial_pos = robot.position.copy()
                    path_controller.stop_start_time = None
                    path_controller.was_stopped = False
                    path_controller.last_speed = 0.0  # Reset speed smoothing on start
                    print(f"Movement started: {path_mode} direction")
                else:
                    robot.stop()
                    print("Movement stopped")
                    
            if ord('p') in keys:
                path_mode_index = (path_mode_index + 1) % len(path_modes)
                path_mode = path_modes[path_mode_index]
                print(f"Path mode changed: {path_mode}")
                
            if ord('s') in keys:
                # Cycle within current speed category
                if path_controller.speed_category == 'fixed':
                    path_controller.current_fixed_speed_index = (path_controller.current_fixed_speed_index + 1) % len(path_controller.fixed_speeds)
                    print(f"Fixed speed changed: {path_controller.fixed_speeds[path_controller.current_fixed_speed_index]:.2f} m/s")
                else:
                    path_controller.current_speed_mode = (path_controller.current_speed_mode + 1) % len(path_controller.speed_modes)
                    mode_name = path_controller.speed_modes[path_controller.current_speed_mode]
                    print(f"Speed profile changed: {mode_name}")
            
            if ord('c') in keys:
                # Toggle speed category
                if path_controller.speed_category == 'profile':
                    path_controller.speed_category = 'fixed'
                    print(f"Speed category changed: FIXED ({path_controller.fixed_speeds[path_controller.current_fixed_speed_index]:.2f} m/s)")
                else:
                    path_controller.speed_category = 'profile'
                    print(f"Speed category changed: PROFILE ({path_controller.speed_modes[path_controller.current_speed_mode]})")
                
            if ord('r') in keys:
                robot.stop()
                robot.position = [0.0, 0.0, 0.35]
                robot.yaw = 0.0
                is_walking = False
                print("Position reset complete")
                
            if 27 in keys:  # ESC key pressed
                print("\nESC pressed - Emergency stop")
                if robot:
                    robot.stop()
                break

            # Movement control
            if is_walking and start_time is not None:
                current_pos = robot.position
                elapsed_time = current_time - start_time
                
                # Calculate distance traveled
                if initial_pos is not None:
                    if path_mode == 'forward':
                        distance_traveled = current_pos[0] - initial_pos[0]  # Use X for forward
                    else:  # leftward or leftward_zigzag - both use Y for forward progress
                        distance_traveled = -(current_pos[1] - initial_pos[1])  # Use Y for left
                    
                    # Debug distance tracking every 50 loops
                    if int(current_time * 50) % 100 == 0:  # Every ~2 seconds
                        print(f"\n[DEBUG] Distance={distance_traveled:.2f}m: current_pos={current_pos}, initial_pos={initial_pos}")
                else:
                    distance_traveled = 0

                # Check if path is complete first
                if distance_traveled >= path_controller.path_length:
                    print(f"\nPath complete. Total distance: {distance_traveled:.2f} m")
                    robot.stop()
                    is_walking = False
                    # Reset path state for next run
                    initial_pos = None
                    path_controller.stop_start_time = None
                    path_controller.was_stopped = False
                    path_controller.last_speed = 0.0
                    robot.position = [0.0, 0.0, 0.35]  # Reset position tracking
                    robot.yaw = 0.0
                    
                    print("\n" + "="*70)
                    print("                    READY FOR NEXT PATH")
                    print("="*70)
                    print("Press SPACE to start a new path")
                    print("Press P to change direction | Press S to change speed profile")
                    print("="*70 + "\n")
                    continue  # Skip velocity commands when path is complete

                # Get target speed based on speed category and path mode
                if path_mode == 'leftward_zigzag':
                    target_speed = 1.0  # Fixed speed for zigzag
                    path_controller.last_speed = target_speed
                elif path_controller.speed_category == 'fixed':
                    # Use fixed constant speed
                    target_speed = path_controller.get_fixed_speed(distance_traveled)
                else:
                    # Use variable speed profile
                    target_speed = path_controller.get_speed(distance_traveled, current_time, path_mode)

                # Periodic status output
                is_stopped = (target_speed < 0.05)
                
                if current_time - last_status_time > 2.0 and not is_stopped:
                    actual_speed = math.sqrt(robot.current_vx**2 + robot.current_vy**2)
                    distance_to_goal = path_controller.path_length - distance_traveled
                    speed_desc = path_controller.get_current_speed_description()
                    
                    status = f"\nDistance: {distance_traveled:.1f} m"
                    status += f" | Target: {target_speed:.2f}m/s"
                    status += f" | Actual: {actual_speed:.2f}m/s ({speed_desc})"
                    status += f" | To Goal: {distance_to_goal:.1f}m"
                    
                    print(status)
                    last_status_time = current_time
                    
                # Generate control commands based on path mode
                if path_mode == 'forward':
                    vx = target_speed
                    vy = 0.0
                    omega = 0.0
                    
                elif path_mode == 'leftward_zigzag':
                    # Zigzag path: linear (0-5m), zigzag (5-11m), linear (11-16m)
                    zigzag_start = 5.0
                    zigzag_end = 11.0
                    zigzag_cycles = 2.0  # Number of complete zigzags
                    lateral_fraction = 0.5  # 50% of speed allocated to lateral motion
                    
                    if zigzag_start <= distance_traveled < zigzag_end:
                        # During zigzag zone
                        zigzag_progress = (distance_traveled - zigzag_start) / (zigzag_end - zigzag_start)
                        # Oscillate lateral velocity (vx = forward/backward in robot frame when facing left)
                        lateral_velocity = target_speed * lateral_fraction * math.sin(2 * math.pi * zigzag_cycles * zigzag_progress)
                        # Calculate forward component to maintain constant overall speed
                        forward_component = math.sqrt(max(0.0, target_speed**2 - lateral_velocity**2))
                        
                        vx = lateral_velocity  # Lateral (left/right oscillation)
                        vy = -forward_component  # Forward along path (negative Y = leftward)
                        omega = 0.0
                    else:
                        # Before or after zigzag zone - straight leftward motion
                        vx = 0.0
                        vy = -target_speed
                        omega = 0.0
                        
                else:  # leftward (straight)
                    vx = 0.0
                    vy = -target_speed
                    omega = 0.0
                
                # Send velocity command to robot
                robot.set_velocity(vx, vy, omega)
                        
            else:
                # Not walking - ensure robot is stopped
                if (abs(robot.current_vx) > 0.01 or 
                    abs(robot.current_vy) > 0.01 or 
                    abs(robot.current_omega) > 0.01):
                    robot.set_velocity(0.0, 0.0, 0.0)

            time.sleep(0.02)  # 50Hz control loop
            
    except KeyboardInterrupt:
        print("\nEmergency stop triggered")
        if robot:
            robot.stop()
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if robot:
            try:
                robot.close()
            except Exception:
                pass
            print("Robot safely stopped and seated")
        
        keyboard.stop()
        print("Control ended")

if __name__ == '__main__':
    main()
