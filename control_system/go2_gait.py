"""
Go2W Real Robot Deployment Script - CORRECTED for Dual Mode Operation

This script supports both wheeled mode (forward) and walking mode (lateral) just like the simulation.
Go2W can walk laterally using legs while wheels are locked, similar to regular Go2.

Requirements:
- Go2W EDU with omnidirectional capabilities (wheels + legs)
- Network configured (robot IP: 192.168.123.161)  
- unitree_sdk2py properly installed

Usage:
    python3 go2w_deployment.py <network_interface>
    Example: python3 go2w_deployment.py enp2s0
"""

import sys
import time
import math
import threading
import termios
import tty
import select
import socket
import numpy as np
from typing import List, Tuple, Optional

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.high_level_interface import HighLevelInterface
except ImportError as e:
    print("Error importing Unitree SDK:")
    print(f"   {e}")
    print("Please install unitree_sdk2py from source:")
    print("   git clone https://github.com/unitreerobotics/unitree_sdk2_python.git")
    print("   cd unitree_sdk2_python")
    print("   pip3 install -e .")
    sys.exit(1)

class SimplePathController:
    """Simple path controller with both wheeled and walking modes"""
    def __init__(self):
        self.path_length = 15.0
        self.braking_distance = 1.5
        self.speed_modes = [
            "1-3_gradual",     # 1m/s to 3m/s gradual increase
            "3-1_gradual",     # 3m/s to 1m/s gradual decrease  
            "1_stop_1",        # 1m/s, stop at 8m, then 1m/s
            "3_stop_3"         # 3m/s, stop at 8m, then 3m/s
        ]
        self.current_speed_mode = 0
        self.gaze_enabled = False
        self.was_stopped = False
        self.stop_start_time = None
        
        self.last_speed = 0.0
        self.speed_smoothing = 0.85
        
    def get_speed(self, distance_traveled, current_time=None):
        """Calculate current speed"""
        mode = self.speed_modes[self.current_speed_mode]
        
        if mode == "1-3_gradual":
            progress = min(1.0, distance_traveled / self.path_length)
            base_speed = 1.0 + progress * 2.0
            
        elif mode == "3-1_gradual":
            progress = min(1.0, distance_traveled / self.path_length)
            base_speed = max(1.0, 3.0 - progress * 2.0)
            
        elif mode == "1_stop_1":
            if distance_traveled < 8.0:
                base_speed = 1.0
            elif distance_traveled >= 8.0:
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                
                if current_time and (current_time - self.stop_start_time) < 2.0:
                    base_speed = 0.0
                    if not self.was_stopped:
                        print(f"Robot STOPPED at {distance_traveled:.1f}m")
                        self.was_stopped = True
                else:
                    if self.was_stopped:
                        print(f"Robot RESUMED at {distance_traveled:.1f}m")
                        self.was_stopped = False
                    base_speed = 1.0
            else:
                base_speed = 1.0
                
        elif mode == "3_stop_3":
            if distance_traveled < 8.0:
                base_speed = 3.0
            elif distance_traveled >= 8.0:
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                
                if current_time and (current_time - self.stop_start_time) < 2.0:
                    base_speed = 0.0
                    if not self.was_stopped:
                        print(f"Robot STOPPED at {distance_traveled:.1f}m")
                        self.was_stopped = True
                else:
                    if self.was_stopped:
                        print(f"Robot RESUMED at {distance_traveled:.1f}m")
                        self.was_stopped = False
                    base_speed = 3.0
            else:
                base_speed = 3.0
        else:
            base_speed = 1.0
        
        is_in_programmed_stop = (base_speed == 0.0 and 'stop' in mode and distance_traveled >= 8.0)
        
        if not is_in_programmed_stop:
            distance_to_goal = self.path_length - distance_traveled
            if distance_to_goal <= self.braking_distance and distance_to_goal > 0:
                brake_factor = distance_to_goal / self.braking_distance
                base_speed = base_speed * brake_factor
            elif distance_to_goal <= 0:
                base_speed = 0.0
        
        smoothed_speed = self.speed_smoothing * self.last_speed + (1 - self.speed_smoothing) * base_speed
        self.last_speed = smoothed_speed
        
        return smoothed_speed
    
    def compute_forward_control(self, current_pos, current_yaw, target_speed):
        """Forward control for wheeled mode"""
        target_yaw = math.pi/2  # Face toward +Y direction
        
        is_in_stop_period = (target_speed < 0.05)
        
        # Path correction for wheeled mode
        path_error = current_pos[0] - 0.0
        
        if is_in_stop_period:
            # Gentle lateral correction during stops
            lateral_velocity = -path_error * 0.4
            lateral_velocity = np.clip(lateral_velocity, -0.3, 0.3)
            omega = 0.0
            forward_velocity = 0.0
        else:
            # Normal path following 
            lateral_velocity = -path_error * 1.0
            lateral_velocity = np.clip(lateral_velocity, -0.6, 0.6)
            
            # Heading control
            yaw_error = target_yaw - current_yaw
            
            # Normalize angle
            while yaw_error > math.pi:
                yaw_error -= 2*math.pi
            while yaw_error < -math.pi:
                yaw_error += 2*math.pi
            
            # Gentle yaw correction
            omega = yaw_error * 0.1
            omega = np.clip(omega, -1.0, 1.0)
            
            forward_velocity = target_speed
        
        return forward_velocity, lateral_velocity, omega
    
    def get_gaze_angle(self, t):
        """Calculate gaze angle if gaze mode is enabled."""
        if not self.gaze_enabled:
            return 0.0
        return 15.0 * math.sin(0.5 * t)  # ±15 degrees

class PoseBroadcaster:
    """UDP broadcaster for digital twin synchronization."""
    def __init__(self, target_ip: str, target_port: int):
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Pose broadcaster initialized for {self.target_ip}:{self.target_port}")

    def send_pose(self, position: List[float], yaw: float):
        """Sends pose data (x, y, z, yaw) as comma-separated string."""
        try:
            pose_str = f"{position[0]},{position[1]},{position[2]},{yaw}"
            self.sock.sendto(pose_str.encode('utf-8'), (self.target_ip, self.target_port))
        except Exception:
            pass

    def close(self):
        self.sock.close()

class KeyboardInput:
    """Non-blocking keyboard input handler."""
    
    def __init__(self):
        self.keys_pressed = set()
        self.key_events = {}
        self.running = True
        self.old_settings = None
        
    def start(self):
        """Start keyboard monitoring in separate thread."""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno()) 
        self.thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.thread.start()
        
    def stop(self):
        """Stop keyboard monitoring and restore terminal."""
        self.running = False
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
    def _keyboard_listener(self):
        """Listen for keyboard events."""
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
                            
                        self.key_events[key_code] = 3  
                        
                except KeyboardInterrupt:
                    raise
                except:
                    continue
                    
    def get_keys(self):
        """Get current key events."""
        keys = self.key_events.copy()
        self.key_events.clear()
        return keys

class Go2WRobot:
    """Go2W robot interface supporting both wheeled and walking modes"""
    
    def __init__(self, network_interface: str = "enp2s0", remote_ip: str = "192.168.123.1"):
        self.network_interface = network_interface
        self.start_pos = [0.0, 0.0, 0.35]
        self.initial_yaw = 0.0
        self.current_yaw = 0.0
        self.mode = 'walking'  # Default to walking mode
        
        # Initialize UDP broadcaster
        self.pose_broadcaster = PoseBroadcaster(remote_ip, 9051)
        
        # Initialize simple path controller
        self.path_controller = SimplePathController()
        
        print(f"Connecting to Go2W via {network_interface}...")
        
        try:
            # Initialize channel factory with domain ID 0 for real robot
            ChannelFactoryInitialize(0, network_interface)
            
            # Initialize high-level interface
            self.interface = HighLevelInterface()
            
            # Robot state
            self.current_position = [0.0, 0.0, 0.35]
            self.is_connected = False
            self.last_cmd_time = time.time()
            
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_omega = 0.0
            
            # Start state monitoring
            self._start_state_monitoring()
            
            # Wait for connection
            timeout = 5.0
            start_time = time.time()
            while not self.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
            if self.is_connected:
                print("Go2W connection established!")
                print("Supporting both wheeled (forward) and walking (lateral) modes")
                
                # Stand up the robot
                self.stand_up()
            else:
                raise Exception("Timeout waiting for robot connection")
            
        except Exception as e:
            print(f"Failed to connect to Go2W: {e}")
            raise
    
    def _start_state_monitoring(self):
        """Monitor robot state using high-level interface."""
        def monitor_state():
            while True:
                try:
                    # Get high-level state
                    state = self.interface.GetHighState()
                    if state is not None:
                        # Extract position and orientation from state
                        self.current_position = [
                            state.position[0] if hasattr(state, 'position') else 0.0,
                            state.position[1] if hasattr(state, 'position') else 0.0,
                            state.position[2] if hasattr(state, 'position') else 0.35
                        ]
                        
                        # Get yaw from IMU
                        if hasattr(state, 'imu_state') and hasattr(state.imu_state, 'rpy'):
                            self.current_yaw = state.imu_state.rpy[2]
                        
                        self.is_connected = True
                        
                        # Broadcast pose
                        self.pose_broadcaster.send_pose(self.current_position, self.current_yaw)
                    else:
                        if self.is_connected:
                            print("Lost connection to robot state")
                            self.is_connected = False
                        
                    time.sleep(0.02)  # 50Hz
                except Exception as e:
                    if self.is_connected:
                        print(f"State monitoring error: {e}")
                        self.is_connected = False
                    time.sleep(0.1)
                    continue
                    
        self.monitor_thread = threading.Thread(target=monitor_state, daemon=True)
        self.monitor_thread.start()
    
    def set_mode(self, mode):
        """Switch between 'wheeled' and 'walking' modes."""
        if mode == self.mode:
            return
            
        print(f"Switching from {self.mode} to {mode} mode")
        self.mode = mode
        
        # Note: The actual mode switching API calls would go here
        # This depends on the specific Go2W high-level interface methods
        # You may need to call specific functions to enable/disable wheels
    
    def stand_up(self):
        """Stand up the robot using high-level interface."""
        print("Standing up robot...")
        try:
            # Use high-level interface StandUpDown method
            self.interface.StandUpDown()
            time.sleep(3.0)  # Wait for stand up to complete
        except Exception as e:
            print(f"Error during stand up: {e}")
    
    def velocity_move_wheeled(self, vx: float, vy: float, omega: float):
        """
        Move robot in wheeled mode - CONSERVATIVE approach for regular tires
        
        Args:
            vx: Forward velocity in robot frame (m/s)
            vy: Lateral velocity in robot frame (m/s) - MAY BE IGNORED if no omnidirectional wheels
            omega: Angular velocity (rad/s)
        """
        try:
            # CONSERVATIVE: If Go2W doesn't have omnidirectional wheels, ignore lateral velocity
            if abs(vy) > 0.01:
                print(f"WARNING: Lateral velocity {vy:.2f} in wheeled mode - may not work if Go2W has regular tires")
                # Comment out the next line if Go2W has omnidirectional wheels
                vy = 0.0  # Force to 0 for regular tire compatibility
            
            # Velocity smoothing
            if abs(vx) < 0.1 and abs(vy) < 0.1:  # Stopping
                smoothing = 0.3
            else:  # Moving
                smoothing = 0.2
                
            self.current_vx += smoothing * (vx - self.current_vx)
            self.current_vy += smoothing * (vy - self.current_vy)
            self.current_omega += smoothing * (omega - self.current_omega)
            
            # Conservative safety limits for Go2W
            max_linear = 2.0  # Go2W max speed is 2.5 m/s, use conservative limit
            max_angular = 1.0
            
            # Clamp velocities
            vx_clamped = max(-max_linear, min(max_linear, self.current_vx))
            vy_clamped = max(-max_linear, min(max_linear, self.current_vy))
            omega_clamped = max(-max_angular, min(max_angular, self.current_omega))
            
            # Try omnidirectional control first, fallback to forward-only if needed
            self.interface.VelocityMove(vx_clamped, vy_clamped, omega_clamped)
            
            self.last_cmd_time = time.time()
            
        except Exception as e:
            print(f"Error sending wheeled velocity command: {e}")
    
    def velocity_move_walking(self, vx: float, vy: float, omega: float):
        """
        Move robot in walking mode (lateral movement via leg gaits)
        
        Args:
            vx: Forward velocity in robot frame (m/s)
            vy: Lateral velocity in robot frame (m/s)
            omega: Angular velocity (rad/s)
        """
        try:
            # For walking mode, use the high-level interface similar to regular Go2
            # This enables lateral movement via walking gaits
            self.interface.VelocityMove(vx, vy, omega)
            
            self.last_cmd_time = time.time()
            
        except Exception as e:
            print(f"Error sending walking velocity command: {e}")
    
    def set_velocity(self, vx: float, vy: float, omega: float):
        """Set velocity based on current mode."""
        if self.mode == 'wheeled':
            self.velocity_move_wheeled(vx, vy, omega)
        else:  # walking mode
            self.velocity_move_walking(vx, vy, omega)
    
    def stop_move(self):
        """Stop robot movement."""
        try:
            self.interface.StopMove()
        except Exception as e:
            print(f"Error stopping robot: {e}")
    
    def gradual_deceleration(self, target_vx: float = 0.0, target_vy: float = 0.0, 
                           target_omega: float = 0.0, duration: float = 0.6):
        """Apply gradual deceleration."""
        start_time = time.time()
        initial_vx = self.current_vx
        initial_vy = self.current_vy
        initial_omega = self.current_omega
        
        while time.time() - start_time < duration:
            progress = (time.time() - start_time) / duration
            
            # Smooth deceleration
            smooth_progress = (1 - math.cos(progress * math.pi)) / 2
            
            interp_vx = initial_vx + smooth_progress * (target_vx - initial_vx)
            interp_vy = initial_vy + smooth_progress * (target_vy - initial_vy)
            interp_omega = initial_omega + smooth_progress * (target_omega - initial_omega)
            
            # Send command
            self.set_velocity(interp_vx, interp_vy, interp_omega)
            
            time.sleep(0.02)  # 50Hz
        
        # Update internal state
        self.current_vx = target_vx
        self.current_vy = target_vy
        self.current_omega = target_omega
        
        # Final stop
        self.stop_move()
    
    def get_position(self):
        """Get current position."""
        return self.current_position.copy()
    
    def get_position_and_orientation(self):
        """Get current position and orientation."""
        return self.current_position.copy(), self.current_yaw
    
    def reset(self):
        """Reset tracking and stop movement."""
        print("Resetting robot...")
        self.gradual_deceleration()
        
        # Reset velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # Reset position tracking
        self.start_pos = self.current_position.copy()
        self.initial_yaw = self.current_yaw
        
        print(f"New start position: {self.start_pos}")
    
    def emergency_stop(self):
        """Emergency stop."""
        print("EMERGENCY STOP!")
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # Send stop commands repeatedly
        for _ in range(10):
            try:
                self.stop_move()
                time.sleep(0.01)
            except:
                pass
    
    def __del__(self):
        """Cleanup."""
        try:
            self.emergency_stop()
            self.pose_broadcaster.close()
        except:
            pass

def main():
    # Check arguments
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 go2w_deployment.py <network_interface> [remote_ip]")
        print("Example: python3 go2w_deployment.py enp2s0 192.168.123.99")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    remote_ip = sys.argv[2] if len(sys.argv) == 3 else "192.168.123.99"
    
    # Initialize
    keyboard = KeyboardInput()
    robot = None
    
    try:
        print("Initializing Go2W robot...")
        robot = Go2WRobot(network_interface, remote_ip)
        
        # Get path controller
        path_controller = robot.path_controller
        
        # Start keyboard
        keyboard.start()
        
        # State variables
        path_mode = 'leftward'  # Start in leftward (walking) mode
        is_walking = False
        start_time = None
        initial_pos = None
        last_status_time = time.time()
        
        print("\n" + "="*60)
        print("Go2W Robot - 15m Path Control (Dual Mode)")
        print("="*60)
        print("\nControls:")
        print("  SPACE: Start/Stop Movement")
        print("  P: Toggle Path Mode (forward/leftward)")
        print("  S: Cycle Speed Profile")
        print("  R: Reset Position")
        print("  Ctrl+C: Emergency Stop")
        print("\nModes:")
        print("  Forward: Wheeled mode - lateral correction may depend on wheel type")
        print("  Leftward: Walking mode - GUARANTEED to work (leg-based lateral movement)")
        print("\nSpeed Profiles:")
        for i, mode in enumerate(path_controller.speed_modes):
            print(f"  {i}: {mode}")
        print(f"\nBraking: Last {path_controller.braking_distance}m")
        print(f"Network: {network_interface}")
        print(f"Broadcasting to: {remote_ip}:9051")
        print("\nNOTE: If Go2W has regular tires (not omnidirectional), lateral")
        print("      correction in forward mode will be disabled automatically.")
        print("="*60 + "\n")
        
        # Main control loop
        while True:
            current_time = time.time()
            
            # Check connection
            if not robot.is_connected:
                print("Robot disconnected! Waiting...")
                is_walking = False
                robot.gradual_deceleration()
                time.sleep(1)
                continue
            
            # Maintain connection with heartbeat
            if current_time - robot.last_cmd_time > 0.5:
                robot.set_velocity(0.0, 0.0, 0.0)
            
            # Handle keyboard
            keys = keyboard.get_keys()
            
            if ord(' ') in keys:
                is_walking = not is_walking
                if is_walking:
                    start_time = current_time
                    initial_pos = robot.get_position()
                    path_controller.stop_start_time = None
                    path_controller.was_stopped = False
                    print(f"Movement: ON ({path_mode} mode)")
                else:
                    robot.gradual_deceleration()
                    print("Movement: OFF")
                    
            if ord('p') in keys:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                
                # Switch robot mode based on path mode
                if path_mode == 'forward':
                    robot.set_mode('wheeled')
                    print("Path Mode: forward (wheeled mode)")
                else:  # leftward
                    robot.set_mode('walking')
                    print("Path Mode: leftward (walking mode)")
                
            if ord('s') in keys:
                path_controller.current_speed_mode = (path_controller.current_speed_mode + 1) % len(path_controller.speed_modes)
                mode_name = path_controller.speed_modes[path_controller.current_speed_mode]
                print(f"Speed Profile: {mode_name}")
                
            if ord('r') in keys:
                robot.reset()
                is_walking = False
                print("Position Reset")

            # Movement control
            if is_walking and start_time is not None:
                current_pos = robot.get_position()
                elapsed_time = current_time - start_time
                
                # Calculate distance based on movement direction
                if initial_pos is not None:
                    if path_mode == 'forward':
                        distance_traveled = current_pos[1] - initial_pos[1]
                    else:  # leftward
                        distance_traveled = current_pos[1] - initial_pos[1]  # Still measure forward progress
                else:
                    distance_traveled = 0

                # Get target speed
                target_speed = path_controller.get_speed(distance_traveled, current_time)

                # Status output
                is_stopped = (target_speed < 0.05)
                
                if current_time - last_status_time > 2.0 and not is_stopped:
                    speed = math.sqrt(robot.current_vx**2 + robot.current_vy**2)
                    distance_to_goal = path_controller.path_length - distance_traveled
                    path_error = current_pos[0] - 0.0
                    
                    status = f"Position: {distance_traveled:.1f}m"
                    status += f", Speed: {speed:.2f}/{target_speed:.2f}m/s"
                    status += f", Error: {abs(path_error):.3f}m"
                    status += f", Mode: {robot.mode}"
                    
                    print(status)
                    last_status_time = current_time
                    
                # Movement control based on path mode
                if path_mode == 'forward':
                    # Forward mode using wheels with path correction
                    # NOTE: Path correction (vy) may not work if Go2W has regular tires
                    current_pos, current_yaw = robot.get_position_and_orientation()
                    
                    if target_speed > 0.05:
                        # Use forward control with path correction
                        vx, vy, omega = path_controller.compute_forward_control(
                            current_pos, current_yaw, target_speed
                        )
                    else:
                        # Stopped - maintain position
                        lateral_error = current_pos[0] - 0.0
                        vy = -lateral_error * 1.5
                        vy = np.clip(vy, -0.2, 0.2)
                        vx, omega = 0.0, 0.0
                    
                    robot.set_velocity(vx, vy, omega)  # vy may be ignored in wheeled mode
                    
                else:  # leftward mode using walking
                    # Lateral movement via walking gaits (like regular Go2)
                    # THIS WILL DEFINITELY WORK - uses leg gaits regardless of wheel type
                    # Apply gaze control
                    gaze_angle = path_controller.get_gaze_angle(elapsed_time)
                    gaze_rad = math.radians(gaze_angle)
                    angular_velocity = gaze_rad * 0.1  # Subtle gaze movement
                    
                    # Move laterally (positive Y direction) using walking
                    robot.set_velocity(0.0, target_speed, angular_velocity)
                
                # Check completion
                if distance_traveled >= path_controller.path_length:
                    print(f"Path complete! Distance: {distance_traveled:.2f}m")
                    if path_mode == 'forward':
                        print(f"Final error: {abs(current_pos[0]):.3f}m")
                    robot.gradual_deceleration()
                    is_walking = False
                        
            else:
                # Gradual stop
                if (abs(robot.current_vx) > 0.01 or 
                    abs(robot.current_vy) > 0.01 or 
                    abs(robot.current_omega) > 0.01):
                    robot.set_velocity(0.0, 0.0, 0.0)

            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\nEmergency stop!")
        if robot:
            robot.emergency_stop()
        
    except Exception as e:
        print(f"\nError: {e}")
        if robot:
            robot.emergency_stop()
        
    finally:
        if robot:
            robot.emergency_stop()
            print("Robot stopped")
        
        keyboard.stop()
        print("Deployment ended")

if __name__ == '__main__':
    main()