"""
Go2W Ethernet Control - Using Official Unitree SDK over Ethernet

This solution uses the official Unitree SDK with ethernet connection for
the most reliable and lowest latency control possible.

Requirements:
    - Official Unitree SDK installed at /home/unitree/unitree_sdk2_python
    - Ethernet connection to robot at 192.168.123.18
    - numpy for calculations
    
Usage:
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
from typing import Optional

sys.path.append('/home/unitree/unitree_sdk2_python')

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    _SDK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Official Unitree SDK not available: {e}")
    _SDK_AVAILABLE = False

# Network interface for ethernet connection
NETWORK_INTERFACE = "eth0"  # Default ethernet interface
ROBOT_IP = "192.168.123.18"  # Ethernet IP address

class Go2EthernetControl:
    """Official Unitree SDK control over Ethernet - Most reliable method"""
    
    def __init__(self, network_interface="eth0"):
        if not _SDK_AVAILABLE:
            raise RuntimeError("Official Unitree SDK not available. Please install it first.")
        
        # Initialize the channel factory with network interface
        print(f"🔌 Initializing ethernet connection via {network_interface}...")
        ChannelFactoryInitialize(0, network_interface)
        
        # Initialize sport client
        self.client = SportClient()
        self.client.SetTimeout(10.0) 
        self.client.Init()
        
        # Robot state for UI
        self.position = [0.0, 0.0, 0.35]
        self.yaw = 0.0
        self.is_standing = False
        
        # Velocity smoothing for UI display
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        print(f"✅ Ethernet SDK connection initialized via {network_interface}")
        print(f"📡 Using official SportClient API")
        
    def stand_up(self):
        """Stand up the robot using official SDK"""
        print("🤖 Standing up...")
        code = self.client.StandUp()
        if code == 0:
            print("✅ Stand up successful")
            balance_code = self.client.BalanceStand()
            if balance_code == 0:
                print("✅ Balance stand enabled")
                self.is_standing = True
                return True
            else:
                print(f"❌ Balance stand failed: {balance_code}")
                return False
        else:
            print(f"❌ Stand up failed: {code}")
            return False
    
    def sit_down(self):
        """Sit down the robot using official SDK"""
        print("🤖 Sitting down...")
        code = self.client.StandDown()
        if code == 0:
            print("✅ Sit down successful")
            self.is_standing = False
            return True
        else:
            print(f"⚠️ StandDown failed ({code}), trying Damp mode...")
            damp_code = self.client.Damp()
            if damp_code == 0:
                print("✅ Damp mode activated (robot should relax)")
                self.is_standing = False
                return True
            else:
                print(f"❌ Both StandDown and Damp failed: StandDown={code}, Damp={damp_code}")
                return False
        
    def set_velocity(self, vx, vy, omega):
        """Send velocity command using official SDK"""
        # Update smoothed values ONLY for UI display - don't use for actual commands
        if abs(vx) < 0.1 and abs(vy) < 0.1:
            smoothing = 0.3
        else:
            smoothing = 0.2
            
        self.current_vx += smoothing * (vx - self.current_vx)
        self.current_vy += smoothing * (vy - self.current_vy)
        self.current_omega += smoothing * (omega - self.current_omega)
        
        # Use the ACTUAL input velocities, not the smoothed ones
        vx_clamped = np.clip(vx, -3.0, 3.0)  # Forward/backward limits
        vy_clamped = np.clip(vy, -1.5, 1.5)  # Lateral movement typically has lower limits
        omega_clamped = np.clip(omega, -1.5, 1.5)  # Rotation limits
        
        # Debug when lateral speeds are being limited
        if abs(vy) > 1.5 and abs(vy_clamped) <= 1.5:
            print(f"\n[LATERAL_LIMIT] Requested vy={vy:.2f} clamped to {vy_clamped:.2f}")
        
        # Send stop command if all velocities are zero
        if abs(vx_clamped) < 0.01 and abs(vy_clamped) < 0.01 and abs(omega_clamped) < 0.01:
            code = self.client.StopMove()
        else:
            # Send move command via official SDK
            code = self.client.Move(vx_clamped, vy_clamped, omega_clamped)
        
        # Update position estimate (dead reckoning for UI)
        dt = 0.02
        old_pos = self.position.copy()
        self.position[0] += vx_clamped * dt
        self.position[1] += vy_clamped * dt
        self.yaw += omega_clamped * dt
        
        # Debug position updates occasionally
        if abs(vx_clamped) > 0.1 or abs(vy_clamped) > 0.1:
            if int(time.time() * 10) % 50 == 0:  # Every ~5 seconds when moving
                print(f"\n[POSITION] Old: {old_pos}, New: {self.position}, Delta: vx={vx_clamped*dt:.3f}, vy={vy_clamped*dt:.3f}")
        
        # Debug output
        if abs(vx_clamped) > 0.01 or abs(vy_clamped) > 0.01 or abs(omega_clamped) > 0.01:
            print(f"\rMove cmd: vx={vx_clamped:.2f}, vy={vy_clamped:.2f}, ω={omega_clamped:.2f}, result={code}", end='', flush=True)
        elif code == 0:  # Show stop commands too
            print(f"\rStop cmd sent, result={code}", end='', flush=True)
        
        return code == 0
        
    def stop(self):
        """Stop movement using official SDK"""
        code = self.client.StopMove()
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        return code == 0
        
    def get_state(self):
        """Get robot state using official SDK"""
        keys = ["pose", "velocity", "imu"]
        code, data = self.client.GetState(keys)
        if code == 0:
            return data
        return None
        
    def close(self):
        """Cleanup - sit down robot safely"""
        print("🔄 Safely shutting down...")
        self.stop()
        time.sleep(1.0)  
        self.sit_down()
        time.sleep(2.0)  

class SimplePathController:
    """Path controller with multiple speed profiles"""
    def __init__(self):
        self.path_length = 15.0
        self.braking_distance = 1.5
        self.speed_modes = [
            "0.5-3.0_gradual",  # 0.5m/s to 3.0m/s gradual increase 
            "3.0-0.5_gradual",  # 3.0m/s to 0.5m/s gradual decrease  
            "1.0_stop_1.0",     # 1.0m/s, stop at 8m, then 1.0m/s
            "3.0_stop_3.0"      # 3.0m/s, stop at 8m, then 3.0m/s
        ]
        self.current_speed_mode = 0
        self.gaze_enabled = False
        self.was_stopped = False
        self.stop_start_time = None
        self.last_speed = 0.0
        self.speed_smoothing = 0.85
        
    def get_speed(self, distance_traveled, current_time=None):
        """Calculate current speed based on selected profile"""
        mode = self.speed_modes[self.current_speed_mode]
        
        if mode == "0.5-3.0_gradual":
            progress = min(1.0, distance_traveled / self.path_length)
            base_speed = 0.5 + progress * 2.5  # 0.5 to 3.0 m/s
            
        elif mode == "3.0-0.5_gradual":
            progress = min(1.0, distance_traveled / self.path_length)
            base_speed = max(0.5, 3.0 - progress * 2.5)  # 3.0 to 0.5 m/s
            
        elif mode == "1.0_stop_1.0":
            if distance_traveled < 8.0:
                base_speed = 1.0
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
                    base_speed = 1.0
            else:  # distance_traveled >= 10.0 - continue at 1.0 without stopping again
                base_speed = 1.0
                
        elif mode == "3.0_stop_3.0":
            if distance_traveled < 8.0:
                base_speed = 3.0
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
                    base_speed = 3.0
            else:  # distance_traveled >= 10.0 - continue at 3.0 without stopping again
                base_speed = 3.0
        else:
            base_speed = 1.0
        
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
        path_mode = 'forward'
        is_walking = False
        start_time = None
        initial_pos = None
        last_status_time = time.time()
        
        print("\n" + "="*70)
        print("READY FOR CONTROL")
        print("="*70)
        
        print("\n🎮 Controls:")
        print("   SPACE  : Start/Stop Movement")
        print("   P      : Toggle Path Direction (forward/leftward)")
        print("   S      : Cycle Speed Profile")
        print("   R      : Reset Position")
        print("   ESC    : Emergency Stop")
        print("   Ctrl+C : Force Exit")
        
        print(f"\n📏 Path Settings:")
        print(f"   Length         : {path_controller.path_length}m")
        print(f"   Braking Distance: {path_controller.braking_distance}m")
        
        print("\n🚀 Press SPACE to start movement!")
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
                    print(f"🚀 Movement Started: {path_mode} direction")
                else:
                    robot.stop()
                    print("⏹️ Movement Stopped")
                    
            if ord('p') in keys:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                print(f"📍 Path Direction Changed: {path_mode}")
                
            if ord('s') in keys:
                path_controller.current_speed_mode = (path_controller.current_speed_mode + 1) % len(path_controller.speed_modes)
                mode_name = path_controller.speed_modes[path_controller.current_speed_mode]
                print(f"⚡ Speed Profile Changed: {mode_name}")
                
            if ord('r') in keys:
                robot.stop()
                robot.position = [0.0, 0.0, 0.35]
                robot.yaw = 0.0
                is_walking = False
                print("🔄 Position Reset Complete")
                
            if 27 in keys:  # ESC key pressed
                print("\n🛑 ESC pressed - Emergency stop!")
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
                    else:  # leftward
                        distance_traveled = -(current_pos[1] - initial_pos[1])  # Use Y for left
                    
                    # Debug distance tracking every 50 loops
                    if int(current_time * 50) % 100 == 0:  # Every ~2 seconds
                        print(f"\n[DEBUG] Distance={distance_traveled:.2f}m: current_pos={current_pos}, initial_pos={initial_pos}")
                else:
                    distance_traveled = 0

                # Check if path is complete first
                if distance_traveled >= path_controller.path_length:
                    print(f"\n🎯 Path Complete! Total distance: {distance_traveled:.2f}m")
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
                    print("🚀 Press SPACE to start a new path!")
                    print("📍 Press P to change direction | ⚡ Press S to change speed profile")
                    print("="*70 + "\n")
                    continue  # Skip velocity commands when path is complete

                # Get target speed
                target_speed = path_controller.get_speed(distance_traveled, current_time)

                # Periodic status output
                is_stopped = (target_speed < 0.05)
                
                if current_time - last_status_time > 2.0 and not is_stopped:
                    speed = math.sqrt(robot.current_vx**2 + robot.current_vy**2)
                    distance_to_goal = path_controller.path_length - distance_traveled
                    mode_name = path_controller.speed_modes[path_controller.current_speed_mode]
                    
                    status = f"\n📊 Distance: {distance_traveled:.1f}m"
                    status += f" | Target: {target_speed:.2f}m/s ({mode_name})"
                    status += f" | To Goal: {distance_to_goal:.1f}m"
                    
                    print(status)
                    last_status_time = current_time
                    
                # Generate control commands with actual target speed (not 1.0)
                if path_mode == 'forward':
                    vx = target_speed
                    vy = 0.0
                    omega = 0.0
                else:  # leftward
                    vx = 0.0
                    vy = -target_speed
                    omega = 0.0
                
                # Ensure we're sending the actual target speed, not smoothed display speed
                robot.set_velocity(vx, vy, omega)
                        
            else:
                # Not walking - ensure robot is stopped
                if (abs(robot.current_vx) > 0.01 or 
                    abs(robot.current_vy) > 0.01 or 
                    abs(robot.current_omega) > 0.01):
                    robot.set_velocity(0.0, 0.0, 0.0)

            time.sleep(0.02)  # 50Hz control loop
            
    except KeyboardInterrupt:
        print("\n🛑 Emergency stop triggered!")
        if robot:
            robot.stop()
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if robot:
            try:
                robot.close()
            except Exception:
                pass
            print("✅ Robot safely stopped and seated")
        
        keyboard.stop()
        print("👋 Control ended")

if __name__ == '__main__':
    main()