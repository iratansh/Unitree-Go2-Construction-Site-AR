"""
Go2W Real Robot Deployment Script - Simplified Version

The Go2W automatically handles movement modes:
- Forward/backward motion: Uses wheels
- Lateral motion: Uses crab walk (leg gaits)

This is handled internally by the SportClient.Move() API.

Requirements:
- Go2W EDU robot
- Network configured (robot IP: 192.168.123.18)  
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
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    SDK_AVAILABLE = True
    print("Successfully imported Unitree SDK2 (Go2 SportClient)")
except ImportError as e:
    print(f"SDK import failed: {e}") 
    sys.exit(1)

class SimplePathController:
    """Path controller with multiple speed profiles"""
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
        """Calculate current speed based on selected profile"""
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
    
    def compute_forward_control(self, current_pos, current_yaw, target_speed):
        """Compute control for forward movement (uses wheels automatically)"""
        target_yaw = math.pi/2  # Face toward +Y direction
        
        is_in_stop_period = (target_speed < 0.05)
        
        # Lateral correction to maintain straight path
        path_error = current_pos[0] - 0.0  # Deviation from X=0
        
        if is_in_stop_period:
            # Gentle correction during stops
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
        """Calculate gaze angle for lateral movement"""
        if not self.gaze_enabled:
            return 0.0
        return 15.0 * math.sin(0.5 * t)  # ±15 degrees

class PoseBroadcaster:
    """UDP broadcaster for digital twin synchronization"""
    def __init__(self, target_ip: str, target_port: int):
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Pose broadcaster initialized for {self.target_ip}:{self.target_port}")

    def send_pose(self, position: List[float], yaw: float):
        """Send pose data as comma-separated string"""
        try:
            pose_str = f"{position[0]},{position[1]},{position[2]},{yaw}"
            self.sock.sendto(pose_str.encode('utf-8'), (self.target_ip, self.target_port))
        except Exception:
            pass

    def close(self):
        self.sock.close()

class KeyboardInput:
    """Non-blocking keyboard input handler"""
    
    def __init__(self):
        self.keys_pressed = set()
        self.key_events = {}
        self.running = True
        self.old_settings = None
        
    def start(self):
        """Start keyboard monitoring in separate thread"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno()) 
        self.thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.thread.start()
        
    def stop(self):
        """Stop keyboard monitoring and restore terminal"""
        self.running = False
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
    def _keyboard_listener(self):
        """Listen for keyboard events"""
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
        """Get current key events"""
        keys = self.key_events.copy()
        self.key_events.clear()
        return keys

class Go2WRobot:
    """Simplified Go2W robot interface - leverages built-in movement modes"""
    
    def __init__(self, network_interface: str = "enp2s0", remote_ip: str = "192.168.123.18"):
        self.network_interface = network_interface
        self.start_pos = [0.0, 0.0, 0.35]
        self.initial_yaw = 0.0
        self.current_yaw = 0.0
        
        # Initialize UDP broadcaster
        self.pose_broadcaster = PoseBroadcaster(remote_ip, 9051)
        
        # Initialize path controller
        self.path_controller = SimplePathController()
        
        print(f"Connecting to Go2W via {network_interface}...")
        
        try:
            # Initialize channel factory with domain ID 0 for real robot
            ChannelFactoryInitialize(0, network_interface)
            
            # Initialize clients
            print("Initializing SportClient and RobotStateClient...")
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            
            self.state_client = RobotStateClient()
            self.state_client.SetTimeout(10.0)
            self.state_client.Init()
            
            # Robot state
            self.current_position = [0.0, 0.0, 0.35]
            self.is_connected = False
            self.last_cmd_time = time.time()
            
            # Velocity smoothing
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
                print("Robot will automatically use wheels for forward motion and crab walk for lateral motion")
                
                # Stand up the robot
                self.stand_up()
            else:
                print("Warning: Could not verify connection, but proceeding...")
            
        except Exception as e:
            print(f"Failed to connect to Go2W: {e}")
            raise
    
    def _start_state_monitoring(self):
        """Monitor robot state in background thread"""
        def monitor_state():
            while True:
                try:
                    code, state = self.state_client.GetRobotState()
                    if code == 0 and state is not None:
                        if hasattr(state, 'position'):
                            self.current_position = [
                                state.position[0],
                                state.position[1], 
                                state.position[2] if len(state.position) > 2 else 0.35
                            ]
                        
                        if hasattr(state, 'imu_state') and hasattr(state.imu_state, 'rpy'):
                            self.current_yaw = state.imu_state.rpy[2]
                        
                        self.is_connected = True
                        self.pose_broadcaster.send_pose(self.current_position, self.current_yaw)
                    else:
                        if self.is_connected:
                            print(f"Lost connection (code: {code})")
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
    
    def stand_up(self):
        """Stand up the robot"""
        print("Standing up robot...")
        try:
            code = self.sport_client.StandUp()
            if code != 0:
                print(f"Warning: StandUp returned code {code}")
            time.sleep(3.0)  # Wait for stand up to complete
        except Exception as e:
            print(f"Error during stand up: {e}")
    
    def set_velocity(self, vx: float, vy: float, omega: float):
        """
        Set velocity - Go2W automatically handles movement mode:
        - Forward/backward (vx): Uses wheels
        - Lateral (vy): Uses crab walk
        
        Args:
            vx: Forward velocity in robot frame (m/s)
            vy: Lateral velocity in robot frame (m/s)
            omega: Angular velocity (rad/s)
        """
        try:
            # Velocity smoothing for smoother motion
            if abs(vx) < 0.1 and abs(vy) < 0.1:  # Stopping
                smoothing = 0.3
            else:  # Moving
                smoothing = 0.2
                
            self.current_vx += smoothing * (vx - self.current_vx)
            self.current_vy += smoothing * (vy - self.current_vy)
            self.current_omega += smoothing * (omega - self.current_omega)
            
            # Safety limits
            max_linear = 2.0  # Conservative limit for Go2W
            max_angular = 1.0
            
            # Clamp velocities
            vx_clamped = np.clip(self.current_vx, -max_linear, max_linear)
            vy_clamped = np.clip(self.current_vy, -max_linear, max_linear)
            omega_clamped = np.clip(self.current_omega, -max_angular, max_angular)
            
            # Send velocity command - robot handles mode automatically
            code = self.sport_client.Move(vx_clamped, vy_clamped, omega_clamped)
            if code != 0:
                print(f"Warning: Move command returned code {code}")
            
            self.last_cmd_time = time.time()
            
        except Exception as e:
            print(f"Error sending velocity command: {e}")
    
    def stop_move(self):
        """Stop robot movement"""
        try:
            code = self.sport_client.StopMove()
            if code != 0:
                print(f"Warning: StopMove returned code {code}")
        except Exception as e:
            print(f"Error stopping robot: {e}")
    
    def gradual_deceleration(self, target_vx: float = 0.0, target_vy: float = 0.0, 
                           target_omega: float = 0.0, duration: float = 0.6):
        """Apply gradual deceleration for smooth stops"""
        start_time = time.time()
        initial_vx = self.current_vx
        initial_vy = self.current_vy
        initial_omega = self.current_omega
        
        while time.time() - start_time < duration:
            progress = (time.time() - start_time) / duration
            
            # Smooth deceleration curve
            smooth_progress = (1 - math.cos(progress * math.pi)) / 2
            
            interp_vx = initial_vx + smooth_progress * (target_vx - initial_vx)
            interp_vy = initial_vy + smooth_progress * (target_vy - initial_vy)
            interp_omega = initial_omega + smooth_progress * (target_omega - initial_omega)
            
            # Send interpolated command
            self.set_velocity(interp_vx, interp_vy, interp_omega)
            
            time.sleep(0.02)  # 50Hz
        
        # Update internal state
        self.current_vx = target_vx
        self.current_vy = target_vy
        self.current_omega = target_omega
        
        # Final stop
        self.stop_move()
    
    def get_position(self):
        """Get current position"""
        return self.current_position.copy()
    
    def get_position_and_orientation(self):
        """Get current position and orientation"""
        return self.current_position.copy(), self.current_yaw
    
    def reset(self):
        """Reset tracking and stop movement"""
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
        """Emergency stop - immediately halt all motion"""
        print("EMERGENCY STOP!")
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # Send multiple stop commands for safety
        for _ in range(10):
            try:
                self.sport_client.StopMove()
                time.sleep(0.01)
            except:
                pass
    
    def __del__(self):
        """Cleanup on deletion"""
        try:
            self.emergency_stop()
            self.pose_broadcaster.close()
        except:
            pass

def main():
    # Check arguments
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 go2w_deployment.py <network_interface> [remote_ip]")
        print("Example: python3 go2w_deployment.py enp2s0 192.168.123.18")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    remote_ip = sys.argv[2] if len(sys.argv) == 3 else "192.168.123.18"
    
    # Initialize components
    keyboard = KeyboardInput()
    robot = None
    
    try:
        print("Initializing Go2W robot...")
        robot = Go2WRobot(network_interface, remote_ip)
        
        # Get path controller reference
        path_controller = robot.path_controller
        
        # Start keyboard input
        keyboard.start()
        
        # State variables
        path_mode = 'leftward'  # Start with lateral movement
        is_walking = False
        start_time = None
        initial_pos = None
        last_status_time = time.time()
        
        print("\n" + "="*60)
        print("Go2W Robot - 15m Path Control")
        print("="*60)
        print("\nThe Go2W automatically selects movement mode:")
        print("  • Forward motion → Uses wheels")
        print("  • Lateral motion → Uses crab walk")
        print("\nControls:")
        print("  SPACE: Start/Stop Movement")
        print("  P: Toggle Path Direction (forward/leftward)")
        print("  S: Cycle Speed Profile")
        print("  R: Reset Position")
        print("  Ctrl+C: Emergency Stop")
        print("\nPath Directions:")
        print("  Forward: Move forward using wheels (with lateral correction)")
        print("  Leftward: Move laterally using crab walk")
        print("\nSpeed Profiles:")
        for i, mode in enumerate(path_controller.speed_modes):
            print(f"  {i}: {mode}")
        print(f"\nPath Length: {path_controller.path_length}m")
        print(f"Braking Distance: {path_controller.braking_distance}m")
        print(f"Network: {network_interface}")
        print(f"Broadcasting to: {remote_ip}:9051")
        print("="*60 + "\n")
        
        # Main control loop
        while True:
            current_time = time.time()
            
            # Check connection status
            if not robot.is_connected:
                if is_walking:
                    print("Robot disconnected! Stopping...")
                    is_walking = False
                    robot.gradual_deceleration()
                time.sleep(0.5)
                continue
            
            # Send heartbeat to maintain connection
            if current_time - robot.last_cmd_time > 0.5:
                robot.set_velocity(0.0, 0.0, 0.0)
            
            # Process keyboard input
            keys = keyboard.get_keys()
            
            if ord(' ') in keys:
                is_walking = not is_walking
                if is_walking:
                    start_time = current_time
                    initial_pos = robot.get_position()
                    path_controller.stop_start_time = None
                    path_controller.was_stopped = False
                    print(f"Movement: ON ({path_mode} direction)")
                else:
                    robot.gradual_deceleration()
                    print("Movement: OFF")
                    
            if ord('p') in keys:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                print(f"Path Direction: {path_mode}")
                if path_mode == 'forward':
                    print("  → Will use wheels for forward motion")
                else:
                    print("  → Will use crab walk for lateral motion")
                
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
                
                # Calculate distance traveled
                if initial_pos is not None:
                    if path_mode == 'forward':
                        # Measure forward progress (Y direction)
                        distance_traveled = current_pos[1] - initial_pos[1]
                    else:  # leftward
                        # Measure lateral progress (X direction)
                        distance_traveled = -(current_pos[0] - initial_pos[0])  # Negative because leftward is -X
                else:
                    distance_traveled = 0

                # Get target speed from controller
                target_speed = path_controller.get_speed(distance_traveled, current_time)

                # Periodic status output
                is_stopped = (target_speed < 0.05)
                
                if current_time - last_status_time > 2.0 and not is_stopped:
                    speed = math.sqrt(robot.current_vx**2 + robot.current_vy**2)
                    distance_to_goal = path_controller.path_length - distance_traveled
                    
                    if path_mode == 'forward':
                        path_error = abs(current_pos[0] - 0.0)  # Deviation from X=0
                    else:
                        path_error = abs(current_pos[1] - initial_pos[1])  # Deviation from initial Y
                    
                    status = f"Distance: {distance_traveled:.1f}m"
                    status += f", Speed: {speed:.2f}/{target_speed:.2f}m/s"
                    status += f", Path Error: {path_error:.3f}m"
                    status += f", To Goal: {distance_to_goal:.1f}m"
                    
                    print(status)
                    last_status_time = current_time
                    
                # Generate control commands based on path direction
                if path_mode == 'forward':
                    # Forward motion - robot will use wheels automatically
                    current_pos, current_yaw = robot.get_position_and_orientation()
                    
                    if target_speed > 0.05:
                        # Compute forward control with path correction
                        vx, vy, omega = path_controller.compute_forward_control(
                            current_pos, current_yaw, target_speed
                        )
                    else:
                        # Stopped - maintain position with gentle corrections
                        lateral_error = current_pos[0] - 0.0
                        vy = -lateral_error * 1.5
                        vy = np.clip(vy, -0.2, 0.2)
                        vx, omega = 0.0, 0.0
                    
                    robot.set_velocity(vx, vy, omega)
                    
                else:  # leftward mode
                    # Lateral motion - robot will use crab walk automatically
                    # Apply optional gaze control
                    gaze_angle = path_controller.get_gaze_angle(elapsed_time)
                    gaze_rad = math.radians(gaze_angle)
                    angular_velocity = gaze_rad * 0.1  # Subtle gaze movement
                    
                    # Move laterally (negative X is leftward)
                    robot.set_velocity(0.0, -target_speed, angular_velocity)  # Negative for leftward
                
                # Check if path is complete
                if distance_traveled >= path_controller.path_length:
                    print(f"\n✓ Path complete! Total distance: {distance_traveled:.2f}m")
                    if path_mode == 'forward':
                        final_error = abs(current_pos[0])
                        print(f"  Final lateral error: {final_error:.3f}m")
                    else:
                        final_error = abs(current_pos[1] - initial_pos[1])
                        print(f"  Final forward error: {final_error:.3f}m")
                    
                    robot.gradual_deceleration()
                    is_walking = False
                        
            else:
                # Not walking - ensure robot is stopped
                if (abs(robot.current_vx) > 0.01 or 
                    abs(robot.current_vy) > 0.01 or 
                    abs(robot.current_omega) > 0.01):
                    robot.set_velocity(0.0, 0.0, 0.0)

            time.sleep(0.02)  # 50Hz control loop
            
    except KeyboardInterrupt:
        print("\n⚠ Emergency stop triggered!")
        if robot:
            robot.emergency_stop()
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        if robot:
            robot.emergency_stop()
        
    finally:
        if robot:
            robot.emergency_stop()
            print("Robot safely stopped")
        
        keyboard.stop()
        print("Deployment ended")

if __name__ == '__main__':
    main()