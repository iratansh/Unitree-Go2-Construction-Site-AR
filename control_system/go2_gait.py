#!/usr/bin/env python3
"""
Go2 Real Robot Deployment Script

This script adapts the go2_gait_simulation.py for real Go2 EDU hardware.
It imports the PathController class and replicates the interface for seamless deployment.

Requirements:
- Go2 EDU connected via Ethernet
- Network configured (robot IP: 192.168.123.161, computer IP: 192.168.123.99)

Usage:
    python3 go2_gait.py <network_interface>
    Example: python3 go2_gait.py enp2s0

Safety:
- Always test in open, safe area with padding/mats
- Have emergency stop ready (Ctrl+C)
- Start with slow speeds
- Ensure stable network connection
"""

import sys
import time
import math
import threading
import termios
import tty
import select
from typing import List, Tuple, Optional

try:
    from go2_gait_simulation import PathController
except ImportError:
    print("Error: Cannot import from go2_gait_simulation.py")
    sys.exit(1)

try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeCmd_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import Thread
except ImportError as e:
    print("Error importing Unitree SDK:")
    print(f"   {e}")
    print("\nPlease install unitree_sdk2py:")
    print("   git clone https://github.com/unitreerobotics/unitree_sdk2_python.git")
    print("   cd unitree_sdk2_python")
    print("   pip3 install -e .")
    sys.exit(1)

class KeyboardInput:
    """Non-blocking keyboard input handler for real-time control"""
    
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
                        # Convert to ASCII code for consistency with PyBullet
                        key_code = ord(key.lower()) if len(key) == 1 else 0
                        
                        # Special key mappings
                        if key == '\x1b':  # ESC sequence start
                            # Read arrow keys
                            if select.select([sys.stdin], [], [], 0.1)[0]:
                                key2 = sys.stdin.read(1)
                                if key2 == '[' and select.select([sys.stdin], [], [], 0.1)[0]:
                                    key3 = sys.stdin.read(1)
                                    if key3 == 'A': key_code = 65362  # UP
                                    elif key3 == 'B': key_code = 65364  # DOWN
                                    elif key3 == 'C': key_code = 65363  # RIGHT
                                    elif key3 == 'D': key_code = 65361  # LEFT
                        elif key == ' ':
                            key_code = ord(' ')
                        elif key == '\x03':  # Ctrl+C
                            self.running = False
                            raise KeyboardInterrupt()
                            
                        # Record key event
                        self.key_events[key_code] = 3  # KEY_WAS_TRIGGERED equivalent
                        
                except KeyboardInterrupt:
                    raise
                except:
                    continue
                    
    def get_keys(self):
        """Get current key events (mimics PyBullet interface)"""
        keys = self.key_events.copy()
        self.key_events.clear()
        return keys

class Go2RealRobot:
    """Real Go2 robot interface that matches the simulation robot interface"""
    
    def __init__(self, network_interface: str = "enp2s0"):
        self.network_interface = network_interface
        self.start_pos = [0.0, 0.0, 0.28]  # Default starting position
        self.initial_yaw = 0.0
        self.current_yaw = 0.0
        
        # Initialize DDS communication
        print(f"Connecting to Go2 via {network_interface}...")
        
        try:
            # Publishers for robot control
            self.cmd_publisher = ChannelPublisher("rt/sportmodecommand", SportModeCmd_)
            self.cmd_publisher.Init()
            
            # Subscribers for robot state  
            self.state_subscriber = ChannelSubscriber("rt/sportmodestate", unitree_go_msg_dds__SportModeState_)
            self.state_subscriber.Init()
            
            # Robot state
            self.current_position = [0.0, 0.0, 0.28]
            self.is_connected = False
            self.last_cmd_time = time.time()
            
            # Start state monitoring
            self._start_state_monitoring()
            
            # Wait for connection
            timeout = 5.0
            start_time = time.time()
            while not self.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
            if self.is_connected:
                print("Go2 connection established")
                print("SAFETY: Robot is now under software control")
            else:
                raise Exception("Timeout waiting for robot connection")
            
        except Exception as e:
            print(f"Failed to connect to Go2: {e}")
            raise
    
    def _start_state_monitoring(self):
        """Start monitoring robot state in background thread"""
        def monitor_state():
            while True:
                try:
                    msg = self.state_subscriber.Read()
                    if msg:
                        self.current_position = [
                            msg.position[0], 
                            msg.position[1], 
                            msg.body_height
                        ]
                        self.current_yaw = msg.imu_state.rpy[2]  # Yaw angle
                        self.is_connected = True
                    time.sleep(0.02)  # 50Hz monitoring
                except Exception as e:
                    if self.is_connected:
                        print(f"Lost connection to robot: {e}")
                        self.is_connected = False
                    continue
                    
        self.monitor_thread = threading.Thread(target=monitor_state, daemon=True)
        self.monitor_thread.start()
    
    def set_base_velocity(self, linear_velocity: List[float], angular_velocity: List[float]):
        """Set robot base velocity (matches simulation interface)"""
        try:
            # Safety check - limit velocities
            max_linear_speed = 3.5  # m/s
            max_angular_speed = 2.0  # rad/s
            
            # Clamp velocities
            vx = max(-max_linear_speed, min(max_linear_speed, linear_velocity[0]))
            vy = max(-max_linear_speed, min(max_linear_speed, linear_velocity[1]))
            vyaw = max(-max_angular_speed, min(max_angular_speed, angular_velocity[2]))
            
            cmd = SportModeCmd_()
            cmd.mode = 2  # Speed mode
            cmd.gait_type = 1  # Trot gait
            cmd.speed_level = 0  # Normal speed
            cmd.foot_raise_height = 0.08  # Foot lift height
            cmd.body_height = 0.28  # Body height
            cmd.position = [0.0, 0.0]  # Not used in speed mode
            
            # Set velocities
            cmd.velocity = [float(vx), float(vy), float(vyaw)]
            cmd.yaw_speed = float(vyaw)
            
            # Calculate CRC for command validation
            crc = CRC()
            cmd.crc = crc.Crc(cmd)
            
            self.cmd_publisher.Write(cmd)
            self.last_cmd_time = time.time()
            
        except Exception as e:
            print(f"Error sending velocity command: {e}")
    
    def get_position(self) -> List[float]:
        """Get current robot position (matches simulation interface)"""
        return self.current_position.copy()
    
    def reset(self):
        """Reset robot to starting position and stop movement"""
        print("Resetting robot...")
        # Stop all movement
        self.set_base_velocity([0, 0, 0], [0, 0, 0])
        
        # Reset position tracking (robot doesn't physically teleport)
        self.start_pos = self.current_position.copy()
        self.initial_yaw = self.current_yaw
        print(f"New starting position: {self.start_pos}")
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all movement"""
        print("EMERGENCY STOP!")
        for _ in range(10):  # Send multiple stop commands
            self.set_base_velocity([0, 0, 0], [0, 0, 0])
            time.sleep(0.01)
    
    def __del__(self):
        """Cleanup on deletion"""
        try:
            self.emergency_stop()
        except:
            pass

def print_safety_warning():
    """Print important safety information"""
    print("\n" + "*" * 20)
    print("           SAFETY WARNING")
    print("*" * 20)
    print("• Robot is under software control")
    print("• Ensure clear 20m+ space around robot")
    print("• Keep emergency stop ready (Ctrl+C)")
    print("• Start with slow speeds")
    print("• Have someone supervise operation")
    print("• Use padding/mats around robot")
    print("*" * 20 + "\n")
    
    response = input("Type 'I UNDERSTAND' to continue: ")
    if response != "I UNDERSTAND":
        print("Operation cancelled for safety.")
        sys.exit(1)

def main():
    # Check command line arguments
    if len(sys.argv) != 2:
        print("Usage: python3 go2_gait.py <network_interface>")
        print("Example: python3 go2_gait.py enp2s0")
        print("\nTo find your network interface, run: ifconfig")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    
    # Safety warning
    print_safety_warning()
    
    # Initialize keyboard input
    keyboard = KeyboardInput()
    robot = None
    
    try:
        # Initialize robot and controller
        print("Initializing Go2 robot...")
        robot = Go2RealRobot(network_interface)
        
        # Use the same PathController from simulation
        path_controller = PathController()
        
        # Initialize keyboard monitoring
        keyboard.start()
        
        # Simulation state (matches original)
        path_mode = 'leftward'  # Start in leftward mode
        is_walking = False
        start_time = None
        initial_pos = None
        last_status_time = time.time()
        
        print("\n" + "="*50)
        print("Go2 Real Robot - 15m Linear Path")
        print("="*50)
        print("Controls:")
        print("  SPACE: Start/Stop Walking")
        print("  p: Switch Path Mode (forward/leftward)")
        print("  s: Cycle Speed Mode")
        print("  g: Toggle Gaze Mode")
        print("  r: Reset Robot")
        print("  Ctrl+C: Emergency Stop")
        print("\nSpeed Modes:")
        for i, mode in enumerate(path_controller.speed_modes):
            print(f"  {i}: {mode}")
        print(f"\nStarting Mode: {path_mode}")
        print(f"Network Interface: {network_interface}")
        print("="*50 + "\n")
        
        # Main control loop
        while True:
            current_time = time.time()
            
            # Connection watchdog
            if not robot.is_connected:
                print("Robot disconnected! Attempting to reconnect...")
                is_walking = False
                robot.set_base_velocity([0, 0, 0], [0, 0, 0])
                time.sleep(1)
                continue
            
            # Send heartbeat command to maintain connection
            if current_time - robot.last_cmd_time > 0.5:
                robot.set_base_velocity([0, 0, 0], [0, 0, 0])
            
            # Handle keyboard input (matches simulation logic)
            keys = keyboard.get_keys()
            
            if ord(' ') in keys:
                is_walking = not is_walking
                if is_walking:
                    start_time = current_time
                    initial_pos = robot.get_position()
                    path_controller.was_stopped = False
                    path_controller.stop_start_time = None  # Reset stop timer when starting
                    print(f"Walking: ON")
                else:
                    robot.set_base_velocity([0, 0, 0], [0, 0, 0])
                    print(f"Walking: OFF")
                    
            if ord('p') in keys:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                print(f" Path Mode: {path_mode}")
                print("   Note: Robot will gradually adjust orientation while moving")
                
            if ord('s') in keys:
                path_controller.current_speed_mode = (path_controller.current_speed_mode + 1) % len(path_controller.speed_modes)
                current_mode = path_controller.speed_modes[path_controller.current_speed_mode]
                gaze_status = "with gaze" if path_controller.gaze_enabled else "no gaze"
                print(f" Speed Mode: {current_mode} ({gaze_status})")
                
            if ord('g') in keys:
                path_controller.gaze_enabled = not path_controller.gaze_enabled
                current_mode = path_controller.speed_modes[path_controller.current_speed_mode]
                gaze_status = "with gaze" if path_controller.gaze_enabled else "no gaze"
                print(f" Gaze Mode: {'ON' if path_controller.gaze_enabled else 'OFF'}")
                print(f"   Current: {current_mode} ({gaze_status})")
                
            if ord('r') in keys:
                robot.reset()
                is_walking = False
                path_mode = 'leftward'
                path_controller.was_stopped = False
                path_controller.stop_start_time = None  # Reset stop timer
                print(" Robot Reset")

            # Main walking logic (matches simulation)
            if is_walking and start_time is not None:
                elapsed_time = current_time - start_time
                current_pos = robot.get_position()
                
                # Calculate distance traveled
                if initial_pos is not None:
                    distance_traveled = math.sqrt((current_pos[0] - initial_pos[0])**2 + 
                                                (current_pos[1] - initial_pos[1])**2)
                else:
                    distance_traveled = 0
                
                # Get current speed from path controller
                target_speed = path_controller.get_speed(distance_traveled, current_time)
                gaze_angle = path_controller.get_gaze_angle(elapsed_time)
                
                # Handle stop/resume console output
                is_stopped = (target_speed == 0.0)
                if is_stopped and not path_controller.was_stopped:
                    print(f"⏸  Robot STOPPED at {distance_traveled:.1f}m")
                    path_controller.was_stopped = True
                elif not is_stopped and path_controller.was_stopped:
                    print(f" Robot RESUMED at {distance_traveled:.1f}m")
                    path_controller.was_stopped = False
                
                # Status update every 2 seconds
                if current_time - last_status_time > 2.0 and target_speed > 0:
                    print(f" Distance: {distance_traveled:.1f}m, Speed: {target_speed:.1f}m/s")
                    last_status_time = current_time
                
                # Calculate velocities based on path mode
                if path_mode == 'forward':
                    # Move forward (robot's X direction when facing forward)
                    linear_velocity = [target_speed, 0, 0]
                    
                    # Add rotation to face forward if needed
                    yaw_error = -robot.current_yaw  # Target yaw is 0 for forward
                    yaw_correction = max(-0.5, min(0.5, yaw_error * 0.5))  # P-controller
                    
                else:  # leftward
                    # Crab-walk sideways (move in Y direction)
                    linear_velocity = [0, target_speed, 0]
                    
                    # Maintain perpendicular orientation (90 degrees)
                    target_yaw = math.pi / 2
                    yaw_error = target_yaw - robot.current_yaw
                    yaw_correction = max(-0.5, min(0.5, yaw_error * 0.5))  # P-controller
                
                # Apply gaze control through angular velocity
                gaze_rad = math.radians(gaze_angle) if path_controller.gaze_enabled else 0
                angular_velocity = [0, 0, yaw_correction + gaze_rad * 0.1]
                
                # Send commands to robot
                robot.set_base_velocity(linear_velocity, angular_velocity)
                
                # Check if path completed
                if distance_traveled >= path_controller.path_length:
                    print(f"Path completed! Distance: {distance_traveled:.2f}m")
                    robot.set_base_velocity([0, 0, 0], [0, 0, 0])
                    is_walking = False
                    
            else:
                # Stop and maintain stationary
                robot.set_base_velocity([0, 0, 0], [0, 0, 0])

            time.sleep(0.02)  # 50Hz control loop
            
    except KeyboardInterrupt:
        print("\nEmergency stop activated!")
        if robot:
            robot.emergency_stop()
        
    except Exception as e:
        print(f"\nError: {e}")
        if robot:
            robot.emergency_stop()
        
    finally:
        # Cleanup
        if robot:
            robot.set_base_velocity([0, 0, 0], [0, 0, 0])
            print("Robot movement stopped")
        
        keyboard.stop()
        print("Program ended")

if __name__ == '__main__':
    main()