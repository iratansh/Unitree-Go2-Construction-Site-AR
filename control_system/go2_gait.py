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
    Example: python3 go2_gait.py wlo1 192.168.123.18
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
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
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
        self.remote_ip = remote_ip
        self.start_pos = [0.0, 0.0, 0.35]
        self.initial_yaw = 0.0
        self.current_yaw = 0.0
        self.mode = 'walking'  # Default to walking mode
        
        # Initialize UDP broadcaster
        self.pose_broadcaster = PoseBroadcaster(remote_ip, 9051)
        
        # Initialize path controller
        self.path_controller = SimplePathController()
        
        print(f"Connecting to Go2W via {network_interface}...")
        
        try:
            # Try different domain IDs to avoid conflicts
            domain_ids_to_try = [0, 1, 42]  # Common domain IDs used by Unitree
            channel_init_success = False
            
            for domain_id in domain_ids_to_try:
                try:
                    print(f"Trying ChannelFactory initialization with domain ID {domain_id}...")
                    ChannelFactoryInitialize(domain_id, network_interface)
                    print(f"✓ ChannelFactory initialized successfully with domain ID {domain_id}")
                    channel_init_success = True
                    break
                except Exception as e:
                    print(f"Failed with domain ID {domain_id}: {e}")
                    continue
            
            if not channel_init_success:
                print("Trying ChannelFactory initialization without specific interface binding...")
                try:
                    ChannelFactoryInitialize(0)
                    print("✓ ChannelFactory initialized without interface binding")
                    channel_init_success = True
                except Exception as e:
                    print(f"Failed without interface binding: {e}")
                    raise Exception("Could not initialize ChannelFactory with any configuration")
            if not channel_init_success:
                raise Exception("Could not initialize ChannelFactory")
            
            # Initialize clients with proper timeout and retries
            print("Initializing SportClient...")
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(30.0)  # Increased timeout for better reliability
            
            # Initialize the client
            for attempt in range(3):  # Try up to 3 times
                print(f"SportClient initialization attempt {attempt + 1}/3...")
                try:
                    sport_init_code = self.sport_client.Init()
                    print(f"SportClient.Init() -> {sport_init_code}")
                    if sport_init_code == 0 or sport_init_code is None:  # Success or no error
                        break
                    time.sleep(2)  # Wait before retry
                except Exception as e:
                    print(f"Init attempt {attempt + 1} failed: {e}")
                    if attempt == 2:  # Last attempt
                        raise
                    time.sleep(2)
            
            # Enhanced lease acquisition with multiple attempts
            print("🔍 Attempting to acquire control lease...")
            lease_acquired = False
            max_lease_attempts = 5
            
            for attempt in range(max_lease_attempts):
                try:
                    print(f"Lease acquisition attempt {attempt + 1}/{max_lease_attempts}")
                    
                    # Try to get current lease status
                    current_lease = self.sport_client.GetLeaseId()
                    print(f"Current lease ID: {current_lease}")
                    
                    if current_lease is not None and current_lease != 0:
                        print(f"✓ Valid lease already acquired: {current_lease}")
                        lease_acquired = True
                        break
                    
                    # Wait for lease to be applied with timeout
                    print("Waiting for lease to be applied...")
                    lease_wait_start = time.time()
                    lease_timeout = 10.0  # 10 second timeout
                    
                    try:
                        # Non-blocking lease wait with timeout
                        while time.time() - lease_wait_start < lease_timeout:
                            current_lease = self.sport_client.GetLeaseId()
                            if current_lease is not None and current_lease != 0:
                                print(f"✓ Lease acquired during wait: {current_lease}")
                                lease_acquired = True
                                break
                            time.sleep(0.5)  # Check every 500ms
                        
                        if not lease_acquired:
                            print(f"⚠️  Lease acquisition timeout after {lease_timeout}s")
                    except Exception as e:
                        print(f"Lease wait error: {e}")
                    
                    if lease_acquired:
                        break
                        
                    print(f"Attempt {attempt + 1} failed, retrying in 3 seconds...")
                    time.sleep(3)
                    
                except Exception as e:
                    print(f"Lease attempt {attempt + 1} error: {e}")
                    time.sleep(2)
            
            time.sleep(1.0)  # Additional settling time
            
            # Set up state subscriber
            print("Attempting to subscribe to SportModeState...")
            try:
                self.state_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
                self.state_subscriber.Init(self._state_callback, 10)
                print("✓ SportModeState subscriber initialized")
            except Exception as e:
                print(f"⚠️  SportModeState subscription failed: {e}")
                print("   Continuing without state subscription...")
            
            # Robot state
            self.current_position = [0.0, 0.0, 0.35]
            self.is_connected = False
            self.last_cmd_time = time.time()
            
            # Velocity smoothing
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_omega = 0.0
            
            # Test connection by trying to send a command
            print("🧪 Testing robot communication...")
            
            if lease_acquired:
                print("✓ Control lease acquired successfully")
            else:
                print("⚠️  Control lease not confirmed - attempting to proceed")
            
            # Test robot responsiveness with simple commands
            robot_responsive = False
            test_commands = [
                ("BalanceStand", lambda: self.sport_client.BalanceStand()),
                ("RecoveryStand", lambda: self.sport_client.RecoveryStand()),
                ("StandUp", lambda: self.sport_client.StandUp()),
            ]
            
            for cmd_name, cmd_func in test_commands:
                try:
                    print(f"Testing {cmd_name}...")
                    result = cmd_func()
                    print(f"SportClient.{cmd_name}() -> {result}")
                    
                    if result == 0:  # Success
                        print(f"✓ {cmd_name} successful!")
                        robot_responsive = True
                        break
                    elif result == 3102:
                        print(f"❌ {cmd_name} failed with 3102 (communication error)")
                    else:
                        print(f"⚠️  {cmd_name} returned code {result}")
                        
                    time.sleep(1)  # Brief pause between commands
                    
                except Exception as e:
                    print(f"❌ {cmd_name} exception: {e}")
            
            if robot_responsive:
                print("🎉 Go2W connection established and robot is responsive!")
                print("✓ Robot will automatically use wheels for forward motion and crab walk for lateral motion")
                self.is_connected = True
            else:
                print("⚠️  Robot communication test completed but no success codes received")
                print("   This may indicate:")
                print("   1. Robot is not fully initialized (wait longer)")
                print("   2. Another controller has the lease")
                print("   3. Robot is not in the correct mode")
                print("   4. Network communication issues persist")
                
                # Allow script to continue for testing
                print("   Continuing with caution for diagnostic purposes...")
                self.is_connected = True
            
        except Exception as e:
            print(f"Failed to connect to Go2W: {e}")
            raise
    
    def _state_callback(self, msg):
        """Callback for SportModeState messages"""
        try:
            print(f"DEBUG: State callback received message: {msg is not None}")
            if msg is not None:
                print(f"DEBUG: Message type: {type(msg)}")
                print(f"DEBUG: Message attributes: {dir(msg)}")
                
                # Update position if available
                if hasattr(msg, 'position') and len(msg.position) >= 3:
                    self.current_position = [msg.position[0], msg.position[1], msg.position[2]]
                    print(f"DEBUG: Updated position: {self.current_position}")
                
                # Update orientation if available  
                if hasattr(msg, 'imu_state') and hasattr(msg.imu_state, 'rpy') and len(msg.imu_state.rpy) >= 3:
                    self.current_yaw = msg.imu_state.rpy[2]
                    print(f"DEBUG: Updated yaw: {self.current_yaw}")
                
                # Mark as connected on first successful message
                if not self.is_connected:
                    print("✓ SportModeState received - robot connected!")
                    self.is_connected = True
                    
                # Send pose update
                self.pose_broadcaster.send_pose(self.current_position, self.current_yaw)
                
        except Exception as e:
            print(f"State callback error: {e}")
    
    def _start_state_monitoring(self):
        """State monitoring is now handled by callback - this is a placeholder"""
        print("State monitoring using SportModeState callback")
    
    def stand_up(self):
        """Stand up the robot"""
        print("🤖 Standing up robot...")
        try:
            code = self.sport_client.StandUp()
            if code != 0:
                print(f"⚠️  Warning: StandUp returned code {code}")
            else:
                print("✅ Robot standing up...")
            time.sleep(3.0)  # Wait for stand up to complete
        except Exception as e:
            print(f"❌ Error during stand up: {e}")
    
    def set_velocity(self, vx: float, vy: float, omega: float):
        """
        Set velocity - Go2W automatically handles movement mode:
        - Forward/backward (vx): Uses wheels
        - Lateral (vy): Uses crab walk
        
        Args:
            vx: Forward velocity in robot frame (m/s)
            vy: Lateral velocity in robot frame (m/s) - MAY BE IGNORED if no omnidirectional wheels
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
        print("🔄 Resetting robot position and state...")
        self.gradual_deceleration()
        
        # Reset velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # Reset position tracking
        self.start_pos = self.current_position.copy()
        self.initial_yaw = self.current_yaw
        
        print(f"✅ New start position: [{self.start_pos[0]:.2f}, {self.start_pos[1]:.2f}, {self.start_pos[2]:.2f}]")
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all motion"""
        print("🚨 EMERGENCY STOP ACTIVATED!")
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

def network_diagnostics(remote_ip: str, network_interface: str):
    """Perform network diagnostics to help troubleshoot connectivity issues"""
    print("\n🔍 NETWORK DIAGNOSTICS")
    print("="*50)
    
    # Check if interface exists
    try:
        import subprocess
        result = subprocess.run(['ip', 'link', 'show', network_interface], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ Network interface '{network_interface}' exists")
        else:
            print(f"❌ Network interface '{network_interface}' not found")
            return False
    except Exception as e:
        print(f"❌ Could not check network interface: {e}")
        return False
    
    # Test basic ping connectivity
    try:
        result = subprocess.run(['ping', '-c', '3', '-W', '2', remote_ip], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ ICMP ping to {remote_ip} successful")
        else:
            print(f"❌ ICMP ping to {remote_ip} failed")
            return False
    except Exception as e:
        print(f"❌ Ping test failed: {e}")
        return False
    
    # Check firewall status for DDS ports
    try:
        result = subprocess.run(['sudo', 'ufw', 'status'], 
                              capture_output=True, text=True, timeout=5)
        if 'inactive' in result.stdout.lower():
            print("⚠️  Firewall is inactive - DDS traffic should pass through")
        elif '7400' in result.stdout and '7410' in result.stdout:
            print("✓ Firewall has DDS-specific rules configured")
        else:
            print("❌ Firewall is active but missing DDS rules")
            print("   Run: sudo ufw allow in on wlo1 proto udp from 192.168.12.0/24 to any port 7400:7650")
    except Exception as e:
        print(f"⚠️  Could not check firewall status: {e}")
    
    # Test if DDS ports are reachable (basic UDP socket test)
    import socket
    dds_ports = [7400, 7401, 7410, 7411]
    for port in dds_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            # Try to send a test packet (this won't be valid DDS but tests connectivity)
            sock.sendto(b'test', (remote_ip, port))
            sock.close()
            print(f"✓ UDP port {port} reachable")
        except Exception:
            print(f"⚠️  UDP port {port} connectivity test failed (expected for DDS)")
    
    print("="*50)
    return True

def main():
    # Check arguments
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 go2w_deployment.py <network_interface> [remote_ip]")
        print("Example: python3 go2w_deployment.py enp2s0 192.168.123.18")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    remote_ip = sys.argv[2] if len(sys.argv) == 3 else "192.168.123.18"
    
    # Run network diagnostics first
    if not network_diagnostics(remote_ip, network_interface):
        print("❌ Network diagnostics failed. Please resolve network issues before proceeding.")
        sys.exit(1)
    
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
        
        print("\n" + "="*70)
        print("           Go2W Robot - 15m Path Control (Dual Mode)")
        print("="*70)
        
        print("\n🤖 Movement Modes:")
        print("   • Forward motion → Uses wheels automatically")
        print("   • Lateral motion → Uses crab walk automatically")
        
        print("\n🎮 Controls:")
        print("   SPACE  : Start/Stop Movement")
        print("   P      : Toggle Path Direction (forward/leftward)")
        print("   S      : Cycle Speed Profile")
        print("   R      : Reset Position")
        print("   Ctrl+C : Emergency Stop")
        
        print("\n📍 Path Directions:")
        print("   Forward  : Move forward using wheels (with lateral correction)")
        print("   Leftward : Move laterally using crab walk")
        
        print("\n⚡ Speed Profiles:")
        for i, mode in enumerate(path_controller.speed_modes):
            print(f"   {i}: {mode}")
        
        print(f"\n📏 Path Settings:")
        print(f"   Length         : {path_controller.path_length}m")
        print(f"   Braking Distance: {path_controller.braking_distance}m")
        
        print(f"\n🌐 Network:")
        print(f"   Interface      : {network_interface}")
        print(f"   Broadcasting to: {remote_ip}:9051")
        
        print("\n⚠️  NOTE: If Go2W has regular tires (not omnidirectional),")
        print("          lateral correction in forward mode will be disabled.")
        print("="*70)
        
        # Main control loop
        while True:
            current_time = time.time()
            
            # Check connection status
            if not robot.is_connected:
                if is_walking:
                    print("❌ Robot disconnected! Stopping movement...")
                    is_walking = False
                    robot.gradual_deceleration()
                time.sleep(0.5)
                continue
            
            # Send heartbeat to maintain connection
            if current_time - robot.last_cmd_time > 0.5:
                robot.set_velocity(0.0, 0.0, 0.0)
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
                    print(f"🚀 Movement Started: {path_mode} direction")
                else:
                    robot.gradual_deceleration()
                    print("⏹️  Movement Stopped")
                    
            if ord('p') in keys:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                print(f"📍 Path Direction Changed: {path_mode}")
                if path_mode == 'forward':
                    print("   → Will use wheels for forward motion")
                else:
                    print("   → Will use crab walk for lateral motion")
                
            if ord('s') in keys:
                path_controller.current_speed_mode = (path_controller.current_speed_mode + 1) % len(path_controller.speed_modes)
                mode_name = path_controller.speed_modes[path_controller.current_speed_mode]
                print(f"⚡ Speed Profile Changed: {mode_name}")
                
            if ord('r') in keys:
                robot.reset()
                is_walking = False
                print("🔄 Position Reset Complete")

            # Movement control
            if is_walking and start_time is not None:
                current_pos = robot.get_position()
                elapsed_time = current_time - start_time
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
                    
                    status = f"📊 Distance: {distance_traveled:.1f}m"
                    status += f" | Speed: {speed:.2f}/{target_speed:.2f}m/s"
                    status += f" | Path Error: {path_error:.3f}m"
                    status += f" | To Goal: {distance_to_goal:.1f}m"
                    
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
                    print(f"\n🎯 Path Complete! Total distance: {distance_traveled:.2f}m")
                    if path_mode == 'forward':
                        final_error = abs(current_pos[0])
                        print(f"   Final lateral error: {final_error:.3f}m")
                    else:
                        final_error = abs(current_pos[1] - initial_pos[1])
                        print(f"   Final forward error: {final_error:.3f}m")
                    
                    robot.gradual_deceleration()
                    is_walking = False
                        
            else:
                # Not walking - ensure robot is stopped
                if (abs(robot.current_vx) > 0.01 or 
                    abs(robot.current_vy) > 0.01 or 
                    abs(robot.current_omega) > 0.01):
                    robot.set_velocity(0.0, 0.0, 0.0)
                    robot.set_velocity(0.0, 0.0, 0.0)

            time.sleep(0.02)  # 50Hz control loop
            
    except KeyboardInterrupt:
        print("\n🛑 Emergency stop triggered!")
        if robot:
            robot.emergency_stop()
        
    except Exception as e:
        print(f"\n❌ Critical Error: {e}")
        import traceback
        traceback.print_exc()
        if robot:
            robot.emergency_stop()
        
    finally:
        if robot:
            robot.emergency_stop()
            print("✅ Robot safely stopped")
        
        keyboard.stop()
        print("🏁 Deployment ended")

if __name__ == '__main__':
    main()