"""
Unified Go2 Control System
Supports both Unity simulation and real Go2 robot with identical interface
"""

import socket
import json
import threading
import time
import math
import numpy as np
from enum import Enum
from typing import Dict, Any, Optional, Tuple
from abc import ABC, abstractmethod
from dataclasses import dataclass

try:
    import unitree_legged_sdk
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False
    print("Unitree SDK not available - simulation mode only")

@dataclass
class RobotState:
    """Common robot state representation"""
    position: Tuple[float, float, float]
    orientation: float  # yaw in degrees
    velocity: float
    is_moving: bool
    distance_traveled: float
    mode: str

class PathPlannerBase(ABC):
    """Base class for path planning algorithms"""
    
    @abstractmethod
    def calculate_next_position(self, current_state: RobotState, dt: float) -> Tuple[float, float, float]:
        pass
    
    @abstractmethod
    def calculate_orientation(self, current_state: RobotState, target_gaze: float) -> float:
        pass

class StraightPathPlanner(PathPlannerBase):
    """Simple straight-line path planner with gaze control"""
    
    def __init__(self, path_length: float = 10.0):
        self.path_length = path_length
        self.gaze_smoothing = 0.1  # Smoothing factor for gaze transitions
        
    def calculate_next_position(self, current_state: RobotState, dt: float) -> Tuple[float, float, float]:
        """Calculate next position along straight path"""
        if not current_state.is_moving:
            return current_state.position
        
        # Calculate movement based on mode
        if current_state.mode == "rightward":
            # Move along X axis (rightward)
            dx = current_state.velocity * dt
            dy = 0
        else:
            # Move along Z axis (forward) - Unity uses Z for forward
            dx = 0
            dy = current_state.velocity * dt
        
        # Apply movement
        new_x = current_state.position[0] + dx
        new_y = current_state.position[1]  # Height stays constant
        new_z = current_state.position[2] + dy
        
        return (new_x, new_y, new_z)
    
    def calculate_orientation(self, current_state: RobotState, target_gaze: float) -> float:
        """Calculate smooth orientation transitions"""
        # Smooth transition to target gaze angle
        current_yaw = current_state.orientation
        yaw_diff = target_gaze - current_yaw
        
        # Normalize angle difference to [-180, 180]
        while yaw_diff > 180:
            yaw_diff -= 360
        while yaw_diff < -180:
            yaw_diff += 360
        
        # Apply smoothing
        new_yaw = current_yaw + yaw_diff * self.gaze_smoothing
        
        return new_yaw

class AdaptivePathPlanner(PathPlannerBase):
    """Advanced path planner with construction site awareness"""
    
    def __init__(self, path_length: float = 10.0):
        self.path_length = path_length
        self.obstacle_avoidance_distance = 1.0  # meters
        self.worker_comfort_zone = 2.0  # meters
        self.gaze_adaptation_rate = 0.15
        
    def calculate_next_position(self, current_state: RobotState, dt: float) -> Tuple[float, float, float]:
        """Calculate position with subtle lateral adjustments for realism"""
        if not current_state.is_moving:
            return current_state.position
        
        base_movement = StraightPathPlanner().calculate_next_position(current_state, dt)
        
        # Add realistic swaying motion
        time_factor = time.time()
        sway_amplitude = 0.02  # 2cm lateral sway
        sway_frequency = 1.5  # Hz
        
        if current_state.mode == "forward":
            # Lateral sway when moving forward
            lateral_offset = math.sin(time_factor * sway_frequency * 2 * math.pi) * sway_amplitude
            return (base_movement[0] + lateral_offset, base_movement[1], base_movement[2])
        else:
            # Forward/back sway when moving rightward
            forward_offset = math.sin(time_factor * sway_frequency * 2 * math.pi) * sway_amplitude
            return (base_movement[0], base_movement[1], base_movement[2] + forward_offset)
    
    def calculate_orientation(self, current_state: RobotState, target_gaze: float) -> float:
        """Calculate orientation with worker-aware adjustments"""
        base_orientation = StraightPathPlanner().calculate_orientation(current_state, target_gaze)
        
        # Add subtle head movements for realism
        time_factor = time.time()
        head_motion = math.sin(time_factor * 0.5) * 5  # ±5 degree head motion
        
        return base_orientation + head_motion

class RobotControllerBase(ABC):
    """Base controller interface for both simulation and real robot"""
    
    def __init__(self):
        self.current_state = RobotState(
            position=(0, 0, 0),
            orientation=0,
            velocity=0,
            is_moving=False,
            distance_traveled=0,
            mode="forward"
        )
        self.target_speed = 0.5
        self.target_gaze = 0
        self.path_planner = StraightPathPlanner()
        
    @abstractmethod
    def connect(self) -> bool:
        pass
    
    @abstractmethod
    def disconnect(self):
        pass
    
    @abstractmethod
    def send_movement_command(self, velocity: Tuple[float, float, float], yaw_rate: float):
        pass
    
    @abstractmethod
    def get_sensor_data(self) -> Dict[str, Any]:
        pass
    
    def update(self, dt: float):
        """Common update logic"""
        if self.current_state.is_moving:
            # Update position
            new_position = self.path_planner.calculate_next_position(self.current_state, dt)
            
            # Calculate distance traveled
            dx = new_position[0] - self.current_state.position[0]
            dz = new_position[2] - self.current_state.position[2]
            distance_delta = math.sqrt(dx*dx + dz*dz)
            self.current_state.distance_traveled += distance_delta
            
            # Update state
            self.current_state.position = new_position
            self.current_state.orientation = self.path_planner.calculate_orientation(
                self.current_state, self.target_gaze
            )
            
            # Check if reached end of path
            if self.current_state.distance_traveled >= self.path_planner.path_length:
                self.stop_movement()

class UnitySimulationController(RobotControllerBase):
    """Controller for Unity simulation via UDP"""
    
    def __init__(self, unity_ip="127.0.0.1", send_port=12345, receive_port=12346):
        super().__init__()
        self.unity_ip = unity_ip
        self.send_port = send_port
        self.receive_port = receive_port
        self.send_socket = None
        self.receive_socket = None
        self.receive_thread = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """Establish connection with Unity"""
        try:
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket.bind(("", self.receive_port))
            self.receive_socket.settimeout(0.1)  # Non-blocking with timeout
            
            self.is_connected = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            print(f"Connected to Unity simulation at {self.unity_ip}:{self.send_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to Unity: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Unity"""
        self.is_connected = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.send_socket:
            self.send_socket.close()
        if self.receive_socket:
            self.receive_socket.close()
    
    def _receive_loop(self):
        """Receive status updates from Unity"""
        while self.is_connected:
            try:
                data, addr = self.receive_socket.recvfrom(1024)
                status = json.loads(data.decode())
                # Update internal state from Unity
                self.current_state.position = tuple(status.get("position", [0, 0, 0]))
                self.current_state.orientation = status.get("orientation", 0)
                self.current_state.distance_traveled = status.get("distanceTraveled", 0)
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_connected:
                    print(f"Receive error: {e}")
    
    def send_movement_command(self, velocity: Tuple[float, float, float], yaw_rate: float):
        """Send movement command to Unity"""
        if not self.is_connected:
            return
        
        command = {
            "velocity": list(velocity),
            "yawRate": yaw_rate,
            "speed": self.target_speed,
            "mode": self.current_state.mode,
            "gazeAngle": self.target_gaze,
            "isMoving": self.current_state.is_moving
        }
        
        try:
            data = json.dumps(command).encode()
            self.send_socket.sendto(data, (self.unity_ip, self.send_port))
        except Exception as e:
            print(f"Send error: {e}")
    
    def get_sensor_data(self) -> Dict[str, Any]:
        """Get simulated sensor data"""
        return {
            "obstacles": [],  # Simulated obstacle positions
            "workers": [],    # Simulated worker positions
            "surface_type": "concrete"  # Simulated surface
        }

class RealGo2Controller(RobotControllerBase):
    """Controller for real Go2 robot"""
    
    def __init__(self):
        super().__init__()
        self.udp = None
        self.cmd = None
        self.state = None
        self.control_thread = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """Connect to real Go2 robot"""
        if not UNITREE_SDK_AVAILABLE:
            print("Unitree SDK not available")
            return False
        
        try:
            # Initialize UDP communication
            self.udp = unitree_legged_sdk.UDP(unitree_legged_sdk.HIGHLEVEL, 
                                             "192.168.123.15", 8080)
            
            self.cmd = unitree_legged_sdk.HighCmd()
            self.state = unitree_legged_sdk.HighState()
            
            # Initialize command
            self.cmd.mode = 2  # Walking mode
            self.cmd.gaitType = 1  # Trot
            self.cmd.velocity = [0, 0]
            self.cmd.yawSpeed = 0
            
            self.is_connected = True
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            
            print("Connected to real Go2 robot")
            return True
        except Exception as e:
            print(f"Failed to connect to Go2: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Go2"""
        self.is_connected = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        
        # Send stop command
        if self.cmd:
            self.cmd.velocity = [0, 0]
            self.cmd.yawSpeed = 0
            if self.udp:
                self.udp.Send(self.cmd)
    
    def _control_loop(self):
        """Main control loop for real robot"""
        while self.is_connected:
            try:
                # Send command
                if self.udp and self.cmd:
                    self.udp.Send(self.cmd)
                
                # Receive state
                if self.udp:
                    self.udp.Recv(self.state)
                    
                    # Update internal state from robot
                    # TODO: Transform coordinates later
                    self.current_state.position = (
                        self.state.position[0],
                        self.state.position[1], 
                        self.state.position[2]
                    )
                    self.current_state.orientation = math.degrees(self.state.yaw)
                
                time.sleep(0.01)  # 100Hz control loop
            except Exception as e:
                if self.is_connected:
                    print(f"Control loop error: {e}")
    
    def send_movement_command(self, velocity: Tuple[float, float, float], yaw_rate: float):
        """Send movement command to real robot"""
        if not self.is_connected or not self.cmd:
            return
        
        # Convert to robot coordinate system
        if self.current_state.mode == "rightward":
            self.cmd.velocity = [0, -velocity[0]]  # Lateral movement
        else:
            self.cmd.velocity = [velocity[2], 0]  # Forward movement
        
        self.cmd.yawSpeed = yaw_rate
    
    def get_sensor_data(self) -> Dict[str, Any]:
        """Get real sensor data from robot"""
        if not self.state:
            return {}
        
        return {
            "imu": {
                "roll": self.state.imu.rpy[0],
                "pitch": self.state.imu.rpy[1],
                "yaw": self.state.imu.rpy[2]
            },
            "foot_force": self.state.footForce,
            "battery": self.state.battery,
            "temperature": self.state.temperature
        }

class UnifiedGo2Controller:
    """Unified controller that can switch between simulation and real robot"""
    
    def __init__(self, mode="simulation"):
        self.mode = mode
        self.controller = None
        self.is_running = False
        
        # Initialize appropriate controller
        if mode == "simulation":
            self.controller = UnitySimulationController()
        elif mode == "real" and UNITREE_SDK_AVAILABLE:
            self.controller = RealGo2Controller()
        else:
            print(f"Invalid mode or SDK not available: {mode}")
            self.controller = UnitySimulationController()
            self.mode = "simulation"
        
        # Control parameters
        self.speed = 0.5  # m/s
        self.gaze_angle = 0  # degrees
        self.movement_mode = "forward"
        
    def connect(self):
        """Connect to robot/simulation"""
        return self.controller.connect()
    
    def disconnect(self):
        """Disconnect from robot/simulation"""
        self.stop()
        self.controller.disconnect()
    
    def start(self):
        """Start movement"""
        self.controller.current_state.is_moving = True
        self.controller.target_speed = self.speed
        self.controller.target_gaze = self.gaze_angle
        self.controller.current_state.mode = self.movement_mode
        self.is_running = True
        
        # Start update thread
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        print(f"Started {self.movement_mode} movement at {self.speed} m/s")
    
    def stop(self):
        """Stop movement"""
        self.controller.current_state.is_moving = False
        self.is_running = False
        if hasattr(self, 'update_thread'):
            self.update_thread.join(timeout=1.0)
        
        # Send stop command
        self.controller.send_movement_command((0, 0, 0), 0)
        print("Stopped movement")
    
    def reset(self):
        """Reset to start position"""
        self.stop()
        self.controller.current_state.position = (0, 0, 0)
        self.controller.current_state.orientation = 0
        self.controller.current_state.distance_traveled = 0
        print("Reset to start position")
    
    def _update_loop(self):
        """Main update loop"""
        dt = 0.01  # 100Hz update rate
        
        while self.is_running:
            # Update controller
            self.controller.update(dt)
            
            # Calculate movement command
            if self.controller.current_state.is_moving:
                if self.movement_mode == "rightward":
                    velocity = (self.speed, 0, 0)
                else:
                    velocity = (0, 0, self.speed)
                
                # Calculate yaw rate for gaze control
                yaw_error = self.gaze_angle - self.controller.current_state.orientation
                yaw_rate = yaw_error * 0.1  # Proportional control
                
                self.controller.send_movement_command(velocity, yaw_rate)
            
            time.sleep(dt)
    
    def set_speed(self, speed: float):
        """Set movement speed"""
        self.speed = max(0.1, min(2.0, speed))
        self.controller.target_speed = self.speed
        print(f"Speed set to {self.speed} m/s")
    
    def set_mode(self, mode: str):
        """Set movement mode"""
        if mode in ["forward", "rightward"]:
            self.movement_mode = mode
            self.controller.current_state.mode = mode
            print(f"Mode set to {mode}")
    
    def set_gaze(self, angle: float):
        """Set gaze angle"""
        self.gaze_angle = max(-90, min(90, angle))
        self.controller.target_gaze = self.gaze_angle
        print(f"Gaze angle set to {self.gaze_angle}°")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        return {
            "mode": self.mode,
            "position": self.controller.current_state.position,
            "orientation": self.controller.current_state.orientation,
            "distance_traveled": self.controller.current_state.distance_traveled,
            "is_moving": self.controller.current_state.is_moving,
            "speed": self.speed,
            "gaze_angle": self.gaze_angle,
            "movement_mode": self.movement_mode,
            "sensors": self.controller.get_sensor_data()
        }
    
    def switch_path_planner(self, planner_type: str):
        """Switch between path planning algorithms"""
        if planner_type == "straight":
            self.controller.path_planner = StraightPathPlanner()
        elif planner_type == "adaptive":
            self.controller.path_planner = AdaptivePathPlanner()
        print(f"Switched to {planner_type} path planner")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Unified Go2 Controller")
    parser.add_argument("--mode", choices=["simulation", "real"], 
                       default="simulation", help="Control mode")
    parser.add_argument("--planner", choices=["straight", "adaptive"],
                       default="straight", help="Path planner type")
    
    args = parser.parse_args()
    
    # Create controller
    controller = UnifiedGo2Controller(mode=args.mode)
    controller.switch_path_planner(args.planner)
    
    # Connect
    if not controller.connect():
        print("Failed to connect!")
        exit(1)
    
    print(f"\nGo2 Controller ({args.mode} mode with {args.planner} planner)")
    print("Commands: start, stop, reset, speed <value>, mode <forward/rightward>, gaze <angle>, status, quit")
    
    try:
        while True:
            cmd = input("\n> ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == "quit":
                break
            elif cmd[0] == "start":
                controller.start()
            elif cmd[0] == "stop":
                controller.stop()
            elif cmd[0] == "reset":
                controller.reset()
            elif cmd[0] == "speed" and len(cmd) > 1:
                controller.set_speed(float(cmd[1]))
            elif cmd[0] == "mode" and len(cmd) > 1:
                controller.set_mode(cmd[1])
            elif cmd[0] == "gaze" and len(cmd) > 1:
                controller.set_gaze(float(cmd[1]))
            elif cmd[0] == "status":
                status = controller.get_status()
                print(json.dumps(status, indent=2))
            else:
                print("Unknown command")
    
    finally:
        controller.disconnect()
        print("Disconnected")