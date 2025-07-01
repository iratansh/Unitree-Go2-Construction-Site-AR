#!/usr/bin/env python3
"""
Unitree SDK Integration for Go2 Robot
Optional: Use this if you want to control a real Go2 robot instead of Unity simulation
"""

import time
import math
from enum import Enum
from typing import Optional, Tuple

try:
    from unitree_sdk import Go2Robot, MotionCommand
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False
    print("Unitree SDK not found. Using simulation mode.")

class RobotMode(Enum):
    SIMULATION = "simulation"
    REAL_ROBOT = "real_robot"

class Go2RobotController:
    """
    Unified controller that can work with both Unity simulation and real Go2 robot
    """
    
    def __init__(self, mode: RobotMode = RobotMode.SIMULATION):
        self.mode = mode
        self.robot = None
        
        if mode == RobotMode.REAL_ROBOT and UNITREE_SDK_AVAILABLE:
            self._init_real_robot()
        
        # Movement parameters
        self.current_speed = 0.5  # m/s
        self.current_gaze_angle = 0.0  # degrees
        self.movement_mode = "forward"
        
    def _init_real_robot(self):
        """Initialize connection to real Go2 robot"""
        try:
            self.robot = Go2Robot()
            self.robot.init()
            print("Connected to real Go2 robot")
        except Exception as e:
            print(f"Failed to connect to real robot: {e}")
            self.mode = RobotMode.SIMULATION
    
    def move_straight(self, distance: float, speed: float):
        """Move the robot straight for a given distance"""
        if self.mode == RobotMode.REAL_ROBOT and self.robot:
            # Real robot movement
            duration = distance / speed
            
            motion_cmd = MotionCommand()
            motion_cmd.mode = 2  # Walking mode
            motion_cmd.velocity = [speed, 0, 0]  # Forward velocity
            motion_cmd.yaw_rate = 0
            
            start_time = time.time()
            while time.time() - start_time < duration:
                self.robot.send_motion_command(motion_cmd)
                time.sleep(0.01)  # 100Hz control loop
            
            # Stop the robot
            motion_cmd.velocity = [0, 0, 0]
            self.robot.send_motion_command(motion_cmd)
        else:
            # Simulation mode - commands handled by Unity
            print(f"Simulation: Moving {distance}m at {speed}m/s")
    
    def move_with_gaze(self, distance: float, speed: float, gaze_angle: float, 
                       lateral_movement: bool = False):
        """
        Move the robot with specific gaze control
        
        Args:
            distance: Distance to travel in meters
            speed: Movement speed in m/s
            gaze_angle: Gaze angle in degrees relative to movement direction
            lateral_movement: If True, robot moves sideways (rightward)
        """
        if self.mode == RobotMode.REAL_ROBOT and self.robot:
            duration = distance / speed
            
            motion_cmd = MotionCommand()
            motion_cmd.mode = 2  # Walking mode
            
            if lateral_movement:
                # Rightward movement
                motion_cmd.velocity = [0, -speed, 0]  # Negative Y for rightward
            else:
                # Forward movement
                motion_cmd.velocity = [speed, 0, 0]
            
            # Convert gaze angle to yaw rate for subtle body orientation
            # This creates a slight body rotation while moving
            yaw_rate = math.radians(gaze_angle) * 0.1  # Scaled down for subtlety
            motion_cmd.yaw_rate = yaw_rate
            
            # Add body pose adjustment for gaze
            motion_cmd.body_height = 0.0  # Default height
            motion_cmd.roll = 0.0
            motion_cmd.pitch = 0.0
            motion_cmd.yaw = math.radians(gaze_angle) * 0.5  # Partial body rotation
            
            start_time = time.time()
            while time.time() - start_time < duration:
                # Add subtle swaying motion for realism
                t = time.time() - start_time
                sway_amplitude = 0.02  # 2cm sway
                motion_cmd.body_height = math.sin(t * 2.0) * sway_amplitude
                
                self.robot.send_motion_command(motion_cmd)
                time.sleep(0.01)
            
            # Stop the robot
            motion_cmd.velocity = [0, 0, 0]
            motion_cmd.yaw_rate = 0
            self.robot.send_motion_command(motion_cmd)
        else:
            # Simulation mode
            mode_str = "rightward" if lateral_movement else "forward"
            print(f"Simulation: Moving {mode_str} {distance}m at {speed}m/s with {gaze_angle}° gaze")
    
    def set_posture(self, height_offset: float = 0.0, roll: float = 0.0, 
                    pitch: float = 0.0, yaw: float = 0.0):
        """
        Set robot body posture
        
        Args:
            height_offset: Height offset from default standing height (meters)
            roll: Body roll angle (degrees)
            pitch: Body pitch angle (degrees)  
            yaw: Body yaw angle (degrees)
        """
        if self.mode == RobotMode.REAL_ROBOT and self.robot:
            motion_cmd = MotionCommand()
            motion_cmd.mode = 1  # Force-stand mode for posture control
            motion_cmd.body_height = height_offset
            motion_cmd.roll = math.radians(roll)
            motion_cmd.pitch = math.radians(pitch)
            motion_cmd.yaw = math.radians(yaw)
            
            self.robot.send_motion_command(motion_cmd)
    
    def emergency_stop(self):
        """Emergency stop the robot"""
        if self.mode == RobotMode.REAL_ROBOT and self.robot:
            motion_cmd = MotionCommand()
            motion_cmd.mode = 0  # Damping mode (safe stop)
            motion_cmd.velocity = [0, 0, 0]
            self.robot.send_motion_command(motion_cmd)
        else:
            print("Emergency stop triggered")
    
    def get_robot_state(self) -> Optional[dict]:
        """Get current robot state"""
        if self.mode == RobotMode.REAL_ROBOT and self.robot:
            state = self.robot.get_state()
            return {
                "position": state.position,
                "velocity": state.velocity,
                "orientation": state.orientation,
                "battery_level": state.battery_level,
                "temperature": state.temperature
            }
        return None


class SafetyMonitor:
    """
    Safety monitoring for real robot operation
    """
    
    def __init__(self, controller: Go2RobotController):
        self.controller = controller
        self.emergency_distance = 0.5  # meters
        self.max_speed = 1.5  # m/s
        
    def check_proximity_sensors(self) -> bool:
        """Check if path is clear using robot sensors"""
        if self.controller.mode == RobotMode.REAL_ROBOT:
            # This would interface with actual proximity sensors
            # TODO: Implement this later
            return True
        return True
    
    def monitor_battery(self) -> Tuple[bool, float]:
        """Monitor battery level"""
        state = self.controller.get_robot_state()
        if state and "battery_level" in state:
            battery = state["battery_level"]
            return battery > 20.0, battery  # Safe if above 20%
        return True, 100.0  # Default safe
    
    def validate_speed(self, requested_speed: float) -> float:
        """Validate and clamp speed to safe limits"""
        return min(requested_speed, self.max_speed)


if __name__ == "__main__":
    # Choose mode based on availability
    mode = RobotMode.REAL_ROBOT if UNITREE_SDK_AVAILABLE else RobotMode.SIMULATION
    
    # Create controller
    controller = Go2RobotController(mode)
    safety = SafetyMonitor(controller)
    
    print(f"Running in {mode.value} mode")
    
    # Example movement sequence
    if mode == RobotMode.REAL_ROBOT:
        print("Starting real robot movement sequence...")
        
        # Safety check
        if not safety.check_proximity_sensors():
            print("Path blocked! Aborting.")
            exit(1)
        
        battery_ok, battery_level = safety.monitor_battery()
        if not battery_ok:
            print(f"Low battery ({battery_level}%)! Aborting.")
            exit(1)
        
        # Execute movement
        try:
            # Forward movement with direct gaze
            print("Moving forward 5m...")
            controller.move_with_gaze(distance=5.0, speed=0.5, gaze_angle=0.0)
            
            time.sleep(2)
            
            # Rightward movement facing participant
            print("Moving rightward 5m...")
            controller.move_with_gaze(distance=5.0, speed=0.5, gaze_angle=0.0, 
                                    lateral_movement=True)
            
        except KeyboardInterrupt:
            print("\nInterrupted! Stopping robot...")
            controller.emergency_stop()
        except Exception as e:
            print(f"Error: {e}")
            controller.emergency_stop()
    else:
        print("Simulation mode - integrate with Unity controller")