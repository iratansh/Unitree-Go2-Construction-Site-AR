import pybullet as p
import pybullet_data
import time
import math
import sys
import numpy as np
import os

class Go2WRobot:
    """A class to manage the Go2W robot with both omnidirectional wheels and walking modes."""
    
    def __init__(self, urdf_path, start_pos):
        self.start_pos = start_pos
        # Load the robot facing perpendicular to path (0 degrees orientation)
        self.robot_id = p.loadURDF(urdf_path, start_pos, p.getQuaternionFromEuler([0, 0, 0]))
        
        # Identify and store leg joint indices
        self.joint_indices = {}
        self.wheel_joints = []
        self.joint_names = []
        
        # Get all joints
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            # Store leg joints
            if 'hip' in joint_name or 'thigh' in joint_name or 'calf' in joint_name:
                self.joint_indices[joint_name] = i
                self.joint_names.append(joint_name)
                
            # Store wheel joints
            if 'foot' in joint_name and joint_type == p.JOINT_REVOLUTE:
                self.wheel_joints.append(i)
                print(f"Found wheel joint: {joint_name} (index: {i})")

        # Print joint information for debugging
        print("Available leg joints:")
        for name, idx in self.joint_indices.items():
            print(f"  {name}: {idx}")

        # Gait parameters (similar to regular Go2)
        self.gait_frequency = 1.5  # Hz
        self.gait_amplitude = 0.3  # Radians
        self.step_height = 0.15
        
        # Initialize in walking mode
        self.mode = 'walking'
        self.set_standing_pose()
        
        # Identify specific wheels for omnidirectional control
        self.fl_wheel = None  # Front Left
        self.fr_wheel = None  # Front Right
        self.rl_wheel = None  # Rear Left
        self.rr_wheel = None  # Rear Right
        
        # Wheel assignment
        for idx in self.wheel_joints:
            joint_info = p.getJointInfo(self.robot_id, idx)
            name = joint_info[1].decode('utf-8').lower()
            
            if any(pattern in name for pattern in ['fl', 'front_left', 'frontleft', 'lf']):
                self.fl_wheel = idx
                print(f"Assigned FL wheel: {name} (index: {idx})")
            elif any(pattern in name for pattern in ['fr', 'front_right', 'frontright', 'rf']):
                self.fr_wheel = idx
                print(f"Assigned FR wheel: {name} (index: {idx})")
            elif any(pattern in name for pattern in ['rl', 'rear_left', 'rearleft', 'lb', 'bl']):
                self.rl_wheel = idx
                print(f"Assigned RL wheel: {name} (index: {idx})")
            elif any(pattern in name for pattern in ['rr', 'rear_right', 'rearright', 'rb', 'br']):
                self.rr_wheel = idx
                print(f"Assigned RR wheel: {name} (index: {idx})")
        
        # Fallback assignment
        if None in [self.fl_wheel, self.fr_wheel, self.rl_wheel, self.rr_wheel]:
            print("Warning: Could not identify all wheels by name. Using fallback assignment.")
            if len(self.wheel_joints) >= 4:
                self.fl_wheel = self.wheel_joints[0]
                self.fr_wheel = self.wheel_joints[1] 
                self.rl_wheel = self.wheel_joints[2]
                self.rr_wheel = self.wheel_joints[3]
        
        # Velocity smoothing for simulation consistency with real robot
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0

    def apply_trot_gait(self, t, speed_factor=1.0):
        """Apply a procedural trot gait similar to regular Go2."""
        
        # Adjust gait frequency based on speed
        adjusted_frequency = self.gait_frequency * speed_factor
        
        # Diagonal pairs of legs move together in a trot
        phase1 = math.sin(2 * math.pi * adjusted_frequency * t)
        phase2 = math.sin(2 * math.pi * adjusted_frequency * t + math.pi)

        # Define base angles for proper standing pose
        hip_angle = 0.0
        thigh_angle = 0.8
        calf_angle = -1.4
        
        # Get lifting motion for step height
        lift1 = max(0, math.sin(2 * math.pi * adjusted_frequency * t)) * self.step_height
        lift2 = max(0, math.sin(2 * math.pi * adjusted_frequency * t + math.pi)) * self.step_height

        # Try to match the joint names from the reference code
        joint_mapping = {
            'FR_hip': ['FR_hip_joint', 'fr_hip_joint', 'front_right_hip_joint'],
            'RL_hip': ['RL_hip_joint', 'rl_hip_joint', 'rear_left_hip_joint'],
            'FL_hip': ['FL_hip_joint', 'fl_hip_joint', 'front_left_hip_joint'],
            'RR_hip': ['RR_hip_joint', 'rr_hip_joint', 'rear_right_hip_joint'],
            'FR_thigh': ['FR_thigh_joint', 'fr_thigh_joint', 'front_right_thigh_joint'],
            'RL_thigh': ['RL_thigh_joint', 'rl_thigh_joint', 'rear_left_thigh_joint'],
            'FL_thigh': ['FL_thigh_joint', 'fl_thigh_joint', 'front_left_thigh_joint'],
            'RR_thigh': ['RR_thigh_joint', 'rr_thigh_joint', 'rear_right_thigh_joint'],
            'FR_calf': ['FR_calf_joint', 'fr_calf_joint', 'front_right_calf_joint'],
            'RL_calf': ['RL_calf_joint', 'rl_calf_joint', 'rear_left_calf_joint'],
            'FL_calf': ['FL_calf_joint', 'fl_calf_joint', 'front_left_calf_joint'],
            'RR_calf': ['RR_calf_joint', 'rr_calf_joint', 'rear_right_calf_joint']
        }
        
        def find_joint(desired_joint):
            """Find the actual joint name that exists in our robot."""
            possible_names = joint_mapping.get(desired_joint, [desired_joint])
            for name in possible_names:
                if name in self.joint_indices:
                    return self.joint_indices[name]
            return None

        # Apply hip joint control
        hip_oscillation = phase1 * 0.1
        fr_hip = find_joint('FR_hip')
        rl_hip = find_joint('RL_hip')
        if fr_hip: p.setJointMotorControl2(self.robot_id, fr_hip, p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation)
        if rl_hip: p.setJointMotorControl2(self.robot_id, rl_hip, p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation)
        
        hip_oscillation2 = phase2 * 0.1
        fl_hip = find_joint('FL_hip')
        rr_hip = find_joint('RR_hip')
        if fl_hip: p.setJointMotorControl2(self.robot_id, fl_hip, p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation2)
        if rr_hip: p.setJointMotorControl2(self.robot_id, rr_hip, p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation2)

        # Animate Pair 1 (FR, RL)
        thigh1 = thigh_angle + phase1 * self.gait_amplitude - lift1 * 0.5
        calf1 = calf_angle - phase1 * self.gait_amplitude * 0.8 + lift1 * 1.2
        
        fr_thigh = find_joint('FR_thigh')
        fr_calf = find_joint('FR_calf')
        rl_thigh = find_joint('RL_thigh')
        rl_calf = find_joint('RL_calf')
        
        if fr_thigh: p.setJointMotorControl2(self.robot_id, fr_thigh, p.POSITION_CONTROL, targetPosition=thigh1)
        if fr_calf: p.setJointMotorControl2(self.robot_id, fr_calf, p.POSITION_CONTROL, targetPosition=calf1)
        if rl_thigh: p.setJointMotorControl2(self.robot_id, rl_thigh, p.POSITION_CONTROL, targetPosition=thigh1)
        if rl_calf: p.setJointMotorControl2(self.robot_id, rl_calf, p.POSITION_CONTROL, targetPosition=calf1)

        # Animate Pair 2 (FL, RR)
        thigh2 = thigh_angle + phase2 * self.gait_amplitude - lift2 * 0.5
        calf2 = calf_angle - phase2 * self.gait_amplitude * 0.8 + lift2 * 1.2
        
        fl_thigh = find_joint('FL_thigh')
        fl_calf = find_joint('FL_calf')
        rr_thigh = find_joint('RR_thigh')
        rr_calf = find_joint('RR_calf')
        
        if fl_thigh: p.setJointMotorControl2(self.robot_id, fl_thigh, p.POSITION_CONTROL, targetPosition=thigh2)
        if fl_calf: p.setJointMotorControl2(self.robot_id, fl_calf, p.POSITION_CONTROL, targetPosition=calf2)
        if rr_thigh: p.setJointMotorControl2(self.robot_id, rr_thigh, p.POSITION_CONTROL, targetPosition=thigh2)
        if rr_calf: p.setJointMotorControl2(self.robot_id, rr_calf, p.POSITION_CONTROL, targetPosition=calf2)

    def set_standing_pose(self):
        """Set the robot to a natural standing position."""
        hip_angle = 0.0
        thigh_angle = 0.8
        calf_angle = -1.4
        
        for joint_name, joint_idx in self.joint_indices.items():
            if 'hip' in joint_name:
                p.setJointMotorControl2(self.robot_id, joint_idx, p.POSITION_CONTROL, targetPosition=hip_angle)
            elif 'thigh' in joint_name:
                p.setJointMotorControl2(self.robot_id, joint_idx, p.POSITION_CONTROL, targetPosition=thigh_angle)
            elif 'calf' in joint_name:
                p.setJointMotorControl2(self.robot_id, joint_idx, p.POSITION_CONTROL, targetPosition=calf_angle)

    def set_base_velocity(self, linear_velocity, angular_velocity):
        """Sets the velocity of the robot's base - same as regular Go2."""
        p.resetBaseVelocity(self.robot_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)

    def set_omnidirectional_velocity(self, vx, vy, omega):
        """Set omnidirectional wheel velocities with velocity smoothing."""
        if self.mode != 'wheeled':
            return
        
        # Velocity smoothing for simulation consistency
        # Different smoothing rates for different situations
        if abs(vx) < 0.1 and abs(vy) < 0.1:  # Stopping
            smoothing = 0.3  # Faster smoothing when stopping
        else:  # Moving
            smoothing = 0.2  # Moderate smoothing when moving
            
        self.current_vx += smoothing * (vx - self.current_vx)
        self.current_vy += smoothing * (vy - self.current_vy)
        self.current_omega += smoothing * (omega - self.current_omega)
        
        # Robot and wheel parameters
        wheel_radius = 0.05
        robot_width = 0.4
        robot_length = 0.5
        
        # Standard mecanum wheel kinematics
        geometry_factor = (robot_width + robot_length) / 2
        
        fl_vel = (self.current_vx - self.current_vy - geometry_factor * self.current_omega) / wheel_radius
        fr_vel = (self.current_vx + self.current_vy + geometry_factor * self.current_omega) / wheel_radius
        rl_vel = (self.current_vx + self.current_vy - geometry_factor * self.current_omega) / wheel_radius
        rr_vel = (self.current_vx - self.current_vy + geometry_factor * self.current_omega) / wheel_radius
        
        max_force = 25
        
        def apply_wheel_velocity(wheel_idx, velocity):
            if wheel_idx is not None:
                p.setJointMotorControl2(
                    self.robot_id, wheel_idx,
                    p.VELOCITY_CONTROL,
                    targetVelocity=velocity,
                    force=max_force
                )
        
        apply_wheel_velocity(self.fl_wheel, fl_vel)
        apply_wheel_velocity(self.fr_wheel, fr_vel) 
        apply_wheel_velocity(self.rl_wheel, rl_vel)
        apply_wheel_velocity(self.rr_wheel, rr_vel)

    def set_mode(self, mode):
        """Switch between 'wheeled' and 'walking' modes."""
        if mode == self.mode:
            return
            
        print(f"Switching from {self.mode} to {mode} mode")
        self.mode = mode
        
        if mode == 'wheeled':
            # Lock wheels for wheeled mode
            self.stop_all_wheels()
        elif mode == 'walking':
            # Stop wheels for walking mode
            self.stop_all_wheels()

    def stop_all_wheels(self):
        """Stop all wheels."""
        for idx in self.wheel_joints:
            p.setJointMotorControl2(
                self.robot_id, idx,
                p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=50
            )

    def get_position(self):
        """Get current position of the robot."""
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        return pos
        
    def get_position_and_orientation(self):
        """Get current position and orientation."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        return pos, euler[2]
        
    def reset(self):
        """Reset to starting position."""
        p.resetBasePositionAndOrientation(
            self.robot_id, 
            self.start_pos, 
            p.getQuaternionFromEuler([0, 0, 0])
        )
        p.resetBaseVelocity(self.robot_id, [0,0,0], [0,0,0])
        self.set_standing_pose()
        
        # Reset velocity smoothing
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0

class FixedPathController:
    """Path controller with proper speed enforcement and smoother behavior."""
    
    def __init__(self):
        self.path_length = 15.0
        self.braking_distance = 1.5  # Distance before goal to start braking
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
        
        # Speed smoothing for simulation consistency with real robot
        self.last_speed = 0.0
        self.speed_smoothing = 0.85  # Higher value = smoother but slower response
        
    def get_speed(self, distance_traveled, current_time=None):
        """Calculate current speed with smoother transitions."""
        mode = self.speed_modes[self.current_speed_mode]
        
        # Get base speed from mode
        if mode == "1-3_gradual":
            # Linear increase from 1 to 3 over full path
            progress = min(1.0, distance_traveled / self.path_length)
            base_speed = 1.0 + progress * 2.0
            
        elif mode == "3-1_gradual":
            # Linear decrease from 3 to 1 over full path - ENFORCE MINIMUM 1m/s
            progress = min(1.0, distance_traveled / self.path_length)
            base_speed = max(1.0, 3.0 - progress * 2.0)  # FIXED: Clamp to minimum 1m/s
            
        elif mode == "1_stop_1":
            if distance_traveled < 8.0:
                base_speed = 1.0
            elif distance_traveled >= 8.0:
                if self.stop_start_time is None:
                    self.stop_start_time = current_time
                
                if current_time and (current_time - self.stop_start_time) < 2.0:
                    base_speed = 0.0
                else:
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
                else:
                    base_speed = 3.0
            else:
                base_speed = 3.0
        else:
            base_speed = 1.0
        
        # Apply braking zone ONLY if not in a programmed stop
        is_in_programmed_stop = (base_speed == 0.0 and 'stop' in mode and distance_traveled >= 8.0)
        
        if not is_in_programmed_stop:
            distance_to_goal = self.path_length - distance_traveled
            if distance_to_goal <= self.braking_distance and distance_to_goal > 0:
                brake_factor = distance_to_goal / self.braking_distance
                base_speed = base_speed * brake_factor
            elif distance_to_goal <= 0:
                base_speed = 0.0
        
        # Apply velocity smoothing for physical deployment consistency
        smoothed_speed = self.speed_smoothing * self.last_speed + (1 - self.speed_smoothing) * base_speed
        self.last_speed = smoothed_speed
        
        return smoothed_speed
    
    def get_gaze_angle(self, t):
        """Calculate gaze angle if gaze mode is enabled."""
        if not self.gaze_enabled:
            return 0.0
        return 15.0 * math.sin(0.5 * t)  # ±15 degrees

def main():
    # Setup - use DIRECT mode for headless operation
    try:
        p.connect(p.GUI)
        print("Running with GUI")
    except:
        print("GUI failed, running headless")
        p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    # Path setup - 15m straight path
    start_pos = [0, 0, 0.35]
    end_pos = [0, 15, 0.35]  # 15m forward path
    
    print("Loading Go2W robot...")
    
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        urdf_path = os.path.join(script_dir, "URDF/go2w_description/urdf/go2w_description.urdf")
        robot = Go2WRobot(urdf_path, start_pos)
    except p.error as e:
        print(f"Failed to load robot: {e}")
        p.disconnect()
        return

    # Initialize path controller
    path_controller = FixedPathController()
    robot.set_standing_pose()
    
    # Simulation state
    path_mode = 'leftward'  # Start in leftward mode
    is_walking = False
    start_time = None
    initial_pos = None
    
    # Draw 15m path
    p.addUserDebugLine(start_pos, end_pos, [1, 0, 0], 3)
    
    # Add distance markers every 2.5m
    for i in range(1, 7):
        marker_pos = [0, i * 2.5, 0.35]
        p.addUserDebugLine([marker_pos[0], marker_pos[1], marker_pos[2] - 0.2], 
                          [marker_pos[0], marker_pos[1], marker_pos[2] + 0.2], 
                          [0, 1, 0], 2)
    
    # Add braking zone marker
    brake_start = [0, path_controller.path_length - path_controller.braking_distance, 0.35]
    p.addUserDebugLine(
        [brake_start[0]-0.5, brake_start[1], brake_start[2]], 
        [brake_start[0]+0.5, brake_start[1], brake_start[2]], 
        [1, 1, 0], 3  # Yellow line for brake zone
    )
    
    print("\n" + "="*50)
    print("Go2W Path Simulation - 15m Linear Path")
    print("="*50)
    print("\nControls:")
    print("  SPACE: Start/Stop Walking")
    print("  P: Switch Path Mode (forward/leftward)")
    print("  S: Cycle Speed Mode")
    print("  G: Toggle Gaze Mode")
    print("  R: Reset Robot")
    print("\nSpeed Modes:")
    for i, mode in enumerate(path_controller.speed_modes):
        print(f"  {i}: {mode}")
    print(f"\nStarting Mode: {path_mode}")
    print(f"Braking zone: Last {path_controller.braking_distance}m (yellow marker)")
    print("="*50 + "\n")

    try:
        while p.isConnected():
            current_time = time.time()
            
            # Handle input
            keys = p.getKeyboardEvents()
            if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                is_walking = not is_walking
                if is_walking:
                    start_time = current_time
                    initial_pos = robot.get_position()
                    path_controller.was_stopped = False
                    path_controller.stop_start_time = None
                print(f"Walking: {'ON' if is_walking else 'OFF'}")
                
            if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                
                # Set robot mode and orientation based on path mode
                current_pos = robot.get_position()
                if path_mode == 'forward':
                    # Face toward goal (Y direction = 90 degrees)
                    new_orientation = p.getQuaternionFromEuler([0, 0, math.pi/2])
                    robot.set_mode('wheeled')
                else:  # leftward
                    # Face perpendicular to path (0 degrees) 
                    new_orientation = p.getQuaternionFromEuler([0, 0, 0])
                    robot.set_mode('walking')
                
                p.resetBasePositionAndOrientation(robot.robot_id, current_pos, new_orientation)
                print(f"Path Mode: {path_mode} (robot mode: {robot.mode})")
                
            if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
                path_controller.current_speed_mode = (path_controller.current_speed_mode + 1) % len(path_controller.speed_modes)
                current_mode = path_controller.speed_modes[path_controller.current_speed_mode]
                gaze_status = "with gaze" if path_controller.gaze_enabled else "no gaze"
                print(f"Speed Mode: {current_mode} ({gaze_status})")
                
            if ord('g') in keys and keys[ord('g')] & p.KEY_WAS_TRIGGERED:
                path_controller.gaze_enabled = not path_controller.gaze_enabled
                current_mode = path_controller.speed_modes[path_controller.current_speed_mode]
                gaze_status = "with gaze" if path_controller.gaze_enabled else "no gaze"
                print(f"Gaze Mode: {'ON' if path_controller.gaze_enabled else 'OFF'}")
                print(f"Current: {current_mode} ({gaze_status})")
                
            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                robot.reset()
                is_walking = False
                path_mode = 'leftward'
                robot.set_mode('walking')
                path_controller.was_stopped = False
                path_controller.stop_start_time = None
                print("Robot Reset")

            if is_walking and start_time is not None:
                # Calculate elapsed time and distance
                elapsed_time = current_time - start_time
                current_pos = robot.get_position()
                
                # Calculate distance traveled from start
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
                    print(f"Robot STOPPED at {distance_traveled:.1f}m (with gentle corrections)")
                    path_controller.was_stopped = True
                elif not is_stopped and path_controller.was_stopped:
                    print(f"Robot RESUMED at {distance_traveled:.1f}m")
                    path_controller.was_stopped = False
                
                # Calculate velocities based on path mode
                if path_mode == 'forward' and robot.mode == 'wheeled':
                    # Use omnidirectional wheels for forward movement with path correction
                    current_pos, current_yaw = robot.get_position_and_orientation()
                    target_yaw = math.pi/2  # Face toward +Y direction
                    
                    # Check if we're in a stop period
                    mode = path_controller.speed_modes[path_controller.current_speed_mode]
                    is_in_stop_period = (target_speed < 0.05 and 
                                       ('stop' in mode) and 
                                       distance_traveled >= 8.0)
                    
                    # path correction - prevent oscillations at high speeds
                    path_error = current_pos[0] - 0.0
                    abs_error = abs(path_error)

                    # Only correct laterally if error is significant or we're in stop period
                    if is_in_stop_period or abs_error > 0.1:
                        # Allow gentle lateral correction when deviation is large or we're stopped
                        lateral_correction = -path_error * 0.3
                        lateral_correction = max(-0.2, min(0.2, lateral_correction))
                    else:
                        lateral_correction = 0.0

                    # Disable yaw correction for straight-line movement
                    yaw_correction = 0.0

                    # Apply the velocity to the robot
                    robot.set_omnidirectional_velocity(
                        target_speed,        # Forward speed
                        lateral_correction,  # Lateral speed (usually zero)
                        yaw_correction       # Angular velocity (zero)
                    )
                    
                    if is_in_stop_period:
                        # GENTLE corrections during stop periods to prevent divergence
                        lateral_correction = -path_error * 0.4  # Gentle correction
                        lateral_correction = max(-0.3, min(0.3, lateral_correction))
                        yaw_correction = 0.0  # No yaw correction during stops
                    else:
                        # corrections - less aggressive to prevent oscillations
                        # Reduce gain at higher speeds to prevent overcorrection
                        base_gain = 1.0  # Reduced from 1.2
                        speed_reduction = min(0.3, target_speed / 10.0)  # Slight reduction at high speed
                        adaptive_gain = base_gain - speed_reduction
                        
                        lateral_correction = -path_error * adaptive_gain
                        # Conservative limits to prevent overcorrection
                        lateral_correction = max(-0.6, min(0.6, lateral_correction))
                        
                        # relaxed yaw control - don't fight lateral corrections
                        yaw_error = target_yaw - current_yaw
                        while yaw_error > math.pi:
                            yaw_error -= 2*math.pi
                        while yaw_error < -math.pi:
                            yaw_error += 2*math.pi
                        
                        # Disable corrections when very close to goal
                        distance_to_goal = path_controller.path_length - distance_traveled
                        if distance_to_goal < 0.5:
                            lateral_correction = 0.0
                            yaw_correction = 0.0
                        else:
                            # Much gentler yaw correction that doesn't interfere with forward movement
                            yaw_correction = 0.05 * yaw_error  # Reduced from 0.1
                    
                    # prioritize forward movement over perfect orientation
                    yaw_threshold = 0.25  # Increased tolerance before stopping to rotate
                    if not is_in_stop_period and abs(yaw_error) > yaw_threshold:
                        # Only stop for very large yaw errors, and correct more gently
                        robot.set_omnidirectional_velocity(
                            target_speed * 0.5,  # Keep some forward movement
                            lateral_correction * 0.5,  # Reduced lateral correction during yaw fix
                            1.0 * yaw_error  # Moderate yaw correction
                        )
                    else:  # Normal forward movement with smooth corrections
                        robot.set_omnidirectional_velocity(
                            target_speed,           # Full forward movement
                            lateral_correction,     # Smooth lateral correction
                            yaw_correction         # Gentle yaw correction
                        )
                    
                else:  # leftward mode using walking
                    # Move toward goal (positive Y) while facing perpendicular to path
                    # Same as regular Go2 - just set base velocity and apply gait
                    linear_velocity = [0, target_speed, 0]  # Move along Y-axis toward goal
                    
                    # Apply gaze control
                    gaze_rad = math.radians(gaze_angle)
                    angular_velocity = [0, 0, gaze_rad * 0.1]  # Subtle gaze movement
                    
                    # Set base velocity (same as regular Go2)
                    robot.set_base_velocity(linear_velocity, angular_velocity)
                    
                    # Apply gait only when moving at reasonable speed and NOT in braking zone
                    distance_to_goal = path_controller.path_length - distance_traveled
                    in_braking_zone = distance_to_goal <= path_controller.braking_distance
                    
                    if target_speed > 0.3 and not in_braking_zone:  # Higher threshold and exclude braking zone
                        speed_factor = max(0.5, min(1.5, target_speed / 2.0))  # More conservative speed factor
                        robot.apply_trot_gait(elapsed_time, speed_factor)
                    else:
                        robot.set_standing_pose()  # Stand still for low speeds or braking zone
                
                # Check if path completed
                if distance_traveled >= path_controller.path_length:
                    print(f"Path completed! Distance: {distance_traveled:.2f}m")
                    is_walking = False
                    if hasattr(path_controller, 'brake_announced'):
                        delattr(path_controller, 'brake_announced')
                
                # Check if in braking zone
                distance_to_goal = path_controller.path_length - distance_traveled
                if 0 < distance_to_goal <= path_controller.braking_distance:
                    if not hasattr(path_controller, 'brake_announced'):
                        print(f"Entering brake zone at {distance_traveled:.1f}m")
                        path_controller.brake_announced = True
                elif hasattr(path_controller, 'brake_announced'):
                    delattr(path_controller, 'brake_announced')
                    
            else:
                # Stop and maintain standing pose
                robot.set_base_velocity([0, 0, 0], [0, 0, 0])
                robot.set_standing_pose()

            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == '__main__':
    main()