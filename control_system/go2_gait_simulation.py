import pybullet as p
import pybullet_data
import time
import math
import numpy as np

class Go2Robot:
    """A class to manage the Go2 robot, including joint control for walking."""
    
    def __init__(self, urdf_path, start_pos):
        self.start_pos = start_pos
        # Load the robot facing 90 degrees from path (perpendicular to Y-axis = along X-axis = 0 degrees)
        self.robot_id = p.loadURDF(urdf_path, start_pos, p.getQuaternionFromEuler([0, 0, 0]))
        
        # Identify and store leg joint indices
        self.joint_indices = {}
        self.joint_names = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if 'hip' in joint_name or 'thigh' in joint_name or 'calf' in joint_name:
                self.joint_indices[joint_name] = i
                self.joint_names.append(joint_name)

        # Print joint information for debugging
        print("Available joints:")
        for name, idx in self.joint_indices.items():
            print(f"  {name}: {idx}")

        # Gait parameters
        self.gait_frequency = 1.5  # Hz
        self.gait_amplitude = 0.3  # Radians
        self.step_height = 0.15

    def apply_trot_gait(self, t, speed_factor=1.0):
        """Applies a procedural trot gait with speed-dependent frequency."""
        
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

        # Apply hip joint control
        hip_oscillation = phase1 * 0.1
        p.setJointMotorControl2(self.robot_id, self.joint_indices['FR_hip_joint'], p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['RL_hip_joint'], p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation)
        
        hip_oscillation2 = phase2 * 0.1
        p.setJointMotorControl2(self.robot_id, self.joint_indices['FL_hip_joint'], p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation2)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['RR_hip_joint'], p.POSITION_CONTROL, targetPosition=hip_angle + hip_oscillation2)

        # Animate Pair 1 (FR, RL)
        thigh1 = thigh_angle + phase1 * self.gait_amplitude - lift1 * 0.5
        calf1 = calf_angle - phase1 * self.gait_amplitude * 0.8 + lift1 * 1.2
        
        p.setJointMotorControl2(self.robot_id, self.joint_indices['FR_thigh_joint'], p.POSITION_CONTROL, targetPosition=thigh1)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['FR_calf_joint'], p.POSITION_CONTROL, targetPosition=calf1)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['RL_thigh_joint'], p.POSITION_CONTROL, targetPosition=thigh1)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['RL_calf_joint'], p.POSITION_CONTROL, targetPosition=calf1)

        # Animate Pair 2 (FL, RR)
        thigh2 = thigh_angle + phase2 * self.gait_amplitude - lift2 * 0.5
        calf2 = calf_angle - phase2 * self.gait_amplitude * 0.8 + lift2 * 1.2
        
        p.setJointMotorControl2(self.robot_id, self.joint_indices['FL_thigh_joint'], p.POSITION_CONTROL, targetPosition=thigh2)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['FL_calf_joint'], p.POSITION_CONTROL, targetPosition=calf2)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['RR_thigh_joint'], p.POSITION_CONTROL, targetPosition=thigh2)
        p.setJointMotorControl2(self.robot_id, self.joint_indices['RR_calf_joint'], p.POSITION_CONTROL, targetPosition=calf2)

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
        """Sets the velocity of the robot's base."""
        p.resetBaseVelocity(self.robot_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)
        
    def get_position(self):
        """Get current position of the robot."""
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        return pos
        
    def reset(self):
        # Reset to starting position facing 90 degrees from path (0 degrees orientation)
        p.resetBasePositionAndOrientation(self.robot_id, self.start_pos, p.getQuaternionFromEuler([0, 0, 0]))
        p.resetBaseVelocity(self.robot_id, [0,0,0], [0,0,0])
        self.set_standing_pose()

class PathController:
    """Controls speed and gaze along the 15m path."""
    
    def __init__(self):
        self.path_length = 15.0
        self.speed_modes = [
            "1-3_gradual",     # 1m/s to 3m/s gradual increase
            "3-1_gradual",     # 3m/s to 1m/s gradual decrease  
            "1-3_abrupt",      # 1m/s to 3m/s abrupt at 8m
            "3-1_abrupt",      # 3m/s to 1m/s abrupt at 8m
            "1_stop_1",        # 1m/s, stop at 8m, then 1m/s
            "3_stop_3"         # 3m/s, stop at 8m, then 3m/s
        ]
        self.current_speed_mode = 0
        self.gaze_enabled = False
        self.was_stopped = False  # Track if robot was previously stopped
        
    def get_speed(self, distance_traveled):
        """Calculate current speed based on mode and distance."""
        mode = self.speed_modes[self.current_speed_mode]
        
        if mode == "1-3_gradual":
            # Linear interpolation from 1 to 3 m/s over 15m
            return 1.0 + (distance_traveled / self.path_length) * 2.0
            
        elif mode == "3-1_gradual":
            # Linear interpolation from 3 to 1 m/s over 15m
            return 3.0 - (distance_traveled / self.path_length) * 2.0
            
        elif mode == "1-3_abrupt":
            # 1 m/s until 8m, then 3 m/s
            return 1.0 if distance_traveled < 8.0 else 3.0
            
        elif mode == "3-1_abrupt":
            # 3 m/s until 8m, then 1 m/s
            return 3.0 if distance_traveled < 8.0 else 1.0
            
        elif mode == "1_stop_1":
            # 1 m/s, stop at 8m, then 1 m/s after 8.5m
            if distance_traveled < 8.0:
                return 1.0
            elif distance_traveled < 8.5:
                return 0.0  # Stop
            else:
                return 1.0
                
        elif mode == "3_stop_3":
            # 3 m/s, stop at 8m, then 3 m/s after 8.5m
            if distance_traveled < 8.0:
                return 3.0
            elif distance_traveled < 8.5:
                return 0.0  # Stop
            else:
                return 3.0
                
        return 1.0  # Default
    
    def get_gaze_angle(self, t):
        """Calculate gaze angle if gaze mode is enabled."""
        if not self.gaze_enabled:
            return 0.0
        # Subtle oscillating gaze movement
        return 15.0 * math.sin(0.5 * t)  # ±15 degrees

def main():
    # Setup
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    # Path setup - 15m straight path
    start_pos = [0, 0, 0.4]
    end_pos = [0, 15, 0.4]  # 15m forward path
    
    try:
        robot = Go2Robot("control_system/URDF/go2_description.urdf", start_pos)
    except p.error as e:
        print(f"Failed to load robot: {e}")
        p.disconnect()
        return

    # Initialize path controller
    path_controller = PathController()
    robot.set_standing_pose()
    
    # Simulation state
    path_mode = 'leftward'  # Start in leftward mode since robot starts facing 90 degrees from path
    is_walking = False
    start_time = None
    initial_pos = None
    
    # Draw 15m path
    p.addUserDebugLine(start_pos, end_pos, [1, 0, 0], 3)
    
    # Add distance markers every 2.5m
    for i in range(1, 7):
        marker_pos = [0, i * 2.5, 0.4]
        p.addUserDebugLine([marker_pos[0], marker_pos[1], marker_pos[2] - 0.2], 
                          [marker_pos[0], marker_pos[1], marker_pos[2] + 0.2], 
                          [0, 1, 0], 2)
    
    print("\n" + "="*50)
    print("Go2 Path Simulation - 15m Linear Path")
    print("="*50)
    print("Controls:")
    print("  SPACE: Start/Stop Walking")
    print("  P: Switch Path Mode (forward/leftward)")
    print("  S: Cycle Speed Mode")
    print("  G: Toggle Gaze Mode")
    print("  R: Reset Robot")
    print("\nSpeed Modes:")
    for i, mode in enumerate(path_controller.speed_modes):
        print(f"  {i}: {mode}")
    print(f"\nStarting Mode: {path_mode}")
    print("="*50 + "\n")

    try:
        while True:
            current_time = time.time()
            
            # Handle input
            keys = p.getKeyboardEvents()
            if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                is_walking = not is_walking
                if is_walking:
                    start_time = current_time
                    initial_pos = robot.get_position()
                    path_controller.was_stopped = False  # Reset stop state when starting
                print(f"Walking: {'ON' if is_walking else 'OFF'}")
                
            if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
                path_mode = 'leftward' if path_mode == 'forward' else 'forward'
                
                # Instantly set orientation based on mode
                current_pos = robot.get_position()
                if path_mode == 'forward':
                    # Face toward goal (Y direction = 90 degrees)
                    new_orientation = p.getQuaternionFromEuler([0, 0, math.pi/2])
                else:  # leftward
                    # Face 90 degrees from path (perpendicular to path = 0 degrees) 
                    new_orientation = p.getQuaternionFromEuler([0, 0, 0])
                
                p.resetBasePositionAndOrientation(robot.robot_id, current_pos, new_orientation)
                print(f"Path Mode: {path_mode}")
                
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
                path_mode = 'leftward'  # Reset to starting mode
                path_controller.was_stopped = False  # Reset stop state
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
                target_speed = path_controller.get_speed(distance_traveled)
                gaze_angle = path_controller.get_gaze_angle(elapsed_time)
                
                # Handle stop/resume console output
                is_stopped = (target_speed == 0.0)
                if is_stopped and not path_controller.was_stopped:
                    print(f"Robot STOPPED at {distance_traveled:.1f}m")
                    path_controller.was_stopped = True
                elif not is_stopped and path_controller.was_stopped:
                    print(f"Robot RESUMED at {distance_traveled:.1f}m")
                    path_controller.was_stopped = False
                
                # Calculate velocities based on path mode
                if path_mode == 'forward':
                    # Move forward along Y-axis
                    linear_velocity = [0, target_speed, 0]
                    
                    # Apply only gaze control (robot already faces correct direction)
                    gaze_rad = math.radians(gaze_angle)
                    angular_velocity = [0, 0, gaze_rad * 0.1]  # Subtle gaze movement only
                    
                else:  # leftward
                    # Crab-walk toward goal (positive Y) while facing perpendicular to path
                    linear_velocity = [0, target_speed, 0]  # Same direction as forward mode
                    
                    # Apply only gaze control (robot already faces correct direction)
                    gaze_rad = math.radians(gaze_angle)
                    angular_velocity = [0, 0, gaze_rad * 0.1]  # Subtle gaze movement only
                
                # Update robot
                robot.set_base_velocity(linear_velocity, angular_velocity)
                
                # Apply gait only when moving, standing pose when stopped
                if target_speed > 0:
                    speed_factor = max(0.1, target_speed / 2.0)  # Scale gait frequency with speed
                    robot.apply_trot_gait(elapsed_time, speed_factor)
                else:
                    # Robot is stopped - use standing pose instead of walking animation
                    robot.set_standing_pose()
                
                # Check if path completed
                if distance_traveled >= path_controller.path_length:
                    print(f"Path completed! Distance: {distance_traveled:.2f}m")
                    is_walking = False
                    
            else:
                # Stop and maintain standing pose
                robot.set_base_velocity([0, 0, 0], [0, 0, 0])
                robot.set_standing_pose()

            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    except p.error:
        pass
    finally:
        p.disconnect()

if __name__ == '__main__':
    main()