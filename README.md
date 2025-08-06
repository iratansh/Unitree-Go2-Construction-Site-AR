# InteraConstruction: AR Construction Site & Python Control for Go2 Robot

## Project Summary

**InteraConstruction** is a research platform combining a Unity-based augmented reality (AR) environment with a Python-based control system for a physical Unitree Go2 robot. The system projects a virtual construction site as AR holograms into the real world via a Meta Quest headset.

The robot's movement is managed by a standalone Python control system, which includes:
1.  A **PyBullet simulation** (`go2_gait_simulation.py`) for developing and testing gait and movement logic in a virtual environment.
2.  A **real-robot deployment script** (`go2_gait.py`) that uses the Unitree SDK to operate the physical Go2, executing the same control logic tested in the simulation.

Currently, the robot follows a predefined 15-meter straight-line path with various configurable speed modes, allowing for controlled experiments in human-robot interaction within the AR-enhanced space. The Unity application's role is to provide the visual AR overlay, while the Python system handles all robot control.

## Core Components

### 1. Unity AR Environment
- Renders a virtual construction site as AR holograms.
- Deploys to a Meta Quest headset for an immersive, passthrough AR experience.
- Provides spatial audio for realism.

### 2. Python Control System
- **Standalone control**: Decoupled from Unity for direct, low-latency robot communication.
- **Simulation and Reality Parity**: Test gaits and paths in PyBullet, then deploy the same code to the real robot.
- **Direct SDK Integration**: Uses the `unitree_sdk2py` to send velocity commands to the Go2.
- **Keyboard Control**: Real-time control over starting, stopping, speed modes, and other parameters.

## Technologies Used

- **Unity** (2020.3.19f1): For the 3D environment and AR projection.
- **Meta Quest (Oculus) SDK**: For AR passthrough and headset deployment.
- **Python 3**: For the robot control system.
- **PyBullet**: For physics simulation of the Go2 robot.
- **Unitree Go2 DDS SDK**: For real-time communication with the physical robot.

## Project Structure

```
InteraConstruction-main/
│
├── control_system/
│   ├── go2_gait_simulation.py  # PyBullet simulation for the robot
│   ├── go2_gait.py             # Real robot deployment script
│   └── URDF/                   # Robot model for simulation
│
├── Assets/
│   ├── _Scripts/               # C# scripts for Unity AR environment
│   ├── _Materials/, _Models/, Prefabs/
│   ├── Scenes/
│   └── ... (other Unity assets)
│
├── ProjectSettings/
├── Packages/
└── README.md
```

## How to Use

### 1. Unity AR Environment
- Open the project in Unity Hub (using version 2020.3.19f1).
- Build and deploy the `Construction Environment AR Supported` scene to a connected Meta Quest headset.

### 2. Python Control System

**Prerequisites:**
- Install Python 3.
- Install required packages: `pip install pybullet numpy`
- Install the Unitree SDK:
  ```bash
  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
  cd unitree_sdk2_python
  pip3 install -e .
  ```

**A. Running the Simulation:**
Navigate to the `control_system` directory and run:
```bash
python3 go2_gait_simulation.py
```
This will open a PyBullet window where you can test the robot's movement.

**B. Running on the Real Robot:**
1.  Connect the Go2 robot to your computer via Ethernet.
2.  Configure your network interface to be on the same subnet as the robot (e.g., computer IP `192.168.123.99`, robot IP `192.168.123.161`).
3.  Find your network interface name (e.g., `eth0` or `enp2s0`) using `ifconfig` (Linux) or `ipconfig` (Windows).
4.  Run the deployment script with your network interface as an argument:
```bash
cd control_system
python3 go2_gait.py <your_network_interface>
```
**Safety is critical.** Follow the on-screen warnings before proceeding.


## Implementation Logic

### Dual-Mode Robot Operation

The system implements a sophisticated dual-mode control architecture that allows the Go2 robot to operate in two distinct movement modes:

#### 1. Walking Mode (Lateral Movement)
**Logic**: Uses the robot's natural quadruped gaits to move laterally while facing perpendicular to the path.
- **Gait Generation**: Implements a procedural trot gait with diagonal leg pairs moving together
- **Phase Control**: Uses sinusoidal functions with 180° phase offset between diagonal pairs (FR+RL vs FL+RR)
- **Speed Adaptation**: Gait frequency and amplitude scale with target velocity for natural movement
- **Leg Kinematics**: 
  - Hip joints provide lateral steering oscillation
  - Thigh joints control forward/backward leg swing
  - Calf joints handle step height and ground clearance
- **Base Velocity**: Robot body moves forward using PyBullet's `resetBaseVelocity` (simulation) or Unitree SDK's `VelocityMove` (real robot)

#### 2. Wheeled Mode (Forward Movement)
**Logic**: Utilizes omnidirectional wheels for precise forward movement with lateral path correction.
- **Mecanum Kinematics**: Implements standard mecanum wheel equations for omnidirectional control
- **Path Correction**: PID-style lateral error correction to maintain straight-line trajectory
- **Velocity Smoothing**: Applies exponential smoothing to prevent jerky movements and oscillations
- **Adaptive Control**: Reduces correction gains at higher speeds to prevent overcorrection

### Speed Profile System

The implementation includes four distinct speed profiles that demonstrate different robot behaviors:

1. **Gradual Acceleration (1-3 m/s)**: Linear speed increase over the full path length
2. **Gradual Deceleration (3-1 m/s)**: Linear speed decrease with enforced minimum of 1 m/s
3. **Stop-and-Go Patterns**: Programmed stops at 8m mark with 2-second pause duration
4. **Braking Zone**: Automatic deceleration in the final 1.5m using distance-based speed scaling

**Smoothing Logic**: All speed transitions use exponential smoothing (α = 0.85) to ensure physical plausibility and reduce actuator stress.

### Simulation-to-Reality Transfer

#### Parity Design
Both simulation and deployment scripts share identical control logic:
- **Unified Path Controller**: Same speed calculation algorithms and state machines
- **Consistent Timing**: Both systems operate at 50Hz control frequency
- **Velocity Smoothing**: Identical smoothing parameters ensure consistent behavior
- **Safety Constraints**: Same velocity limits and error checking

#### Simulation-Specific Features (PyBullet)
- **Physics Integration**: Uses PyBullet's built-in physics for realistic dynamics
- **Joint Control**: Direct position/velocity control for individual leg joints
- **Visual Debugging**: Real-time path visualization and distance markers
- **URDF Loading**: Loads robot model from standardized URDF description

#### Deployment-Specific Features (Real Robot)
- **SDK Integration**: Uses Unitree's high-level interface for robust communication
- **Network Handling**: DDS communication over Ethernet with automatic reconnection
- **State Monitoring**: Continuous IMU and position feedback at 50Hz
- **Safety Systems**: Emergency stop, gradual deceleration, and connection monitoring
- **Digital Twin Sync**: UDP broadcasting of robot pose for external visualization

### Gaze Control System

**Logic**: Implements human-like attention patterns during movement.
- **Sinusoidal Pattern**: ±15° head turning using `sin(0.5 * t)` for natural rhythm
- **Speed Independence**: Gaze timing remains constant regardless of movement speed
- **Mode Integration**: Applies as angular velocity offset in both walking and wheeled modes

### Path Following Architecture

#### Walking Mode Path Following
```
Target Direction: +Y axis (forward)
Robot Orientation: 0° (perpendicular to path)
Movement Vector: [0, target_speed, gaze_omega]
```

#### Wheeled Mode Path Following
```
Path Error: current_x - target_x (0.0)
Lateral Correction: -error * gain (adaptive)
Heading Control: target_yaw (90°) - current_yaw
Control Vector: [forward_speed, lateral_correction, yaw_correction]
```

### Error Handling and Robustness

- **Connection Monitoring**: Continuous health checking with automatic reconnection
- **Velocity Limiting**: Hard limits prevent dangerous speeds or accelerations
- **Gradual Transitions**: All mode switches include smooth velocity interpolation
- **Emergency Protocols**: Multi-level stop systems from gentle deceleration to immediate halt

This architecture ensures that behaviors tested in simulation translate reliably to the physical robot, while maintaining safety and robustness in real-world deployment.

## Acknowledgements

The base environment for this project is adapted from the open-source **InteraConstruction** simulator, a Unity 3D simulation game developed to provide a realistic construction site setting.

- **Original Repository**: [F21-G1-S5/InteraConstruction](https://github.com/F21-G1-S5/InteraConstruction)
- **License**: MIT License

We extend our gratitude to the original contributors for making their work available to the community.
