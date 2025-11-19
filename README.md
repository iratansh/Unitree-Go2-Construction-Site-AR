# Unitree Go2 Construction Site AR + Python Control

## Overview

The repository combines a Unity-based construction-site visualization with
Python scripts that command a Unitree Go2 robot. Unity renders an AR scene
for a Meta Quest headset, while the Python tooling handles a PyBullet test
bed and the real-robot gait controller that runs on the Go2W onboard
computer via the official Unitree SDK.

## Hardware & Software Requirements

### Hardware
- Unitree Go2 robot with the Go2W computer module
- Development workstation for Unity/Python (Windows/Linux/macOS)
- Meta Quest headset for the AR experience
- Gigabit Ethernet cable between the workstation and the Go2W (for
  transferring scripts and tethering during control)
- Optional: EmotiBit sensor for physiological data logging

### Software
- Unity 2020.3.19f1 (recorded in `ProjectSettings/ProjectVersion.txt`)
- Python 3.10+ with `conda` or `venv`
- Unitree `unitree_sdk2_python` checked out on the Go2W at
  `/home/unitree/unitree_sdk2_python`
- Meta Quest build support installed inside Unity if you plan to deploy AR

## Repository Layout

```
.
├── Assets/                        # Unity scenes, prefabs, and scripts
├── control_system/
│   ├── go2_gait.py                # Real-robot gait controller using the SDK
│   ├── go2_gait_simulation.py     # PyBullet parity test for the controller
│   └── URDF/                      # Go2W description used in simulation
├── data_processing/
│   └── emotibit.py                # Quick-look plots for EmotiBit exports
├── environment.yml                # Conda definition for Python tooling
├── ip_address.env                 # Optional helper for storing IP/interface
├── Packages/, ProjectSettings/, UserSettings/, Assets/ (Unity project)
├── LICENSE.md
└── README.md
```

## Python Environment Setup

Create an environment on your development machine for simulation and data
processing utilities.

**Conda**

```bash
conda env create -f environment.yml
conda activate go2-ar
```

**Python venv**

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install pybullet numpy
```

Install the Unitree SDK Python bindings on the Go2W (they are only needed
for `go2_gait.py`).

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /home/unitree/unitree_sdk2_python
pip install -e /home/unitree/unitree_sdk2_python
```

If the SDK ends up elsewhere, add it to `PYTHONPATH` on the Go2W before
running the control scripts, for example:

```bash
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

## Real-Robot Control Workflow (Go2W)

`control_system/go2_gait.py` **must** run directly on the Go2W computer so
that it can open the `eth0` interface used by the SportClient API. A typical
workflow looks like:

1. Connect your workstation to the Go2W Ethernet port. The factory default
   addressing is `192.168.123.18` on the robot.
2. Copy the control scripts onto the Go2W (example using `rsync`):

   ```bash
   rsync -av control_system unitree@192.168.123.18:~/go2_ar_control
   ```

3. SSH into the robot (`ssh unitree@192.168.123.18`, password `123`).
4. Source any desired environment variables (you can place them into
   `ip_address.env` and `source ip_address.env`).
5. Run the gait controller, explicitly passing the Ethernet interface the
   SDK should bind to:

   ```bash
   cd ~/go2_ar_control
   python3 go2_gait.py eth0
   ```

   The script will stand the robot up, open a 50 Hz loop, and expose a
   keyboard UI:
   - `SPACE`: start/stop the current scripted path
   - `P`: cycle between forward/leftward/zigzag paths
   - `S`: cycle speed profiles (gradual ramp, stop-and-go, etc.)
   - `R`: reset the estimated pose so the next run restarts the 16 m path
   - `ESC` or `Ctrl+C`: emergency stop and exit

Keep an E-stop close by and ensure the operator area is clear before
enabling torque. Because the Go2W Ethernet port is dedicated to the robot
bus, avoid running through Wi-Fi bridges; a direct `eth0` connection keeps
latency deterministic.

## Simulation Workflow

`control_system/go2_gait_simulation.py` mirrors the logic of
`go2_gait.py` using PyBullet. Run it on your workstation to vet new speed
profiles or zigzag logic before pushing code to the robot:

```bash
python3 control_system/go2_gait_simulation.py
```

The script attempts to open the GUI; if unavailable, it falls back to
headless `DIRECT` mode. Keyboard bindings match the real-robot script. The
URDFs in `control_system/URDF` are kept in sync with Unitree's public model.

## Unity AR Environment

Open the repository root inside Unity 2020.3.19f1 (or newer 2020 LTS). The
XR settings, scenes, and scripts live under `Assets/`. Build targets should
be configured for Meta Quest. Unity-specific settings (graphics, input, XR)
are tracked in `ProjectSettings/` so teammates can reproduce the build.

## Optional: EmotiBit Data Capture

`data_processing/emotibit.py` plots EmotiBit CSV exports for quick QA. Set
the `file_path` constant to your raw CSV and run the script from your
workstation environment:

```bash
python3 data_processing/emotibit.py
```

Each numeric sensor channel is plotted with timestamps and the console
prints row counts per sensor type.

## IP Configuration Helper

`ip_address.env` is intentionally excluded from source control. Use it to
store deployment-specific values (robot IP, workstation static IP, etc.) so
they can be sourced before running scripts on the Go2W. Example contents:

```
export GO2_IP=192.168.123.18
export GO2_INTERFACE=eth0
```

## License and Credits

The project is released under the MIT License (see `LICENSE.md`). Parts of
the Unity environment build upon the open-source
[InteraConstruction](https://github.com/F21-G1-S5/InteraConstruction)
simulator—thanks to the original authors for making their work available.
