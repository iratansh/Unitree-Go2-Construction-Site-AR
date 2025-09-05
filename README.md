# Unitree Go2 Construction Site AR + Python Control

## Overview

This project combines a Unity-based AR construction site with Python control for a physical Unitree Go2 robot. Unity renders the environment to a Meta Quest headset, while Python scripts handle robot control in simulation and on the real robot using the Unitree SDK.

Current capabilities:

- Unity AR environment targeting Meta Quest
- PyBullet simulation of Go2 locomotion
- Real-robot control via Unitree SDK
- Low-latency native control client using official SDK (new)
- Optional modules: EmotiBit data capture

## Repository Structure

```
.
├── environment.yml                  # Conda env for Python tooling
├── ip_address.env                   # Optional network/IP config
├── LICENSE.md
├── README.md
├── Assets/                          # Unity assets (models, scenes, scripts, XR, etc.)
├── ProjectSettings/                 # Unity project settings (Unity 2020.3.19f1)
├── Packages/                        # Unity package manifests
├── control_system/
│   ├── go2_gait_simulation.py       # PyBullet simulation
│   ├── go2_gait.py                  # Real-robot (gait) control via Unitree SDK
│   └── URDF/                        # Robot model for simulation
├── data_processing/
   └── emotibit.py                  # Optional EmotiBit data capture
```

## Unity AR Environment

- Unity version: 2020.3.19f1 (confirmed in `ProjectSettings/ProjectVersion.txt`).
- Open this folder as a Unity project and build for Meta Quest.
- Scenes and XR configuration are under `Assets/`.

## Python Environment Setup

You can use conda (recommended) or plain Python.

Option A: Conda

```bash
# From repo root
conda env create -f environment.yml
conda activate go2-ar
```

Option B: Python + pip

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install pybullet numpy
```

Install the Unitree SDK Python bindings (required for real-robot control):

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /home/unitree/unitree_sdk2_python
pip install -e /home/unitree/unitree_sdk2_python
```

If your SDK path differs, either edit the path appended in `control_system/native_control.py` or set PYTHONPATH, e.g.:

```bash
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

Optional modules:

- EmotiBit data capture: handled by `data_processing/emotibit.py` (hardware required).

## Running

Safety first: Ensure a clear area around the robot. Keep an E-stop accessible.

1) Simulation (PyBullet)

```bash
python3 control_system/go2_gait_simulation.py
```

2) Real Robot – Gait Control

# Example interface: eth0

python3 control_system/go2_gait.py <network_interface>

3) Real Robot – Native SDK Control (new)
   Low-latency direct control using Unitree SportClient.

```bash
python3 control_system/native_control.py <network_interface>
```

Controls (keyboard):

- SPACE: toggle auto 15 m straight-line motion
- W/A/S/D: translate (vx/vy)
- Q/E: rotate (yaw)
- R: stand up
- F: sit down
- ESC: exit

Notes:

- Default auto speed: 0.5 m/s, distance: 15 m
- Script uses `ChannelFactoryInitialize(0, <network_interface>)`

## Networking

Typical Go2 default: robot IP 192.168.123.18. Connect via ethernet and ssh into the go2w using ssh unitree@192.168.123.18. Password is 123. Transfer control system over to the ssh instance and run the go2_gait script. Supply the host interface name to the Python scripts. You may store and source values from `ip_address.env` if desired.

## License

See `LICENSE.md`.

## Acknowledgements

This project builds upon the open-source InteraConstruction simulator (MIT). Thanks to the original authors.

- Original repository: https://github.com/F21-G1-S5/InteraConstruction
