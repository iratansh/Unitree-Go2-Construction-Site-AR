# Prerequisites:

1. Power on the robot dog
2. Ensure the ethernet cable is connected to the robot dog and the laptop
3. Run `ssh unitree@192.168.123.18` in the terminal
4. Run one of the commands below

# Current Experiment Settings

- Total path length: **14m** (halfway point: **7m**)
- Zigzag window: **3.5m -> 10.5m** (total zigzag distance: **7m**)

# Commands (3x2x4)

# 0.75m/s:

# Linear path

python3 go2_gait2.py eth0 --speed 0.75 --path linear_forward --gaze no_stop
python3 go2_gait2.py eth0 --speed 0.75 --path linear_forward --gaze stop_gaze_forward
python3 go2_gait2.py eth0 --speed 0.75 --path linear_forward --gaze stop_rotate_gaze

# Zigzag path

python3 go2_gait2.py eth0 --speed 0.75 --path forward_zigzag --gaze no_stop --participant-side right
python3 go2_gait2.py eth0 --speed 0.75 --path forward_zigzag --gaze stop_gaze_forward --participant-side right
python3 go2_gait2.py eth0 --speed 0.75 --path forward_zigzag --gaze stop_rotate_gaze --participant-side right

# 1.25m/s:

# Linear path

python3 go2_gait2.py eth0 --speed 1.25 --path linear_forward --gaze no_stop
python3 go2_gait2.py eth0 --speed 1.25 --path linear_forward --gaze stop_gaze_forward
python3 go2_gait2.py eth0 --speed 1.25 --path linear_forward --gaze stop_rotate_gaze

# Zigzag path

python3 go2_gait2.py eth0 --speed 1.25 --path forward_zigzag --gaze no_stop --participant-side right
python3 go2_gait2.py eth0 --speed 1.25 --path forward_zigzag --gaze stop_gaze_forward --participant-side right
python3 go2_gait2.py eth0 --speed 1.25 --path forward_zigzag --gaze stop_rotate_gaze --participant-side right

# 1.75m/s:

# Linear path

python3 go2_gait2.py eth0 --speed 1.75 --path linear_forward --gaze no_stop --speed-feedback
python3 go2_gait2.py eth0 --speed 1.75 --path linear_forward --gaze stop_gaze_forward --speed-feedback
python3 go2_gait2.py eth0 --speed 1.75 --path linear_forward --gaze stop_rotate_gaze --speed-feedback

# Zigzag path

python3 go2_gait2.py eth0 --speed 1.75 --path forward_zigzag --gaze no_stop --participant-side right --speed-feedback
python3 go2_gait2.py eth0 --speed 1.75 --path forward_zigzag --gaze stop_gaze_forward --participant-side right --speed-feedback
python3 go2_gait2.py eth0 --speed 1.75 --path forward_zigzag --gaze stop_rotate_gaze --participant-side right --speed-feedback

# 2.25m/s:

# Linear path

python3 go2_gait2.py eth0 --speed 2.25 --path linear_forward --gaze no_stop --speed-feedback
python3 go2_gait2.py eth0 --speed 2.25 --path linear_forward --gaze stop_gaze_forward --speed-feedback
python3 go2_gait2.py eth0 --speed 2.25 --path linear_forward --gaze stop_rotate_gaze --speed-feedback

# Zigzag path

 python3 go2_gait2.py eth0 --speed 2.25 --path forward_zigzag --gaze no_stop --participant-side right --speed-feedback
python3 go2_gait2.py eth0 --speed 2.25 --path forward_zigzag --gaze stop_gaze_forward --participant-side right --speed-feedback
python3 go2_gait2.py eth0 --speed 2.25 --path forward_zigzag --gaze stop_rotate_gaze --participant-side right --speed-feedback
