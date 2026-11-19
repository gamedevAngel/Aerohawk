AeroHawk

Software for the AeroHawk food-delivery robot: a 1/10 RC crawler driven by an NVIDIA Jetson Orin Nano running ROS 2 Humble (Ubuntu 22.04).

Current stage: manual driving from a PC over Wi-Fi.
Next stage: autonomous navigation with the RPLiDAR C1.

Hardware
Part	Role
Jetson Orin Nano Super Dev Kit	Main computer
PCA9685 (I2C)	Sends every control signal: steering servo + IBT-2 inputs
Steering servo	Steering, on one PCA9685 channel
IBT-2 (BTS7960)	Powers the 35T 550 drive motor; RPWM, LPWM, R_EN, L_EN come from PCA9685 channels
RPLiDAR C1	Mapping / obstacle detection (autonomy stage)

Channel numbers and all other settings live in aerohawk_bringup/config/drive_params.yaml. The channel numbers there are placeholders until confirmed on the real wiring.

Repo layout
AeroHawk/
├── aerohawk_bringup/          launch files + settings (no logic)
│   ├── config/drive_params.yaml   every drive setting, in one place
│   └── launch/robot.launch.py     starts the robot
└── aerohawk_drive/            makes the car move
    └── aerohawk_drive/
        ├── hardware.py            PCA9685 board, steering servo, IBT-2 motor (no ROS)
        ├── drive_node.py          /cmd_vel -> steering + motor, with a safety stop
        └── teleop_keyboard.py     MANUAL MODE ONLY - keyboard driving over SSH

How it fits together: anything that wants to move the car publishes a geometry_msgs/Twist on /cmd_vel. drive_node turns that into steering and motor output. Today the keyboard publishes it; later the RC receiver and Nav2 will. drive_node never needs to change.

One-time setup (on the Jetson)
bash
# I2C access
sudo apt install python3-smbus
sudo usermod -aG i2c $USER        # then log out and back in

# Check the PCA9685 shows up (look for "40")
sudo i2cdetect -y -r 7

# Workspace: the repo goes inside src/
mkdir -p ~/Aerohawk_Code/src
cd ~/Aerohawk_Code/src
git clone <repo-url> AeroHawk
Build
bash
cd ~/Aerohawk_Code
colcon build --symlink-install
source install/setup.bash

--symlink-install means edits to Python files and drive_params.yaml take effect without rebuilding. Rebuild after adding new files.

Test the hardware without ROS
bash
cd ~/Aerohawk_Code/src/AeroHawk/aerohawk_drive/aerohawk_drive
python3 hardware.py servo     # steering sweep
python3 hardware.py motor     # WHEELS OFF THE GROUND first
Drive

Both terminals are SSH sessions from your PC into the Jetson.

Terminal 1: start the robot

bash
cd ~/Aerohawk_Code && source install/setup.bash
ros2 launch aerohawk_bringup robot.launch.py

Terminal 2: keyboard control

bash
cd ~/Aerohawk_Code && source install/setup.bash
ros2 run aerohawk_drive teleop_keyboard
Key	Action
w / s	faster / slower (keep pressing s to reverse)
a / d	steer left / right
x	straighten steering
space	stop
q	quit (also stops)
Safety
If drive_node gets no command for 0.5 s (Wi-Fi drop, teleop closed), it stops the motor and centers the steering.
Motor power is capped by max_duty (40% by default) and speed changes are ramped.
Stopping drive_node (Ctrl+C) turns every PCA9685 output off, including the IBT-2 enables.
Test with the wheels off the ground after any wiring or settings change.
Moving to autonomy
Delete aerohawk_drive/aerohawk_drive/teleop_keyboard.py and its line in aerohawk_drive/setup.py.
Add the LiDAR, SLAM, Nav2 and twist_mux to robot.launch.py.
drive_node.py and hardware.py stay as they are.