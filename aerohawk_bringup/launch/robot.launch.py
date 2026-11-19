"""
Start the robot:  ros2 launch aerohawk_bringup robot.launch.py

Right now: drive_node with drive_params.yaml.
Teleop is NOT started here - run it separately so this file never depends on manual mode.
Autonomy later: LiDAR, SLAM, Nav2 and twist_mux get added to the list below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drive_params = os.path.join(
        get_package_share_directory('aerohawk_bringup'), 'config', 'drive_params.yaml')

    return LaunchDescription([
        Node(
            package='aerohawk_drive',
            executable='drive_node',
            name='drive_node',           # must match the top key in drive_params.yaml
            parameters=[drive_params],
            output='screen',
            emulate_tty=True,            # keeps log colors / live output in the terminal
        ),
    ])
