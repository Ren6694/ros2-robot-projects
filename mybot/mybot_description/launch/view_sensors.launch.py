"""D5: 传感器可视化——独立 RViz 视图（与 gazebo_control.launch.py 并行使用）。

用法（两个终端）：
  终端 A: ros2 launch mybot_description gazebo_control.launch.py
  终端 B: ros2 launch mybot_description view_sensors.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mybot_description')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'mybot_sensors.rviz')],
            output='screen',
        ),
    ])
