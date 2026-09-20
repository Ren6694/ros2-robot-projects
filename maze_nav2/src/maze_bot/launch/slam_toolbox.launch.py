#!/usr/bin/env python3
"""
slam_toolbox 建图：完整仿真 + slam_toolbox（online_async 异步模式）
用法：
  ros2 launch maze_bot slam_toolbox.launch.py
  ros2 launch maze_bot slam_toolbox.launch.py gui:=false use_rviz:=false
  ros2 launch maze_bot slam_toolbox.launch.py \
      slam_params_file:=/path/to/other.yaml

产物：
  话题 /map（nav_msgs/OccupancyGrid）
  服务 /slam_toolbox/save_map（也可用 nav2_map_server 的 map_saver_cli 保存）
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # 完整仿真（Gazebo + 小车 + robot_state_publisher）
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulate.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'use_rviz': use_rviz,
        }.items(),
    )

    # slam_toolbox 异步建图（复用官方 launch，只替换参数文件）
    #   online_async 用独立线程做匹配，不阻塞 /scan 回调 ——
    #   这一点比 sync 版本更适合本项目（同步版在 CPU 紧张时会丢帧）
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config',
                                       'maze_slam_toolbox.yaml'),
            description='slam_toolbox 参数文件路径'),

        sim,
        slam,
    ])
