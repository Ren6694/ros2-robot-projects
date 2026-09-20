#!/usr/bin/env python3
"""
Nav2 导航：完整仿真 + AMCL 定位 + Nav2 导航栈
用法：
  ros2 launch maze_bot navigate.launch.py
  ros2 launch maze_bot navigate.launch.py map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 完整仿真
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulate.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'use_rviz': 'false',       # RViz 由 nav2_bringup 统一启动
        }.items(),
    )

    # Nav2 官方 bringup（含 map_server + amcl + 全套导航节点）
    # 注意：bringup 自带的 rviz 在 WSLg 软渲染下经常卡住不开窗口（进程活着没界面），
    # 所以这里把 use_rviz 显式传给 bringup，由外层（启动脚本）自己单独拉 rviz。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            # bringup 的内嵌 rviz 在 WSLg 软渲染下会卡住不开窗口，
            # 统一改为外层自己拉 rviz（本 launch 里 use_rviz:=true 时手动 Node 启动）
            'use_rviz': 'false',
        }.items(),
    )

    # 自己拉 RViz：nohup 式独立启动比 bringup 内嵌的可靠（同上，窗口能正常出）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'maze_map.yaml'),
            description='地图 yaml（默认用迷宫真值地图）'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件'),

        sim,
        nav2,
        rviz_node,
    ])
