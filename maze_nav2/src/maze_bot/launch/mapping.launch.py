#!/usr/bin/env python3
"""
SLAM 建图：完整仿真 + Cartographer
用法：
  ros2 launch maze_bot mapping.launch.py
  ros2 launch maze_bot mapping.launch.py gui:=false use_rviz:=false   # 后台建图
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot')
    carto_share = get_package_share_directory('cartographer_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    config_dir = LaunchConfiguration('configuration_directory')
    config_basename = LaunchConfiguration('configuration_basename')
    scan_time_offset = LaunchConfiguration('scan_time_offset')
    carto_scan_topic = LaunchConfiguration('carto_scan_topic')

    # 完整仿真
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulate.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'use_rviz': use_rviz,
        }.items(),
    )

    # ---- 激光时间戳修正 relay（可选）----
    # 背景（2026-09-18 实测）：
    #   Gazebo ray sensor 的 /scan header.stamp 比数据实际采集时刻晚约
    #   半个采样周期（12Hz → 41.7ms）。对【直接用 TF 投影激光点】的消费者
    #   （例如纯里程计投影建图），修正后精度从 R@±3 91.2% 提升到 98.4%。
    #
    # ⚠️ 但对【做 scan matching 的 Cartographer】实测【有害】：
    #       未修正  R@±3 = 17.4%   自由空间IoU = 0.359
    #       修正后  R@±3 = 14.9%   自由空间IoU = 0.295
    #   原因：scan matching 本身能纠正初值里 2cm 级的偏差，而改动 scan 时间戳
    #   会破坏它 scan↔odom 的时间模型（日志出现 Queue too short for velocity
    #   estimation），反而降低位姿质量。
    #
    # 因此 relay 默认【启动但不接入】，Cartographer 默认仍订阅 /scan。
    # 需要时用 launch 参数切换：
    #       ros2 launch maze_bot mapping.launch.py carto_scan_topic:=scan_fixed
    scan_fix = Node(
        package='maze_bot',
        executable='scan_time_fix.py',
        name='scan_time_fix',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'offset': scan_time_offset,
            'in_topic': 'scan',
            'out_topic': 'scan_fixed',
        }],
    )

    # Cartographer SLAM 节点
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename,
        ],
        remappings=[
            # 默认 'scan'（不修正）；改 'scan_fixed' 可启用时间戳修正
            ('scan', carto_scan_topic),
            ('odom', 'odom'),
        ],
    )

    # 把 Cartographer 的 submap 转成可用的栅格地图（/map 话题）
    carto_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(pkg_share, 'config')),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='maze_2d.lua'),
        DeclareLaunchArgument(
            'scan_time_offset',
            default_value='-0.0417',
            description='激光时间戳修正量（秒）。默认 -1/(2*12Hz)，'
                        '补偿 Gazebo 时间戳滞后半个采样周期。设 0 可关闭修正。'),
        DeclareLaunchArgument(
            'carto_scan_topic',
            default_value='scan',
            description="Cartographer 订阅的激光话题。'scan' = 原始（默认，"
                        "对 scan matching 更友好）；'scan_fixed' = 经时间戳修正。"
                        "relay 始终启动，保证 scan_fixed 随时可用。"),

        scan_fix,
        sim,
        carto_node,
        carto_grid,
    ])
