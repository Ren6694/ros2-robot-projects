#!/usr/bin/env python3
"""
完整仿真：迷宫世界 + 小车 + 状态发布 +（可选）RViz2
用法：
  ros2 launch maze_bot simulate.launch.py
  ros2 launch maze_bot simulate.launch.py gui:=false          # 无 Gazebo 界面（headless，省资源）
  ros2 launch maze_bot simulate.launch.py use_rviz:=false
  ros2 launch maze_bot simulate.launch.py x:=-3.5 y:=-2.5     # 指定出生点
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'maze.world')
    default_urdf = os.path.join(pkg_share, 'urdf', 'maze_bot.urdf.xacro')

    world = LaunchConfiguration('world')
    urdf_file = LaunchConfiguration('urdf_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    verbose = LaunchConfiguration('verbose')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    scan_time_offset = LaunchConfiguration('scan_time_offset')

    # ---- Gazebo（含 gazebo_ros_init / factory / state 插件）----
    # ⚠️ 必须用 gazebo_ros 的 gazebo.launch.py，
    #    直接用 `gazebo` 命令启动不会挂载 ROS 插件，spawn_entity 会失败
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': verbose,
        }.items(),
    )

    # ---- 机器人状态发布 ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    # ---- 在 Gazebo 里生成小车 ----
    # ⚠️ 关键坑：spawn_entity.py 的 -file 参数**不会做 xacro 展开**，
    #    直接把 .xacro 原文件当 URDF 解析 → ${var} 全部报 "not a valid float"。
    #    正解：用 xacro 命令先展开成临时 .urdf，再 spawn。
    urdf_expanded = '/tmp/maze_bot.expanded.urdf'
    expand_xacro = ExecuteProcess(
        cmd=['xacro', urdf_file, '-o', urdf_expanded],
        output='screen',
        name='expand_xacro',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_maze_bot',
        output='screen',
        arguments=[
            '-entity', 'maze_bot',
            '-file', urdf_expanded,        # ← 用展开后的 urdf
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
            '-timeout', '120',
        ],
    )

    # spawn 必须在 xacro 展开完成后执行
    spawn_after_expand = RegisterEventHandler(
        OnProcessExit(
            target_action=expand_xacro,
            on_exit=[spawn],
        )
    )

    # ---- 激光时间戳修正转发 ----
    # ⚠️ 必需：nav2_params.yaml 里 AMCL(scan_topic) 与代价地图(observation topic)
    #    全部订阅 /scan_fixed，由本节点把 /scan 的 header.stamp 减去 offset 后转发。
    #    若不起它 → /scan_fixed 无人发布 → AMCL/代价地图收不到激光 → 导航必失败。
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

    # ---- RViz2 ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'sim_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('urdf_file', default_value=default_urdf),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo 图形界面（false=headless，软渲染下更快）'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('x', default_value='-3.5',
                              description='出生点 x（默认迷宫起点）'),
        DeclareLaunchArgument('y', default_value='-2.5',
                              description='出生点 y'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('scan_time_offset', default_value='-0.0417',
                              description='激光时间戳修正量(秒)，半采样周期'),

        gz,
        rsp,
        expand_xacro,
        spawn_after_expand,
        scan_fix,
        rviz,
    ])
