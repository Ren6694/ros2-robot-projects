#!/usr/bin/env python3
"""
只启动机器人状态发布（不启动 Gazebo）
用途：在 RViz2 里单独查看小车模型、检查 URDF 与 TF 树
     不依赖 Gazebo，启动快，适合调 URDF
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot')
    default_urdf = os.path.join(pkg_share, 'urdf', 'maze_bot.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'config', 'robot_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file = LaunchConfiguration('urdf_file')
    use_gui = LaunchConfiguration('use_joint_gui')

    return LaunchDescription([
        DeclareLaunchArgument('urdf_file', default_value=default_urdf,
                              description='小车 URDF/Xacro 文件路径'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用仿真时间'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('use_joint_gui', default_value='false',
                              description='是否启动关节状态 GUI'),

        # 解析 xacro 为 URDF，喂给 robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]), value_type=str),
                'use_sim_time': use_sim_time,
            }],
        ),

        # 关节状态发布（固定关节不影响，但连续关节需要）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 可选：关节滑块 GUI，手动拖动看关节运动
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui),
        ),

        # 可选：RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            condition=IfCondition(use_rviz),
        ),
    ])
