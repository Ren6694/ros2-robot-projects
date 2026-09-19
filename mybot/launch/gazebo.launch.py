"""D3 仿真启动：gzserver + gzclient + robot_state_publisher + spawn。

用法：ros2 launch mybot_description gazebo.launch.py
遥控（另开终端）：ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('mybot_description')
    world = os.path.join(pkg_desc, 'worlds', 'mybot_world.world')
    xacro = os.path.join(pkg_desc, 'urdf', 'mybot.urdf.xacro')

    # Gazebo 11 Classic：server 管物理，client 管画面（分开便于按问题6只重启 client）
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose-sdf', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    # URDF -> /robot_description（TF 由 rsp 负责；odom->base TF 由 diff_drive 插件负责）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro]), value_type=str)}],
        output='screen',
    )

    # 把话题里的模型实体化进 Gazebo
    # -timeout 120：WSL 下 factory 插件初始化慢（音频库报错拖累），默认 30s 会抢跑失败（D3 实测）
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mybot', '-timeout', '120'],
        output='screen',
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn,
    ])
