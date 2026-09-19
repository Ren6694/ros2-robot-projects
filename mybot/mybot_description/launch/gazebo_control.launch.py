"""D4 仿真启动：Gazebo + ros2_control（controller_manager + diff_drive_controller）。

用法：ros2 launch mybot_description gazebo_control.launch.py
对比 D3：话题接口不变（/cmd_vel、/odom），但控制栈换成与真机同源的 ros2_control，
且 cmd_vel_timeout 生效——发布器消失 0.5s 后自动停车。
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
    xacro = os.path.join(pkg_desc, 'urdf', 'mybot.ros2c.urdf.xacro')
    controllers = os.path.join(pkg_desc, 'config', 'mybot_controllers.yaml')

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose-sdf', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro, ' ros2c_params_file:=', controllers]),
            value_type=str)}],
        output='screen',
    )

    # controller_manager 节点由插件在实体 spawn 后创建，spawner 会阻塞等待其就绪
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mybot', '-timeout', '120'],
        output='screen',
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])
