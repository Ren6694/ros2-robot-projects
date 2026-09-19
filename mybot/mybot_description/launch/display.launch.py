"""D1 显示链路：robot_state_publisher + joint_state_publisher_gui + RViz。

用法：ros2 launch mybot_description display.launch.py
换模型：ros2 launch mybot_description display.launch.py urdf:=/绝对路径/xxx.urdf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mybot_description')
    # D2 起默认加载 xacro（Command 调 xacro 处理器展开为 URDF 文本）
    default_model = os.path.join(pkg_share, 'urdf', 'mybot.urdf.xacro')

    urdf_arg = DeclareLaunchArgument(
        'urdf', default_value=default_model,
        description='Absolute path of the URDF/xacro file to load')

    # TF 树发布者：读入 URDF 文本 -> /robot_description + 各 link 的 TF
    # Humble 起 Command 结果必须用 ParameterValue(..., value_type=str) 显式声明为字符串
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('urdf')]), value_type=str)}],
        output='screen',
    )

    # 关节滑条面板：手动转轮子，验证 continuous 关节与 TF 联动
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # 自带显示配置（Grid + RobotModel + TF），打开即可见模型
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mybot.rviz')],
        output='screen',
    )

    return LaunchDescription([
        urdf_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
