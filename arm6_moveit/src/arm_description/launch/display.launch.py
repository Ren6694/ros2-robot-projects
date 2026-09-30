# -*- coding: utf-8 -*-
# =============================================================================
#  launch/display.launch.py —— 只看模型（不接 MoveIt / 不接控制器）
#
#  用途有三：
#    1) 建模阶段快速确认几何、颜色、关节轴向是否正确
#    2) 出问题时**二分定位**：这个 launch 里模型就错，就不必去查 MoveIt；
#       反过来 moveit.launch.py 出错而本 launch 正常，说明模型没问题、配置有问题
#    3) 给文档/简历提供"纯模型"截图（没有规划轨迹的干扰线条）
#
#  与 moveit 侧的分工：这里用 joint_state_publisher(/_gui) 手工喂 /joint_states，
#  而 moveit.launch.py 里由 ros2_control 的 joint_state_broadcaster 喂。
#  **两者绝不能同时运行** —— 两个节点发同一个话题会让关节值来回跳，
#  TF 表现为高频抖动，最容易被误判成"渲染问题"。
#
#  注意 xacro 的展开时机：Command(['xacro ', path]) 在 **launch 启动时** 执行，
#  而 path 指向 install/share 里的副本，所以改了 src 下的 xacro 之后
#  必须重新 colcon build（或 bash scripts/sync.sh && colcon build）才会生效。
#  这是 ROS 新手最常见的"我改了模型怎么没变化"，写死在这里省一次排查。
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_description")
    default_model_path = os.path.join(pkg_share, "urdf", "arm6.urdf.xacro")
    default_rviz_path = os.path.join(pkg_share, "rviz", "display_arm.rviz")

    args = [
        DeclareLaunchArgument(
            name="xacro_file",
            default_value=default_model_path,
            description="要加载的 xacro 绝对路径（默认取本包 share/urdf 下的模型）",
        ),
        DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_rviz_path,
            description="rviz2 布局文件（绝对路径）",
        ),
        DeclareLaunchArgument(
            name="use_gui",
            default_value="false",
            description="true = 用 joint_state_publisher_gui 的滑条摆臂（截图取证用）",
        ),
        DeclareLaunchArgument(
            name="show_rviz",
            default_value="true",
            description="false = 无头跑一遍 rsp+jsp，用于 CI 冒烟测试",
        ),
    ]

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("xacro_file")]), value_type=str
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 两个 jsp 二选一，用 IfCondition/UnlessCondition 挂在启动描述上。
    # 不能在 Python 里写 `if LaunchConfiguration('use_gui') == 'true'`：
    # 解析期 LaunchConfiguration 还只是一个符号量，比较结果恒为 False。
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    return LaunchDescription(args + [rsp_node, jsp_node, jsp_gui_node, rviz_node])
