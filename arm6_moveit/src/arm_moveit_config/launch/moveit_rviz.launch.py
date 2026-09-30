# -*- coding: utf-8 -*-
# =============================================================================
#  launch/moveit_rviz.launch.py —— rviz2 + MotionPlanning 插件
#
#  与普通 rviz 启动的唯一区别：要把 MoveIt 的参数树喂给 rviz 进程。
#  generate_moveit_rviz_launch() 传的是
#    planning_pipelines + robot_description_kinematics + joint_limits
#  （robot_description / semantic 由插件自己从话题拿），
#  配置文件路径写死为 <config_pkg>/config/moveit.rviz，可用 rviz_config:= 覆盖。
#
#  截图取证时改这一行即可换窗口布局：
#    ros2 launch arm_moveit_config moveit_rviz.launch.py rviz_config:=<绝对路径>
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
