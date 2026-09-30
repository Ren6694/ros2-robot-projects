# -*- coding: utf-8 -*-
# =============================================================================
#  launch/moveit.launch.py —— 一键起完整仿真（demo）
#
#  用法：
#      ros2 launch arm_moveit_config moveit.launch.py                 # 含 rviz
#      ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false # 无头，跑回归
#      ros2 launch arm_moveit_config moveit.launch.py db:=true        # 另需 mongo 包
#
#  generate_demo_launch() 按依赖顺序拉起 6 件事：
#      static_virtual_joint_tfs -> rsp -> move_group -> rviz(可选)
#      -> ros2_control_node(mock 硬件) -> spawn_controllers
#  其中 ros2_control_node 的参数只有 config/ros2_controllers.yaml，
#  它要用的 URDF 通过 remap (/controller_manager/robot_description <- /robot_description)
#  从 rsp 拿 —— 这也是"rsp 必须先起来"的原因。
#
#  无头模式(use_rviz:=false)是本项目回归脚本的入口：
#  没有 X 显示也能跑完整条"规划 -> 执行 -> 到位校验"链路，
#  因此 CI/脚本里可以稳定复现，不依赖桌面环境。
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
