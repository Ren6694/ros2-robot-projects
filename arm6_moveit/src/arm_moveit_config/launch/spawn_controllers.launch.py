# -*- coding: utf-8 -*-
# =============================================================================
#  launch/spawn_controllers.launch.py —— 把控制器实例化并 activate
#
#  控制器名字列表**不是手写的**：generate_spawn_controllers_launch() 从
#  config/moveit_controllers.yaml 里读
#      moveit_simple_controller_manager.controller_names
#  再自动追加 "joint_state_broadcaster"。
#  这样"MoveIt 要执行的那个控制器"和"实际被 spawner 拉起的控制器"永远是同一个，
#  不会出现"配置里改了名字、launch 里还是旧的、execute 一直超时"这类典型事故。
#
#  本项目最终会 spawner 两个：
#      joint_state_broadcaster -> 发布 /joint_states（TF 与 MoveIt 当前状态之源）
#      arm6_controller         -> JTC，提供 /arm6_controller/follow_joint_trajectory
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
