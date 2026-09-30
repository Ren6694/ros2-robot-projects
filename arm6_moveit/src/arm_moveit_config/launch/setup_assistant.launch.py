# -*- coding: utf-8 -*-
# =============================================================================
#  launch/setup_assistant.launch.py —— MoveIt Setup Assistant（GUI，用于再生成配置）
#
#  本包的 config/ 是**手写**的，但结构与 Setup Assistant 输出完全一致，
#  因此可以直接把它当"编辑器"打开：
#      ros2 launch arm_moveit_config setup_assistant.launch.py
#  它会通过 .setup_assistant 读到 arm_description 的 xacro，并载入现有 SRDF，
#  改完 Save 会覆盖写回 config/ —— 也就是说手写的配置和 GUI 生成的配置可以互认。
#
#  需要 X 显示：WSL2 下 DISPLAY=:0（XFCE/WSLg），本项目在远端桌面里截图取证。
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_setup_assistant_launch(moveit_config)
