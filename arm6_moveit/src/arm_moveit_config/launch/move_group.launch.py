# -*- coding: utf-8 -*-
# =============================================================================
#  launch/move_group.launch.py —— MoveGroup 规划节点（整个系统的"大脑"）
#
#  一个进程里同时承载：规划场景监控(PlanningSceneMonitor)、碰撞检测环境、
#  运动学插件、OMPL 规划管线、轨迹执行管理(TEM) 与 MoveGroup 动作服务。
#  对外接口：/move_group/plan、/move_group/execute、/monitored_planning_scene、
#            /display_planned_path —— demo 节点与 rviz 插件都走这些。
#
#  参数装配全部来自 config/ 下的四份文件（由 MoveItConfigsBuilder 合并）：
#    robot_description            <- arm_description 的 xacro
#    robot_description_semantic   <- config/arm6.srdf
#    robot_description_kinematics <- config/kinematics.yaml
#    robot_description_planning   <- config/joint_limits.yaml
#    planning_pipelines           <- ompl（显式指定，见 rsp.launch.py 注释）
#    trajectory_execution         <- config/moveit_controllers.yaml
#
#  可调试参数：debug:=true 会用 gdb 起节点并加载 launch/gdb_settings.gdb，
#  这是排查"段错误发生在哪个插件"的官方入口。
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
