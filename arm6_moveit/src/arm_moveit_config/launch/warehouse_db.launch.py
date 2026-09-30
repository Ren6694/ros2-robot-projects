# -*- coding: utf-8 -*-
# =============================================================================
#  launch/warehouse_db.launch.py —— 规划结果数据库（可选，默认不启动）
#
#  只有 `db:=true` 时 demo 才会 include 本文件。它起一个 MongoDB 实例
#  (warehouse_ros_mongo)，把规划出来的轨迹/场景存档，用于"回放历史方案"。
#
#  本机**未安装** ros-jazzy-moveit-warehouse-ros-mongo，所以：
#      - 本文件保留（demo 里的 include 带 IfCondition，false 时不会解析它）
#      - package.xml 里也故意不写这个 exec_depend，否则 rosdep install 直接失败
#  需要时：sudo apt install ros-jazzy-moveit-warehouse-ros-mongo && 加回依赖。
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_warehouse_db_launch(moveit_config)
