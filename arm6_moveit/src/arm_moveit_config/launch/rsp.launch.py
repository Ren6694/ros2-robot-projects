# -*- coding: utf-8 -*-
# =============================================================================
#  launch/rsp.launch.py —— Robot State Publisher（URDF -> /robot_description + TF）
#
#  职责：把 xacro 预处理成 URDF，发布到 /robot_description（latched），
#        并订阅 /joint_states 发布每个 link 的 TF。
#        controller_manager 也从这个话题取 URDF 里的 <ros2_control> 段，
#        所以**rsp 必须先于 ros2_control_node 起来**（demo launch 里的顺序即如此）。
#
#  为什么每个 launch 文件都重复一遍 MoveItConfigsBuilder 链，而不是抽成公共模块：
#    这是 MoveIt Setup Assistant 官方模板的写法（templates/launch/generic.launch.py.template）。
#    抽公共模块要给配置包加 ament_cmake_python + ament_python_install_package，
#    为省 5 行代码引入"launch 文件能否 import 到本工程 python 包"这一层不确定性，
#    不值得；社区里所有 moveit_config 包都是复制这几行，评审一眼能看懂。
#
#  planning_pipelines(pipelines=["ompl"]) 是**必须显式写**的一行：
#    不传参时 builder 会把 default_configs 下所有 *_planning.yaml 都装上
#    （ompl / chomp / stomp / pilz_industrial_motion_planner），
#    而 pilz 插件本机未安装 —— 结果是 move_group 启动时报
#    "Unable to find the plugin 'default/pilz_industrial_motion_planner/...'"。
#    显式列 ompl 既避免该报错，也让"本工程用哪些规划器"这件事写在配置里。
# =============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
