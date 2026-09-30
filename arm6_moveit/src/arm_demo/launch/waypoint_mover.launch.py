# =============================================================================
#  launch/waypoint_mover.launch.py —— 只启动点位控制节点本身
#
#  为什么不做成"一个 launch 拉起全套"：
#    move_group + ros2_control 那套 bringup 已经在 arm_moveit_config/moveit.launch.py
#    里了（由 MoveItConfigsBuilder 生成，参数耦合很多）。本节点是"客户端"，
#    它应该能在栈已经起来的情况下单独重启 —— 调节点位表时这一点非常省时间：
#    不用每次都等 move_group 重新载入模型。
#    一键跑整套由 scripts/run_waypoint_demo.sh 负责（后台 bringup + 前台节点 +
#    收摊 + 产物落盘），因为那部分是 shell 的进程管理，不该塞进 launch。
#
#  ★ Jazzy 的一个真实坑：launch 里所有 substitution 最终都是字符串，
#    直接把它交给 double/bool/int 参数会报
#        InvalidParameterTypeException / "The parameter value type does not match"
#    或者更糟 —— 被当成字符串塞进去，节点读到默认值，命令行覆盖"看起来生效了"
#    其实没有。所以数值/布尔参数一律包 ParameterValue(..., value_type=...)。
#
#  用法：
#    ros2 launch arm_demo waypoint_mover.launch.py                    # 规划+执行
#    ros2 launch arm_demo waypoint_mover.launch.py execute:=false     # 只规划（快回归）
#    ros2 launch arm_demo waypoint_mover.launch.py planner_id:=PRM    # 换规划器做对比
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 参数名 -> (默认值, 目标 ROS 参数类型)。写成表是为了让"命令行能覆盖什么"一目了然。
STR_ARGS = {
    "waypoints_file": "",          # 空 = 用包内 config/waypoints.yaml
    "pipeline_id": "ompl",
    "planner_id": "RRTConnect",    # 留空则用 OMPL 默认规划器；写死便于跨次比较
    "log_dir": "",                 # 空 = ~/ros2_ws/logs
    "run_id": "",                  # 空 = 当前时间戳
    # 哪个规划组走夹爪缩放。默认 "gripper"；控制器/SRDF 改名时改这里，不用动代码。
    "gripper_group": "gripper",
    # 下游 JTC 的 action 路径：只用于就绪探测（不发目标）。控制器改名/拆分成
    # 两个 JTC 时改这里即可，节点不会因此起不来（探测失败只 WARN）。
    "jtc_action": "/arm6_controller/follow_joint_trajectory",
}
FLOAT_ARGS = {
    "allowed_planning_time": "5.0",
    "max_velocity_scaling_factor": "1.0",
    "max_acceleration_scaling_factor": "1.0",
    # 臂的缩放默认 1.0。★这里曾经写过一条**错的机理**，值得留着当反面教材：
    #   "跟踪误差 = 关节速度 x 控制环实际周期" ⇒ 降速就能把容差余量买回来，
    #   于是虚拟机默认 0.5。同一套常驻栈、干净开机、各 6 轮的对照把它否了：
    #     0.5 : 6/6 PASS, s/rad 中位 1.457, max_track_err 最大 0.0405, replans 16
    #     1.0 : 6/6 PASS, s/rad 中位 1.027, max_track_err 最大 0.04398, replans 10
    #   速度翻倍误差只涨 8% —— 主导项是**停拍时长**（停多久错多少），不是速度。
    #   降速白付 42% 的时间，还换不来余量。
    #   代价要讲清楚：1.0 时臂的窗口是 0.05 / (3.14~4.19) ≈ 12~16 ms（0.5 时 24~32 ms），
    #   比裸周期探针实测的 10 ms 唤醒 p99=108 ms 小一个量级，所以打断仍是概率性的，
    #   靠下面的 replan_retries 兜 —— 越界不再是 FAIL，只是一次重规划。
    #
    # 夹爪组**单独**一套缩放，算法照旧但默认 1.0：
    #   指  finger*_joint trajectory 0.01 rad（比臂紧 5 倍），URDF v 上限 0.10
    #       -> 窗口 = 0.01 / (0.10 x 本参数)，1.0 时 100 ms、0.5 时 200 ms。
    #   早先"吃满 1.0 会被打断（Position Error 0.018706 = 0.10 x 0.187）"那条来自
    #   冷启动 + 上一场进程没退干净的污染样本；干净开机提权两轮后 13 轮最坏 loop
    #   只有 55 ms，100 ms 的窗口没被碰到。而夹爪陪手臂吃 0.1 缩放实测三张开合花
    #   8.7 s 只走 0.11 rad，白等 7 s。所以夹爪吃满 1.0。
    #   ★教训仍然成立：同一个公式必须代**每个关节自己的**容差，跨组借用等于没算。
    "gripper_velocity_scaling_factor": "1.0",
    "gripper_acceleration_scaling_factor": "1.0",
    # MoveIt 侧的关节目标容差（与 ros2_controllers 的 goal tolerance 是两回事）
    "goal_tolerance": "0.0001",
    # settle_time 语义**已从"固定等待时长"降级为"等待上限"**：节点现在是
    # 泵着回调按验收容差轮询，一停稳就走，到上限才放弃（并如实把误差交给判据）。
    # 改成轮询的真实原因不是省时间，而是旧的固定 sleep 不泵回调 —— 等多久读到的
    # 都是同一份旧快照，到位误差量的是"ExecuteTrajectory 返回那一刻"。
    "settle_time": "0.6",
    "settle_poll_s": "0.02",
    # 重规划前等手臂停稳的秒数：JTC 打断后关节还在惯性收尾，立刻重新规划会拿到
    # 一个仍在动的起点。
    "replan_wait": "0.5",
    "waypoint_gap": "0.1",
    "wait_ready_s": "90.0",
    "arrival_tol_rad": "0.003",
}
INT_ARGS = {
    "num_planning_attempts": "3",
    # 连续多少次采样都在容差内才算"停住了"。单次命中不算 —— 误差在阈值上下抖时
    # 那叫碰巧。设 1 = 只要一次命中就走（最激进，用于测时间下界）。
    "settle_stable_samples": "3",
    # 执行被"瞬间拒绝"(CONTROL_FAILED 且 exec_s<0.2 s) 时的重试上限。
    # 存在的原因是实测到的 DDS 发现竞态：move_group 还没发现 JTC 的 action
    # server 就收到第一条 execute，见 waypoint_mover.INSTANT_REJECT_S 注释。
    # 设 0 = 关闭重试（严格模式，用于确认竞态是否仍在）。
    "exec_retries": "3",
    # 执行**跑起来之后**被打断(CONTROL_FAILED) 时"重新规划 + 再执行"的额度。
    # 与上面的 exec_retries 不是一回事：那个兜毫秒级的 DDS 发现竞态（重发同一条
    # 轨迹即可），这个兜控制环丢拍 —— 手臂停在轨迹半腰，必须从真实当前状态重规划。
    # 触发场景实测于 VirtualBox：宿主把 vCPU 摘下去一次最坏 235 ms，跟踪误差撞上
    # JTC 的 0.05 rad 路径容差。设 0 = 严格模式（虚拟机里会随机 FAIL）。
    # 默认从 2 提到 4 的依据（2026-09-17，热栈连跑 12 轮）：额度 2 时只有 7/12 PASS，
    # 且**每一次打断都落在臂的长行程点**（place / extended / up_final），夹爪组一次
    # 都没有 —— 与上面算出来的窗口完全吻合：臂在 0.5 缩放下只允许丢拍 24~32 ms，
    # 而同一次 bringup 日志里记到的最坏 loop 是 311 ms。也就是说打断必然发生，
    # 2 次恢复额度不够用。注意这**不是**把判据放宽：exec_ok 仍要求 JTC 回 SUCCESS、
    # 到位误差仍按验收容差算，replans 只是 MOTION-SUMMARY 里的观测列，
    # "靠重规划救回来的 PASS"和"一次过的 PASS"在数据里必须是两件事。
    "replan_retries": "4",
}
BOOL_ARGS = {"execute": "true"}


def generate_launch_description():
    actions = []
    for table in (STR_ARGS, FLOAT_ARGS, INT_ARGS, BOOL_ARGS):
        for name, default in table.items():
            actions.append(DeclareLaunchArgument(name, default_value=default))

    params = {}
    for name in STR_ARGS:
        params[name] = LaunchConfiguration(name)
    for name in FLOAT_ARGS:
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    for name in INT_ARGS:
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=int)
    for name in BOOL_ARGS:
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)

    actions.append(
        Node(
            package="arm_demo",
            executable="waypoint_mover",
            name="waypoint_mover",
            output="screen",
            # emulate_tty=True 让节点的 stdout 带上颜色并按行刷出来；
            # 机器可读行本身用裸 print(..., flush=True)，不依赖这个开关。
            emulate_tty=True,
            parameters=[params],
        )
    )
    return LaunchDescription(actions)
