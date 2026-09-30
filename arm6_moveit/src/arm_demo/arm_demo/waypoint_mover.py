#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_mover.py —— 逐点位"规划 → 执行 → 测误差"的 MoveIt 客户端节点
=====================================================================

接口是怎么定下来的（不是照抄教程，是抓图抓出来的）：
    在 Jazzy + MoveIt 2.12.4 上先跑 scripts/probe_moveit_interfaces.sh，
    实测 /move_group 这套图只有 3 个 action：

        /move_action                          moveit_msgs/action/MoveGroup
        /execute_trajectory                   moveit_msgs/action/ExecuteTrajectory
        /arm6_controller/follow_joint_trajectory  control_msgs/action/FollowJointTrajectory

    而本机的 apt 索引（清华镜像, 2026-09-13）里根本没有 ros-jazzy-moveit-commander，
    /opt/ros/jazzy/lib/python3.12/site-packages 下也没有 moveit 目录，所以
    MoveGroupCommander 和 MoveItPy 两套 Python wrapper 都 import 不到。
    结论：直接用消息级接口。好处是规划和执行两步各自计时、各自判失败，
    日志能把"规划不出来"和"规划出来但执行不到位"分清楚 —— 用 wrapper 反而看不到。

为什么规划用 plan_only=True、执行再单独发 /execute_trajectory：
    MoveGroup 的 plan_only=False 也能一条龙执行，但拿回来只有一个总的
    error_code，planning_time 与执行超时混在一起。拆成两步之后：
        plan_s   = 客户端看到结果 - 发出目标        （含 move_group 的往返）
        plan_srv_s = 服务端自己报的 planning_time    （纯规划）
        exec_s   = 执行 action 的往返               （含 TOTG 时间 + settle）
    三者关系不成立（比如 plan_s < plan_srv_s）就说明中间有别的瓶颈。

到位误差为什么要 settle 之后再取：
    JTC 是"轨迹走完就结束"，但 mock hardware / 真机都存在最后一拍的整定过程；
    轨迹结束瞬间立刻取 /joint_states 会把"还没追平"当成误差。
"""

from __future__ import annotations

import os
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from sensor_msgs.msg import JointState

try:  # PyYAML 在 ROS 环境里一定有，但缺了也要给出可操作的报错
    import yaml
except ModuleNotFoundError as exc:  # pragma: no cover
    sys.stderr.write("需要 python3-yaml：sudo apt-get install -y python3-yaml\n")
    raise SystemExit(1) from exc

from arm_demo.motion_log import (  # noqa: E402  (本地模块，需先于 ROS 初始化)
    ARRIVAL_TOL_RAD,
    CSV_FIELDS,
    GRIPPER_ARRIVAL_TOL_RAD,
    MotionLog,
    PeakTracker,
    arrival_error,
    evaluate,
    format_summary,
    joint_vector,
    max_abs,
    path_length,
    sample_rate,
    trajectory_duration,
    worst_joint,
)

# MoveItErrorCodes.SUCCESS = 1；失败码是负数，全部原样进日志便于分类
SUCCESS = 1
CONTROL_FAILED = -4   # 执行侧失败（下游控制器）
INVALID_GOAL = -10    # 轨迹本身不合法（时间/关节数/首末状态）

# ★ CONTROL_FAILED 有两种完全不同的成因，别混为一谈（234501 这次实测踩到）★
#   (a) move_group 的日志：
#         "Action client not connected to action server:
#          arm6_controller/follow_joint_trajectory"
#         "Failed to send trajectory part 1 of 1 to controller arm6_controller"
#       —— JTC 已经 active，但 move_group 这个 DDS 参与者**还没发现**它的
#          action server。特征是 exec_s 只有几毫秒（瞬间返回），且下一条点位
#       往往就正常了。这是竞态，重试即可，不是配置错。
#   (b) 控制器真的 REJECTED 了这条轨迹（例如 allow_partial_joints_goal=false
#       时按 group 下发部分关节目标）。特征同样是秒回，但**重试永远失败**，
#       且 bringup 日志里能看到 JTC 自己的 rejection 原因。
#   所以判据只能是"重试有限次 + 每次都留痕"，不能无限重试把 (b) 藏起来。
INSTANT_REJECT_S = 0.20   # exec_s 小于此值视为"瞬间拒绝"，才允许重试


def _dur(dur) -> float:
    """builtin_interfaces/msg/Duration -> 秒（float）。 MoveIt 轨迹点的时间戳都是这个类型。"""
    return float(dur.sec) + float(dur.nanosec) * 1e-9


class WaypointMover(Node):
    """一个节点干完整件事：读点位表 → 逐点规划/执行 → 落三份产物。"""

    def __init__(self) -> None:
        super().__init__("waypoint_mover")

        # ---------------- 参数：全部可从 launch 覆盖 ----------------
        d = self.declare_parameter
        # 注意：Parameter.as_string 是"方法"，写成属性会拿到一个绑定对象且永不报错，
        # 所以本文件统一用 get_parameter_value().<type>_value，别再用 .as_string。
        self.p_waypoints = d("waypoints_file", "").get_parameter_value().string_value
        self.p_move_ns = d("move_action", "/move_action").get_parameter_value().string_value
        self.p_exec_ns = d("execute_action", "/execute_trajectory").get_parameter_value().string_value
        self.p_ctrl_state = (
            d("controller_state_topic", "/arm6_controller/controller_state")
            .get_parameter_value()
            .string_value
        )
        self.p_pipeline = d("pipeline_id", "ompl").get_parameter_value().string_value
        self.p_planner = d("planner_id", "RRTConnect").get_parameter_value().string_value
        self.p_plan_time = d("allowed_planning_time", 5.0).get_parameter_value().double_value
        self.p_attempts = d("num_planning_attempts", 3).get_parameter_value().integer_value
        self.p_vscale = d("max_velocity_scaling_factor", 0.5).get_parameter_value().double_value
        self.p_ascale = d("max_acceleration_scaling_factor", 0.5).get_parameter_value().double_value
        # ---- 夹爪组单独一套缩放（2026-09-17 实测标定，默认值由 A/B 决定）------
        # 共同的物理量先摆出来：跟踪误差 = 关节速度 x 控制环实际周期。丢拍多久、误差
        # 就攒多大，撞上该关节自己的 constraints.*.trajectory 就 PATH_TOLERANCE_VIOLATED。
        # 所以"允许的最大速度"完全由 (容差 / 丢拍时长) 决定 —— 而臂和指在本案配置里
        # 用的是**两个不同的容差**，这就是它们必须分开调的原因。
        #
        # 丢拍实测：裸周期探针量到这台虚拟机 10 ms 唤醒的 p99 = 108 ms（同一台 Windows
        # 宿主只有 0.64 ms）；bringup 日志里最坏一次 loop 跑完用了 212 ms。
        #
        # 臂：joint1..6 的 trajectory = 0.05 rad，URDF max_velocity 3.14 / 4.19 rad/s。
        #     0.5 缩放下 v 约 1.6~2.1 rad/s ⇒ 容忍窗口 0.05/1.8 只有 24~32 ms，
        #     远小于 p99 的 108 ms ⇒ 手臂被打断是**概率性**的，靠压速度压不干净，
        #     只能靠 replan_retries 从真实当前状态重新规划兜住。
        # 指：finger1/2_joint 的 trajectory 只有 0.01 rad（比臂紧 5 倍），而 URDF
        #     给的 max_velocity 是 0.10 rad/s，于是
        #         容忍窗口 = 0.01 / (0.10 x GVSCALE)
        #     GVSCALE=1.0 ⇒ 100 ms；GVSCALE=0.5 ⇒ 200 ms。
        #     ★下面这段是留着的反面教材：我最初拿臂的 0.05 去算夹爪，得出
        #     "0.10 x 0.108 = 0.011，还剩 4 倍余量"，于是把默认值写成 1.0。
        #     结果 grip_open 立刻多一次 replan，bringup 日志是
        #         Position Error: 0.018706, Position Tolerance: 0.010000
        #     而 0.018706 = 0.10 x 0.187 —— 正好是那次 187 ms 的丢拍。
        #     同一个公式必须代每个关节自己的容差，跨组借用等于没算。
        # 反过来让夹爪跟着手臂吃 0.1 缩放也不对：实测三张开合花 8.7 s 只走了
        # 0.11 rad（平均 0.013 rad/s），那 7 s 是白等的。
        #
        # ★同日稍后的更正（别把上面两个数字当常数）：212 ms / 187 ms 那两次最坏
        # loop 出在**冷启动 + 上一场进程没退干净**的样本上。干净开机、rt_tune 提权
        # 两轮之后，同一套常驻栈连跑 13 轮，最坏一次 loop 只有 55 ms —— 手指那
        # 100 ms 的窗口根本没被碰到，所以 GVSCALE 默认 1.0 站得住。
        # 但"跨组借用容差等于没算"这条教训不变，公式照旧要用各关节自己的容差。
        self.p_grip_group = d("gripper_group", "gripper").get_parameter_value().string_value
        self.p_gvscale = d("gripper_velocity_scaling_factor", 1.0).get_parameter_value().double_value
        self.p_gascale = d("gripper_acceleration_scaling_factor", 1.0).get_parameter_value().double_value
        self.p_tol = d("goal_tolerance", 1.0e-4).get_parameter_value().double_value
        self.p_execute = d("execute", True).get_parameter_value().bool_value
        # settle_time 的语义在这里**降级为上限**，不再是固定等待时长。
        # 为什么必须改（2026-09-17 读代码时发现的真实缺陷，不是性能优化）：
        #   原实现是 `self.sleep(self.p_settle)` 然后读 self._latest_state。
        #   但 sleep() 是本类底部的 @staticmethod = 裸 time.sleep，**不泵回调**；
        #   main() 里也没有后台 executor，只有 spin_until_future_complete 期间
        #   订阅才会推进。所以那 0.6 s 里 _latest_state 是**冻结的**，
        #   到位误差实际取的是"ExecuteTrajectory 返回那一刻"的旧快照 ——
        #   等 0.6 s 和等 6 s 测出来的数一模一样。
        #   改成真的边泵边轮询之后，测量才第一次名副其实，同时省掉白等的时间。
        self.p_settle = d("settle_time", 0.6).get_parameter_value().double_value
        self.p_poll_dt = d("settle_poll_s", 0.02).get_parameter_value().double_value
        # 连续 N 次都在容差内才算"停住了"：单次命中不算，因为误差可能在容差上下抖。
        self.p_stable_n = d("settle_stable_samples", 3).get_parameter_value().integer_value
        self.p_gap = d("waypoint_gap", 0.1).get_parameter_value().double_value
        self.p_ready = d("wait_ready_s", 90.0).get_parameter_value().double_value
        # 下游 JTC 的 action 名：只用来做"发现"检查，永不发目标。
        # 为什么要点名它 —— 见 INSTANT_REJECT_S 处的实测注释 (a)。
        self.p_jtc_ns = d(
            "jtc_action", "/arm6_controller/follow_joint_trajectory"
        ).get_parameter_value().string_value
        # 执行被"瞬间拒绝"时的重试上限（0 = 关闭重试，回到严格模式）
        self.p_retries = d("exec_retries", 3).get_parameter_value().integer_value
        # 执行**跑起来以后**被打断（CONTROL_FAILED / PATH_TOLERANCE_VIOLATED）时，
        # 允许"重新规划 + 再执行"的次数。与 exec_retries 是两回事：那个兜的是
        # DDS 发现竞态（毫秒级、同一条轨迹重发即可），这个兜的是控制环丢拍
        # （必须从真实当前状态重新规划）。设 0 = 严格模式，虚拟机里必挂。
        # 默认 2 -> 4（2026-09-17 热栈 12 轮实测）：额度 2 只有 7/12 PASS，打断全部
        # 落在臂的长行程点。原因是臂在 0.5 缩放下只允许丢拍 24~32 ms，而 bringup
        # 日志记到的最坏 loop 是 311 ms —— 打断是必然事件，2 次恢复额度不够。
        # 判据没有放宽：exec_ok 仍要求 JTC 回 SUCCESS，replans 只是观测列。
        self.p_replans = d("replan_retries", 4).get_parameter_value().integer_value
        self.p_replan_wait = d("replan_wait", 0.5).get_parameter_value().double_value
        self.p_arr_tol = d("arrival_tol_rad", ARRIVAL_TOL_RAD).get_parameter_value().double_value
        self.p_log_dir = d("log_dir", "").get_parameter_value().string_value
        self.p_run_id = d("run_id", "").get_parameter_value().string_value

        # ---------------- action 客户端 ----------------
        self.move_client = ActionClient(self, MoveGroup, self.p_move_ns)
        self.exec_client = ActionClient(self, ExecuteTrajectory, self.p_exec_ns)
        # 只做发现探测，**永远不发目标**：真正的执行者是 move_group 内部的
        # SimpleControllerManager。等自己的客户端能看到 JTC，说明该 action server
        # 已经进了 DDS 发现表，move_group 那边的客户端也就绪了 —— 少了这一步，
        # 第 1 条点位有概率吃到 CONTROL_FAILED(-4)（见 INSTANT_REJECT_S 注释 (a)）。
        self.jtc_probe_client = ActionClient(self, FollowJointTrajectory, self.p_jtc_ns)

        # ---------------- 状态订阅 ----------------
        self._latest_state: Dict[str, float] = {}
        self._state_msgs = 0
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self._tracker = PeakTracker()
        self._recording = False
        self._rec_t0 = 0.0
        self.create_subscription(
            JointTrajectoryControllerState,
            self.p_ctrl_state,
            self._on_controller_state,
            10,
        )

        self._start_state: Dict[str, float] = {}

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg: JointState) -> None:
        # 只留位置；velocity/acceleration 在本演示里不参与判据
        self._latest_state = joint_vector(msg.name, msg.position)
        self._state_msgs += 1

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        if not self._recording:
            return
        err = joint_vector(msg.joint_names, msg.error.positions)
        self._tracker.update(time.monotonic() - self._rec_t0, err)

    # ------------------------------------------------------------------
    # 就绪等待：三件事缺一不可，分别报错，别再让人猜
    # ------------------------------------------------------------------
    def wait_ready(self) -> bool:
        deadline = time.monotonic() + self.p_ready

        self.get_logger().info(f"等待 {self.p_move_ns} ({MoveGroup.__name__}) action server ...")
        while time.monotonic() < deadline and not self.move_client.server_is_ready():
            rclpy.spin_until_future_complete(
                self, rclpy.task.Future(), timeout_sec=0.5, executor=None
            )
        if not self.move_client.server_is_ready():
            self.get_logger().error(
                f"找不到 {self.p_move_ns}：move_group 没起来？"
                f" 先跑 ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false"
            )
            return False

        self.get_logger().info(f"等待 {self.p_exec_ns} ...")
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not self.exec_client.server_is_ready():
            rclpy.spin_until_future_complete(self, rclpy.task.Future(), timeout_sec=0.5)
        if not self.exec_client.server_is_ready():
            self.get_logger().error(
                f"找不到 {self.p_exec_ns}：moveit_simple_controller_manager 没加载？"
                " 检查 config/moveit_controllers.yaml 是否被 trajectory_execution() 发现"
            )
            return False

        self.get_logger().info("等待 /joint_states ...")
        deadline = time.monotonic() + 30.0
        base = self._state_msgs
        while time.monotonic() < deadline and self._state_msgs == base:
            rclpy.spin_until_future_complete(self, rclpy.task.Future(), timeout_sec=0.2)
        if self._state_msgs == base:
            self.get_logger().error(
                "/joint_states 无数据：joint_state_broadcaster 未激活？ ros2 control list_controllers"
            )
            return False

        # ---- 第 4 段：下游 JTC 的 action server 是否已被发现 ----
        # 为什么这一条只 WARN 不 FAIL：
        #   本项目确实需要它（否则第 1 条点位偶发 -4），但 jtc_action 参数名
        #   是部署相关的 —— 换成真机或拆成两个控制器时路径会变。
        #   把它做成硬失败会让"改了控制器名"表现为"节点起不来"，反而更难查。
        #   真正的兜底是 execute() 里的有限重试，两者叠加才够。
        self.get_logger().info(f"等待 {self.p_jtc_ns} (JTC action server) ...")
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not self.jtc_probe_client.server_is_ready():
            rclpy.spin_until_future_complete(self, rclpy.task.Future(), timeout_sec=0.3)
        jtc_ready = self.jtc_probe_client.server_is_ready()
        if not jtc_ready:
            self.get_logger().warn(
                f"20s 内没发现 {self.p_jtc_ns} —— 若控制器命名不同，用 "
                "ros2 param /waypoint_mover jtc_action 改；执行阶段的首条点位"
                "可能吃到 CONTROL_FAILED(-4)，届时由 exec_retries 兜底重试。"
            )

        self.get_logger().info(
            f"READY move_group=ok execute_action=ok jtc={'ok' if jtc_ready else 'timeout'} "
            f"joint_states={self._state_msgs} "
            f"joints={len(self._latest_state)} ctrl_state_topic={self.p_ctrl_state}"
        )
        self._start_state = dict(self._latest_state)
        return True

    # ------------------------------------------------------------------
    # 规划 / 执行
    # ------------------------------------------------------------------
    def _build_request(self, group: str, target: Dict[str, float]) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = group
        req.pipeline_id = self.p_pipeline
        req.planner_id = self.p_planner
        req.allowed_planning_time = self.p_plan_time
        req.num_planning_attempts = self.p_attempts
        # 缩放**按组**取，不再一刀切：手臂受"丢拍 x 速度 = 跟踪误差"的约束，
        # 夹爪不受（理由见 __init__ 里 p_gvscale 的实测注释）。
        if group == self.p_grip_group:
            req.max_velocity_scaling_factor = self.p_gvscale
            req.max_acceleration_scaling_factor = self.p_gascale
        else:
            req.max_velocity_scaling_factor = self.p_vscale
            req.max_acceleration_scaling_factor = self.p_ascale
        # start_state 留空 == 让 move_group 用当前规划场景里的状态。
        # 显式塞状态反而危险：客户端缓存的 /joint_states 可能已经过期。
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=pos,
                tolerance_above=self.p_tol,
                tolerance_below=self.p_tol,
                weight=1.0,
            )
            for name, pos in sorted(target.items())
        ]
        req.goal_constraints = [c]
        return req

    def plan(self, group: str, target: Dict[str, float]) -> Tuple[bool, object, Dict[str, float]]:
        """返回 (是否成功, RobotTrajectory, 计时/错误码信息)。"""
        goal = MoveGroup.Goal(
            request=self._build_request(group, target),
            planning_options=PlanningOptions(plan_only=True),  # 执行是下一步
        )
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if fut.result() is None or not fut.result().accepted:
            return False, None, {"err": "goal_rejected", "code": 0}
        gh = fut.result()

        t0 = time.monotonic()
        rfut = gh.get_result_async()
        # 超时给到 allowed_planning_time 的 3 倍：留出 action 往返和 OMPL setup 的时间
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=3.0 * self.p_plan_time + 10.0)
        dt = time.monotonic() - t0
        if rfut.result() is None:
            self.get_logger().warn(f"规划超时（{dt:.2f}s），取消目标")
            cfut = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cfut, timeout_sec=5.0)
            return False, None, {"err": "client_timeout", "code": -6, "plan_s": dt}

        res = rfut.result().result
        code = int(res.error_code.val)
        return code == SUCCESS, res.planned_trajectory, {
            "code": code,
            "plan_s": dt,
            "plan_srv_s": float(res.planning_time),
        }

    def execute(self, trajectory) -> Tuple[bool, Dict[str, float]]:
        """下发一条轨迹，带**有限**重试。

        重试条件收得很窄，只兜"瞬间拒绝"这一类竞态：
            (err == 'goal_rejected') 或 (code == CONTROL_FAILED 且 exec_s < 0.2 s)
        —— 真跑起来后才失败（超时、跟踪超差）说明问题不在链路上，重试只会
        把时间烧掉并把故障藏起来，所以绝不重试。
        attempts 会进日志：评审要能看出"9/9 一次过"还是"首条重试了 2 次"。
        """
        if trajectory is None:
            return False, {"err": "no_trajectory", "code": 0, "attempts": 0}
        attempt = 0
        while True:
            attempt += 1
            ok, info = self._execute_once(trajectory)
            info["attempts"] = attempt
            if ok:
                return True, info
            instant = info.get("err") == "goal_rejected" or (
                int(info.get("code", 0)) == CONTROL_FAILED
                and float(info.get("exec_s", 1.0)) < INSTANT_REJECT_S
            )
            if not instant or attempt > self.p_retries:
                return False, info
            self.get_logger().warn(
                f"执行被瞬间拒绝 (code={info.get('code')} err={info.get('err')} "
                f"exec_s={info.get('exec_s')})，{attempt}/{self.p_retries} 重试中 —— "
                "典型成因是 move_group 尚未发现 JTC 的 action server；"
                "若每次都失败，请查 ros2_controllers.yaml 的 allow_partial_joints_goal"
            )
            self.sleep(0.5)

    def _execute_once(self, trajectory) -> Tuple[bool, Dict[str, float]]:
        goal = ExecuteTrajectory.Goal(trajectory=trajectory, controller_names=[])
        fut = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if fut.result() is None or not fut.result().accepted:
            return False, {"err": "goal_rejected", "code": 0, "exec_s": 0.0}
        gh = fut.result()

        # 从这里开始统计控制器误差，到执行结束为止
        self._tracker.reset()
        self._rec_t0 = time.monotonic()
        self._recording = True

        t0 = time.monotonic()
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=120.0)
        dt = time.monotonic() - t0
        self._recording = False
        if rfut.result() is None:
            return False, {"err": "client_timeout", "code": -6, "exec_s": dt}
        res = rfut.result().result
        code = int(res.error_code.val)
        return code == SUCCESS, {"code": code, "exec_s": dt}

    # ------------------------------------------------------------------
    # 等"真的停稳"，而不是等一个固定时长
    # ------------------------------------------------------------------
    def wait_settled(self, group: str, target: Dict[str, float]) -> Tuple[float, float]:
        """泵着回调等到位，返回 (实际等待秒数, 最后一次读到的到位误差)。

        与被它替换掉的 `self.sleep(self.p_settle)` 有两点本质区别：

          1) 用 spin_until_future_complete 而不是 sleep。本节点的 self.sleep 是
             裸 time.sleep，不泵回调，而 main() 里没有后台 executor —— 所以旧写法
             在整个等待期间 _latest_state 是**冻结的**，到位误差取的是
             "ExecuteTrajectory 返回那一刻"的旧快照，等 0.6 s 和等 6 s 结果一样。
          2) 判据直接用 motion_log 里的**验收容差**（手臂 3e-3 / 夹爪 5e-3 rad），
             并要求连续 settle_stable_samples 次都在容差内 —— 单次命中不算，
             误差在阈值上下抖动时那叫碰巧，不叫停住。

        超时不等于成功：到 settle_time 上限就带着最后一次的读数返回，
        由调用方的 arr_err 判据如实判 FAIL。这里绝不"等不到就当作到位"。
        """
        tol = (
            GRIPPER_ARRIVAL_TOL_RAD
            if group == self.p_grip_group
            else ARRIVAL_TOL_RAD
        )
        t0 = time.monotonic()
        deadline = t0 + max(0.0, self.p_settle)
        need = max(1, self.p_stable_n)
        stable = 0
        err = float("inf")
        while time.monotonic() < deadline:
            rclpy.spin_until_future_complete(
                self, rclpy.task.Future(), timeout_sec=self.p_poll_dt
            )
            err = max_abs(arrival_error(target, dict(self._latest_state)))
            if err <= tol:
                stable += 1
                if stable >= need:
                    return round(time.monotonic() - t0, 4), round(err, 6)
            else:
                stable = 0
        # 上限到点：再泵一次拿最新值，把"没停稳"如实暴露给 arr_err 判据
        rclpy.spin_until_future_complete(
            self, rclpy.task.Future(), timeout_sec=self.p_poll_dt
        )
        err = max_abs(arrival_error(target, dict(self._latest_state)))
        return round(time.monotonic() - t0, 4), round(err, 6)

    # ------------------------------------------------------------------
    # 一条点位跑完整流程
    # ------------------------------------------------------------------
    def run_one(self, idx: int, wp: Dict[str, object]) -> Dict[str, object]:
        name = str(wp.get("name", f"wp{idx}"))
        group = str(wp.get("group", "arm"))
        target = {str(k): float(v) for k, v in dict(wp.get("joints", {})).items()}

        entry: Dict[str, object] = {
            "idx": idx,
            "name": name,
            "group": group,
            "planner": self.p_planner,
            "target": target,
            "plan_ok": 0,
            "exec_ok": 0,
            "plan_s": 0.0,
            "plan_srv_s": 0.0,
            "exec_s": 0.0,
            "settle_wait_s": 0.0,
            "exec_attempts": 1,
            "replans": 0,
            "traj_pts": 0,
            "traj_s": 0.0,
            "traj_hz": 0.0,
            "path_len_rad": 0.0,
            "track_err_rad": 0.0,
            "track_err_joint": "",
            "track_at_s": None,
            "track_samples": 0,
            "arr_err_rad": 0.0,
            "arr_err_joint": "",
            "arr_err_per_joint": {},
            "err_code": 0,
            "ts": datetime.now().isoformat(timespec="milliseconds"),
        }

        if not target:
            self.get_logger().error(f"{name}: waypoints.yaml 里没有 joints 字段")
            entry["err"] = "empty_target"
            return entry

        # ------------------------------------------------------------------
        # 一次"规划 + 执行"的尝试，失败可按额度重来。
        #
        # 为什么执行被打断后要**重新规划**，而不是把同一条轨迹重发一遍：
        #   轨迹是按"下发那一刻的起点"做时间参数化的。中途被 JTC 打断后手臂停在
        #   轨迹半腰，重发同一条会先撞上 MoveIt 的 allowed_start_tolerance 0.01
        #   （起点校验），即使绕过，参考点与当前状态的差值本身就超容差 —— 必然再失败一次。
        #   重新规划才是从真实当前状态出发的恢复手段。
        #
        # 为什么这条容错是**平台**问题的正当解，而不是把 bug 藏进重试：
        #   2026-09-17 在 VirtualBox 上实测：宿主把 vCPU 线程摘下去一次最坏 235 ms，
        #   控制环丢拍 ⇒ |state - 参考| ≈ 关节速度 × 丢拍时长，撞上 0.05 rad 的
        #   路径容差 ⇒ PATH_TOLERANCE_VIOLATED ⇒ MoveIt 报 CONTROL_FAILED(-4)。
        #   update_rate 已跑满 100 Hz、线程已 SCHED_FIFO 也压不住 —— guest 里的实时
        #   调度管不了宿主怎么调度 vCPU 线程（见 scripts/rt_tune.sh 的实测数据）。
        #   同一份配置在 WSL 上 9/9 一次过，所以差异完全来自 hypervisor 抖动。
        #
        # 判据一个字都没放宽：exec_ok 仍要求 JTC 回 SUCCESS，到位误差仍按
        # arrival_tol_rad 卡，且 replans 进 CSV/JSONL/总结行 —— 评审能看出
        # "9/9 一次过"和"9/9 但重规划了 3 次"是两件事。
        # ------------------------------------------------------------------
        replans = 0
        while True:
            ok, traj, info = self.plan(group, target)
            entry.update(
                plan_ok=1 if ok else 0,
                plan_s=round(float(info.get("plan_s", 0.0)), 4),
                plan_srv_s=round(float(info.get("plan_srv_s", 0.0)), 4),
                err_code=int(info.get("code", 0)),
            )
            if not ok:
                self.get_logger().error(
                    f"[{idx}] {name} 规划失败 code={info.get('code')} ({info.get('err', 'moveit_abort')})"
                )
                entry["err"] = info.get("err", f"moveit_code_{info.get('code')}")
                return entry

            # 轨迹拆成纯 Python 结构：既进日志，也用于路径长度计算
            jt = traj.joint_trajectory
            names = list(jt.joint_names)
            times = [_dur(p.time_from_start) for p in jt.points]
            positions = [[float(v) for v in p.positions] for p in jt.points]
            seq = [dict(zip(names, p)) for p in positions]
            seq.insert(0, {k: self._start_state.get(k, seq[0].get(k, 0.0)) for k in names})

            entry["trajectory"] = {"joint_names": names, "times": times, "positions": positions}
            entry["traj_pts"] = len(positions)
            entry["traj_s"] = round(trajectory_duration(times), 4)
            entry["traj_hz"] = round(sample_rate(times), 2)
            entry["path_len_rad"] = round(path_length(seq, names), 4)
            self._start_state.update(seq[-1])  # 下一条的起点 = 这一条的终点（规划成功时的理论值）

            if not self.p_execute:
                entry["exec_ok"] = 1  # plan-only 模式没有执行，不让它把总结判成 FAIL
                entry["exec_s"] = 0.0
                entry["mode"] = "plan_only"
                self._log_line(entry)
                return entry

            eok, einfo = self.execute(traj)
            entry.update(
                exec_ok=1 if eok else 0,
                exec_s=round(float(einfo.get("exec_s", 0.0)), 4),
                exec_attempts=int(einfo.get("attempts", 1)),
                err_code=int(einfo.get("code", entry["err_code"])),
            )
            entry.pop("err", None)
            if eok:
                break

            code = int(einfo.get("code", 0))
            if code != CONTROL_FAILED or replans >= self.p_replans:
                self.get_logger().error(
                    f"[{idx}] {name} 执行失败 code={code} ({einfo.get('err', 'execution_aborted')})"
                    f"{'，重规划额度已用完' if code == CONTROL_FAILED and replans else ''}"
                )
                entry["err"] = einfo.get("err", f"execute_code_{code}")
                break

            # 可恢复：等手臂停稳 -> 用真实状态覆盖缓存起点 -> 重新规划
            replans += 1
            entry["replans"] = replans
            self.sleep(self.p_replan_wait)
            self._start_state = dict(self._latest_state)
            self.get_logger().warn(
                f"[{idx}] {name} 执行被 {code} 打断（控制环丢拍），"
                f"从当前状态重新规划 {replans}/{self.p_replans} —— "
                f"上条轨迹 exec_s={einfo.get('exec_s')}s / traj_s={entry['traj_s']}s，"
                f"track_err={self._tracker.argmax:.4f} rad"
            )

        # 等稳态，再取到位误差（泵回调 + 连续命中容差；settle_time 只是上限）
        entry["settle_wait_s"], _ = self.wait_settled(group, target)
        actual = dict(self._latest_state)
        err = arrival_error(target, actual)
        err_signed = arrival_error(target, actual, signed=True)
        entry["arr_err_per_joint"] = {k: round(v, 6) for k, v in sorted(err.items())}
        entry["arr_err_signed_per_joint"] = {
            k: round(v, 6) for k, v in sorted(err_signed.items())
        }
        entry["achieved_per_joint"] = {
            k: round(actual[k], 6) for k in sorted(target) if k in actual
        }
        entry["arr_err_rad"] = round(max_abs(err), 6)
        entry["arr_err_joint"] = worst_joint(err) or ""
        entry["track_err_rad"] = round(self._tracker.argmax, 6)
        entry["track_err_joint"] = self._tracker.argmax_joint or ""
        entry["track_at_s"] = (
            round(self._tracker.argmax_time, 4) if self._tracker.argmax_time is not None else None
        )
        entry["track_samples"] = self._tracker.samples
        self._log_line(entry)
        return entry

    @staticmethod
    def _log_line(entry: Dict[str, object]) -> None:
        """裸 print 的机器可读行。

        用 print 而不是 get_logger()：ros2 launch 会给日志行加 `[waypoint_mover-5] `
        前缀和时间戳，机器解析要先剥一层；裸 stdout 保证 `grep -o 'MOTION .*'`
        直接可用。人类可读的诊断信息仍然走 logger。
        """
        line = " ".join(
            f"{k}={entry[k]}"
            for k in (
                "idx",
                "name",
                "group",
                "plan_ok",
                "plan_s",
                "plan_srv_s",
                "traj_pts",
                "traj_s",
                "path_len_rad",
                "exec_ok",
                "exec_attempts",
                "replans",
                "exec_s",
                "settle_wait_s",
                "track_err_rad",
                "arr_err_rad",
                "err_code",
            )
            if k in entry
        )
        print(f"MOTION {line}", flush=True)

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def run(self) -> int:
        path = self.resolve_waypoints()
        wps = self.load_waypoints(path)
        run_id = self.p_run_id or datetime.now().strftime("%Y%m%d-%H%M%S")
        log_dir = self.p_log_dir or os.path.join(os.path.expanduser("~"), "ros2_ws", "logs")
        ml = MotionLog(
            jsonl_path=os.path.join(log_dir, f"motion_{run_id}.jsonl"),
            csv_path=os.path.join(log_dir, f"motion_{run_id}.csv"),
        )
        self.get_logger().info(
            f"点位表={path} 共 {len(wps)} 条 | pipeline={self.p_pipeline} planner={self.p_planner} "
            f"| execute={self.p_execute}"
        )
        self.get_logger().info(f"日志={ml.jsonl_path} / {ml.csv_path}")
        ml.record_config(
            {
                "run_id": run_id,
                "mode": "plan_only" if not self.p_execute else "plan+execute",
                "pipeline_id": self.p_pipeline,
                "planner_id": self.p_planner,
                "allowed_planning_time": self.p_plan_time,
                "num_planning_attempts": self.p_attempts,
                "max_velocity_scaling_factor": self.p_vscale,
                "max_acceleration_scaling_factor": self.p_ascale,
                # 夹爪是**另一套**缩放（见 __init__ 注释）；不记下来的话，
                # 光看 max_velocity_scaling_factor=0.1 会误以为整场都是 0.1。
                "gripper_group": self.p_grip_group,
                "gripper_velocity_scaling_factor": self.p_gvscale,
                "gripper_acceleration_scaling_factor": self.p_gascale,
                # 两种重试额度都要进快照：PASS 时"一次过"和"靠重规划救回"是两件事
                "exec_retries": self.p_retries,
                "replan_retries": self.p_replans,
                "replan_wait": self.p_replan_wait,
                "goal_tolerance": self.p_tol,
                "arrival_tol_rad": self.p_arr_tol,
                # settle_time 现在是**上限**而非固定时长；轮询参数一起记，
                # 才知道"停稳"是按什么判据判的。
                "settle_time": self.p_settle,
                "settle_poll_s": self.p_poll_dt,
                "settle_stable_samples": self.p_stable_n,
                "waypoint_gap": self.p_gap,
                "controller_state_topic": self.p_ctrl_state,
                "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
                "n_waypoints": len(wps),
            }
        )

        rows: List[Dict[str, object]] = []
        t_all = time.monotonic()
        for i, wp in enumerate(wps, start=1):
            # 先落盘再进判据列表：某一条把节点炸了（回调异常、消息字段变更），
            # 前面的记录仍然完整可读 —— 阶段一吃过"崩了就没证据"的亏，这里不再赌。
            entry = self.run_one(i, wp)
            ml.record(entry)
            rows.append(entry)
            self.sleep(self.p_gap)

        summary = evaluate(rows, self.p_arr_tol)
        summary.update(
            {"run_id": run_id, "wall_s": round(time.monotonic() - t_all, 2), "waypoints_file": path}
        )
        ml.record_summary(summary)
        ml.close()

        print(format_summary(summary), flush=True)
        print(f"MOTION-LOG jsonl={ml.jsonl_path} csv={ml.csv_path}", flush=True)
        self.get_logger().info(f"日志已写入 {ml.jsonl_path}")
        return 0 if summary["verdict"] == "PASS" else 2

    # ------------------------------------------------------------------
    # 点位表
    # ------------------------------------------------------------------
    def resolve_waypoints(self) -> str:
        if self.p_waypoints:
            return os.path.abspath(os.path.expanduser(self.p_waypoints))
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("arm_demo"), "config", "waypoints.yaml"
        )

    def load_waypoints(self, path: str) -> List[Dict[str, object]]:
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"点位表不存在: {path}\n  ament_python 包改文件后必须重新 colcon build，"
                "或者用 waypoints_file:=<绝对路径> 直接指源码目录"
            )
        with open(path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        wps = data.get("waypoints") or []
        if not isinstance(wps, list) or not wps:
            raise ValueError(f"{path} 里 waypoints 字段为空或不是列表")
        return wps

    @staticmethod
    def sleep(seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


def main(args: Optional[List[str]] = None) -> int:
    rclpy.init(args=args)
    node = WaypointMover()
    rc = 1
    try:
        if not node.wait_ready():
            return 3
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("用户中断")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
