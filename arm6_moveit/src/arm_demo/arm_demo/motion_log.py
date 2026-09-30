#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motion_log.py —— arm_demo 的"可复算"层（不 import 任何 ROS 模块）
=================================================================

为什么把这一层单独拆出来（和阶段一把 ADC 复算拆进 plot_adc.py 是同一个动机）：

  1) 判据要能离线复算。运动日志落盘后，即便仿真早就关了，也能用同一份代码
     重新算出"到位误差 / 最大跟踪误差 / 轨迹时长"，检查当时打印的 PASS 是不是
     建立在和现在相同的定义上。如果这些公式写在 ROS 节点里，就得重新拉起
     move_group 才能复核 —— 那不叫复算，叫重跑。
  2) 单元测试不需要 ROS。PeakTracker / arrival_error / path_length 都是纯函数，
     在 CI 里 0.1 s 跑完。
  3) 消息类型不泄漏。节点里把 ROS 消息拆成 dict/list 再进来，这个模块只认
     Python 原生结构，因此 moveit_msgs 换版本时这里大概率不用改。

三个误差的定义（务必和文档 04 保持一致，日志字段名就用这些英文）：

  arr_err_rad   到位误差   |q_final - q_target|        —— 执行结束后再等 settle 秒取的稳态值
  track_err_rad 跟踪误差   |q_reference - q_feedback|  —— 执行过程中 JTC 自己算出来的误差峰值
  path_len_rad  路径长度   sum_k ||q_k - q_{k-1}||_2   —— 关节空间折线长度，用来量化
                          "OMPL 是随机规划器，两次轨迹不一样"到底差多少

注意 arr_err 和 track_err 不可互相替代：mock hardware 会完美跟踪参考，所以
track_err 只能证明"控制器链路通"；而 arr_err 才反映 MoveIt 的 goal tolerance
与 JTC 的 goal tolerance 是否对齐（我们的 joint_limits/moveit_controllers 里
写的 goal=0.003 rad 就是靠这个量来验证的）。
"""

from __future__ import annotations

import csv
import json
import math
import os
from typing import Dict, Iterable, List, Optional, Sequence

# 机械臂 6 轴 + 2 指；顺序固定，CSV 列顺序和图例都依赖它
ARM_JOINTS: Sequence[str] = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINTS: Sequence[str] = ("finger1_joint", "finger2_joint")
ALL_JOINTS: Sequence[str] = tuple(ARM_JOINTS) + tuple(GRIPPER_JOINTS)

# 回归判据的默认阈值（弧度）。0.003 与 config/ros2_controllers.yaml 里
# j1~j6 的 goal tolerance 一致；手指给 0.005，因为它的 stopped_velocity 放宽。
ARRIVAL_TOL_RAD = 3.0e-3
GRIPPER_ARRIVAL_TOL_RAD = 5.0e-3
# 跟踪误差这条只看"是否病态"：mock hardware 正常时应当接近 0，真机上超过
# 0.1 rad 说明控制器 update_rate / 轨迹时间参数化出了问题。
TRACK_SANITY_RAD = 0.10


# --------------------------------------------------------------------------
# 基础向量运算
# --------------------------------------------------------------------------
def joint_vector(
    names: Sequence[str],
    positions: Sequence[float],
    only: Optional[Iterable[str]] = None,
) -> Dict[str, float]:
    """把 (names, positions) 两条平行数组拼成 {joint: rad}。

    ROS 的 JointState / JointTrajectoryPoint 都是"名字数组 + 值数组"，
    顺序由发布者决定，因此任何跨消息的比较都必须先转成以关节名为键的字典，
    绝对不能按索引对齐 —— 那是这类代码最常见的静默 bug。
    """
    if len(names) != len(positions):
        raise ValueError(f"joint name/value length mismatch: {len(names)} vs {len(positions)}")
    vec = dict(zip(names, positions))
    if only is not None:
        vec = {k: v for k, v in vec.items() if k in set(only)}
    return vec


def arrival_error(
    target: Dict[str, float], actual: Dict[str, float], signed: bool = False
) -> Dict[str, float]:
    """逐关节到位误差 |actual - target|；只统计 target 里出现的关节。

    未出现在 actual 里的关节记为 inf 而不是抛异常：这样"某关节没被广播"
    这种故障会稳定地表现为 FAIL + 一个显眼的 inf，而不是让整晚的批处理崩掉。

    signed=True 时返回 actual-target（带符号），只给画图用 —— 判据一律用绝对值，
    因为"左右各偏 0.002 rad"在两个点位上互相抵消是绝对不允许的：先求和再取绝对值
    的那种平均值会掩盖它，所以我们既不跨关节求和也不跨点位求和。
    """

    def one(name: str, pos: float) -> float:
        if name not in actual:
            return math.inf
        d = actual[name] - pos
        return d if signed else abs(d)

    return {name: one(name, pos) for name, pos in target.items()}


def max_abs(errors: Dict[str, float]) -> float:
    """字典值的最大绝对值；空字典返回 0.0（用于"这一阶段没有该组的关节"）。"""
    return max((abs(v) for v in errors.values()), default=0.0)


def worst_joint(errors: Dict[str, float]) -> Optional[str]:
    """误差最大的关节名，用于日志里指出"是谁没到位"。"""
    if not errors:
        return None
    return max(errors.items(), key=lambda kv: kv[1])[0]


def path_length(waypoints: Sequence[Dict[str, float]], joints: Sequence[str]) -> float:
    """关节空间折线长度：相邻两个构型的 L2 距离之和（各关节等权）。

    等权是刻意的：单位不同（revolute 是 rad）时任何加权都要引入人为参数，
    而我们只想比较"同一串点位、不同随机种子"下的路径迂回程度。
    """
    total = 0.0
    for prev, cur in zip(waypoints, waypoints[1:]):
        total += math.sqrt(sum((cur.get(j, 0.0) - prev.get(j, 0.0)) ** 2 for j in joints))
    return total


def trajectory_duration(times: Sequence[float]) -> float:
    """轨迹时长 = 最后一个点的时间戳（MoveIt 给的 time_from_start 已是相对量）。"""
    return float(times[-1]) if times else 0.0


def sample_rate(times: Sequence[float]) -> float:
    """轨迹平均采样率(Hz)，用来核对 TOTG 后处理有没有把密集的时间段压掉。"""
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    return (len(times) - 1) / span if span > 0 else 0.0


# --------------------------------------------------------------------------
# 峰值跟踪：执行过程中每个关节出现过的最大 |误差|
# --------------------------------------------------------------------------
class PeakTracker:
    """累积"某向量的逐元素绝对值峰值"，并记住峰值出现的相对时刻。

    用法：执行前 reset()，回调里 update(t, err_vec)，结束时读 peaks / at。
    这里刻意不做滑动窗口：我们要的是"整段执行里最坏的一刻"，滑动窗口会把
    启动瞬间的阶跃误差平均掉，正好掩盖我们要看的那类问题。
    """

    def __init__(self) -> None:
        self._peaks: Dict[str, float] = {}
        self._at: Dict[str, float] = {}
        self.samples = 0

    def reset(self) -> None:
        self._peaks.clear()
        self._at.clear()
        self.samples = 0

    def update(self, t_rel: float, values: Dict[str, float]) -> None:
        self.samples += 1
        for name, value in values.items():
            mag = abs(value)
            if mag > self._peaks.get(name, -1.0):
                self._peaks[name] = mag
                self._at[name] = t_rel

    @property
    def peaks(self) -> Dict[str, float]:
        return dict(self._peaks)

    @property
    def argmax(self) -> float:
        return max_abs(self._peaks)

    @property
    def argmax_joint(self) -> Optional[str]:
        return worst_joint(self._peaks)

    @property
    def argmax_time(self) -> Optional[float]:
        j = self.argmax_joint
        return self._at.get(j) if j is not None else None


# --------------------------------------------------------------------------
# 落盘：JSONL（全量，含轨迹）+ CSV（每条点位一行，给人看/给表格用）
# --------------------------------------------------------------------------
CSV_FIELDS = [
    "idx",
    "name",
    "group",
    "planner",
    "plan_ok",
    "plan_s",
    "plan_srv_s",
    "traj_pts",
    "traj_s",
    "path_len_rad",
    "exec_ok",
    # 1 = 一次过；>1 = 首发被"瞬间拒绝"后重试救回（见 waypoint_mover.INSTANT_REJECT_S）。
    # 只看不判：SUMMARY 的 PASS 仍只看 exec_ok，重试次数用来暴露链路竞态是否在复现。
    "exec_attempts",
    # 与 exec_attempts 是两种不同的恢复：attempts 兜 DDS 发现竞态（同一条轨迹重发），
    # replans 兜控制环丢拍（从真实当前状态重新规划）。同样只看不判。
    "replans",
    "exec_s",
    # 等稳态**实际**花了多久（settle_time 只是上限）。加这一列是为了让提速可核对：
    # 旧实现每条点位固定 sleep 0.6 s，新实现一停稳就走 —— 没有这列就只能靠总时长反推。
    "settle_wait_s",
    "track_err_rad",
    "track_err_joint",
    "arr_err_rad",
    "arr_err_joint",
    "err_code",
]


class MotionLog:
    """一次运行 = 一个 JSONL + 一个 CSV，两者同源，字段名完全一致。

    JSONL 里额外存每条点位的完整轨迹（times/positions），因为画图需要；
    CSV 只放标量指标，方便在 Excel 里排序。
    """

    def __init__(self, jsonl_path: str, csv_path: str) -> None:
        self.jsonl_path = jsonl_path
        self.csv_path = csv_path
        os.makedirs(os.path.dirname(os.path.abspath(jsonl_path)), exist_ok=True)
        os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
        self._jsonl = open(jsonl_path, "w", encoding="utf-8")  # noqa: SIM115(生命周期由 close() 管)
        self._rows: List[Dict[str, object]] = []
        self._summary: Optional[Dict[str, object]] = None

    def record(self, entry: Dict[str, object]) -> None:
        """写一行 JSONL，同时把标量字段挑出来进 CSV。

        每条都 flush：宁可慢一点，也不要出现"节点崩了日志也丢了"的取证事故。
        """
        self._jsonl.write(json.dumps(entry, ensure_ascii=False, sort_keys=True) + "\n")
        self._jsonl.flush()
        self._rows.append({k: entry.get(k, "") for k in CSV_FIELDS})

    def record_config(self, config: Dict[str, object]) -> None:
        """第一条记录 = 本次运行的参数快照。

        画图脚本要把 9 段轨迹拼回一条全局时间轴，必须知道 settle_time 和
        waypoint_gap；这两个值只存在于节点参数里，不落盘就成了"图能画但没人
        知道横轴怎么来的"。凡是产物之间有隐式依赖，就把依赖写进文件。
        """
        self._jsonl.write(
            json.dumps({"config": config}, ensure_ascii=False, sort_keys=True) + "\n"
        )
        self._jsonl.flush()

    def record_summary(self, summary: Dict[str, object]) -> None:
        """总结只进 JSONL。

        CSV 是"一条点位一行"的表，把总结塞进去会在末尾留一行空列，
        在 Excel 里 sort/filter 时非常碍事。
        """
        self._jsonl.write(
            json.dumps({"summary": summary}, ensure_ascii=False, sort_keys=True) + "\n"
        )
        self._jsonl.flush()

    def write_csv(self) -> None:
        with open(self.csv_path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self._rows)

    def close(self) -> None:
        self.write_csv()
        if not self._jsonl.closed:
            self._jsonl.close()


# --------------------------------------------------------------------------
# PASS/FAIL 判据
# --------------------------------------------------------------------------
def evaluate(
    rows: Sequence[Dict[str, object]],
    arrival_tol: float = ARRIVAL_TOL_RAD,
) -> Dict[str, object]:
    """按"每条点位都必须规划成功 + 执行成功 + 到位误差在容差内"判 PASS。

    刻意不把"路径长度/规划耗时"写进判据：OMPL 是随机规划器，这两项天然抖动，
    放进判据只会制造假故障。它们进日志、进图，但不进 PASS 条件。
    """
    n = len(rows)
    if n == 0:
        return {"n": 0, "plan_ok": 0, "exec_ok": 0, "verdict": "FAIL", "reason": "no waypoints"}

    plan_ok = sum(1 for r in rows if int(r.get("plan_ok", 0) or 0) == 1)
    exec_ok = sum(1 for r in rows if int(r.get("exec_ok", 0) or 0) == 1)
    # group 字段用于区分"臂"和"夹爪"：手指的容差比六轴松
    tols = [
        arrival_tol if str(r.get("group", "arm")) == "arm" else GRIPPER_ARRIVAL_TOL_RAD
        for r in rows
    ]
    arr = [float(r.get("arr_err_rad", 0.0) or 0.0) for r in rows]
    worst = max(arr, default=0.0)
    worst_ratio = max((e / t for e, t in zip(arr, tols)), default=0.0)
    track = [float(r.get("track_err_rad", 0.0) or 0.0) for r in rows]

    reasons: List[str] = []
    if plan_ok != n:
        reasons.append(f"plan_ok={plan_ok}/{n}")
    if exec_ok != n:
        reasons.append(f"exec_ok={exec_ok}/{n}")
    if worst_ratio > 1.0:
        reasons.append(f"arrival_ratio={worst_ratio:.2f}")
    if max(track, default=0.0) > TRACK_SANITY_RAD:
        reasons.append(f"track_err={max(track):.3f}")

    return {
        "n": n,
        "plan_ok": plan_ok,
        "exec_ok": exec_ok,
        # 观测项，不进判据：retries>0 说明链路竞态又出现了，
        # 但"重试后成功"本身是合法结果，不该把回归判 FAIL。
        "total_exec_attempts": sum(int(r.get("exec_attempts", 1) or 1) for r in rows),
        "retries": sum(max(0, int(r.get("exec_attempts", 1) or 1) - 1) for r in rows),
        # 重规划总次数（控制环丢拍后从当前状态重来）。同样只看不判。
        # 注意：exec_s / path_len_rad 只统计**最终成功那一轮**，所以 replans>0 时
        # 真实墙上时间比 exec_s 之和更长 —— 看总耗时要用日志时间戳，不是这个字段。
        "replans": sum(int(r.get("replans", 0) or 0) for r in rows),
        "max_arr_err_rad": worst,
        "arrival_tol_ratio": worst_ratio,
        "max_track_err_rad": max(track, default=0.0),
        "total_plan_s": round(sum(float(r.get("plan_s", 0.0) or 0.0) for r in rows), 4),
        "total_exec_s": round(sum(float(r.get("exec_s", 0.0) or 0.0) for r in rows), 4),
        # 等稳态耗时之和。旧实现固定 0.6 s x n，新实现一停稳就走 —— 这一项直接
        # 量化"省下的白等"，不用再去翻日志时间戳做减法。
        "total_settle_s": round(sum(float(r.get("settle_wait_s", 0.0) or 0.0) for r in rows), 4),
        "total_path_len_rad": round(sum(float(r.get("path_len_rad", 0.0) or 0.0) for r in rows), 4),
        "verdict": "PASS" if not reasons else "FAIL",
        "reason": ";".join(reasons) or "ok",
    }


def format_summary(summary: Dict[str, object]) -> str:
    """单行机器可读总结，写法与阶段一的 PLOT-RESULT 完全同构，便于 grep。"""
    parts = [
        f"n={summary.get('n')}",
        f"plan_ok={summary.get('plan_ok')}",
        f"exec_ok={summary.get('exec_ok')}",
        f"retries={summary.get('retries', 0)}",
        f"replans={summary.get('replans', 0)}",
        f"max_arr_err_rad={float(summary.get('max_arr_err_rad', 0.0)):.5f}",
        f"tol_ratio={float(summary.get('arrival_tol_ratio', 0.0)):.2f}",
        f"max_track_err_rad={float(summary.get('max_track_err_rad', 0.0)):.5f}",
        f"plan_s={summary.get('total_plan_s')}",
        f"exec_s={summary.get('total_exec_s')}",
        f"settle_s={summary.get('total_settle_s', 0.0)}",
        f"path_len_rad={summary.get('total_path_len_rad')}",
    ]
    return f"MOTION-SUMMARY {' '.join(parts)} RESULT {summary.get('verdict')} ({summary.get('reason')})"
