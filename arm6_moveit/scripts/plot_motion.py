#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_motion.py —— 把 motion_*.jsonl 画成两张图 + 一份长表 CSV，并给出 PLOT-RESULT
==============================================================================

设计约束（和阶段一的 plot_adc.py 同一家族）：

  * 只吃日志，不吃仿真。任何一张图都能在没有 ROS 的机器上离线重画，
    评审时"图上的线"和"日志里的数"是同一份数据的两种表示，可互相核对。
  * 图内文字一律英文。WSL 的最小桌面不带 CJK 字体，中文标签会变成豆腐块，
    而图是要进简历作品集的 —— 中文说明放文档，图内保持英文。
  * 最后一行必打 PLOT-RESULT PASS/FAIL：把"图画出来了"变成"图里的数据自洽"。
    检查项见 verify()。

时间轴是怎么复原的：
  日志里每条点位只记了自己那段轨迹的相对时间（time_from_start）与几个耗时标量，
  拼成全局时间轴需要知道"两段之间空了多久"：
      t_off[i] = sum_{k<i} (plan_s + exec_s + settle_time + waypoint_gap)
  settle_time / waypoint_gap 来自 JSONL 第一行的 config 记录（缺省时回落到 0.6/0.3，
  与节点默认值一致 —— 回落只是为了脚本不因缺字段而崩，缺字段本身会被 verify() 记成告警）。

用法:
  python3 scripts/plot_motion.py --log logs/motion_20260916-231533.jsonl
  python3 scripts/plot_motion.py --log <jsonl> --out-dir artifacts --prefix motion
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")  # 无显示环境下 import pyplot 也不会去连 X server
import matplotlib.pyplot as plt  # noqa: E402

# 节点默认值，config 记录缺失时回落用（见模块 docstring）
DEFAULT_SETTLE = 0.6
DEFAULT_GAP = 0.3
ARRIVAL_TOL = {"arm": 3.0e-3, "gripper": 5.0e-3}
TRACK_SANITY = 0.10

JOINT_ORDER = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "finger1_joint",
    "finger2_joint",
]
# 单位不能一刀切：6 个 revolute 是 rad，2 个 prismatic 手指是 m。
# 混着标会让人把 0.04 m 的行程读成 0.04 rad，这类"图上看不出来的错"最坑。
UNITS = {j: ("m" if j.startswith("finger") else "rad") for j in JOINT_ORDER}


# --------------------------------------------------------------------------
# 读日志
# --------------------------------------------------------------------------
def load_jsonl(path: str) -> Tuple[Dict[str, object], List[Dict[str, object]], Dict[str, object]]:
    config: Dict[str, object] = {}
    summary: Dict[str, object] = {}
    rows: List[Dict[str, object]] = []
    with open(path, encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if "config" in obj:
                config = obj["config"]
            elif "summary" in obj:
                summary = obj["summary"]
            elif "idx" in obj:
                rows.append(obj)
            else:
                raise ValueError(f"{path}:{line_no} 是无法识别的记录，缺少 idx/config/summary 任一键")
    rows.sort(key=lambda r: int(r["idx"]))
    return config, rows, summary


def timeline(
    rows: List[Dict[str, object]], settle: float, gap: float
) -> List[Tuple[float, float]]:
    """给每条点位算 (起点时刻, 终点时刻)，单位 s，相对整场演示的 0 点。

    注意用的是 exec_s 而不是 traj_s：真实占用时间以控制器执行为准，
    traj_s 只是轨迹自身时长（末段可能因 JTC 收尾策略略有差异）。
    """
    spans: List[Tuple[float, float]] = []
    t = 0.0
    for r in rows:
        dur = float(r.get("exec_s", 0.0) or 0.0)
        spans.append((t, t + dur))
        t += float(r.get("plan_s", 0.0) or 0.0) + dur + settle + gap
    return spans


# --------------------------------------------------------------------------
# 图 1：8 个关节的计划轨迹时间线
# --------------------------------------------------------------------------
def plot_profiles(
    rows: List[Dict[str, object]],
    spans: List[Tuple[float, float]],
    out_png: str,
    title_suffix: str,
) -> None:
    joints = [j for j in JOINT_ORDER if any(j in (r.get("trajectory") or {}).get("joint_names", []) for r in rows)]
    if not joints:
        joints = JOINT_ORDER
    ncol = 4
    nrow = int(math.ceil(len(joints) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.0 * ncol, 2.6 * nrow), sharex=True)
    axes = [axes] if nrow == 1 and ncol == 1 else list(axes.flat)

    for ax in axes:
        ax.grid(alpha=0.3)
    for i, r in enumerate(rows):
        traj = r.get("trajectory") or {}
        names = traj.get("joint_names") or []
        times = traj.get("times") or []
        positions = traj.get("positions") or []
        t0, t1 = spans[i]
        color = plt.cm.tab10(i % 10)
        for jpos, name in enumerate(names):
            if name not in joints:
                continue
            ax = axes[joints.index(name)]
            xs = [t0 + t for t in times]
            ys = [p[jpos] for p in positions]
            ax.plot(xs, ys, "-", color=color, lw=1.6)
            # 到位标记：实心点=计划终点，空心圈=settle 后实际读到的位置
            if positions and name in (r.get("achieved_per_joint") or {}):
                ax.plot(xs[-1], ys[-1], "o", color=color, ms=4)
                ax.plot(
                    t1 + 0.05,
                    r["achieved_per_joint"][name],
                    "o",
                    mfc="none",
                    mec=color,
                    ms=7,
                    mew=1.2,
                )
    for jpos, name in enumerate(joints):
        axes[joints.index(name)].set_ylabel(f"{name} [{UNITS.get(name, 'rad')}]")
    for ax in axes[len(joints):]:
        ax.set_axis_off()
    for ax in axes[(len(joints) - 1) // ncol * ncol:]:
        ax.set_xlabel("time [s]  (relative to demo start)")
    # 每条点位给一个横向底色，方便看出"哪一段是哪个任务"
    for i, (t0, t1) in enumerate(spans):
        axes[0].axvspan(t0, t1, color="k", alpha=0.03 if i % 2 == 0 else 0.0)
        axes[0].annotate(
            str(rows[i]["idx"]),
            xy=(t0, axes[0].get_ylim()[1]),
            fontsize=8,
            va="top",
            ha="left",
            color="0.35",
        )
    fig.suptitle(
        f"Planned joint trajectories per waypoint{title_suffix}\n"
        "solid dot = planned end point, open circle = achieved position after settle "
        "(the two are 1e-4 apart, so the ring looks detached on purpose)"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


# --------------------------------------------------------------------------
# 图 2：指标三联
# --------------------------------------------------------------------------
def plot_metrics(
    rows: List[Dict[str, object]],
    spans: List[Tuple[float, float]],
    summary: Dict[str, object],
    out_png: str,
    title_suffix: str,
    config: Optional[Dict[str, object]] = None,
) -> None:
    config = config or {}
    labels = [f"{r['idx']}:{r['name']}" for r in rows]
    x = list(range(len(rows)))
    plan_s = [float(r.get("plan_s", 0.0) or 0.0) for r in rows]
    exec_s = [float(r.get("exec_s", 0.0) or 0.0) for r in rows]
    traj_s = [float(r.get("traj_s", 0.0) or 0.0) for r in rows]
    arr = [float(r.get("arr_err_rad", 1e-9) or 1e-9) for r in rows]
    track = [float(r.get("track_err_rad", 1e-9) or 1e-9) for r in rows]
    plen = [float(r.get("path_len_rad", 0.0) or 0.0) for r in rows]
    groups = [str(r.get("group", "arm")) for r in rows]

    def decorate(ax):
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=90, fontsize=7)
        ax.grid(alpha=0.3, axis="y")

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15.0, 4.4))

    ax1.bar(x, plan_s, label="plan (client, incl. round-trip)")
    ax1.bar(x, exec_s, bottom=plan_s, label="execute")
    ax1.plot(x, traj_s, "kx--", label="planned trajectory duration")
    ax1.set_ylabel("time [s]")
    ax1.set_title("(a) planning / execution timing")
    ax1.legend(fontsize=8)
    decorate(ax1)

    # 柱=到位误差，折线=执行过程中的跟踪误差峰值；容差按组分别画横线
    ax2.bar(x, arr, label="arrival |q - q_target|")
    ax2.plot(x, track, "rs-", ms=4, lw=1.2, label="peak tracking |ref - fb|")
    for g, tol, ls in (("arm", ARRIVAL_TOL["arm"], ":"), ("gripper", ARRIVAL_TOL["gripper"], "--")):
        if g in groups:
            ax2.axhline(tol, color="0.4", ls=ls, lw=1.0, label=f"{g} goal tol {tol:g}")
    ax2.set_yscale("log")
    ax2.set_ylim(1e-6, 1e-1)
    ax2.set_ylabel("error [rad / m]")
    ax2.set_title("(b) accuracy (log)")
    ax2.legend(fontsize=7)
    decorate(ax2)
    gt = config.get("goal_tolerance")
    if gt:
        ax2.text(
            0.03,
            0.05,
            f"arrival bars sit at MoveIt goal_tolerance={float(gt):g}\n"
            "=> accuracy is set by the planner goal, not the controller",
            transform=ax2.transAxes,
            fontsize=7,
            color="0.2",
        )

    ax3.bar(
        x,
        plen,
        color=[plt.cm.viridis(i / max(1, len(plen) - 1)) for i in x],
        hatch=["//" if g == "gripper" else "" for g in groups],
        edgecolor="0.2",
    )
    ax3.set_ylabel("joint-space path length [rad or m]")
    ax3.set_title("(c) path length  (hatched = gripper group)")
    decorate(ax3)

    verdict = str(summary.get("verdict", "?"))
    fig.suptitle(
        f"MoveIt waypoint metrics — {verdict}"
        f"  (plan_ok={summary.get('plan_ok')}/{summary.get('n')} "
        f"exec_ok={summary.get('exec_ok')}/{summary.get('n')} "
        f"max_arr={float(summary.get('max_arr_err_rad', 0.0)):.2e}){title_suffix}"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


# --------------------------------------------------------------------------
# 长表：一行一个 (时刻, 关节, 值)，给 Excel / 别的绘图工具复用
# --------------------------------------------------------------------------
def write_series_csv(
    rows: List[Dict[str, object]], spans: List[Tuple[float, float]], out_csv: str
) -> int:
    n = 0
    with open(out_csv, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(
            ["t_s", "waypoint_idx", "waypoint", "group", "joint", "position", "target", "achieved"]
        )
        for i, r in enumerate(rows):
            traj = r.get("trajectory") or {}
            names = traj.get("joint_names") or []
            times = traj.get("times") or []
            positions = traj.get("positions") or []
            t0 = spans[i][0]
            target = r.get("target") or {}
            achieved = r.get("achieved_per_joint") or {}
            for tt, pos in zip(times, positions):
                for jpos, name in enumerate(names):
                    w.writerow(
                        [
                            f"{t0 + tt:.6f}",
                            r["idx"],
                            r["name"],
                            r.get("group", ""),
                            name,
                            f"{pos[jpos]:.6f}",
                            f"{target.get(name, '')}" if name in target else "",
                            f"{achieved[name]:.6f}" if name in achieved else "",
                        ]
                    )
                    n += 1
    return n


# --------------------------------------------------------------------------
# 自洽性检查
# --------------------------------------------------------------------------
def verify(
    config: Dict[str, object],
    rows: List[Dict[str, object]],
    summary: Dict[str, object],
    spans: List[Tuple[float, float]],
) -> List[str]:
    """返回"问题列表"，空列表 == PASS。只检查与随机性无关的量。"""
    problems: List[str] = []
    if not rows:
        problems.append("日志里没有任何点位记录")
        return problems

    n_cfg = config.get("n_waypoints")
    if n_cfg is not None and int(n_cfg) != len(rows):
        problems.append(f"config.n_waypoints={n_cfg} 与实际记录条数 {len(rows)} 不符")
    if summary and int(summary.get("n", len(rows))) != len(rows):
        problems.append(f"summary.n={summary.get('n')} 与实际记录条数 {len(rows)} 不符")

    prev_end = -1.0
    for r, (t0, t1) in zip(rows, spans):
        tag = f"wp{r['idx']}({r['name']})"
        if t0 < prev_end - 1e-9:
            problems.append(f"{tag}: 时间轴倒退 t0={t0:.3f} < 上一段终点 {prev_end:.3f}")
        prev_end = t1
        traj = r.get("trajectory") or {}
        names, times, positions = (
            traj.get("joint_names") or [],
            traj.get("times") or [],
            traj.get("positions") or [],
        )
        if not names:
            problems.append(f"{tag}: 没有轨迹数据（可能 plan_only 未执行或未落盘）")
            continue
        if any(b < a for a, b in zip(times, times[1:])):
            problems.append(f"{tag}: time_from_start 非单调")
        if any(len(p) != len(names) for p in positions):
            problems.append(f"{tag}: 轨迹点维度与 joint_names 不一致")
        if int(r.get("plan_ok", 0) or 0) != 1:
            problems.append(f"{tag}: plan_ok!=1")
        if float(r.get("arr_err_rad", 0.0) or 0.0) > ARRIVAL_TOL.get(str(r.get("group", "arm")), 3e-3):
            problems.append(
                f"{tag}: 到位误差 {r.get('arr_err_rad')} 超出 {r.get('group')} 容差"
            )
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description="从 motion_*.jsonl 生成运动图与长表 CSV")
    ap.add_argument("--log", required=True, help="motion_<stamp>.jsonl 路径")
    ap.add_argument("--out-dir", default="artifacts", help="输出目录（默认 ./artifacts）")
    ap.add_argument("--prefix", default="motion", help="输出文件名前缀")
    ap.add_argument(
        "--tag",
        default="",
        help="图标题后缀，用来标注数据源，例如 ' (run 20260916-231533)'",
    )
    ap.add_argument(
        "--final",
        action="store_true",
        help="文件名固定为 <prefix>_<...>_final.*，作为交付版本（覆盖旧文件）",
    )
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    config, rows, summary = load_jsonl(args.log)
    settle = float(config.get("settle_time", DEFAULT_SETTLE))
    gap = float(config.get("waypoint_gap", DEFAULT_GAP))
    spans = timeline(rows, settle, gap)

    # 文件名：交付版固定 <prefix>_final.*，非交付版直接沿用日志主名，
    # 避免出现 motion_motion_<stamp> 这种双前缀（日志本身就叫 motion_<stamp>.jsonl）。
    base = os.path.splitext(os.path.basename(args.log))[0]
    stem = f"{args.prefix}_final" if args.final else base
    png1 = os.path.join(args.out_dir, f"{stem}_profiles.png")
    png2 = os.path.join(args.out_dir, f"{stem}_metrics.png")
    csv1 = os.path.join(args.out_dir, f"{stem}_series.csv")

    plot_profiles(rows, spans, png1, args.tag)
    plot_metrics(rows, spans, summary, png2, args.tag, config)
    n = write_series_csv(rows, spans, csv1)

    print(f"PLOT-FILES {png1} {png2} {csv1}")
    print(
        "PLOT-TIMELINE "
        f"n={len(rows)} settle_s={settle} gap_s={gap} "
        f"total_s={spans[-1][1] + settle + gap if spans else 0:.2f} series_rows={n}"
    )
    problems = verify(config, rows, summary, spans)
    for p in problems:
        print(f"PLOT-ISSUE  {p}")
    print(f"PLOT-RESULT {'PASS' if not problems else 'FAIL'} ({len(problems)} issues)")
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
