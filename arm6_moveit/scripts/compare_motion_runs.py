#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_motion_runs.py —— 把多份 motion_*.csv 按点位对齐比较
==========================================================

用途：回答"回归到底稳不稳"这类问题。单看 MOTION-SUMMARY 的总量不够 ——
     总量相同可能是"两条错得不一样的轨迹互相抵消"，必须逐点位比。

哪些列该比、哪些不该比（这是本脚本存在的核心理由）：
    可比（与随机种子无关）：name / group / plan_ok / exec_ok / arr_err_rad / traj_s / exec_s
    不可比（RRTConnect 是随机规划器）：path_len_rad、traj_pts
    速度相关（必须归一化后才可比）：track_err_rad —— 见下

★ track_err_rad 为什么不能直接比（实测结论，不是猜的）★
    三次运行的 idx=4 pick：track_err_rad = 6.04e-4 / 9.50e-4 / 4.13e-3，
    最大跨次比值 6.8，看起来像"控制退化"。但同一批数据里：
        参考轨迹逐点最大差 = 1.7e-4 rad   （OMPL 随机，但量级远小于峰值误差）
        exec_s           = 2.2843/2.2847/2.2850  （差 0.03 %）
        arr_err_rad      = 6.04e-4→9e-5 级别，三次都 = goal_tolerance
        track_samples    = 229 / 229 / 229      （100 Hz 采样一次没丢）
    把每个点位、每一次运行的误差峰值除以其峰值关节在峰值时刻的参考速度，
    得到一个纯时序量 Δt = err / v：
        231909(无 rviz)  0.44 ~ 0.78 ms
        232746(rviz-A)   0.56 ~ 0.77 ms
        233006(rviz-B)   0.63 ~ 0.92 ms，仅 idx=4 = 3.04 ms
    即：track_err_rad 不是轨迹的属性，而是
        "参考速度 × 反馈相对指令的相位滞后"。
    证据链：所有运行的峰值都落在该关节速度最大的那一段，且绝对误差最大的
    点位恒为 place（joint6 参考速度 1.778 rad/s，全序列最快）。
    所以直接比 track_err_rad 的比值等于在比"采样相位落在哪一毫秒"，是噪声。
    本脚本改用 Δt 判定，并给出物理上界：JTC 周期 10 ms，
    Δt 接近/超过 10 ms 才说明真的丢了一个控制周期（那才是故障）。
    3.04 ms < 10 ms ⇒ 结论：软件渲染（Xvfb + llvmpipe）与 100 Hz 控制环争用
    CPU 造成的单次相位抖动，不是规划或控制退化 —— 已写入 docs/05、docs/07。

用法:
    python3 scripts/compare_motion_runs.py logs/motion_*.csv
    python3 scripts/compare_motion_runs.py logs/motion_*.csv --col track_err_rad

    # 回归集（把两份"缺陷留证"排除；它们不删，是 docs/07 的证据）：
    #   231533 = record() 漏调用产生的空 CSV（节点缺陷，已修）
    #   234501 = idx=1 吃到 CONTROL_FAILED(-4) 的那次（DDS 发现竞态，已修）
    python3 scripts/compare_motion_runs.py --col track_err_rad --exclude 234501,231533
退出码：MUST_MATCH 不一致、或有列的 Δt 越过控制周期上界时返回 1。
        速度相关列的原始比值过大只报 WARN（因为它按定义含采样相位噪声）。

★ 四种"不参与"，方式不同，别混为一谈 ★
    1) --exclude 子串匹配：**人工**决定哪些运行是缺陷留证（见上），连表都不上；
    2) 空 CSV（0 条点位）：只打一行"警告: … 不参与比较"，因为基准改选"记录最多"
       那份之后它自然出局；
    3) plan-only 运行（每一行都 exec_s=0 且 plan_ok=1）：**自动**从所有列剔除，
       打 `EXCLUDED <run> 原因 plan-only（…）`。
       本条以前靠每个调用方各自 `--exclude 005329,011057`：那份名单会随运行数增长，
       抄在 shell 与文档里迟早和实际数据不一致 —— **判据的名单也要有单一真源**。
       （旧版本这里写过"plan-only 仍上 MUST_MATCH 那张表"，是错的：它的 exec_s=0
        会让比值列直接 FAIL，所以才被迫在每处调用里手工排除。）
    4) 算不出 v_max 的运行（JSONL 缺失/无 trajectory）：速度相关列的判据**剔除**它
       并打印 `EXCLUDED <run> 原因 …`，其余运行照常按 Δt 判。
       绝不因为一份不合格就把整批降级成"比原始值" —— 降级照样产出 PASS/FAIL，
       比报错更难发现（本条来自实测的假 FAIL，见 docs/07 §2.6）。
    结果行的后两个字段 `N excluded, M plan-only` 把第 3、4 类摆在台面上：
    一张比对表少了谁，必须写在结论行里，不能只留在上面滚过的日志里。
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import re
import sys

# stdout 编码兜底（Windows 侧实测踩到的）：这里的输出全是中文和 ⇒ / Δ / ↔ 这类符号，
# 而 Windows 的 stdout 默认编码是 GBK。后果不是"乱码"而是**崩溃** ——
# 判据已经算出 `COMPARE-RESULT PASS`，最后打印 WARN 行时抛
# `UnicodeEncodeError: 'gbk' codec can't encode character '\u21d2'`，退出码变成非 0。
# 一个验收脚本因为"打印"而失败，是最难查的假故障（docs/07 §2.6 收了一条）。
# 原则：判据永远比它能被看见更重要，所以对不可编码字符降级替换而不是让它抛。
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# 逐点位必须完全一致的列（名字/分组/成功位）
MUST_MATCH = ["name", "group", "plan_ok", "exec_ok"]
# 允许抖动但量级不能变的列
SCALE_COLS = ["arr_err_rad", "exec_s", "traj_s", "plan_s"]
# 与参考速度成正比的列：判据用 err/v（=相位滞后 Δt），不用 err 本身
VELOCITY_SCALED = ["track_err_rad"]
# JTC 控制周期(ms)。Δt 一旦逼近它，说明丢拍，属于真故障
CONTROL_PERIOD_MS = 10.0
# 原始比值的告警线（只 WARN，不 FAIL）
RATIO_WARN = 5.0


def load(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        return {int(r["idx"]): r for r in csv.DictReader(fh)}


def peak_joint_max_velocity(jsonl_path: str, reason: dict | None = None) -> dict:
    """从同名 JSONL 里取每个点位"峰值关节在整条参考轨迹上的最大 |dq/dt|"。

    只用 CSV 算不出这个 —— CSV 没存轨迹点。JSONL 的 trajectory 字段是
    waypoint_mover 原样落盘的 moveit_msgs/RobotTrajectory（已压成 list）。
    返回 {idx: (joint_name, v_max_rad_s)}；文件/字段缺失就返回空 dict。
    传 reason={} 时可拿到失败原因（见 set_reason），调用方据此**显式剔除**该运行，
    而不是把整批判据降级 —— 绝不静默造假数据。
    """
    out: dict[int, tuple[str, float]] = {}
    if not os.path.isfile(jsonl_path):
        set_reason(reason, jsonl_path, "缺同名 JSONL")
        return out
    try:
        with open(jsonl_path, encoding="utf-8") as fh:
            rows = [json.loads(l) for l in fh if l.strip()]
    except (OSError, json.JSONDecodeError):
        set_reason(reason, jsonl_path, "JSONL 无法解析")
        return out
    for r in rows:
        idx = r.get("idx")
        traj = r.get("trajectory") or {}
        joint = r.get("track_err_joint")
        names = traj.get("joint_names") or []
        pos = traj.get("positions") or []
        tim = traj.get("times") or []
        if idx is None or joint not in names or len(pos) < 2 or len(tim) != len(pos):
            continue
        j = names.index(joint)
        vmax = 0.0
        for i in range(1, len(tim)):
            dt = tim[i] - tim[i - 1]
            if dt > 1e-12:
                vmax = max(vmax, abs((pos[i][j] - pos[i - 1][j]) / dt))
        if vmax > 0:
            out[int(idx)] = (joint, vmax)
    if not out:
        set_reason(reason, jsonl_path,
                   f"JSONL 有 {len(rows)} 行但没有可用的 trajectory/joint 字段"
                   "（plan-only 运行不执行轨迹，本就不该参与执行期误差比对）")
    return out


def set_reason(reason: dict | None, path: str, why: str) -> None:
    """把"这份为什么不能归一化"回传给调用方；reason 为 None 时什么也不做。"""
    if reason is not None:
        reason["why"] = f"{os.path.basename(path)}: {why}"


def tag_of(name: str) -> str:
    m = re.search(r"(\d{8})-(\d{6})", name)
    return (m.group(0)[-12:] if m else name[:12])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="*", help="motion_*.csv；不给就匹配 logs/motion_*.csv")
    ap.add_argument("--col", default="", help="只重点看这一列")
    ap.add_argument(
        "--exclude",
        default="",
        help="逗号分隔的子串；命中的文件不参与比较（用于把\"缺陷留证\"的运行排除在回归集外）",
    )
    args = ap.parse_args()

    files = args.csvs or sorted(glob.glob("logs/motion_*.csv"))
    if args.exclude:
        pats = [p.strip() for p in args.exclude.split(",") if p.strip()]
        keep = [f for f in files if not any(p in os.path.basename(f) for p in pats)]
        for f in files:
            if f not in keep:
                print(f"已排除(缺陷留证): {os.path.basename(f)}")
        files = keep
    if len(files) < 2:
        print(f"COMPARE-RESULT SKIP (只有 {len(files)} 份 CSV，无法比较)")
        return 0
    runs = {os.path.basename(f): load(f) for f in files}
    for t, r in runs.items():
        if not r:
            print(f"警告: {t} 是空表（0 条点位记录），不参与比较")
    # 基准选"记录最多"的那份，而不是字典序第一份：
    # 本目录里就留了一份早期缺陷产生的空 CSV（record() 漏调用），
    # 拿它当基准会让整张比较表变成空 —— 而且还会"PASS"，最坑的就是这种。
    full = {t: r for t, r in runs.items() if r}
    if len(full) < 2:
        print("COMPARE-RESULT SKIP (非空 CSV 少于 2 份)")
        return 0

    # plan-only 运行（execute:=false）自动出局。判据：每一行都 exec_s==0 且 plan_ok==1。
    #   为什么写进代码而不是让每个调用方各自 --exclude：那份名单会随着跑过的次数增长，
    #   抄在 shell 和文档里迟早和实际数据不一致（docs/00 §7 规则 1 防的就是这个）。
    #   为什么不只看 exec_s==0：真实失败的行也可以很快（run 234501 是 0.0029 s），
    #   但它的 plan_ok 序列里有一行 exec_ok=0，且 exec_s 非零 —— 那种必须留在比对里当故障证据。
    def is_plan_only(r: dict) -> bool:
        return bool(r) and all(
            float(row.get("exec_s", 0) or 0) == 0.0 and str(row.get("plan_ok")) == "1"
            for row in r.values()
        )

    plan_only = sorted(t for t in full if is_plan_only(full[t]))
    for t in plan_only:
        print(f"EXCLUDED {t} 原因 plan-only（{len(full[t])} 行全为 exec_s=0："
              "只规划不执行，不参与执行期回归比对）")
    full = {t: r for t, r in full.items() if t not in plan_only}
    if len(full) < 2:
        print("COMPARE-RESULT SKIP (剔除 plan-only 后不足 2 份)")
        return 0
    tags = sorted(full, key=lambda t: (-len(full[t]), t))
    base = tags[0]

    # 每份 CSV 找同名 JSONL，取速度归一化所需的 v_max
    vel = {}
    why: dict[str, str] = {}
    for t in tags:
        stem = os.path.join(os.path.dirname(base_path(files, t)), t[:-4] if t.endswith(".csv") else t)
        rsn: dict = {}
        vel[t] = peak_joint_max_velocity(stem + ".jsonl", rsn)
        if not vel[t]:
            why[t] = rsn.get("why", "无可用 v_max")
    # ★ 速度相关列：算不出 v_max 的那份必须**显式剔除**，不能把整批判据降级。
    #   早期版本的写法是 `have_vel = all(...)`，一份不合格的 CSV 就让全部运行退回
    #   "直接比原始值" —— 实测后果：一次 plan-only 运行（JSONL 里没有 trajectory）
    #   使 idx=4 报出"跨次比值 6.8 ⇒ 量级变化"的假 FAIL，而同一批数据的 Δt 判据
    #   给出 3.04 ms < 控制周期 10 ms（= docs/05 §6.1 已定性的相位抖动）。
    #   判据降级比判据缺失更危险：它照样输出一个看起来合理的 PASS/FAIL。
    dtags = [t for t in tags if vel.get(t)]
    excluded = [t for t in tags if t not in dtags]
    if args.col in VELOCITY_SCALED:
        for t in excluded:
            print(f"EXCLUDED {t} 原因 {why[t]}（不参与 {args.col} 的 Δt 判据）")

    print(f"基准 = {base}")
    print(f"{'idx':>3} {'点位名':<14} " + " ".join(f"{tag_of(t):>14}" for t in tags))
    issues: list[str] = []
    warns: list[str] = []
    for idx in sorted(runs[base]):
        row0 = runs[base][idx]
        cells = []
        for t in tags:
            r = runs[t].get(idx)
            cells.append("缺失" if r is None else f"{float(r.get(args.col, 0.0) or 0.0):.2e}")
            if r is None:
                issues.append(f"{t} 缺少 idx={idx}")
                continue
            for col in MUST_MATCH:
                if str(r.get(col)) != str(row0.get(col)):
                    issues.append(f"{t} idx={idx} {col}: {r.get(col)} != {row0.get(col)}")
        print(f"{idx:>3} {row0.get('name','?'):<14} " + " ".join(f"{c:>14}" for c in cells))

    if args.col:
        normed = args.col in VELOCITY_SCALED
        cols = dtags if normed else tags
        if normed and len(cols) < 2:
            print(f"\n>>> 重点列 {args.col} 需要 v_max 才能归一化，"
                  f"可比运行只有 {len(cols)} 份 ⇒ 无法判 Δt")
            print("COMPARE-RESULT SKIP (速度相关列无可比运行)")
            return 0
        head = ("Δt = " + args.col + "/v_max  (ms，越小越好，上界 = 控制周期 "
                f"{CONTROL_PERIOD_MS:.0f} ms，"
                f"参与 {len(cols)} 份"
                + (f"，剔除 {len(excluded)} 份" if excluded else "")
                + ")") if normed else \
               (f">>> 重点列 {args.col} 的跨次比值（>5 视为量级变化）")
        print(f"\n{head}")
        # Δt 表的列序和上面那张原始值表**不同**（这张剔除了算不出 v_max 的运行），
        # 所以不能靠"回看上一张表头"认列 —— 必须自己印一行，也让 shell 能按列名取值。
        dt_max = {t: 0.0 for t in cols}
        if normed:
            print("DT cols=" + ",".join(tag_of(t) for t in cols))
        for idx in sorted(runs[base]):
            raw = []
            for t in cols:
                r = runs[t].get(idx)
                v = float((r or {}).get(args.col, 0.0) or 0.0)
                if normed:
                    info = vel[t].get(idx)
                    v = v / info[1] * 1000.0 if (info and v > 0) else 0.0
                raw.append(v)
            if normed:
                for t, v in zip(cols, raw):
                    dt_max[t] = max(dt_max[t], v)
            vals = [v for v in raw if v > 0]
            if not vals:
                continue
            b = min(vals)  # 基准份本身可能不是最小值，用最小值做分母更保守
            ratios = [v / b for v in vals]
            over_period = normed and max(vals) >= CONTROL_PERIOD_MS
            if normed:
                flag = "丢拍" if over_period else "OK"
                # 归一化后仍比原始比值：过大只 WARN，因为 Δt 含采样相位噪声
                if max(ratios) > RATIO_WARN and not over_period:
                    warns.append(
                        f"idx={idx} {name_of(runs, idx)}: {args.col} 原始比值 "
                        f"{max(ratios):.1f}，但归一化后 Δt={max(vals):.2f} ms "
                        f"< 控制周期 {CONTROL_PERIOD_MS:.0f} ms ⇒ 采样相位抖动，非控制退化")
                print(f"  idx={idx:<3} {name_of(runs, idx):<14} "
                      + " ".join(f"{v:6.2f}" for v in raw) + f"  {flag}")
            else:
                flag = "OK" if max(ratios) <= RATIO_WARN else "量级变化"
                if flag != "OK":
                    issues.append(f"idx={idx} {args.col} 跨次比值 {max(ratios):.1f}")
                print(f"  idx={idx:<3} {name_of(runs, idx):<14} "
                      + " ".join(f"{v:6.2f}" for v in ratios) + f"  {flag}")
            if over_period:
                issues.append(f"idx={idx} {args.col} 归一化滞后 Δt={max(vals):.2f} ms "
                              f"≥ 控制周期 {CONTROL_PERIOD_MS:.0f} ms（丢拍）")
        if normed:
            # 每份一个机器行：调用方（check_concurrency.sh、文档里的 Δt 表）
            # 不必去猜列序，也不必将就"表里最大值恰好来自哪一份"这种歧义。
            for t in cols:
                print(f"DT max {tag_of(t)} {dt_max[t]:.2f} ms")

    print(f"\nCOMPARE-RESULT {'PASS' if not issues else 'FAIL'} "
          f"({len(issues)} issues, {len(warns)} warns, {len(excluded)} excluded, "
          f"{len(plan_only)} plan-only)")
    for i in issues[:20]:
        print(f"  ISSUE {i}")
    for w in warns[:20]:
        print(f"  WARN  {w}")
    return 0 if not issues else 1


def base_path(files: list[str], tag: str) -> str:
    for f in files:
        if os.path.basename(f) == tag:
            return f
    return tag


def name_of(runs: dict, idx: int) -> str:
    for t in runs:
        r = runs[t].get(idx)
        if r:
            return r.get("name", "?")
    return "?"


if __name__ == "__main__":
    raise SystemExit(main())
