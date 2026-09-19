#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D11 状态机单元测试：合成传感器序列，不依赖 ROS / Gazebo。

跑法（不需要开仿真）：
  ros2 run mybot_control test_avoid.py
  或 python3 mybot_control/scripts/test_avoid.py

为什么这些用例值得写而不是直接在 Gazebo 里试：
  最关键的一条安全性质是"丢线 + 正前方有障碍时，必须停住、不许转身找线"。
  这个状态在真实仿真里要靠"把方块正好摆到挡住线又正好停在 0.30 m 处"才能撞上，
  复现成本高且不稳定；而它恰恰是**唯一会让车撞上去**的那条分支。
  合成序列可以 1 毫秒喂进 10 万种时间线。
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mybot_control.avoid import (                                    # noqa: E402
    DECEL, FOLLOW, OBSTACLE_STOP, SAFE_STOP, SEARCH, AvoidFSM, AvoidParams,
    front_obstacle_distance)
from mybot_control.pd_line import PDGains                           # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'   [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def make_scan(n=360, fill=float('inf'), hits=()):
    """构造一帧 ranges：hits = [(角度°, 距离 m), ...]，其余填 fill。"""
    angle_min = -math.pi
    incr = 2 * math.pi / n
    r = [fill] * n
    for deg, dist in hits:
        i = int(round((math.radians(deg) - angle_min) / incr))
        r[i % n] = dist
    return r, angle_min, incr


def step_to(fsm, now, line_ok=True, e=0.0, theta=0.0, front=None, g=None):
    return fsm.step(now, line_ok, e, theta, front, g or PDGains())


# ───────────────────────── 1. 扇区提取 ─────────────────────────
def test_sector():
    print('\n[1] 前向扇区提取（含自我回波与盲区过滤）')
    p = AvoidParams()

    # 实测复现：车壳自我回波 0.123~0.307 m，角度 |θ|>=98°
    r, amin, incr = make_scan(hits=[(-105.8, 0.123), (-112.8, 0.125), (114.8, 0.127),
                                    (98.8, 0.132), (135.9, 0.196), (174.0, 0.307)])
    f = front_obstacle_distance(r, amin, incr, p)
    check('只有自我回波时前方判定为干净 (None)', f is None, f'实测={f}')

    # 实测复现：0.12 m 方块在 0.64 m 处，占 ±5.4°
    r, amin, incr = make_scan(hits=[(0.5, 0.652), (-4.5, 0.628), (4.5, 0.642)])
    f = front_obstacle_distance(r, amin, incr, p)
    check('正前方 0.63 m 的方块能扫到', f is not None and abs(f - 0.628) < 1e-6, f'实测={f}')

    # 盲区/噪声：前向 0.10 m 的回波必须丢（range_min=0.12，比它更近不可信）
    r, amin, incr = make_scan(hits=[(0.0, 0.10), (2.0, 0.9)])
    f = front_obstacle_distance(r, amin, incr, p)
    check('盲区内读数被丢弃，不误判为贴脸障碍', f is not None and f > 0.8, f'实测={f}')

    # inf / nan / 超距
    r, amin, incr = make_scan(hits=[(0.0, float('nan'))], fill=float('inf'))
    check('全 inf + 一个 nan -> None（不崩）', front_obstacle_distance(r, amin, incr, p) is None)
    r, amin, incr = make_scan(hits=[(0.0, 5.0)])
    check('超出 max_consider 的远回波不参与避障',
          front_obstacle_distance(r, amin, incr, p) is None)

    # 扇区边界：26° 不该算前方
    r, amin, incr = make_scan(hits=[(26.0, 0.4)])
    check('26° 的回波不在 ±25° 扇区内', front_obstacle_distance(r, amin, incr, p) is None)


# ───────────────────────── 2. 基本状态迁移 ─────────────────────────
def test_basic_transitions():
    print('\n[2] 基本状态迁移')
    g = PDGains()

    fsm = AvoidFSM()
    d = step_to(fsm, 1.0, line_ok=True, e=0.02, theta=0.01, front=None, g=g)
    check('线正常 + 前方干净 -> FOLLOW', d.state == FOLLOW, d.state)
    check('FOLLOW 时确实给了前进速度', d.cmd.v > 0.1, f'v={d.cmd.v:.3f}')

    fsm = AvoidFSM()
    d = step_to(fsm, 1.0, front=0.45, g=g)
    check('front 落在 (stop,warn) -> DECEL', d.state == DECEL, f'{d.state} front={d.front}')
    check('DECEL 压低了速度', 0 < d.cmd.v < g.v_max, f'v={d.cmd.v:.3f}')

    fsm = AvoidFSM()
    d = step_to(fsm, 1.0, front=0.25, g=g)
    check('front <= stop -> OBSTACLE_STOP', d.state == OBSTACLE_STOP, d.state)
    check('OBSTACLE_STOP 时 v=0 且 w=0', d.cmd.v == 0.0 and d.cmd.w == 0.0,
          f'v={d.cmd.v} w={d.cmd.w}')

    # 障碍移开后恢复
    fsm = AvoidFSM()
    step_to(fsm, 1.0, front=0.20)
    d = step_to(fsm, 2.0, front=None, e=0.02, g=g)
    check('障碍移开 + 线还在 -> 立刻恢复巡线', d.state == FOLLOW and d.cmd.v > 0.1,
          f'{d.state} v={d.cmd.v:.3f}')


# ───────────────────────── 3. ★安全性质★ ─────────────────────────
def test_safety_properties():
    print('\n[3] 安全性质（这几条挂了就不许合）')
    g = PDGains()

    # 核心：线丢了 + 正前方有障碍 -> 必须 OBSTACLE_STOP，绝不能 SEARCH
    fsm = AvoidFSM()
    step_to(fsm, 0.0, line_ok=True, front=None)            # 先建立"有线"的时间基准
    d = step_to(fsm, 5.0, line_ok=False, front=0.20)       # 丢线且障碍贴脸
    check('★丢线 + 前方有障碍 -> 停住，不转身找线',
          d.state == OBSTACLE_STOP, f'实际={d.state}')
    check('★该状态下没有任何角速度（不会把自己甩进障碍）',
          d.cmd.v == 0.0 and d.cmd.w == 0.0, f'v={d.cmd.v} w={d.cmd.w}')

    # 正在 SEARCH 途中突然冒出障碍 -> 立刻让位给 OBSTACLE_STOP
    fsm = AvoidFSM()
    step_to(fsm, 0.0, line_ok=True)
    step_to(fsm, 2.0, line_ok=False, front=None)           # 进入 SEARCH
    d = step_to(fsm, 2.5, line_ok=False, front=0.25)       # 障碍出现
    check('★SEARCH 中途出现障碍 -> 立刻 OBSTACLE_STOP',
          d.state == OBSTACLE_STOP, f'实际={d.state}')

    # 减速期间转向不能被砍掉（砍掉=把车甩出线外）
    fsm = AvoidFSM()
    d_line = step_to(fsm, 1.0, line_ok=True, e=0.15, theta=0.30, front=None, g=g)
    fsm2 = AvoidFSM()
    d_dec = step_to(fsm2, 1.0, line_ok=True, e=0.15, theta=0.30, front=0.45, g=g)
    check('★DECEL 只压速度、不砍转向',
          abs(d_dec.cmd.w - d_line.cmd.w) < 1e-9 and d_dec.cmd.v < d_line.cmd.v,
          f'w {d_line.cmd.w:.3f}->{d_dec.cmd.w:.3f}, v {d_line.cmd.v:.3f}->{d_dec.cmd.v:.3f}')

    # 瞬时丢线（D10 圆角那 4 帧的情况）不该触发搜索
    fsm = AvoidFSM()
    step_to(fsm, 0.0, line_ok=True)
    d = step_to(fsm, 0.15, line_ok=False, front=None)
    check('短暂丢线(<lost_timeout) 不进入 SEARCH', d.state != SEARCH, d.state)

    # SAFE_STOP 是吸收态
    fsm = AvoidFSM()
    step_to(fsm, 0.0, line_ok=True)
    t = 1.5
    while fsm.state != SAFE_STOP and t < 30:
        step_to(fsm, t, line_ok=False, front=None)
        t += 0.05
    check('持续丢线 + 前方干净 -> 先 SEARCH 后 SAFE_STOP',
          fsm.state == SAFE_STOP, f'{fsm.state} t={t:.1f}')
    d = step_to(fsm, t + 1.0, line_ok=True, e=0.0, front=None)   # 线又回来了
    check('★SAFE_STOP 是吸收态：线回来也不自己复活', d.state == SAFE_STOP, d.state)


# ───────────────────────── 4. 搜索行为 ─────────────────────────
def test_search():
    print('\n[4] 丢线搜索行为')
    p = AvoidParams()
    fsm = AvoidFSM(p=p)
    step_to(fsm, 0.0, line_ok=True)
    d = step_to(fsm, p.lost_timeout_s + 0.1, line_ok=False, e=0.10, front=None)
    check('丢线超时 + 前方干净 -> SEARCH', d.state == SEARCH, d.state)
    check('SEARCH 时 v=0（原地转，不往前拱）', d.cmd.v == 0.0, f'v={d.cmd.v}')
    check('线上次在左(e>0) -> 先左转', d.cmd.w > 0, f'w={d.cmd.w:.3f}')

    # 反向扫描：沿时间轴推进，收集 SEARCH 期间的角速度符号
    g = PDGains()
    fsm2 = AvoidFSM(p=p)
    step_to(fsm2, 0.0, line_ok=True)
    ws = []
    t = p.lost_timeout_s + 0.1
    while t < p.lost_timeout_s + p.search_flip_s * 2.5:
        dd = step_to(fsm2, t, line_ok=False, e=0.10, front=None, g=g)
        if dd.state == SEARCH:
            ws.append((t, dd.cmd.w))
        t += 0.1
    signs = {1 if w > 0 else -1 for _, w in ws}
    check(f'搜索会在 {p.search_flip_s}s 后反向（采到 {len(ws)} 帧，出现 {len(signs)} 个方向）',
          len(signs) >= 2, f'方向集合={sorted(signs)}')

    # 线上次在右 -> 先右转
    fsm3 = AvoidFSM(p=p)
    step_to(fsm3, 0.0, line_ok=True)
    d3 = step_to(fsm3, p.lost_timeout_s + 0.1, line_ok=False, e=-0.10, front=None)
    check('线上次在右(e<0) -> 先右转', d3.cmd.w < 0, f'w={d3.cmd.w:.3f}')


# ───────────────────────── 4b. 迟滞（防阈值抖振） ─────────────────────────
def test_hysteresis():
    print('\n[4b] 迟滞：停在障碍前后，噪声不该让状态来回跳')
    g = PDGains()
    p = AvoidParams()
    fsm = AvoidFSM(p=p)
    step_to(fsm, 0.0, line_ok=True, front=None)
    step_to(fsm, 1.0, line_ok=True, front=0.28, g=g)
    check('前向 0.28 -> OBSTACLE_STOP', fsm.state == OBSTACLE_STOP, fsm.state)

    # 0.32 在 stop(0.30) 与 release(0.40) 之间：已停住时不该脱出
    d = step_to(fsm, 2.0, line_ok=True, front=0.32, g=g)
    check('★ 0.32（stop 与 release 之间）仍保持 OBSTACLE_STOP，不脱出',
          d.state == OBSTACLE_STOP, d.state)

    # 越过 release 门限才让脱出，且脱出到 (stop, warn) 之间应是 DECEL
    d = step_to(fsm, 3.0, line_ok=True, front=0.45, g=g)
    check('0.45（越过 release 0.40）-> 脱出为 DECEL', d.state == DECEL, d.state)

    d = step_to(fsm, 4.0, line_ok=True, front=0.90, g=g)
    check('0.90（越过 warn 0.55）-> FOLLOW', d.state == FOLLOW, d.state)

    # 复现实测噪声：0.289~0.311 摆 200 帧，必须零次状态翻转
    fsm2 = AvoidFSM(p=p)
    step_to(fsm2, 0.0, line_ok=True, front=None)
    step_to(fsm2, 1.0, line_ok=True, front=0.28, g=g)
    states = []
    for i in range(200):
        fr = 0.30 + (0.011 if i % 2 else -0.011)
        states.append(step_to(fsm2, 2.0 + i * 0.05, line_ok=True, front=fr, g=g).state)
    check(f'★ 实测噪声幅度(±11mm)喂 200 帧 -> 状态翻转 0 次',
          len(set(states)) == 1 and states[0] == OBSTACLE_STOP,
          f'出现过的状态={set(states)}')

    # 迟滞不能把"真的更近了"漏掉：停在 0.28 后突然 0.20，仍必须判为 blocked
    fsm3 = AvoidFSM(p=p)
    step_to(fsm3, 0.0, line_ok=True, front=0.28)
    d = step_to(fsm3, 1.0, line_ok=True, front=0.20, g=g)
    check('release 门限不会漏判"障碍又逼近"', d.state == OBSTACLE_STOP, d.state)


# ───────────────────────── 5. 参数自洽性 ─────────────────────────
def test_param_consistency():
    """把阈值和**物理现实**绑在一起校验。不写进测试，就会变成现场"怎么调都撞"。"""
    print('\n[5] 参数自洽性（阈值 vs 传感器与车体物理）')
    p = AvoidParams()
    g = PDGains()

    # /scan 只有 5 Hz：两帧之间车前进的距离必须小于减速带宽度
    scan_rate = 5.0
    per_frame = g.v_max / scan_rate
    band = p.warn_dist_m - p.stop_dist_m
    check(f'减速带 {band:.2f} m > 两帧激光间前进 {per_frame:.2f} m',
          band > per_frame, f'带={band:.3f} 帧距={per_frame:.3f}')

    # stop_dist 必须大于激光盲区，否则"停住"时其实已经贴脸
    lidar_min = 0.12
    check(f'stop_dist {p.stop_dist_m} > 激光盲区 {lidar_min}', p.stop_dist_m > lidar_min)

    # 车头比激光靠前：base 半长 0.20，激光在 x=0.10 -> 车头在激光前 0.10 m
    front_overhang = 0.20 - 0.10
    gap = p.stop_dist_m - front_overhang
    check(f'停住时车头与障碍净空 {gap:.2f} m > 0', gap > 0.05,
          f'front={p.stop_dist_m} 悬出={front_overhang} 净空={gap:.3f}')

    # 自我回波最近角 98°，扇区必须留出安全余量
    check(f'扇区 {p.front_half_angle_deg}° + 余量 <= 自我回波起始角 98°',
          p.front_half_angle_deg + 20.0 <= 98.0)

    # 搜索角速度不该比巡线还猛（原地快转会加剧遮挡、也更容易甩出线外）
    check(f'search_w {p.search_w} < w_max {g.w_max}', p.search_w < g.w_max)

    # 搜索总时长必须明显大于一次翻转周期，否则来不及反向就超时
    check(f'search_max {p.search_max_s}s > 2x flip {p.search_flip_s}s',
          p.search_max_s > 2 * p.search_flip_s)

    # 迟滞门限必须落在减速带内，否则"一脱出就直接全速"，减速带形同虚设
    check(f'stop+release {p.stop_dist_m + p.release_margin_m:.2f} 落在 (stop, warn) 之间',
          p.stop_dist_m < p.stop_dist_m + p.release_margin_m < p.warn_dist_m,
          f'release={p.release_margin_m} stop={p.stop_dist_m} warn={p.warn_dist_m}')
    # 迟滞必须大于激光噪声幅度（实测停在障碍前时前向距离极差 0.023 m）
    check(f'release_margin {p.release_margin_m} > 实测噪声极差 0.023',
          p.release_margin_m > 0.023)

    # 非法参数必须被拦下来
    try:
        AvoidParams(stop_dist_m=0.6, warn_dist_m=0.5)
        check('stop>=warn 会被拒绝', False, '没抛异常')
    except ValueError:
        check('stop>=warn 会被拒绝', True)
    try:
        AvoidParams(front_half_angle_deg=100.0)
        check('扇区>=90° 会被拒绝（会吃到自我回波）', False, '没抛异常')
    except ValueError:
        check('扇区>=90° 会被拒绝（会吃到自我回波）', True)


# ───────────────────────── 6. 长时间随机序列不崩 ─────────────────────────
def test_robust():
    print('\n[6] 病态输入鲁棒性')
    fsm = AvoidFSM()
    g = PDGains()
    ok = True
    try:
        t = 0.0
        while t < 60.0:
            d = fsm.step(t, line_ok=(int(t * 3) % 2 == 0), e=float('nan'),
                         theta=float('nan'), front=0.0 if int(t) % 5 == 0 else None,
                         gains=g)
            assert d.state in (FOLLOW, DECEL, OBSTACLE_STOP, SEARCH, SAFE_STOP)
            assert math.isfinite(d.cmd.v) and math.isfinite(d.cmd.w), d.cmd
            t += 0.05
    except Exception as ex:                                  # noqa: BLE001
        ok = False
        print(f'    异常：{ex!r}')
    check('60 秒病态序列（NaN 偏差 + 0 距离 + 高频翻转）不崩、输出恒有限', ok)

    # 首帧就丢线（last_line_time=0 的边界）
    fsm2 = AvoidFSM()
    d = step_to(fsm2, 0.02, line_ok=False, front=None)
    check('开机第一帧就丢线不会立刻判丢', d.state in (FOLLOW, SEARCH) and d.cmd.v == 0.0,
          f'{d.state} v={d.cmd.v}')


def main() -> int:
    print('=' * 66)
    print('D11 避障状态机单元测试')
    print('=' * 66)
    test_sector()
    test_basic_transitions()
    test_safety_properties()
    test_search()
    test_hysteresis()
    test_param_consistency()
    test_robust()
    print('\n' + '=' * 66)
    if FAILS:
        print(f'❌ 失败 {len(FAILS)} 项：')
        for f in FAILS:
            print(f'   - {f}')
        return 1
    print('✅ 全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
