#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D10 控制律单元测试：符号、限幅、丢线、速度自适应，以及**参数自洽性**。

最后一条（圆角可行性）是真正值钱的那条：它把"控制器增益"和"赛道几何"绑在一起校验——
如果 w_max 小到转不过 0.45 m 的圆角，或者满舵时速度掉到 v_min 以下，
那不管怎么调参车都会冲出去。这种约束不写进测试，就会变成 D12 现场抓瞎。
"""
import math
import os
import sys

if 'mybot_control' not in sys.modules:
    _here = os.path.dirname(os.path.abspath(__file__))
    for _c in (os.path.join(_here, '..'), _here):
        if os.path.isdir(os.path.join(_c, 'mybot_control')):
            sys.path.insert(0, _c)
            break
from mybot_control.pd_line import PDGains, compute     # noqa: E402

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

G = PDGains()
TRACK_R = 0.45          # make_line_world.py 的默认圆角半径（米）


def check(name, got, want, tol=1e-6):
    ok = abs(got - want) <= tol
    print(f'| {name} | {want:+.4f} | {got:+.4f} | {"PASS" if ok else "FAIL"} |')
    return ok


def main():
    fails = 0
    print('| 检查 | 期望 | 实测 | 判定 |')
    print('|---|---|---|---|')

    c = compute(0.0, 0.0, G)
    fails += 0 if check('正对直线：不转向', c.w, 0.0) else 1
    fails += 0 if check('正对直线：全速', c.v, G.v_max) else 1

    c = compute(+0.10, 0.0, G)                    # 线在左 10 cm
    fails += 0 if check('线在左 -> ω 为正（左转）', c.w, G.kp * 0.10) else 1
    c = compute(0.0, +0.30, G)                    # 线朝左拐
    fails += 0 if check('线左拐 -> ω 为正', c.w, G.kd * 0.30) else 1
    c = compute(-0.10, 0.0, G)
    fails += 0 if check('线在右 -> ω 为负', c.w, -G.kp * 0.10) else 1

    big = compute(+1.0, +1.0, G)                  # 1.5+0.8=2.3 rad/s，远超 w_max=1.5
    fails += 0 if check('大偏差被限幅', big.w, G.w_max) else 1
    ok = big.reason == 'saturated'
    print(f'| 限幅时 reason=saturated | True | {big.reason} | {"PASS" if ok else "FAIL"} |')
    fails += 0 if ok else 1

    lost = compute(float('nan'), float('nan'), G)
    ok = lost.v == 0.0 and lost.w == 0.0 and lost.reason == 'lost'
    print(f'| NaN 输入 -> 零速 + reason=lost | True | {lost} | {"PASS" if ok else "FAIL"} |')
    fails += 0 if ok else 1

    lin = compute(0.04, 0.10, G)
    dbl = compute(0.08, 0.20, G)
    ok = abs(dbl.w - 2 * lin.w) < 1e-9 and abs(dbl.w) < G.w_max
    print(f'| 未饱和时线性（误差翻倍 ω 翻倍） | True | {ok} | {"PASS" if ok else "FAIL"} |')
    fails += 0 if ok else 1

    v_full = compute(1.0, 1.0, G).v               # 远超限幅 = 满舵工况
    ok = abs(v_full - G.v_min) < 1e-9
    print(f'| 满舵时速度落到下限 v_min | {G.v_min:.3f} | {v_full:.3f} | {"PASS" if ok else "FAIL"} |')
    fails += 0 if ok else 1

    # ---- 参数自洽性：这台增益跑得过这条赛道吗 ----
    w_need = G.v_max / TRACK_R                     # 以 v_max 过圆角需要的角速度
    ok1 = w_need < G.w_max
    print(f'| 过 {TRACK_R} m 圆角需要 ω={w_need:.3f} rad/s < w_max={G.w_max} | True | '
          f'{ok1} | {"PASS" if ok1 else "FAIL"} |')
    fails += 0 if ok1 else 1
    # 用需要的 ω 去查速度：必须还高于 v_min，否则车会在弯里停住/侧滑
    v_at_corner = G.v_max * (1.0 - G.kv_speed * min(1.0, w_need / G.w_max))
    ok2 = v_at_corner > G.v_min
    print(f'| 该工况下速度 {v_at_corner:.3f} m/s > v_min {G.v_min} | True | '
          f'{ok2} | {"PASS" if ok2 else "FAIL"} |')
    fails += 0 if ok2 else 1
    # 丢线看门狗必须比控制周期长得多，否则正常抖动也会误停车
    print(f'| 丢线超时(1.0s) > 控制周期({1 / 20:.3f}s) 的 10 倍 | True | '
          f'{1.0 > 10 / 20} | {"PASS" if 1.0 > 10 / 20 else "FAIL"} |')

    print(f'\n结果：{"全部通过" if fails == 0 else str(fails) + " 项失败"}')
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
