#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D9 单元测试：用**解析真值**验证 e / θ 算得对不对。

思路（这是本文件的全部价值）：
  1) 给定地面直线 y = e0 + tan(θ0)·x（米、弧度），用相机模型 `ground_to_pixel`
     把它画成一张 mask —— 真值是**构造出来的**，不是人眼估的；
  2) 把这张 mask 丢给 `extract()`，看反解出的 e/θ 与 (e0, θ0) 差多少；
  3) 容差：e ±1 cm、θ ±0.03 rad（≈1.7°）。

覆盖的失败模式：横偏、夹角、干扰块（D8 那块污渍）、断线、以及"线跑出了画面"
——最后一种必须**拒绝输出**而不是给个像模像样的错值，这也是断言的一部分。

跑法：
  ros2 run mybot_control test_line_features.py
  或 python3 scripts/test_line_features.py（需要先 source 过 install，让包可导入）
"""
import math
import os
import sys

import cv2
import numpy as np

# 直接跑脚本时（没走 ros2 run）把仓库里的包目录挂上，方便本地调试
if 'mybot_control' not in sys.modules:
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, '..'), here):
        if os.path.isdir(os.path.join(cand, 'mybot_control')):
            sys.path.insert(0, cand)
            break
from mybot_control.line_features import CamGeom, draw_line, extract   # noqa: E402
from mybot_control.masking import BASELINE, FEATURE, make_mask        # noqa: E402

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

W, H = 320, 240
GEOM = CamGeom.from_hfov(W, H, 1.2)          # 与 mybot_sensors.urdf.xacro 的 hfov 一致
TOL_E, TOL_TH = 0.010, 0.030


def blob(mask, u, v, r=26, value=255):
    """在画面上画一块"像污渍的"干扰区（用来验证逐行最长段 + 歧义行丢弃有效）。"""
    cv2.circle(mask, (u, v), r, value, -1)
    return mask


def gap(mask, v0, v1):
    mask[v0:v1, :] = 0
    return mask


CASES = [
    # 名称,                     e0,     theta0,   变形,                     期望 ok
    ('居中直线',                 0.00,   0.00,     None,                     True),
    ('左偏 5 cm',                0.05,   0.00,     None,                     True),
    ('右偏 3 cm',               -0.03,   0.00,     None,                     True),
    ('朝左拐 10°',               0.00,   np.deg2rad(10), None,               True),
    ('左偏 2 cm + 右偏 8°',      0.02,  -np.deg2rad(8),  None,                True),
    ('带干扰块',                 0.05,   0.00,     lambda m: blob(m, 70, 150), True),
    ('线中间断开',               0.05,   0.00,     lambda m: gap(m, 96, 118),  True),
    ('线已出画面',               0.30,   0.00,     None,                     False),
]


def expect_e(e0: float, th0: float, x_ref: float) -> float:
    """真值直线 y = e0 + tan(th0)·x 在参考平面 x_ref 上的横向偏移。

    e 的定义就是"x_ref 平面上的偏移"（见 line_features.extract 的注释），
    所以断言必须按同一个定义比，不能拿 x=0 的截距去比 —— 那是外推，会放大斜率误差。
    """
    return e0 + math.tan(th0) * x_ref


def main():
    print(f'相机模型: fx=fy={GEOM.fx:.1f}  cx={GEOM.cx}  cy={GEOM.cy}  '
          f'安装 x={GEOM.cam_x} h={GEOM.cam_h} pitch={GEOM.pitch} rad\n')
    print('| 用例 | e 期望 | e 实测 | θ 期望 | θ 实测 | 有效行 | 判定 |')
    print('|---|---|---|---|---|---|---|')
    fails = 0
    for name, e0, th0, mutate, want_ok in CASES:
        m = draw_line((H, W), GEOM, e0, th0)
        if mutate:
            m = mutate(m)
        lf = extract(m, GEOM)
        ok_run = lf.ok == want_ok
        if want_ok and lf.ok:
            ee = expect_e(e0, th0, lf.x_ref)
            de, dth = abs(lf.e - ee), abs(lf.theta - th0)
            ok_val = de <= TOL_E and dth <= TOL_TH
        else:
            ee, de, dth = e0, float('nan'), float('nan')
            ok_val = True                          # 期望失败且确实失败了
        good = ok_run and ok_val
        fails += 0 if good else 1
        ev = f'{lf.e:+.3f}' if lf.ok else '—'
        tv = f'{lf.theta:+.3f}' if lf.ok else '—'
        print(f'| {name} | {ee:+.3f} | {ev} | {th0:+.3f} | {tv} | {lf.n_rows} '
              f'| {"PASS" if good else "FAIL"} |')
        if not good:
            print(f'|   └ 原因: {lf.reason}  |e-e期|={de:.4f} |θ-θ0|={dth:.4f} '
                  f'x_ref={lf.x_ref:.2f} |')

    # 额外自检：投影 <-> 逆投影 必须闭环，否则上面所有断言都无意义
    worst = 0.0
    for x in (0.10, 0.30, 0.55, 0.90):
        for y in (-0.15, 0.0, 0.12):
            px = GEOM.ground_to_pixel(x, y)
            if px is None:
                continue
            back = GEOM.pixel_to_ground(*px)
            worst = max(worst, abs(back[0] - x), abs(back[1] - y))
    print(f'\n投影闭环最大残差: {worst * 1000:.4f} mm（应 <1 mm，否则 CamGeom 的'
          f'正逆变换写反了）')
    if worst > 0.001:
        fails += 1

    # ---- 端到端：走完整链路（BGR 图 -> make_mask -> extract）----
    # 上面几例喂的是"理想 mask"，绕过了 D8 的高斯+开运算。这一组把两种 mask 都跑一遍：
    # 结论是**开运算不能喂给几何提取**（会把远处 3~4 px 的细线咬断成残桩，中点偏），
    # 所以生产路径用 FEATURE，BASELINE 只用于出图与质量指标。
    print('\n端到端（BGR → make_mask → extract），对比开运算有/无：')
    print('| 用例 | mask | e 期望 | e 实测 | θ 期望 | θ 实测 | 判定 |')
    print('|---|---|---|---|---|---|---|')
    for name, e0, th0 in (('正对', 0.0, 0.0), ('左偏5cm+右偏10°', 0.05, np.deg2rad(-10)),
                          ('左偏5cm+左拐12°', 0.05, np.deg2rad(12))):
        floor = np.full((H, W, 3), 179, np.uint8)
        ideal = draw_line((H, W), GEOM, e0, th0)
        img = floor.copy()
        img[ideal > 0] = (26, 26, 26)                       # 黑线
        blob(img, 70, 150, 26, value=(44, 15, 15))          # 暗红污渍（灰度会被骗）
        for tag, cfg in (('hsv+g+open', BASELINE), ('hsv+g 特征用', FEATURE)):
            mask = make_mask(img, **cfg)
            lf = extract(mask, GEOM)
            ee = expect_e(e0, th0, lf.x_ref) if lf.ok else e0
            de = abs(lf.e - ee) if lf.ok else float('inf')
            dth = abs(lf.theta - th0) if lf.ok else float('inf')
            verdict = '—（仅展示）' if tag.startswith('hsv+g+open') else (
                'PASS' if de <= TOL_E and dth <= TOL_TH else 'FAIL')
            if verdict == 'FAIL':
                fails += 1
            print(f'| {name} | {tag} | {ee:+.3f} | {lf.e:+.3f} | {th0:+.3f} | '
                  f'{lf.theta:+.3f} | {verdict} |')
    total = len(CASES) + 1 + 3
    print(f'\n结果：{total - fails} / {total} 通过（开运算那三行只作对照，不计分）')
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
