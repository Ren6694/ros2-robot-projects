# -*- coding: utf-8 -*-
"""D8 的阈值管线与 mask 质量指标（纯算法，不依赖 ROS，便于单测与复用）。

`scripts/line_mask.py` 是它的命令行外壳；D9 的 `line_features.extract()` 吃这里的输出。
选定基线（见 D8 对照实验）：**method=hsv, blur=5, open=3**。
"""
from __future__ import annotations

import cv2
import numpy as np

# 四组对照配置：(名字, 参数)。前两组只用灰度，后两组用 HSV，各自带/不带平滑
COMPARE_CONFIGS = [
    ('gray',        dict(method='gray', gray=80,  blur=0, opensz=0)),
    ('gray+gauss',  dict(method='gray', gray=80,  blur=5, opensz=0)),
    ('hsv',         dict(method='hsv',  v_hi=70,  blur=0, opensz=0)),
    ('hsv+g+open',  dict(method='hsv',  v_hi=70,  blur=5, opensz=3)),
]

# D8 实验选定的基线：给人看、算质量指标用
BASELINE = dict(method='hsv', v_hi=70, blur=5, opensz=3)

# D9 特征提取专用：不做开运算。
# 说明（避免以讹传讹）：我一开始以为"θ 偏 7°"是开运算把远处 3~4 px 的细线咬断造成的，
# 单变量对照后发现**两种 mask 结果完全一样**，真凶是干扰物与线粘连（已由
# line_features.gate_by_continuity 的逐行连续性门控解决）。
# 保留这个不带开运算的档位，只是因为远处细线被腐蚀掉对提取没有任何好处，没必要冒这个风险；
# 出图与质量指标继续用 BASELINE（开运算让 mask 更好看、指标更干净）。
FEATURE = dict(method='hsv', v_hi=70, blur=5, opensz=0)


def make_mask(img_bgr, method='hsv', gray=80, v_hi=70, s_lo=0, s_hi=255,
              blur=0, opensz=0):
    """BGR 图 -> 二值 mask（线=255）。每一步都可单独关，方便对照实验。"""
    if method == 'gray':
        m = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        m = cv2.GaussianBlur(m, (blur, blur), 0) if blur > 1 else m
        m = cv2.threshold(m, gray, 255, cv2.THRESH_BINARY_INV)[1]
    else:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # 黑线特征：V 低。灰地面：V 高、S≈0。主判据是 V，S 范围留给彩色干扰
        lo = np.array([0, s_lo, 0], dtype=np.uint8)
        hi = np.array([179, s_hi, v_hi], dtype=np.uint8)
        m = cv2.inRange(hsv, lo, hi)
        if blur > 1:
            m = cv2.GaussianBlur(m, (blur, blur), 0)
            m = cv2.threshold(m, 127, 255, cv2.THRESH_BINARY)[1]
    if opensz > 1:                      # 开运算：先腐蚀后膨胀，吃掉孤立噪点与色块边缘
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (opensz, opensz))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return m


def measure(mask):
    """给一张 mask 打分 -> (覆盖率%, 连通域数, 最大域占比%, 底部中心偏移)。

    只看覆盖率会被误检骗到；**连通域=1 且最大域占比>95%** 才算"能喂给下游"。
    """
    total = mask.size
    white = int(np.count_nonzero(mask))
    if white == 0:
        return 0.0, 0, 0.0, float('nan')
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    areas = stats[1:, cv2.CC_STAT_AREA]          # 0 号是背景
    h, w = mask.shape
    band = mask[int(h * 0.8):, :]                # 最下方 1/5 行 = 离车最近、最可信
    cols = np.flatnonzero(band.any(axis=0))
    offset = float('nan')
    if cols.size:
        offset = (float(cols.min() + cols.max()) / 2.0 - w / 2.0) / (w / 2.0)
    return (100.0 * white / total, int(n - 1),
            100.0 * float(areas.max()) / white, offset)


def overlay(img_bgr, mask):
    """把 mask 涂成半透明绿叠回原图，肉眼检查"框住的是不是线"。"""
    out = img_bgr.copy()
    green = np.zeros_like(out)
    green[:, :, 1] = 255
    out[mask > 0] = cv2.addWeighted(out[mask > 0], 0.45, green[mask > 0], 0.55, 0)
    return out
