# -*- coding: utf-8 -*-
"""D10 巡线控制律：把 (e, θ) 变成 (v, ω)。纯函数，不依赖 ROS，便于单测与整定。

控制律
------
    ω = Kp·e + Kd·θ            （限幅 ±w_max）
    v = clamp(v_max·(1 - k_v·|ω|/w_max), v_min, v_max)

为什么是这两项
--------------
· `e`（米，线在车左为正）是**位置误差**，负责把车拽回线上方；
· `θ`（弧度，线朝左拐为正）是**航向误差**，负责提前打方向，是这台车能过 0.45 m 圆角的关键。
  只用 e 的纯 P 控制必然在圆角处滞后：车已经偏出去了才反应，冲出去 → 回摆 → 振荡。
  所以 θ 这一项在控制意义上就是"D"（对误差变化率的近似），不需要再单独求导。
· 速度随 |ω| 自动降：转向越急走得越慢，防止在圆角处因轮胎侧滑丢线；
  同时设下限 v_min，太慢会让 diff_drive 的轮速进入静摩擦区、车干脆不动。

符号约定（与 line_features 一致）：e>0 = 线在左 → ω>0 = 左转（REP-103 右手系）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PDGains:
    kp: float = 1.5
    kd: float = 0.8
    v_max: float = 0.25
    v_min: float = 0.08
    w_max: float = 1.5          # rad/s；diff_drive 的限位是 2.0，留余量
    kv_speed: float = 0.7       # |ω|/w_max 满舵时速度降到 (1-kv) 倍


@dataclass
class Cmd:
    v: float
    w: float
    reason: str = 'ok'          # ok / lost / saturated


def compute(e: float, theta: float, g: PDGains) -> Cmd:
    """(e, θ) -> (v, ω)。输入 NaN（没看到线）时返回零速，由节点的看门狗决定怎么办。"""
    if e != e or theta != theta:                      # NaN 检查
        return Cmd(0.0, 0.0, 'lost')
    w = g.kp * e + g.kd * theta
    saturated = abs(w) > g.w_max
    w = max(-g.w_max, min(g.w_max, w))
    v = g.v_max * (1.0 - g.kv_speed * min(1.0, abs(w) / g.w_max))
    v = max(g.v_min, min(g.v_max, v))
    return Cmd(v, w, 'saturated' if saturated else 'ok')
