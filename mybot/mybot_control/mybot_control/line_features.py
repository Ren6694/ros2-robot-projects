# -*- coding: utf-8 -*-
"""D9 线特征提取：mask -> 逐行线中心 -> 逆透视投影 -> 米制横向偏差 e 与夹角 θ。

为什么不用像素偏差
------------------
`snap_camera.py`（D7）里那个"偏差 -1~+1"是像素归一化的，它能看但不能用：
  · 同一个 30 px 偏差，在 0.15 m 处和在 0.9 m 处对应的实际横向距离差 6 倍；
  · 像素宽度随俯角、地面距离非线性变化，PD 增益就没法整定，也没法验证"算得对不对"。
所以这里把每行线中心**逆投影到地面坐标系**（单位：米），再对近地面一段做直线拟合：

      y = e + tan(θ) · x        （x 前向、y 左侧为正，车体系）

  e  = 拟合直线在 x=0 处的横向截距 —— 线在车左边为正 → cmd_angular.z 取正即可转过去
  θ  = 拟合直线的局部航向角 —— 线朝左拐为正
两个量都是**车体几何**决定的，和相机分辨率、俯角的具体数值解耦，D10 的 PD 可以直接用。

约定（务必和 mybot_core.urdf.xacro 一致）
----------------------------------------
相机在车体系位置 (cam_x, 0, cam_h)，绕 y 轴 **正角=低头**（URDF 右手系，+x 转向 −z）。
camera_link 用 x前/y左/z上；光学系用 REP-103 的 x右/y下/z前。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


# ---------------------------------------------------------------- 相机几何 --
@dataclass
class CamGeom:
    """针孔模型 + 相机在车体系里的安装位姿。"""
    fx: float = 233.9
    fy: float = 233.9
    cx: float = 159.5
    cy: float = 119.5
    cam_x: float = 0.21      # 相机在 base_link 前方的距离（D7 挂点）
    cam_h: float = 0.154     # 相机离地高度
    pitch: float = 0.6       # rad，正角低头

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov: float, **kw) -> "CamGeom":
        """Gazebo 的相机是方形像素：fx = (w/2)/tan(hfov/2)。"""
        f = (width / 2.0) / math.tan(hfov / 2.0)
        return cls(fx=f, fy=f, cx=(width - 1) / 2.0, cy=(height - 1) / 2.0, **kw)

    @classmethod
    def from_camera_info(cls, msg, **kw) -> "CamGeom":
        """从 /camera/camera_info 的 K 矩阵构造（节点里用这个，别硬编码内参）。"""
        k = list(msg.k)
        return cls(fx=k[0], fy=k[4], cx=k[2], cy=k[5], **kw)

    # 车体系 <- 相机系：先绕 y 转 pitch（正角低头），再平移到安装点
    def _rot(self) -> np.ndarray:
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    def pixel_to_ground(self, u: float, v: float):
        """像素 -> 地面点 (x 前向, y 左侧)，单位米。光线打不到地面返回 None。"""
        d_opt = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        # 光学(x右,y下,z前) -> camera_link(x前,y左,z上)
        d_cl = np.array([d_opt[2], -d_opt[0], -d_opt[1]])
        d_body = self._rot() @ d_cl
        if d_body[2] >= -1e-9:                     # 射线朝上，永远碰不到地面
            return None
        t = self.cam_h / (-d_body[2])
        return (self.cam_x + t * d_body[0], t * d_body[1])

    def ground_to_pixel(self, x: float, y: float):
        """地面点 -> 像素（单元测试用它造"解析真值"的图）。"""
        d_body = np.array([x - self.cam_x, y, -self.cam_h])
        d_cl = self._rot().T @ d_body              # 逆旋转
        if d_cl[0] <= 1e-9:                        # 在相机背后
            return None
        d_opt = np.array([-d_cl[1], -d_cl[2], d_cl[0]])
        return (self.fx * d_opt[0] / d_opt[2] + self.cx,
                self.fy * d_opt[1] / d_opt[2] + self.cy)


# ------------------------------------------------------------ mask -> 中心点 --
@dataclass
class LineFeatures:
    e: float = float('nan')          # 米，在 x_ref 平面上量的横向偏移，左为正
    theta: float = float('nan')      # rad，线朝左拐为正
    x_ref: float = float('nan')      # e 的参考平面（默认=最近可用行，不做外推）
    e_axis: float = float('nan')     # 外推到车轴 x=0 的值，仅参考：外推会放大斜率误差
    ok: bool = False
    reason: str = ''
    n_rows: int = 0                  # 候选行数（过完行级筛选）
    n_inl: int = 0                   # RANSAC 内点数，即真正参与拟合的行数
    pts: list = field(default_factory=list)      # [(x, y, u, v, width_px)]
    fit_span: tuple = ()             # 实际用到的 x 范围，方便判断"看到多近/多远"

    def e_at(self, x: float) -> float:
        """把偏差换算到任意前向平面 x（D10 想要"车轴处"或"预瞄点处"的偏差时用）。"""
        if not self.ok:
            return float('nan')
        return self.e + math.tan(self.theta) * (x - self.x_ref)


def row_centers(mask: np.ndarray, min_run: int = 2, second_ratio: float = 0.35,
                v_from: int = 0, v_to: int | None = None, drop_border: bool = True):
    """逐行取**最长连续白段**的中心。

    两条硬规矩，都是实测换来的：
    1) 一行里出现"第二大段且长度超过最长段的 second_ratio"时，这行直接丢弃 ——
       这就是 D8 那块干扰色块不会污染 e 的原因（取中位数/均值的做法会被它拽偏）。
    2) **触到画面左右边缘的行也丢弃**（drop_border）：线朝一侧拐到冲出画面时，
       被裁断的那条弦的中点已经不是线中心了。实测这会让 e 少 39 mm、θ 多 0.065 rad，
       而且只在"线往左拐"时出现（左边界裁切），是个方向相关的系统偏差 —— 比纯噪声毒得多。
    """
    h, w = mask.shape[0], mask.shape[1]
    v_to = h if v_to is None else min(v_to, h)
    out = []
    for v in range(v_from, v_to):
        cols = np.flatnonzero(mask[v] > 0)
        if cols.size < min_run:
            continue
        # 按连续性切段
        splits = np.flatnonzero(np.diff(cols) > 1)
        runs = np.split(cols, splits + 1)
        runs.sort(key=lambda r: r.size, reverse=True)
        top = runs[0]
        if drop_border and (top.min() <= 0 or top.max() >= w - 1):
            continue                                # 被画面边缘裁过 -> 中点不可信
        if len(runs) > 1 and runs[1].size >= second_ratio * top.size:
            continue                                # 有势均力敌的第二段 -> 不可信
        out.append((v, float(top.min() + top.max()) / 2.0, int(top.size)))
    return out


def fit_line_ransac(xs, ys, tol: float = 0.008, iters: int = 300, seed: int = 0):
    """2 点 RANSAC 拟合 y = b·x + a，返回 (b, a, 内点数, 内点掩码)。

    为什么不用顺序跟踪式门控：实测过一版"从最近行往上、预测下一行中心"的门控，
    结果**一旦被污染点骗过一次，模型就歪了，后面把好的行全拒掉**，基线缩短反而更差
    （θ 误差从 0.006 涨到 0.041）。干扰物与线粘连属于"少数样本错得离谱"，
    这正是 RANSAC 的适用场景，而顺序预测不是。
    固定 seed 保证单元测试可复现。
    """
    rng = np.random.default_rng(seed)
    n = len(xs)
    best = (-1.0, 0.0, -1)
    if n < 2:
        return 0.0, float('nan'), 0, np.zeros(n, bool)
    for _ in range(iters):
        i, j = rng.integers(0, n, size=2)
        if i == j:
            continue
        x1, y1 = xs[i], ys[i]
        x2, y2 = xs[j], ys[j]
        if abs(x2 - x1) < 1e-6:
            continue
        b = (y2 - y1) / (x2 - x1)
        a = y1 - b * x1
        res = np.abs(ys - (b * xs + a))
        inl = res <= tol
        cnt = int(inl.sum())
        if cnt > best[2]:
            best = (b, a, cnt, inl)
    b, a, cnt, inl = best
    if cnt >= 3:                                   # 用内点做最小二乘精修
        bb, aa = np.polyfit(xs[inl], ys[inl], 1)
        res = np.abs(ys - (bb * xs + aa))
        inl = res <= tol
        return float(bb), float(aa), int(inl.sum()), inl
    return float(b), float(a), max(cnt, 0), inl


def extract(mask: np.ndarray, geom: CamGeom, x_lo: float = 0.05, x_hi: float = 0.60,
            min_rows: int = 6, min_run: int = 2, min_width: int = 4,
            x_ref: float | None = None, ransac: bool = True,
            ransac_tol: float = 0.008) -> LineFeatures:
    """mask -> LineFeatures。

    四个实测换来的取舍：
    · **x_hi=0.60**：远处那截被透视压成 1~2 px，横向分辨力从近处 ~1 mm/px 恶化到 ~4 mm/px。
    · **min_width=4**：再把"线宽太窄的行"挡掉，远处抖动大的样本不进拟合。
    · **ransac**：抗"干扰物与线粘连"。这种行的中点被整段拽偏，`row_centers` 的
      歧义行规则看不见（只剩一段了）。实测 12° 夹角 + 污渍：普通最小二乘 θ=0.338
      （真值 0.209），RANSAC 回到 0.21x。
    · **e 定义在参考平面 x_ref 上，而不是外推到车轴 x=0**：画面最近只能看到 x≈0.29 m，
      外推到 0 等于把斜率误差乘 0.3~0.5 再灌进 e —— 实测 12° 夹角时这一项贡献 68 mm
      的"假偏差"。要车轴处的值用 `e_at(0.0)` 换算，但 PD 反馈建议直接用 x_ref 平面的 e。
    """
    lf = LineFeatures()
    rows = [r for r in row_centers(mask, min_run=min_run) if r[2] >= min_width]
    pts = []
    for v, u, w in rows:
        g = geom.pixel_to_ground(u, v)
        if g is None:
            continue
        x, y = g
        if x_lo <= x <= x_hi:                      # 只留近场可用段
            pts.append((x, y, u, v, w))
    if len(pts) < min_rows:
        lf.reason = f'有效行太少（{len(pts)} < {min_rows}），线可能丢出画面'
        lf.n_rows = len(pts)
        return lf
    pts.sort()
    lf.pts = pts
    lf.n_rows = len(pts)
    xs = np.array([p[0] for p in pts])
    ys = np.array([p[1] for p in pts])
    if ransac:
        b, a, n_inl, _ = fit_line_ransac(xs, ys, tol=ransac_tol)
        lf.n_inl = n_inl
        if n_inl < min_rows:
            lf.reason = f'内点太少（{n_inl} < {min_rows}），线可能被大面积污染'
            return lf
    else:
        b, a = np.polyfit(xs, ys, 1)               # y = b*x + a
        lf.n_inl = len(pts)
    lf.theta = float(math.atan(b))
    lf.fit_span = (float(xs.min()), float(xs.max()))
    lf.x_ref = float(max(x_lo, lf.fit_span[0]) if x_ref is None else x_ref)
    lf.e = float(a + b * lf.x_ref)                 # 参考平面上的横向偏移（不外推）
    lf.e_axis = float(a)                           # 车轴处的值，仅参考
    lf.ok = True
    lf.reason = 'ok'
    return lf


def draw_line(mask_shape, geom: CamGeom, e: float, theta: float,
              width: float = 0.05, x_from: float = 0.02, x_to: float = 1.6,
              value: int = 255) -> np.ndarray:
    """按解析真值把一条地面直线画进 mask（单元测试造图用）。

    做法：沿 x 采样 -> 取线的左右两条边投影成像素 -> 拼成多边形 fillPoly。
    比逐行涂像素可靠：不会因为取整在行间留缝（有缝就会让"最长连续段"逻辑丢行）。
    """
    m = np.zeros(mask_shape, np.uint8)
    t = math.tan(theta)
    left, right = [], []
    for x in np.arange(x_from, x_to, 0.01):
        y = e + t * x
        pl = geom.ground_to_pixel(x, y + width / 2.0)
        pr = geom.ground_to_pixel(x, y - width / 2.0)
        if pl is None or pr is None:
            continue
        for acc, p in ((left, pl), (right, pr)):
            acc.append((int(round(p[0])), int(round(p[1]))))
    if len(left) >= 2 and len(right) >= 2:
        # fillPoly 会把画面外的部分自动裁掉，所以这里不用手工 clip
        cv2.fillPoly(m, [np.array(left + right[::-1], np.int32)], value)
    return m
