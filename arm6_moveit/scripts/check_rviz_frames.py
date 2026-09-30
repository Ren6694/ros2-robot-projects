#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/check_rviz_frames.py —— 三张 rviz 截图的内容判据（可离线重跑）

原来是内联在 capture_rviz.sh 第 6 步的一段 python，判的是
"3D 视口里变化像素占**视口**像素的比例"：moving > 0.20%、final < 0.35%。
2026-09-30 把 Orbit 从 Distance 3.2 改到 1.62（外观改造后要看得清零件）之后，
同一套流程判成 final=0.69% > 0.35% FAIL —— 但手臂并没有不回位。

量出来的原因：那个比例的分母是**视口面积**，是个与取景无关的常数；
分子（变化像素）却随手臂在屏幕上多大而成比例增长。实测两批：

    取景          手臂扫过 bbox    mv 原始    fn 原始
    3.2 m (09-17)   20 016 px      0.58 %     0.13 %
    1.62 m(09-30)   78 364 px      1.99 %     0.69 %     ← 面积 3.9 倍，读数 3.4~5.3 倍

所以把分母换成**同一张图里手臂自己的投影面积**（取 idle↔moving 差异像素的包围盒，
不需要按颜色分割手臂）：

    取景          mv 归一化    fn 归一化
    3.2 m          24.9 %       5.6 %
    1.62 m         21.8 %       7.5 %      ← 两批落在同一量级，取景不再影响判据

新判据（两条都是无量纲比值，不再有任何"视口占比"常数）：
    [1] mv_norm > 12 %          手臂真的动了（相对自身大小）
    [2] fn_norm < 0.6 × mv_norm 回到了起点：残差必须明显小于运动幅度
两条都在归档数据上验过，含一条**真实红样**：

    批次                       mv_norm  fn_norm  0.6*mv   判定
    09-16 Loop Animation 鬼影   33.5 %   26.8 %   20.1 %   FAIL ← 该红
    09-17 修好后                24.9 %    5.6 %   14.9 %   PASS
    09-30 新取景                21.8 %    7.5 %   13.1 %   PASS

[2] 用"相对 mv"而不是绝对数，是因为鬼影那种失败的表现恰好是"final 与 idle 差得
和 moving 一样多"；早期阈值定在 0.8×mv 时，鬼影批算出来 26.8 vs 26.8 **正好压在边界**，
这种判据换个批次就会翻脸，所以收到 0.6。

  用法: python3 scripts/check_rviz_frames.py <idle.png> <moving.png> <final.png>
  结果行: FRAMES-RESULT PASS/FAIL mv_norm=… fn_norm=… arm_bbox=…
  退出码: 0 PASS / 1 FAIL / 2 读图失败
"""
import sys

VIEWPORT_X_FROM = 0.44   # 左侧三个面板固定占约 700/1600 px，只看 3D 视口
CHANGED_THRESHOLD = 25   # 任一通道差 >25 记为变化，滤掉抗锯齿与右下角 fps 计数
MV_MIN_PCT = 12.0        # 判据 [1]
FN_RATIO_MAX = 0.6       # 判据 [2]：fn_norm < 0.6 × mv_norm


def load(path):
    import numpy as np
    from matplotlib import image as mpimg
    a = np.asarray(mpimg.imread(path)[:, :, :3], dtype=float)
    if a.max() <= 1.0:
        # matplotlib 读 PNG 给的是 [0,1] 浮点，不乘回 255 会把正常图判成"全黑"（06 §3 踩过）
        a *= 255.0
    h, w, _ = a.shape
    return a[:, int(w * VIEWPORT_X_FROM):, :]


def changed_mask(x, y):
    import numpy as np
    return np.abs(x - y).max(axis=2) > CHANGED_THRESHOLD


def bbox_area(mask):
    ys, xs = mask.nonzero()
    if len(xs) == 0:
        return 1, 0, 0
    return (int(xs.max()) - int(xs.min()) + 1) * (int(ys.max()) - int(ys.min()) + 1), xs.max() - xs.min(), ys.max() - ys.min()


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    try:
        idle, moving, final = (load(p) for p in sys.argv[1:4])
    except Exception as e:                       # 读图失败 ≠ 判红，别让"缺文件"伪装成"内容不对"
        print("FRAMES-RESULT FAIL (读图失败: %s)" % e)
        return 2
    mv, fn = changed_mask(idle, moving), changed_mask(idle, final)
    area, w_px, h_px = bbox_area(mv)
    mv_raw, fn_raw = mv.mean() * 100.0, fn.mean() * 100.0
    mv_n = mv.sum() / float(area) * 100.0
    fn_n = fn.sum() / float(area) * 100.0
    c1 = mv_n > MV_MIN_PCT
    c2 = fn_n < FN_RATIO_MAX * mv_n
    print("DIFF idle<->moving changed=%.2f%% arm_norm=%.1f%%  %s" % (
        mv_raw, mv_n, "OK" if c1 else "FAIL(手臂没动?)"))
    print("DIFF idle<->final  changed=%.2f%% arm_norm=%.1f%%  %s (界 %.1f%% = %.1f x mv)" % (
        fn_raw, fn_n, "OK" if c2 else "FAIL(未归位/有多余机器人)", FN_RATIO_MAX * mv_n, FN_RATIO_MAX))
    print("ARM bbox=%d px (%dx%d)  视口占比=%.2f%%  ← 取景变了要看这个数，不是看 changed" % (
        area, w_px, h_px, area / float(mv.shape[0] * mv.shape[1]) * 100.0))
    ok = c1 and c2
    print("FRAMES-RESULT %s mv_norm=%.1f fn_norm=%.1f arm_bbox=%d" % (
        "PASS" if ok else "FAIL", mv_n, fn_n, area))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
