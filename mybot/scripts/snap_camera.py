#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D7：抓一帧相机图像存盘，并打印"黑线是否真的在画面里"的量化统计。

为什么不用 rqt_image_view 看一眼就算完：
  GUI 只能"看着像对"，写进 README 需要数字。本脚本顺带做了 D8 的前置动作——
  灰度化 + 固定阈值二值化 + 逐行取线中心，所以它既是 D7 的验收证据，
  也是 D9「线中心提取」的最小原型。

用法：
  ros2 run mybot_description snap_camera.py --out /tmp/d7_straight.png
  ros2 run mybot_description snap_camera.py --topic /camera/image_raw --thresh 80 --rows 6
"""
import argparse
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class OneShot(Node):
    """只取第一帧就收工，避免为了拍照把节点常驻。"""

    def __init__(self, topic: str):
        super().__init__('snap_camera')
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg: Image):
        if self.frame is None:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def analyse(img, thresh: int, n_rows: int):
    """返回 (mask, 统计文本)。灰度 < thresh 视为黑线。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = (gray < thresh).astype(np.uint8) * 255
    h, w = gray.shape
    lines = [f'分辨率      : {w}x{h}',
             f'灰度均值    : {gray.mean():.1f}（地面 0.7*255≈179 / 黑线 0.1*255≈26）',
             f'黑像素占比  : {100.0 * np.count_nonzero(mask) / mask.size:.1f}%  (阈值 gray<{thresh})',
             '逐行线中心  :']
    for i in range(n_rows):
        row = int(h - 1 - i * (h // (n_rows + 1)))     # 从画面底部往上取样，底部离车最近
        cols = np.flatnonzero(mask[row] > 0)
        if cols.size:
            # 一段连续黑色区间的中心；断开时取整段中位数，D9 再细化
            center = int(np.median(cols))
            lines.append(f'  y={row:3d}: 宽 {cols.size:3d}px 中心 x={center:3d} '
                         f'偏差 {(center - w / 2) / (w / 2):+.2f}')
        else:
            lines.append(f'  y={row:3d}: 未见黑线')
    return mask, '\n'.join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description='抓一帧相机图像并统计黑线')
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--out', default='/tmp/camera_snap.png')
    ap.add_argument('--thresh', type=int, default=80)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

    rclpy.init()
    node = OneShot(args.topic)
    spin_timeout = 0.0
    while rclpy.ok() and node.frame is None and spin_timeout < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        spin_timeout += 0.1
    if node.frame is None:
        print(f'[FAIL] {args.timeout:.0f}s 内没收到 {args.topic}，检查 gzserver 与相机插件')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    mask, report = analyse(node.frame, args.thresh, args.rows)
    cv2.imwrite(args.out, node.frame)
    side = cv2.hconcat([node.frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
    stem, dot, ext = args.out.rpartition('.')
    cv2.imwrite(f'{stem or args.out}_side{dot}{ext or "png"}', side)

    print(report)
    print(f'原图/二值图 : {args.out}  |  {stem}_side{dot}{ext}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
