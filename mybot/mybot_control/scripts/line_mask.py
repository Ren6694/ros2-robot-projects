#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D8：cv_bridge 桥接 + 灰度/高斯/HSV 阈值实验（命令行外壳，算法在 mybot_control.masking）。

三种模式：
  stream   持续发布 /line/mask (mono8) 与 /line/debug (bgr8)，可顺带存图
             ros2 run mybot_control line_mask.py --save 3 --outdir /tmp/d8
  compare  抓同一帧跑四组配置对照，打印指标表并各存一张 mask（D8 的主验收）
             ros2 run mybot_control line_mask.py --mode compare --outdir /tmp/d8
  tune     OpenCV 滑条实时调参（本机 WSLg 不显示该窗口，见 README 已知限制）
"""
import argparse
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

if 'mybot_control' not in sys.modules:                 # 支持从源码直接 python3 跑
    _here = os.path.dirname(os.path.abspath(__file__))
    for _c in (os.path.join(_here, '..'), _here):
        if os.path.isdir(os.path.join(_c, 'mybot_control')):
            sys.path.insert(0, _c)
            break
from mybot_control.masking import (COMPARE_CONFIGS, make_mask, measure,  # noqa: E402
                                   overlay)


class LineMask(Node):
    def __init__(self, args, cfg):
        super().__init__('line_mask')
        self.args, self.cfg = args, cfg
        self.bridge = CvBridge()
        self.latest = None
        self.frames = 0
        self.pub_mask = self.create_publisher(Image, '/line/mask', 10)
        self.pub_debug = self.create_publisher(Image, '/line/debug', 10)
        self.create_subscription(Image, args.topic, self._cb, 10)

    def _cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest = (img, msg.header)
        if self.args.mode != 'stream':
            return                                   # compare/tune 自己取帧，不用一直算
        mask = make_mask(img, **self.cfg)
        # header 必须透传：下游拿 mask 和 /odom、/tf 对时间全靠它
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8',
                                                        header=msg.header))
        if self.args.debug:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(overlay(img, mask),
                                                             encoding='bgr8',
                                                             header=msg.header))
        if self.args.save and self.frames < self.args.save:
            self.dump(img, mask, tag=f'f{self.frames}')
            self.frames += 1

    def dump(self, img, mask, tag, cfg_name=''):
        d = self.args.outdir
        os.makedirs(d, exist_ok=True)
        sfx = f'_{cfg_name}' if cfg_name else ''
        cv2.imwrite(os.path.join(d, f'{tag}{sfx}_orig.png'), img)
        cv2.imwrite(os.path.join(d, f'{tag}{sfx}_mask.png'), mask)
        cv2.imwrite(os.path.join(d, f'{tag}{sfx}_overlay.png'), overlay(img, mask))


def wait_frame(node, timeout=40.0):
    t0 = time.time()
    while rclpy.ok() and node.latest is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        print(f'[FAIL] {timeout:.0f}s 内没收到 {node.args.topic}，仿真和相机插件起了吗？')
    return node.latest


def run_compare(node):
    """同一帧跑四组配置：这是 D8 的验收证据，也是 D11/D12 改赛道后的回归入口。"""
    got = wait_frame(node)
    if got is None:
        return 1
    img, _header = got
    os.makedirs(node.args.outdir, exist_ok=True)
    tiles = []
    print('\n| 配置 | 覆盖率% | 连通域 | 最大域占比% | 底部中心偏移 |')
    print('|---|---|---|---|---|')
    for name, cfg in COMPARE_CONFIGS:
        mask = make_mask(img, **cfg)
        cov, ncomp, big, off = measure(mask)
        print(f'| {name} | {cov:.1f} | {ncomp} | {big:.1f} | {off:+.2f} |')
        cv2.imwrite(os.path.join(node.args.outdir, f'compare_{name}_mask.png'), mask)
        tile = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(tile, name, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        tiles.append(tile)
    path = os.path.join(node.args.outdir, 'compare_grid.png')
    cv2.imwrite(path, cv2.hconcat([img] + tiles))
    print(f'\n原图(最左) + 四组 mask 已拼成：{path}')
    return 0


def run_tune(node):
    """滑条实时调参。按 s 存当前帧，q 退出。"""
    cv2.namedWindow('tune', cv2.WINDOW_NORMAL)

    def bar(name, init, mx):
        cv2.createTrackbar(name, 'tune', init, mx, lambda _: None)
        return name

    names = [bar('hsv', 1, 1), bar('gray', 80, 200), bar('vhi', 70, 200),
             bar('blur', 5, 15), bar('open', 3, 9)]
    print('滑条：hsv 0=灰度 1=HSV；gray/vhi 阈值；blur/open 核大小（0=关）。s 存图，q 退出')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest is None:
            continue
        img, _ = node.latest
        v = {n: cv2.getTrackbarPos(n, 'tune') for n in names}
        cfg = dict(method='hsv' if v['hsv'] else 'gray', gray=max(1, v['gray']),
                   v_hi=max(1, v['vhi']),
                   blur=v['blur'] | 1 if v['blur'] > 1 else 0,
                   opensz=v['open'] | 1 if v['open'] > 1 else 0)
        mask = make_mask(img, **cfg)
        cv2.imshow('tune', cv2.hconcat([img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                                        overlay(img, mask)]))
        key = cv2.waitKey(30) & 0xFF
        if key == ord('s'):
            node.dump(img, mask, tag='tune', cfg_name=f"{cfg['method']}{cfg['gray']}")
            print('已存 tune_*.png 到', node.args.outdir)
        elif key == ord('q'):
            break
    cv2.destroyAllWindows()
    return 0


def main():
    ap = argparse.ArgumentParser(description='D8 cv_bridge 阈值实验')
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--mode', choices=['stream', 'compare', 'tune'], default='stream')
    ap.add_argument('--method', choices=['gray', 'hsv'], default='hsv')
    ap.add_argument('--gray', type=int, default=80, help='灰度阈值（小于它是线）')
    ap.add_argument('--v-hi', type=int, default=70, help='HSV 的 V 上限')
    ap.add_argument('--blur', type=int, default=5, help='高斯核，奇数；0=关')
    ap.add_argument('--open', dest='opensz', type=int, default=3, help='开运算核；0=关')
    ap.add_argument('--save', type=int, default=0, help='存几帧后停止存图（stream 模式）')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='stream 模式跑多少墙上秒后退出；0=一直跑')
    ap.add_argument('--outdir', default='/tmp/mybot_masks')
    ap.add_argument('--no-debug', dest='debug', action='store_false', default=True)
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

    cfg = dict(method=args.method, gray=args.gray, v_hi=args.v_hi,
               blur=args.blur, opensz=args.opensz)
    rclpy.init()
    node = LineMask(args, cfg)
    if args.mode == 'compare':
        rc = run_compare(node)
    elif args.mode == 'tune':
        rc = run_tune(node)
    else:
        print(f'stream：{args.topic} -> /line/mask + /line/debug，配置 {cfg}')
        print(f'存图目录：{args.outdir}（--save N 存 N 帧；--duration 秒 到点退出）')
        rc = 0
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0 and time.time() - t0 >= args.duration:
                break
            if args.save and node.frames >= args.save:
                node.args.save = 0                   # 存够就停存图，继续发布
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
