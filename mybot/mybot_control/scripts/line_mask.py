#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D8：cv_bridge 桥接 + 灰度/高斯/HSV 阈值实验。

一句话：把 /camera/image_raw 变成一张"线是白、地是黑"的二值图，并用数字告诉你这张图好不好。

三种模式：
  stream   持续发布 /line/mask (mono8) 与 /line/debug (bgr8)，可顺带存图
             ros2 run mybot_control line_mask.py --save 3 --outdir /tmp/d8
  compare  抓同一帧，跑四组配置做对照，打印指标表并各存一张 mask（D8 的主验收）
             ros2 run mybot_control line_mask.py --mode compare --outdir /tmp/d8
  tune     OpenCV 滑条实时调 HSV/灰度阈值（GUI 可用时最直观）
             ros2 run mybot_control line_mask.py --mode tune

指标为什么是这几个（都直接决定 D9/D10 好不好做）：
  覆盖率        白像素占比。巡线直道上经验值 8%~20%，过大=阈值太松，过小=线丢了
  连通域数      白区被切成几块。理想 1；>3 说明有噪点或线被反光切断
  最大域占比    最大连通域 / 全部白像素。衡量"主线是否完整"，应 >95%
  底部中心偏移  画面最下方 1/5 行的白区中心，-1(最左) ~ +1(最右)。这就是 D10 的 e 信号
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# 四组对照配置：(名字, 参数)。前两组只用灰度，后两组用 HSV，各自带/不带平滑
COMPARE_CONFIGS = [
    ('gray',        dict(method='gray', gray=80,  blur=0, opensz=0)),
    ('gray+gauss',  dict(method='gray', gray=80,  blur=5, opensz=0)),
    ('hsv',         dict(method='hsv',  v_hi=70,  blur=0, opensz=0)),
    ('hsv+g+open',  dict(method='hsv',  v_hi=70,  blur=5, opensz=3)),
]


def make_mask(img_bgr, method='hsv', gray=80, v_hi=70, s_lo=0, s_hi=255,
              blur=0, opensz=0):
    """BGR 图 -> 二值 mask（线=255）。每一步都可单独关，方便对照实验。"""
    if method == 'gray':
        m = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        m = cv2.GaussianBlur(m, (blur, blur), 0) if blur > 1 else m
        m = cv2.threshold(m, gray, 255, cv2.THRESH_BINARY_INV)[1]
    else:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # 黑线特征：V 低。灰地面：V 高、S≈0。所以主判据是 V，S 范围留给彩色干扰
        lo = np.array([0, s_lo, 0], dtype=np.uint8)
        hi = np.array([179, s_hi, v_hi], dtype=np.uint8)
        m = cv2.inRange(hsv, lo, hi)
        if blur > 1:
            m = cv2.GaussianBlur(m, (blur, blur), 0)
            m = cv2.threshold(m, 127, 255, cv2.THRESH_BINARY)[1]
    if opensz > 1:                      # 开运算：先腐蚀后膨胀，吃掉孤立噪点
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (opensz, opensz))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return m


def measure(mask):
    """给一张 mask 打分。返回 (覆盖率%, 连通域数, 最大域占比%, 底部中心偏移)"""
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
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8',
                                                        header=msg.header))
        if self.args.debug:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(overlay(img, mask),
                                                             encoding='bgr8',
                                                             header=msg.header))
        if self.args.save > 0 and self.frames < self.args.save:
            self._dump(img, mask, tag=f'f{self.frames}')
            self.frames += 1
            if self.frames >= self.args.save:
                self.args.save = -1                  # 存够就停，节点继续跑

    def _dump(self, img, mask, tag, cfg_name=''):
        d = self.args.outdir
        os.makedirs(d, exist_ok=True)
        sfx = f'_{cfg_name}' if cfg_name else ''
        cv2.imwrite(os.path.join(d, f'{tag}{sfx}_orig.png'), img)
        cv2.imwrite(os.path.join(d, f'{tag}{sfx}_mask.png'), mask)
        cv2.imwrite(os.path.join(d, f'{tag}{sfx}_overlay.png'), overlay(img, mask))
        return os.path.join(d, f'{tag}{sfx}')


def wait_frame(node, timeout=40.0):
    t0 = time.time()
    while rclpy.ok() and node.latest is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        print(f'[FAIL] {timeout:.0f}s 内没收到 {node.args.topic}，仿真和相机插件起了吗？')
    return node.latest


def run_compare(node):
    """同一帧跑四组配置，打印对照表 + 各存一张 mask + 拼一张大图。"""
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
    grid = cv2.hconcat([cv2.hconcat([img] + tiles)])
    path = os.path.join(node.args.outdir, 'compare_grid.png')
    cv2.imwrite(path, grid)
    print(f'\n原图(最左) + 四组 mask 已拼成：{path}')
    return 0


def run_tune(node):
    """滑条实时调参。按 s 存当前帧，q 退出。"""
    cv2.namedWindow('tune', cv2.WINDOW_NORMAL)
    keys = {'hsv': 1, 'gray': 80, 'vhi': 70, 'blur': 0, 'open': 0}

    def bar(name, init, mx):
        cv2.createTrackbar(name, 'tune', init, mx, lambda _: None)
        return name

    names = [bar('hsv', 1, 1), bar('gray', keys['gray'], 200),
             bar('vhi', keys['vhi'], 200), bar('blur', 0, 15), bar('open', 0, 9)]
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
            node._dump(img, mask, tag='tune', cfg_name=f"{cfg['method']}{cfg['gray']}")
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
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
