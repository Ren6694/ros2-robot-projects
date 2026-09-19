#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D10：PD 巡线节点。订阅 /line/pose，按固定节拍发 /cmd_vel，跑完整条线。

    /line/pose (PoseStamped)  →  pd_line.compute(e, θ)  →  /cmd_vel (Twist)

三个不是可有可无的细节：
1. **控制环自己定时**（--rate，默认 20 Hz），不在图像回调里发指令。
   图像频率会随负载抖动（本机 headless ~15 Hz、开 GUI ~5 Hz），
   绑在回调上等于让控制周期跟着抖，PD 就没法整定。
2. **丢线看门狗**：超过 --lost-timeout 没收到新的 /line/pose 就发零速停车，
   而不是"沿用上一次的方向"。D9 的线特征节点在无效时**不发布**，就是为了配合这里。
3. **自动判定跑完一圈**：里程 >6 m 且回到起点 0.35 m 内 → 记一次 lap 并退出，
   这样 `--duration` 只是安全上限，不用猜时间。

用法：
  ros2 run mybot_control line_follow_pd.py                       # 一直跑，跑完一圈自动停
  ros2 run mybot_control line_follow_pd.py --kp 2.0 --kd 1.0 --csv /tmp/d10.csv
  ros2 run mybot_control line_follow_pd.py --duration 120        # 最多 120 秒墙上时间
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

if 'mybot_control' not in sys.modules:
    _here = os.path.dirname(os.path.abspath(__file__))
    for _c in (os.path.join(_here, '..'), _here):
        if os.path.isdir(os.path.join(_c, 'mybot_control')):
            sys.path.insert(0, _c)
            break
from mybot_control.pd_line import PDGains, compute      # noqa: E402

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass


class LineFollowPD(Node):
    def __init__(self, args):
        super().__init__('line_follow_pd')
        self.args = args
        self.gains = PDGains(kp=args.kp, kd=args.kd, v_max=args.v_max,
                             v_min=args.v_min, w_max=args.w_max, kv_speed=args.kv)
        self.e = float('nan')
        self.theta = float('nan')
        self.pose_seen = 0.0          # 最近一次收到 /line/pose 的墙上时间
        self.vx_now = 0.0
        self.start_xy = None
        self.path = 0.0
        self.prev_xy = None
        self.rows = []
        self.lost_frames = 0
        self.sat_frames = 0
        self.laps = 0
        self.t0 = time.time()
        self.done = False

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/line/pose', self._pose_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_timer(1.0 / args.rate, self._tick)

    def _pose_cb(self, msg):
        self.e = msg.pose.position.y
        q = msg.pose.orientation
        self.theta = 2.0 * math.atan2(q.z, q.w)
        self.pose_seen = time.time()

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.vx_now = msg.twist.twist.linear.x
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
        if self.prev_xy is not None:
            self.path += math.hypot(p.x - self.prev_xy[0], p.y - self.prev_xy[1])
        self.prev_xy = (p.x, p.y)

    def _tick(self):
        if self.done or self.start_xy is None:
            return
        fresh = (time.time() - self.pose_seen) <= self.args.lost_timeout
        if fresh:
            cmd = compute(self.e, self.theta, self.gains)
        else:
            cmd = compute(float('nan'), float('nan'), self.gains)   # 丢线 -> 零速
            self.lost_frames += 1
        if cmd.reason == 'saturated':
            self.sat_frames += 1
        out = Twist()
        out.linear.x = cmd.v
        out.angular.z = cmd.w
        self.pub.publish(out)
        self.rows.append((round(time.time() - self.t0, 2), self.e, self.theta,
                          cmd.v, cmd.w, self.vx_now, self.prev_xy[0], self.prev_xy[1],
                          cmd.reason))
        # 一圈判定：走够里程且回到起点附近
        dx = self.prev_xy[0] - self.start_xy[0]
        dy = self.prev_xy[1] - self.start_xy[1]
        if self.path > 6.0 and math.hypot(dx, dy) < 0.35:
            self.laps += 1
            if self.args.laps <= 1 or self.laps >= self.args.laps:
                self.done = True

    def summary(self):
        el = time.time() - self.t0
        errs = [abs(r[1]) for r in self.rows if r[1] == r[1]]
        if not errs:
            return '没有任何有效样本（一直没看到线？）'
        return (f'墙上 {el:.0f}s  里程 {self.path:.2f} m  圈数 {self.laps}\n'
                f'  |e| 均值 {sum(errs) / len(errs) * 100:.1f} cm / 最大 {max(errs) * 100:.1f} cm\n'
                f'  丢线帧 {self.lost_frames} / {len(self.rows)}   饱和帧 {self.sat_frames}\n'
                f'  样本 {len(self.rows)} 帧（控制 {self.args.rate} Hz）')


def main():
    ap = argparse.ArgumentParser(description='D10 PD 巡线')
    ap.add_argument('--kp', type=float, default=1.5)
    ap.add_argument('--kd', type=float, default=0.8)
    # 默认速度 0.30 m/s：实测 0.25 与 0.35 都能跑完整圈（0.35 反而跟得更紧，
    # |e| 均值 1.3 cm），但 0.50 在第一个圆角入口就丢线被看门狗停住。
    # 取 0.30 = 已验证上限的 0.85 倍，留一点余量。
    ap.add_argument('--v-max', dest='v_max', type=float, default=0.30)
    ap.add_argument('--v-min', dest='v_min', type=float, default=0.08)
    ap.add_argument('--w-max', dest='w_max', type=float, default=1.5)
    ap.add_argument('--kv', type=float, default=0.7, help='满舵时速度降到 (1-kv)·v_max')
    ap.add_argument('--rate', type=float, default=20.0, help='控制频率 Hz')
    ap.add_argument('--lost-timeout', type=float, default=1.0, help='多久没线特征就停车(秒)')
    ap.add_argument('--laps', type=int, default=1, help='跑够几圈退出')
    ap.add_argument('--duration', type=float, default=300.0, help='墙上时间安全上限(秒)')
    ap.add_argument('--csv', default='/tmp/d10_pd.csv')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = LineFollowPD(args)
    print(f'PD 巡线：kp={args.kp} kd={args.kd} v∈[{args.v_min},{args.v_max}] '
          f'w_max={args.w_max} rate={args.rate}Hz  等 /odom 建立起点…')
    t0 = time.time()
    last = 0.0
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.time() - t0
            if now > args.duration:
                print(f'[timeout] 到 {args.duration}s 上限，主动退出')
                break
            if not args.quiet and now - last >= 5:
                last = now
                e = node.e * 100 if node.e == node.e else float('nan')
                print(f'  t={now:5.0f}s 里程={node.path:5.2f}m e={e:+6.1f}cm '
                      f'v={node.vx_now:.2f} 丢线={node.lost_frames}')
    except KeyboardInterrupt:
        print('\n[Ctrl-C] 停车后退出')
        node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

    if args.csv:
        d = os.path.dirname(os.path.abspath(args.csv))
        os.makedirs(d, exist_ok=True)
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_wall', 'e_m', 'theta_rad', 'cmd_v', 'cmd_w', 'odom_vx',
                        'odom_x', 'odom_y', 'reason'])
            w.writerows(node.rows)
        print(f'CSV: {args.csv}（{len(node.rows)} 行）')
    print(node.summary())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
