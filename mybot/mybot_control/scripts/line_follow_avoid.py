#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D11：巡线 + 避障 + 丢线恢复的组合节点。

    /line/pose (PoseStamped) ┐
                             ├→ AvoidFSM.step() → /cmd_vel (Twist)
    /scan (LaserScan)        ┘                    → /avoid/state (String)

和 D10 的关系：**控制律完全没改**（还是 pd_line.compute），这一层只在它外面套了一个
"什么时候允许动、往哪儿动"的仲裁。状态逻辑全在 mybot_control/avoid.py（纯函数、可单测），
本文件只做 ROS 胶水。

三个和 ROS 时序有关的决定（不看代码容易踩）：

1. **激光比控制环慢 4 倍，必须处理"数据年龄"。**
   /scan 只有 5 Hz，控制环 20 Hz → 每个控制周期里读的激光平均已经 0.1 s 老。
   这本身没问题（阈值就是按 5 Hz 算的），但如果 /scan **彻底断了**，
   "前方无回波"和"激光挂了"在数据上长得一模一样。所以这里做**失效安全**处理：
   激光超过 --scan-timeout 没更新就当作"前方有障碍"停住，而不是当作"前方干净"继续跑。
   宁可假停，不可真撞。

2. **状态机用墙上时间，不用仿真时间。**
   与 D10 的 --lost-timeout 保持同一口径（都是墙上秒），否则 RTF≠1 时两处超时
   会互相矛盾。代价：慢仿真下 search_max 这类时长会按墙上时间走。
   实测本机空载 RTF=1.00，两者等价；有漂移时以 --clock 为准的读者需要知道这点。

3. **退出前必发一次零速。**
   Ctrl-C / 超时 / 跑完圈三条路径都要把 /cmd_vel 归零。
   cmd_vel_timeout 虽然会在 2 s 仿真时间后自动清，但那 2 s 足够车再往前拱 0.4 m。

用法：
  ros2 run mybot_control line_follow_avoid.py
  ros2 run mybot_control line_follow_avoid.py --stop-dist 0.35 --csv /tmp/d11.csv
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

if 'mybot_control' not in sys.modules:
    _here = os.path.dirname(os.path.abspath(__file__))
    for _c in (os.path.join(_here, '..'), _here):
        if os.path.isdir(os.path.join(_c, 'mybot_control')):
            sys.path.insert(0, _c)
            break
from mybot_control.avoid import (                          # noqa: E402
    AvoidFSM, AvoidParams, front_obstacle_distance)
from mybot_control.pd_line import PDGains                  # noqa: E402

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass


class LineFollowAvoid(Node):
    def __init__(self, args):
        super().__init__('line_follow_avoid')
        self.args = args
        self.gains = PDGains(kp=args.kp, kd=args.kd, v_max=args.v_max,
                             v_min=args.v_min, w_max=args.w_max, kv_speed=args.kv)
        p = AvoidParams(front_half_angle_deg=args.front_half_angle,
                        stop_dist_m=args.stop_dist,
                        warn_dist_m=args.warn_dist,
                        lost_timeout_s=args.lost_timeout,
                        search_w=args.search_w,
                        search_flip_s=args.search_flip,
                        search_max_s=args.search_max)
        self.fsm = AvoidFSM(p=p)

        self.e = float('nan')
        self.theta = float('nan')
        self.pose_seen = None           # None = 还没收到过线（宽限期，见 avoid.py）
        self.ranges = None
        self.angle_min = 0.0
        self.angle_incr = 0.0
        self.scan_seen = None
        self.vx_now = 0.0
        self.start_xy = None
        self.path = 0.0
        self.prev_xy = None
        self.rows = []
        self.state_counts = {}
        self.laps = 0
        self.t0 = time.time()
        self.done = False

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/avoid/state', 10)
        self.create_subscription(PoseStamped, '/line/pose', self._pose_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_timer(1.0 / args.rate, self._tick)

    def _pose_cb(self, msg):
        self.e = msg.pose.position.y
        q = msg.pose.orientation
        self.theta = 2.0 * math.atan2(q.z, q.w)
        self.pose_seen = time.time()

    def _scan_cb(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_incr = msg.angle_increment
        self.scan_seen = time.time()

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.vx_now = msg.twist.twist.linear.x
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
        if self.prev_xy is not None:
            self.path += math.hypot(p.x - self.prev_xy[0], p.y - self.prev_xy[1])
        self.prev_xy = (p.x, p.y)

    def _front_distance(self, now):
        """返回 (距离 or None, 激光是否可信)。

        失效安全：没收到过 /scan、或 /scan 超时 -> 视为"前方有障碍"(0.0)，
        让状态机走 OBSTACLE_STOP。绝不能用 None 表示"激光挂了"——
        None 在状态机里的语义是"前方干净"，那等于把传感器故障当成放行。
        """
        if self.ranges is None or self.scan_seen is None:
            return 0.0, False
        if now - self.scan_seen > self.args.scan_timeout:
            return 0.0, False
        return front_obstacle_distance(self.ranges, self.angle_min, self.angle_incr,
                                       self.fsm.p), True

    def _tick(self):
        if self.done or self.start_xy is None:
            return
        now = time.time()
        line_ok = (self.pose_seen is not None
                   and (now - self.pose_seen) <= self.args.lost_timeout)
        front, _lidar_ok = self._front_distance(now)
        d = self.fsm.step(now, line_ok, self.e, self.theta, front, self.gains)

        out = Twist()
        out.linear.x = d.cmd.v
        out.angular.z = d.cmd.w
        self.pub.publish(out)
        self.state_pub.publish(String(data=d.state))
        self.state_counts[d.state] = self.state_counts.get(d.state, 0) + 1
        self.rows.append((round(now - self.t0, 2), d.state,
                          self.e if line_ok else float('nan'),
                          self.theta if line_ok else float('nan'),
                          front if front is not None else float('nan'),
                          d.cmd.v, d.cmd.w, self.vx_now,
                          self.prev_xy[0], self.prev_xy[1], d.cmd.reason))

        dx = self.prev_xy[0] - self.start_xy[0]
        dy = self.prev_xy[1] - self.start_xy[1]
        if self.path > 6.0 and math.hypot(dx, dy) < 0.35:
            self.laps += 1
            if self.args.laps <= 1 or self.laps >= self.args.laps:
                self.done = True

    def summary(self):
        el = time.time() - self.t0
        # 行布局见 _tick 里的 append：(t, state, e, theta, front, v, w, vx, x, y, reason)
        # —— 索引必须和那里同步改。D11 比 D10 多插了一列 state，所以 e 是 r[2] 不是 r[1]，
        # 搞错就是 abs('FOLLOW') 这种崩法（第一次跑就栽在这，靠 CSV 才把验收数据捞回来）。
        errs = [abs(r[2]) for r in self.rows if r[2] == r[2]]
        dists = [r[4] for r in self.rows if r[4] == r[4] and r[4] > 0.05]
        s = (f'墙上 {el:.0f}s  里程 {self.path:.2f} m  圈数 {self.laps}\n'
             f'  状态帧数 ' + '  '.join(f'{k}={v}' for k, v in sorted(self.state_counts.items())))
        if errs:
            s += (f'\n  |e| 均值 {sum(errs) / len(errs) * 100:.1f} cm / '
                  f'最大 {max(errs) * 100:.1f} cm')
        if dists:
            s += f'\n  前向最近障碍 {min(dists):.3f} m（停住门限 {self.args.stop_dist} m）'
        s += f'\n  状态迁移 {self.fsm.n_transitions} 次  样本 {len(self.rows)} 帧'
        return s


def main():
    ap = argparse.ArgumentParser(description='D11 巡线+避障')
    ap.add_argument('--kp', type=float, default=1.5)
    ap.add_argument('--kd', type=float, default=0.8)
    ap.add_argument('--v-max', dest='v_max', type=float, default=0.30)
    ap.add_argument('--v-min', dest='v_min', type=float, default=0.08)
    ap.add_argument('--w-max', dest='w_max', type=float, default=1.5)
    ap.add_argument('--kv', type=float, default=0.7)
    ap.add_argument('--rate', type=float, default=20.0)
    # 阈值默认值全部来自 avoid.py 的实测推导；改之前先看那边的注释与 test_avoid.py 第 [5] 组
    ap.add_argument('--front-half-angle', dest='front_half_angle', type=float, default=25.0,
                    help='前向扇区半角；>=90 会吃到车体自我回波（实测最近 98°）')
    ap.add_argument('--stop-dist', dest='stop_dist', type=float, default=0.30)
    ap.add_argument('--warn-dist', dest='warn_dist', type=float, default=0.55)
    ap.add_argument('--lost-timeout', dest='lost_timeout', type=float, default=1.0)
    ap.add_argument('--scan-timeout', dest='scan_timeout', type=float, default=1.0,
                    help='超过这么久没有 /scan 就失效安全停车')
    ap.add_argument('--search-w', dest='search_w', type=float, default=0.35)
    ap.add_argument('--search-flip', dest='search_flip', type=float, default=1.5)
    ap.add_argument('--search-max', dest='search_max', type=float, default=6.0)
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--duration', type=float, default=300.0)
    ap.add_argument('--csv', default='/tmp/d11_avoid.csv')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = LineFollowAvoid(args)
    print(f'避障巡线：kp={args.kp} kd={args.kd} v_max={args.v_max} '
          f'stop={args.stop_dist} warn={args.warn_dist} 扇区±{args.front_half_angle}° '
          f'rate={args.rate}Hz  等 /odom…')
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
                print(f'  t={now:5.0f}s 里程={node.path:5.2f}m 状态={node.fsm.state:<13} '
                      f'e={e:+6.1f}cm v={node.vx_now:.2f}')
    except KeyboardInterrupt:
        print('\n[Ctrl-C] 停车后退出')
    node.pub.publish(Twist())          # 三条退出路径共用：归零再走
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()

    if args.csv:
        d = os.path.dirname(os.path.abspath(args.csv))
        os.makedirs(d, exist_ok=True)
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_wall', 'state', 'e_m', 'theta_rad', 'front_m',
                        'cmd_v', 'cmd_w', 'odom_vx', 'odom_x', 'odom_y', 'reason'])
            w.writerows(node.rows)
        print(f'CSV: {args.csv}（{len(node.rows)} 行）')
    print(node.summary())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
