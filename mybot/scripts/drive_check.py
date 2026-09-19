#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D7：赛道"能跑"验收脚本 —— 按给定速度开一段仿真时间，记录 /odom 轨迹并打表。

为什么不用 teleop 键盘手动开：
  1) 手动开没有可复现的数字，写不进 README；
  2) diff_drive_controller 的 cmd_vel_timeout=0.5 s（D4 结论）正好在这里当安全检查：
     本脚本退出后 0.5 s 内车必须自己停住，表格里最后几行的 x 不再增长就是证据。

用法：
  ros2 run mybot_description drive_check.py --speed 0.2 --turn 0.0 --duration 8
  ros2 run mybot_description drive_check.py --speed 0.1 --turn 0.5 --duration 6   # 原地+前进模拟过弯
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    """四元数 -> 偏航角（只用 z/w，平面运动够用）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveCheck(Node):
    def __init__(self, speed, turn, rate):
        super().__init__('drive_check')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.last = None
        self.path_len = 0.0
        self.create_subscription(Odometry, '/odom', self._cb, 10)
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn
        self.timer = self.create_timer(1.0 / rate, lambda: self.pub.publish(msg))

    def _cb(self, msg):
        p = msg.pose.pose.position
        if self.last is not None:
            self.path_len += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.last = (p.x, p.y)
        self.odom = msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2, help='线速度 m/s')
    ap.add_argument('--turn', type=float, default=0.0, help='角速度 rad/s')
    ap.add_argument('--duration', type=float, default=8.0, help='仿真时间秒')
    ap.add_argument('--rate', type=float, default=10.0, help='cmd_vel 发布频率 Hz')
    ap.add_argument('--wall-cap', type=float, default=600.0, help='墙上时间上限，防卡死')
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

    rclpy.init()
    node = DriveCheck(args.speed, args.turn, args.rate)
    t0_wall = time.time()
    t0_sim = None
    step = max(1.0, args.duration / 8.0)
    next_log = 0.0
    print('  t_sim     x       y      yaw    v_x     v_z')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None:
            if time.time() - t0_wall > 30:
                print('[FAIL] 30 s 没收到 /odom，检查控制器是否 active')
                break
            continue
        now_sim = node.odom.header.stamp.sec + node.odom.header.stamp.nanosec * 1e-9
        if t0_sim is None:
            t0_sim = now_sim
        el = now_sim - t0_sim
        if el >= next_log:
            p = node.odom.pose.pose.position
            v = node.odom.twist.twist
            print(f'{el:6.1f} {p.x:8.3f} {p.y:8.3f} {yaw_of(node.odom.pose.pose.orientation):7.3f} '
                  f'{v.linear.x:7.3f} {v.angular.z:7.3f}')
            next_log += step
        if el >= args.duration or time.time() - t0_wall > args.wall_cap:
            break

    # 停发指令，等 cmd_vel_timeout 真正触发（v_x 归零）之后再开始量滑移，
    # 否则量到的是"超时窗口内的正常续行"，不是滑移。
    node.destroy_timer(node.timer)
    wait0 = time.time()
    while rclpy.ok() and time.time() - wait0 < 40:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom and abs(node.odom.twist.twist.linear.x) < 1e-4:
            break
    stop_x = node.odom.pose.pose.position.x if node.odom else 0.0
    settle0 = time.time()
    drift = 0.0
    while rclpy.ok() and time.time() - settle0 < 12:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom:
            drift = max(drift, abs(node.odom.pose.pose.position.x - stop_x))
    p = node.odom.pose.pose.position
    print(f'\n仿真时长  : {el:.1f} s（墙上 {time.time() - t0_wall:.0f} s，RTF≈{el / (time.time() - t0_wall):.2f}）')
    print(f'末位姿    : x={p.x:.3f} y={p.y:.3f} yaw={yaw_of(node.odom.pose.pose.orientation):.3f}')
    print(f'里程(odom): {node.path_len:.3f} m，理论 {args.speed * el:.3f} m')
    print(f'停车滑移  : {drift * 1000:.1f} mm（停发后 12 s 内，应 <20 mm = timeout 生效）')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
