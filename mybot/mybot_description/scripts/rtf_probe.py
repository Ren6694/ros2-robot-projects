#!/usr/bin/env python3
"""RTF 探针：用 /clock 的仿真时间增量 ÷ 墙上时间增量，直接量 Real Time Factor。

为什么要单独写这个脚本
----------------------
`ros2 topic hz /clock` 只能告诉你**发布频率**，读不出 RTF（Gazebo 的 clock 发布节奏
和 max_step_size / real_time_update_rate 绑在一起）。判 RTF 只认一个数：
**仿真时间走了多少 ÷ 墙上过了多少**。

为什么值得每次实验前量一遍
--------------------------
本项目里 `cmd_vel_timeout`、传感器 `update_rate` 都是**按仿真时间**计时的，
它们的表现完全由 RTF 决定。仓库文档里先后出现过 0.16 / 0.11~0.22 / 0.15 三个常数，
09-19 晚重测发现**这些低值都不可复现**（是在残留多个 gzserver 的污染状态下测的）：
车跑动时 gzclient + rqt 全开实测 RTF = 1.00，空转时读到 0.53。
结论不是"换成一个新常数"，而是**别照抄常数，现量**。

用法（WSL，已 source ROS 环境）
------------------------------
    ros2 run mybot_description rtf_probe.py 15
    # 或 python3 scripts/rtf_probe.py 15

坑：订阅 /clock 必须用 BEST_EFFORT QoS。它是 best_effort 发布的，
默认 reliable 订阅会报 `incompatible QoS ... RELIABILITY` 且**一条都收不到**，
看起来像"仿真没起"，其实是 QoS 不匹配。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock


def main() -> int:
    span = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node('rtf_probe')
    samples = []

    def cb(msg):
        samples.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9))

    # create_subscription 的参数顺序是 (msg_type, topic, callback, qos)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(Clock, '/clock', cb, qos)

    t0 = time.monotonic()
    while time.monotonic() - t0 < span and len(samples) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(samples) < 2:
        print('拿不到 /clock —— 仿真没起？先跑 `bash ros2_workbench.sh status` 看 gzserver 数量')
        return 1
    first = samples[0]
    while time.monotonic() - t0 < span:
        rclpy.spin_once(node, timeout_sec=0.2)
    last = samples[-1]

    sim_dt = last[1] - first[1]
    wall_dt = max(last[0] - first[0], 1e-6)
    print(f'RTF = {sim_dt / wall_dt:.2f}   (仿真 {sim_dt:.2f} s / 墙上 {wall_dt:.2f} s)')
    print(f'样本 {len(samples)} 条 /clock，平均 {len(samples) / wall_dt:.1f} Hz')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
