#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光时间戳修正 relay
====================
问题（2026-09-18 实测确认）
--------------------------
Gazebo 的 ray sensor 发布的 /scan 消息，其 header.stamp 比【数据实际被采集的
那一刻】晚约【半个采样周期】。

在 12 Hz 下，半个周期 = 41.7 ms。

危害
----
Cartographer（以及任何用 msg.header.stamp 去查 TF 的消费者）会拿
"t + 41.7ms 时刻的机器人位姿" 去投影 "t 时刻采集的激光点"。

    小车 0.5 m/s、转角 0.7 rad/s 时：
        位置偏差 = 0.5 × 0.0417 = 2.1 cm
        角度偏差 = 0.7 × 0.0417 = 1.7 deg
这两种偏差随运动方向改变符号，于是同一面墙在不同位置被观测到，
占用概率累积不起来 —— 地图表现为"发虚、重影、墙缺失"。

实测证据（纯里程计投影基准，同一次轨迹，只改 TF 查询时间偏移）
--------------------------------------------------------------
    offset     R@±3      准确率
    +0ms       91.2%      99%+      ← 当前系统行为
    -20ms      93.4%
    -40ms      98.4%     100%       ← 峰值，与 半周期 41.7ms 吻合
    -60ms      97.4%
    -80ms      95.6%
    -100ms     92.8%

本节点做什么
------------
订阅 /scan，把 header.stamp 减去 offset（默认 41.7ms），
以 scan_fixed 话题转发。Cartographer 订阅 scan_fixed 即可。

注意：修正后位姿与数据对齐，但 Cartographer 会读到"更早的时间"。
由于 TF 缓冲保存历史，查询仍然成功，且拿到的是正确的位姿。

用法
----
    ros2 run <pkg> scan_time_fix.py --ros-args -p offset:=-0.0417
或直接
    python3 scan_time_fix.py --ros-args -p offset:=-0.0417 -p use_sim_time:=true
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time


class ScanTimeFix(Node):

    def __init__(self):
        super().__init__('scan_time_fix')

        # 默认 -1/(2 * 12Hz) = -41.7ms
        self.declare_parameter('offset', -0.0417)
        self.declare_parameter('in_topic', 'scan')
        self.declare_parameter('out_topic', 'scan_fixed')
        self.declare_parameter('report_every', 20.0)

        self.off = float(self.get_parameter('offset').value)
        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.report_every = float(self.get_parameter('report_every').value)

        # 订阅用 best_effort（与 gazebo 插件一致）；
        # 发布用 reliable —— reliable 发布端可被 best_effort 与 reliable 订阅端同时接收，
        # 兼容性最好（Cartographer 内部用 SensorDataQoS = best_effort）。
        sub_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(LaserScan, self.out_topic, pub_qos)
        self.sub = self.create_subscription(
            LaserScan, self.in_topic, self.cb, sub_qos)

        self.n = 0
        self.dts = []
        self.t_report = self.get_clock().now()

        self.get_logger().info(
            'scan 时间戳修正: %s -> %s   偏移 %+.4f s (%.1f ms)'
            % (self.in_topic, self.out_topic, self.off, self.off * 1000))
        self.get_logger().info(
            '依据: Gazebo 激光时间戳滞后半个采样周期；12Hz 时 = 41.7ms')

    def cb(self, msg: LaserScan):
        t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) + self.off
        s = int(math.floor(t))
        ns = int(round((t - s) * 1e9))
        if ns >= 1000000000:
            s += 1
            ns -= 1000000000
        if ns < 0:
            s -= 1
            ns += 1000000000
        msg.header.stamp = Time(sec=s, nanosec=ns)
        self.pub.publish(msg)

        self.n += 1
        now = self.get_clock().now()
        if (now - self.t_report).nanoseconds * 1e-9 >= self.report_every:
            self.t_report = now
            self.get_logger().info('已转发 %d 帧 (偏移 %+.1f ms)'
                                   % (self.n, self.off * 1000))


def main():
    rclpy.init()
    node = ScanTimeFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            node.get_logger().info('最终转发 %d 帧' % node.n)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
