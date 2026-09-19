#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D9：把相机画面变成**米制**的横向偏差 e 与夹角 θ，供 D10 的 PD 直接用。

数据流：
  /camera/image_raw ─┐
  /camera/camera_info ┤→ masking.make_mask(hsv+blur5+open3) → line_features.extract()
                     └→ /line/pose   (geometry_msgs/PoseStamped: position.y=e[米,左为正],
                                      orientation=绕 z 的 θ[rad]; header 原样透传)
                       /line/centers (bgr8 可视化：逐行中心 + 拟合直线)
                       /line/mask    (mono8，沿用 D8 的话题名)

为什么用 PoseStamped 而不是 Pose2D：`geometry_msgs/Pose2D` **没有 header**，
下游就无法判断这帧数据有多旧（D10 的看门狗、以及和 /odom 做时间对齐都要靠时间戳）。

符号约定（和 line_features 一致，D10 整定时别搞反）：
  e > 0  线在车**左**边 → cmd_vel.angular.z 取正（左转）
  θ > 0  线朝**左**拐   → 同样给正转向

用法：
  ros2 run mybot_control line_features_node.py
  ros2 run mybot_control line_features_node.py --publish-image false   # 省 CPU
  ros2 topic echo /line/pose
"""
import argparse
import math
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

if 'mybot_control' not in sys.modules:
    _here = os.path.dirname(os.path.abspath(__file__))
    for _c in (os.path.join(_here, '..'), _here):
        if os.path.isdir(os.path.join(_c, 'mybot_control')):
            sys.path.insert(0, _c)
            break
from mybot_control.line_features import CamGeom, extract   # noqa: E402
from mybot_control.masking import FEATURE, make_mask       # noqa: E402

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass


class LineFeaturesNode(Node):
    def __init__(self, args):
        super().__init__('line_features')
        self.args = args
        self.bridge = CvBridge()
        self.geom = None                 # 内参等 /camera/camera_info 到达后再建
        self.last_print = 0.0
        self.n_ok = 0
        self.n_bad = 0
        self.last = None

        self.pub_pose = self.create_publisher(PoseStamped, '/line/pose', 10)
        self.pub_mask = self.create_publisher(Image, '/line/mask', 10)
        self.pub_img = self.create_publisher(Image, '/line/centers', 10)
        self.create_subscription(CameraInfo, args.info_topic, self._info_cb, 10)
        self.create_subscription(Image, args.topic, self._img_cb, 10)

    def _info_cb(self, msg):
        if self.geom is not None:
            return
        self.geom = CamGeom.from_camera_info(
            msg, cam_x=self.args.cam_x, cam_h=self.args.cam_h, pitch=self.args.pitch)

    def _img_cb(self, msg):
        if self.geom is None:
            return                        # 没有内参就不算，避免用错 fx 得出假米制
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 注意用 FEATURE 而不是 BASELINE：开运算会把远处 3~4 px 的细线咬断，
        # 让 θ 多算 7°（单元测试里能看到两种 mask 的对照行）
        mask = make_mask(img, **FEATURE)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8',
                                                        header=msg.header))
        lf = extract(mask, self.geom, x_lo=self.args.x_lo, x_hi=self.args.x_hi)

        if lf.ok:
            self.n_ok += 1
            self.last = lf
            out = PoseStamped()
            out.header = msg.header                   # 时间戳+frame 原样透传
            out.header.frame_id = 'base_link'         # e/θ 是车体系的量
            out.pose.position.y = lf.e                # 米，左为正
            out.pose.orientation.z = math.sin(lf.theta / 2.0)
            out.pose.orientation.w = math.cos(lf.theta / 2.0)
            self.pub_pose.publish(out)
        else:
            self.n_bad += 1
            # 故意不发布"上一次的值"：宁可让 D10 的看门狗发现没数据，
            # 也不要拿旧数据把车开出去 —— 这是仿真阶段就该立的规矩
            pass

        if self.args.publish_image:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(self._draw(img, lf),
                                                           encoding='bgr8',
                                                           header=msg.header))

        now = time.time()
        if now - self.last_print >= self.args.print_every:
            self.last_print = now
            if lf.ok:
                span = f'{lf.fit_span[0]:.2f}~{lf.fit_span[1]:.2f} m'
                print(f'e={lf.e * 100:+6.1f} cm@x={lf.x_ref:.2f}m  '
                      f'θ={math.degrees(lf.theta):+6.1f}°  '
                      f'行={lf.n_inl}/{lf.n_rows}(内点/候选)  拟合段 x∈{span}  '
                      f'ok/bad={self.n_ok}/{self.n_bad}', flush=True)
            else:
                print(f'无有效线特征：{lf.reason}（ok/bad={self.n_ok}/{self.n_bad}）',
                      flush=True)

    def _draw(self, img, lf):
        out = img.copy()
        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (90, 90, 90), 1)
        for x, y, u, v, _wd in lf.pts:
            cv2.circle(out, (int(u), int(v)), 1, (0, 0, 255), -1)
        if lf.ok:
            # 把拟合直线画回画面：沿 x 取首尾两点投影成像素，看它有没有贴着线心
            for (x0, x1) in [(lf.fit_span[0], lf.fit_span[1])]:
                p0 = self.geom.ground_to_pixel(x0, lf.e + math.tan(lf.theta) * x0)
                p1 = self.geom.ground_to_pixel(x1, lf.e + math.tan(lf.theta) * x1)
                if p0 and p1:
                    cv2.line(out, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])),
                             (0, 255, 0), 1)
            cv2.putText(out, f"e={lf.e * 100:+.1f}cm th={math.degrees(lf.theta):+.1f}deg",
                        (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        else:
            cv2.putText(out, 'NO LINE', (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 1)
        return out


def main():
    ap = argparse.ArgumentParser(description='D9 线特征：米制 e 与 θ')
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--info-topic', default='/camera/camera_info')
    ap.add_argument('--cam-x', type=float, default=0.21, help='相机在车体系前向位置(米)')
    ap.add_argument('--cam-h', type=float, default=0.154, help='相机离地高度(米)')
    ap.add_argument('--pitch', type=float, default=0.6, help='相机俯角(rad，正=低头)')
    ap.add_argument('--x-lo', type=float, default=0.05, help='拟合最近距离(米)')
    ap.add_argument('--x-hi', type=float, default=0.60, help='拟合最远距离(米)')
    ap.add_argument('--publish-image', default='true', help='true/false，是否发可视化图')
    ap.add_argument('--print-every', type=float, default=1.0, help='终端打印间隔(墙上秒)')
    ap.add_argument('--duration', type=float, default=0.0, help='>0 则到时退出')
    args = ap.parse_args()
    args.publish_image = str(args.publish_image).lower() in ('1', 'true', 'yes')

    rclpy.init()
    node = LineFeaturesNode(args)
    print(f'等 {args.info_topic} 给内参（cam_x={args.cam_x} cam_h={args.cam_h} '
          f'pitch={args.pitch}），然后 /line/pose2d 开始出数')
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration and time.time() - t0 >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
