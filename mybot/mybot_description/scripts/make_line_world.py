#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D7：巡线赛道世界生成器（line_following.world）。

为什么不直接手写 .world：
  赛道 = 2 条长直道 + 2 条短直道 + 4 段圆弧。圆弧在 SDF 里只能用直盒段（chord）近似，
  一段一个 <visual>，手写 28 个盒子既容易算错 yaw，也没法在 D12 调参时改一个数就重生成。
  所以把中心线参数化，由本脚本一次性产出 .world，赛道几何进 git、生成器也进 git（可复现）。

几何约定（俯视，右手系，z 向上）：
      a = --half-straight   长直道半长（底/顶直道 x ∈ [-a, a]，y = ∓b）
      b = --half-width      长直道的 y 位置（赛道半宽）
      r = --corner-radius   四角圆弧半径
      于是整条环线外轮廓尺寸 = (2a + 2r) × (2b)，本脚本默认 3.30 m × 1.80 m

黑线怎么画：
  · 视觉-only（**不写 <collision>**）—— 否则 3 mm 的台阶会干扰驱动轮接触，污染 /odom；
  · 每段盒子长度 = 弦长 + line_width（两端各多画半格），保证相邻段重叠、接头不露底；
  · 圆弧按 --arc-step-deg 切分，默认 15°：r=0.45 时弦长 0.117 m，接缝缺口 <0.3 mm，阈值分割看不出来。

用法（Windows 侧改，再 cp 进 WSL 构建）：
  python scripts/make_line_world.py --out worlds/line_following.world
"""
from __future__ import annotations

import argparse
import math
import os
import sys

WORLD_TEMPLATE = """<?xml version="1.0"?>
<!-- 由 scripts/make_line_world.py 生成，请勿手改；改赛道改参数重跑生成器。
     参数：{params}
     几何：环线 {perimeter:.2f} m，{nseg} 段黑线（直道 {straight_n} + 圆弧 {arc_n}） -->
<sdf version="1.6">
  <world name="line_following">

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- 地面：Gazebo/Grey（diffuse 0.7）+ 摩擦 mu=100/mu2=50，比自建平面抓地更好，
         与黑线 FlatBlack（0.1）形成 7:1 亮度对比，D8 阈值分割余量充足 -->
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- 关掉阴影与地面网格：WSL2 上 RTF 只有 ~0.16，省渲染；
         网格是 gzclient 叠在地面上的半透明线，会进相机画面，给 D8 添噪声
         （注意：SDF 1.6 的 <scene> 不认 <grid>，实测该元素被忽略，见 --no-scene 二分记录）
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <shadows>false</shadows>
      <grid>
        <enabled>false</enabled>
      </grid>
    </scene>
     -->
{scene}

    <!-- ROS 世界接口（D6 结论：Humble 已把 gazebo_ros_api_plugin 拆成三块）
         libgazebo_ros_factory.so 由 gzserver -s 加载（/spawn_entity）；
         下面两个挂在 world 里：state -> /gazebo/{{get,set}}_entity_state，
         properties -> /pause_physics、/unpause_physics、/gazebo/{{get,set}}_light_properties -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
    </plugin>
    <plugin name="gazebo_ros_properties" filename="libgazebo_ros_properties.so">
      <ros><namespace>/gazebo</namespace></ros>
    </plugin>

    <!-- ============ 巡线黑线：一个 static model、一个 link、{nseg} 个 visual ============ -->
    <model name="line_track">
      <static>true</static>
      <link name="line">
{segments}
      </link>
    </model>

    <!-- 起跑参考块（黄色，中心在黑线外侧 {pad_offset_y:.3f} m，即 y={y_start:.3f}，
         与黑线不相连：D9 做线中心提取时它不会被当成线的一部分） -->
    <model name="start_pad">
      <static>true</static>
      <link name="pad">
        <visual name="v">
          <pose>0.0 {y_start:.3f} 0.0015 0 0 0</pose>
          <geometry><box><size>0.03 0.16 0.003</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Yellow</name>
          </script></material>
        </visual>
      </link>
    </model>
{stain}
  </world>
</sdf>
"""

# D8 干扰色块：暗红"地板污渍"。它的灰度亮度约 44（会被 gray<80 误判成线），
# 但 HSV 的 V = max(R,G,B) 约 110，远大于黑线的 V 约 26，所以 HSV 阈值能排除它。
# 专门用来回答"D8 为什么最后选 HSV 而不是灰度"这个问题。--no-stain 可关掉。
STAIN_BLOCK = """
    <!-- D8 干扰色块：故意放在黑线左侧 18 cm、车开不到的地方 -->
    <model name="floor_stain">
      <static>true</static>
      <link name="stain">
        <visual name="v">
          <pose>0.60 -0.72 0.0015 0 0 0</pose>
          <geometry><box><size>0.16 0.12 0.003</size></box></geometry>
          <material>
            <lighting>true</lighting>
            <ambient>0.20 0.02 0.02 1</ambient>
            <diffuse>0.45 0.05 0.05 1</diffuse>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""

SEGMENT_TEMPLATE = """        <visual name="seg_{idx:02d}">
          <pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length:.4f} {width:.3f} {thk:.4f}</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/FlatBlack</name>
          </script></material>
        </visual>"""

# --scene 三档，用于二分"相机传感器出图全灰"这个问题：
#   none  : 完全不写 <scene>，与 D5 的 mybot_world.world 保持一致（已验证传感器能出图）
#   basic : 只写 SDF 1.6 认的 ambient / shadows
#   full  : 再加 <grid>（SDF 1.6 不认，会被忽略）
SCENE_BLOCKS = {
    'none': '',
    'basic': """    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <shadows>false</shadows>
    </scene>""",
    'full': """    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <shadows>false</shadows>
      <grid><enabled>false</enabled></grid>
    </scene>""",
}


def build_centerline(a: float, b: float, r: float, arc_step_deg: int):
    """按逆时针顺序返回闭合中心线顶点（首尾同为 (-a,-b)）。"""
    pts = [(-a, -b), (a, -b)]                      # 底部长直道

    def arc(cx, cy, a0, a1):
        step = math.radians(arc_step_deg)
        ang = a0 + step
        while ang < a1 - 1e-9:
            pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
            ang += step

    arc(a, -(b - r), -math.pi / 2, 0.0)            # 右下 90° -> 到达 (a+r, -(b-r))
    pts.append((a + r, -(b - r)))
    pts.append((a + r, (b - r)))                   # 右侧短直道
    arc(a, (b - r), 0.0, math.pi / 2)
    pts.append((a, b))                             # 右上
    pts.append((-a, b))                            # 顶部长直道
    arc(-a, (b - r), math.pi / 2, math.pi)
    pts.append((-(a + r), (b - r)))                # 左上
    pts.append((-(a + r), -(b - r)))               # 左侧短直道
    arc(-a, -(b - r), math.pi, 1.5 * math.pi)
    pts.append((-a, -b))                           # 回到起点，闭合
    return pts


def main() -> int:
    try:  # Windows 控制台默认 cp936，重定向时中文 print 会炸；统一按 UTF-8 输出
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="生成巡线赛道 .world")
    ap.add_argument("--half-straight", type=float, default=1.2, dest="a")
    ap.add_argument("--half-width", type=float, default=0.9, dest="b")
    ap.add_argument("--corner-radius", type=float, default=0.45, dest="r")
    ap.add_argument("--line-width", type=float, default=0.05, dest="w")
    ap.add_argument("--line-thickness", type=float, default=0.003, dest="thk")
    ap.add_argument("--arc-step-deg", type=int, default=15, dest="step")
    ap.add_argument("--out", default="worlds/line_following.world")
    ap.add_argument("--scene", choices=sorted(SCENE_BLOCKS), default="basic",
                    help="写哪种 <scene> 块（二分相机传感器出图问题用）")
    ap.add_argument("--no-stain", dest="stain", action="store_false", default=True,
                    help="不放 D8 的暗红干扰色块")
    args = ap.parse_args()

    if args.r >= args.b:
        raise SystemExit(f"corner-radius({args.r}) 必须小于 half-width({args.b})，否则圆角自交")

    pts = build_centerline(args.a, args.b, args.r, args.step)
    zc = args.thk / 2.0                             # 盒子中心高度：贴地、上半截露出
    segs, perimeter = [], 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        chord = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        length = chord + args.w                     # 两端各延长半格，接头不露底
        perimeter += chord
        segs.append(SEGMENT_TEMPLATE.format(
            idx=len(segs), cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, z=zc,
            yaw=yaw, length=length, width=args.w, thk=args.thk))

    arc_n = sum(1 for (x0, y0), (x1, y1) in zip(pts, pts[1:])
                if abs(math.hypot(x1 - x0, y1 - y0) - 2 * args.r * math.sin(math.radians(args.step / 2))) < 1e-6)
    params = (f"half_straight={args.a} half_width={args.b} corner_radius={args.r} "
              f"line_width={args.w} line_thickness={args.thk} arc_step_deg={args.step}")

    pad_offset_y = args.w / 2 + 0.10          # 黑线半宽 + 10 cm 间隙
    xml = WORLD_TEMPLATE.format(params=params, perimeter=perimeter, nseg=len(segs),
                                straight_n=len(segs) - arc_n, arc_n=arc_n,
                                segments="\n".join(segs), y_start=-(args.b + pad_offset_y),
                                pad_offset_y=pad_offset_y, scene=SCENE_BLOCKS[args.scene],
                                stain=STAIN_BLOCK if args.stain else '')

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(xml)

    print(f"环线周长  : {perimeter:.2f} m")
    print(f"黑线段数  : {len(segs)}（直道 {len(segs) - arc_n} / 圆弧 {arc_n}）")
    print(f"外轮廓    : {2 * (args.a + args.r):.2f} x {2 * args.b:.2f} m")
    print(f"线宽/厚度 : {args.w * 1000:.0f} mm / {args.thk * 1000:.0f} mm")
    print(f"起点位姿  : x=0 y={-args.b} yaw=0（底部长直道中点，车头朝 +x）")
    print(f"已写出    : {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
