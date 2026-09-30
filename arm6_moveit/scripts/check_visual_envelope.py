#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/check_visual_envelope.py

机械臂"外观改造"的两条硬判据（都在离线 URDF 上跑，不需要 ROS、不需要仿真）：

  [A] BASELINE-RESULT  ——（锚点原本叫 COLLISION-RESULT，让给了 update_collisions.sh
      的"碰撞矩阵收敛"，同名两个含义会让 grep 串台）—— 与改造前的基线 URDF 比，**所有 <collision> 是否逐几何体未变**。
      为什么要有这一条：本轮只允许动 <visual>。一旦 collision 被顺手改了一个毫米，
      MoveIt 的碰撞矩阵、SRDF、以及 08 篇里所有计时/到位数字就全部作废，
      而这些东西**不会报错**，只会悄悄变成另一台机器。语义比较（link 名 + 位姿 +
      几何体类型与尺寸），比逐字符 diff 更严：它无视注释与排版，只看几何。

  [B] ENVELOPE-RESULT   —— 每个 link 的 visual 并集 AABB 相对 collision 并集 AABB
      的外扩量。规则三条（数值来自"真机看起来对、又不穿模"的折中）：
        1) 侧向 (x/y) ≤ 6 mm —— 侧向才是"看着穿进桌子、规划却说不碰"的方向；
        2) 端面朝向工件的 link（腕滚 / 夹爪座 / 两指 / tool0）±z ≤ 6 mm
           —— 这里多出来的几毫米会直接变成"抓到了但没夹住"；
        3) 其余 link 的 ±z 只作信息打印，但不得超过 HUB_SANITY_MM：
           俯仰毂本来就是**绕着自己的轴**画的，往轴后凸出一个毂半径是几何必然。

  用法:
    python3 scripts/check_visual_envelope.py [artifacts/arm6.urdf]
    python3 scripts/check_visual_envelope.py --collision-baseline artifacts/arm6_old.urdf
    python3 scripts/check_visual_envelope.py --selftest

  退出码: 0 全 PASS / 1 有 FAIL / 2 用法或解析错误
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

LAT_LIMIT_MM = 6.0
END_AXIS_LIMIT_MM = 6.0
HUB_SANITY_MM = 40.0

# 端面朝向工件的那几个 link：它们的 ±z 外扩会影响抓取，按硬判据算。
END_LINKS = {"link4", "link6", "gripper_base", "finger1_link", "finger2_link", "tool0"}


def _parse(v):
    return [float(x) for x in (v or "0 0 0").split()]


def _rot(rpy):
    """URDF 的 rpy 是固定角 XYZ 外旋：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]


def _aabb_only(link, tag):
    """link 下所有 <visual>（或所有 <collision>）的 AABB 并集；无几何返回 None。"""


def _collisions(root):
    """{link_name: [规范化几何串]}，只取 collision —— 判 A 用。入参是 <robot> 元素。"""
    out = {}
    for link in root.findall("link"):
        items = []
        for c in link.findall("collision"):
            origin = c.find("origin")
            xyz = [round(v, 6) for v in (_parse(origin.get("xyz")) if origin is not None else [0, 0, 0])]
            rpy = [round(v, 6) for v in (_parse(origin.get("rpy")) if origin is not None else [0, 0, 0])]
            g = c.find("geometry").find("*")
            if g.tag == "box":
                geo = "box " + " ".join("%.6f" % float(x) for x in g.get("size").split())
            elif g.tag == "cylinder":
                geo = "cyl %.6f %.6f" % (float(g.get("radius")), float(g.get("length")))
            elif g.tag == "sphere":
                geo = "sph %.6f" % float(g.get("radius"))
            else:
                geo = g.tag
            items.append("%s|%s|%s" % (xyz, rpy, geo))
        out[link.get("name")] = sorted(items)
    return out


def check_envelope(urdf_path):
    tree = ET.parse(urdf_path)
    rows, fails = [], []
    for link in tree.getroot().findall("link"):
        name = link.get("name")
        c_box = _aabb_only(link, "collision")
        v_box = _aabb_only(link, "visual")
        if c_box is None:
            rows.append((name, None, None, None, "no-collision"))
            continue
        if v_box is None:
            rows.append((name, None, None, None, "no-visual"))
            continue
        (vlo, vhi), (clo, chi) = v_box, c_box
        over = [max(0.0, vhi[a] - chi[a]) * 1000.0 for a in range(3)]
        under = [max(0.0, clo[a] - vlo[a]) * 1000.0 for a in range(3)]
        lat = max(over[0], under[0], over[1], under[1])
        ax = max(over[2], under[2])
        bad = []
        if lat > LAT_LIMIT_MM + 1e-6:
            bad.append("侧向 %.2f mm > %.1f" % (lat, LAT_LIMIT_MM))
        if name in END_LINKS and ax > END_AXIS_LIMIT_MM + 1e-6:
            bad.append("端面 %.2f mm > %.1f" % (ax, END_AXIS_LIMIT_MM))
        if ax > HUB_SANITY_MM:
            bad.append("轴向 %.2f mm > 毂上限 %.1f" % (ax, HUB_SANITY_MM))
        rows.append((name, lat, ax, name in END_LINKS, bad))
        if bad:
            fails.append("%s: %s" % (name, "; ".join(bad)))
    return rows, fails


def _aabb_only(link, tag):
    """与 _aabb 同逻辑，但只累计指定 tag（visual 或 collision）。"""
    lo, hi, found = [math.inf] * 3, [-math.inf] * 3, False
    for elem in link.findall(tag):
        origin = elem.find("origin")
        xyz = _parse(origin.get("xyz")) if origin is not None else [0.0, 0.0, 0.0]
        rpy = _parse(origin.get("rpy")) if origin is not None else [0.0, 0.0, 0.0]
        g = elem.find("geometry")
        if g is None:
            continue
        box, cyl, sph = g.find("box"), g.find("cylinder"), g.find("sphere")
        if box is not None:
            s = [float(x) for x in box.get("size").split()]
            half = [s[0] / 2.0, s[1] / 2.0, s[2] / 2.0]
        elif cyl is not None:
            half = [float(cyl.get("radius"))] * 2 + [float(cyl.get("length")) / 2.0]
        elif sph is not None:
            half = [float(sph.get("radius"))] * 3
        else:
            raise ValueError("link %s 的 %s 用了 mesh，AABB 算不了" % (link.get("name"), tag))
        R = _rot(rpy)
        for i in (0, 1):
            for j in (0, 1):
                for k in (0, 1):
                    p = [(i * 2 - 1) * half[0], (j * 2 - 1) * half[1], (k * 2 - 1) * half[2]]
                    for a in range(3):
                        v = xyz[a] + sum(R[a][b] * p[b] for b in range(3))
                        lo[a] = min(lo[a], v)
                        hi[a] = max(hi[a], v)
        found = True
    return (lo, hi) if found else None


def report(rows, fails):
    print("  link              侧向外扩   端向外扩   端面link  判定")
    for name, lat, ax, is_end, bad in rows:
        if lat is None:
            print("  %-16s %s" % (name, bad))
            continue
        mark = "FAIL" if bad else ("ok" if is_end else "info")
        print("  %-16s %8.2f %10.2f %8s   %s" % (name, lat, ax, "Y" if is_end else "-", mark))
    print("  阈值: 侧向 ≤ %.1f mm / 端面 link ±z ≤ %.1f mm / 任意 link ±z ≤ %.1f mm"
          % (LAT_LIMIT_MM, END_AXIS_LIMIT_MM, HUB_SANITY_MM))
    return fails


def selftest():
    """判据自己也要被验一次 —— 造三种坏 URDF，必须都被抓。

    ★ 红样只改 <visual> 那一行。第一版图省事直接 base.replace('size="0.02 0.02', ...)，
      结果 str.replace 默认**全替换**，visual 和 collision 一起变大了，
      外扩量恒为 0，三条红样全"通过" —— 一个永远不会红的判据比没有判据更糟。
    """
    import tempfile
    VIS = '    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="%s"/></geometry></visual>'
    COLL = '    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.02 0.02 0.10"/></geometry></collision>'

    def urdf(vis_size, link_name="arm"):
        return ('<robot name="t">\n  <link name="%s">\n%s\n%s\n  </link></robot>'
                % (link_name, VIS % vis_size, COLL))

    cases = [
        ("好样：visual 与 collision 同尺寸", urdf("0.02 0.02 0.10"), None, True),
        ("红样 1：visual 侧向凸 30 mm", urdf("0.08 0.02 0.10"), None, False),
        ("红样 2：端面 link 的 visual 沿 z 长 40 mm", urdf("0.02 0.02 0.14", "link6"), None, False),
        ("红样 3：非端面 link 沿 z 凸 60 mm（超毂安全上限）", urdf("0.02 0.02 0.22"), None, False),
        ("对照：非端面 link 沿 z 凸 20 mm（毂绕自己的轴，允许）", urdf("0.02 0.02 0.14"), None, True),
    ]
    ok = True
    for desc, xml, _extra, expect_pass in cases:
        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as f:
            f.write(xml)
            path = f.name
        try:
            _rows, fails = check_envelope(path)
            got_pass = not fails
        finally:
            os.unlink(path)
        good = got_pass == expect_pass
        ok = ok and good
        print("  [%s] %s -> %s (期望 %s)" % (
            "PASS" if good else "FAIL", desc,
            "无外扩告警" if got_pass else "报外扩", "应通过" if expect_pass else "应报错"))
    # 判据 A 的红样：改一个 collision 尺寸必须被基线比对抓出来
    good_xml = urdf("0.02 0.02 0.10")
    bad_xml = good_xml.replace(COLL, COLL.replace("0.10", "0.095"))
    ca = _collisions(ET.fromstring(good_xml))
    cb = _collisions(ET.fromstring(bad_xml))
    good = ca != cb
    ok = ok and good
    print("  [%s] 红样 4：collision 少 5 mm 必须被基线比对抓出" % ("PASS" if good else "FAIL"))
    # 判据 A 的假红对照：基线里根本没有 dummy 根 world，不该算差异
    with_empty = '<robot name="t"><link name="world"/><link name="arm">%s%s</link></robot>' % (VIS % "0.02 0.02 0.10", COLL)
    without = '<robot name="t"><link name="arm">%s%s</link></robot>' % (VIS % "0.02 0.02 0.10", COLL)
    ea = _collisions(ET.fromstring(with_empty))
    eb = _collisions(ET.fromstring(without))
    good = all((ea.get(n) or []) == (eb.get(n) or []) for n in set(ea) | set(eb))
    ok = ok and good
    print("  [%s] 好样 5：基线缺 dummy 根 world 不该报假红" % ("PASS" if good else "FAIL"))
    print("SELFTEST-RESULT %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", nargs="?", default="artifacts/arm6.urdf")
    ap.add_argument("--collision-baseline", default=None)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()

    if not os.path.exists(a.urdf):
        print("ENVELOPE-RESULT FAIL urdf 不存在: %s" % a.urdf)
        return 2

    rows, fails = check_envelope(a.urdf)
    report(rows, fails)
    print("ENVELOPE-RESULT %s (max 侧向 %.2f mm, 端面 %.2f mm, %d 个 link 超限)" % (
        "PASS" if not fails else "FAIL",
        max([r[1] for r in rows if r[1] is not None] or [0.0]),
        max([r[2] for r in rows if r[2] is not None] or [0.0]),
        len(fails)))
    for line in fails:
        print("  ISSUE %s" % line)

    rc = 0 if not fails else 1
    if a.collision_baseline:
        if not os.path.exists(a.collision_baseline):
            print("BASELINE-RESULT FAIL 基线不存在: %s" % a.collision_baseline)
            return 2
        cur = _collisions(ET.parse(a.urdf).getroot())
        old = _collisions(ET.parse(a.collision_baseline).getroot())
        # "基线里没有这个 link" 与 "这个 link 没有任何 collision 几何体" 是同一件事：
        # dummy 根 world 就是这种空 link，它在不在这份存档里都不影响几何。
        # 不这样归一化，判据 A 会报一条假红，而假红会把人推回去"修"一个没坏的东西。
        diffs = [name for name in sorted(set(cur) | set(old))
                 if (cur.get(name) or []) != (old.get(name) or [])]
        if diffs:
            print("BASELINE-RESULT FAIL 以下 link 的 collision 与基线不同: %s" % ", ".join(diffs))
            for name in diffs:
                print("  %-14s 基线 %s" % (name, old.get(name)))
                print("  %-14s 现在 %s" % ("", cur.get(name)))
            rc = 1
        else:
            n = sum(len(v) for v in cur.values())
            print("BASELINE-RESULT PASS (%d 个 link / %d 个 collision 几何体逐项一致)" % (len(cur), n))
    return rc


if __name__ == "__main__":
    sys.exit(main())
