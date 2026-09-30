#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/gen_viewer_parts.py —— 把 URDF 里的 <visual> 零件清单注入 arm6-viewer.html

为什么生成而不是手抄：viewer 的标题写着"URDF 参数 1:1 复刻"，而它原来是把每个
link 的几何体**手抄**成 pushBox/pushCyl 各一行。手抄的东西没有强制力 —— 造型一改
（一个 link 从 1 个几何体变成 10 个），网页就会悄悄变成另一台机器人，
而它看起来仍然"很合理"。改成从 artifacts/arm6.urdf 读，网页与 RViz 就不可能不一致。

  用法: python3 scripts/gen_viewer_parts.py [--urdf artifacts/arm6.urdf] [--html arm6-viewer.html]
        python3 scripts/gen_viewer_parts.py --check     # 只校验不写盘（回归用）

  结果行: VIEWER-RESULT PASS (N 个零件 / M 种材料)   或   VIEWER-RESULT FAIL 原因
  退出码: 0 成功 / 1 校验不通过 / 2 文件缺失
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

BEGIN = "/*__PARTS_BEGIN__*/"
END = "/*__PARTS_END__*/"

# URDF link 名 → viewer 正运动学算出来的帧名。viewer 的 fk() 只认这些帧，
# 所以这里是一张**封闭映射**：出现表外的带几何 link 就报错，而不是悄悄漏画。
FRAME = {
    "base_link": "f0", "link1": "f1", "link2": "f2", "link3": "f3",
    "link4": "f4", "link5": "f5", "link6": "f6",
    "gripper_base": "gb", "finger1_link": "fg1", "finger2_link": "fg2",
    "tool0": "t0",
}


def _f(s, default=0.0):
    return default if s is None else float(s)


def _num(v):
    """紧凑数字：0.0175 -> '0.0175'，0.0 -> '0'，去掉浮点噪声。"""
    s = ("%.6f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def collect(urdf_path):
    root = ET.parse(urdf_path).getroot()
    mats = {}
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None:
            mats[m.get("name")] = [_f(x) for x in c.get("rgba").split()[:3]]
    parts, unknown = [], []
    for link in root.findall("link"):
        name = link.get("name")
        vis = link.findall("visual")
        if not vis:
            continue
        if name not in FRAME:
            unknown.append(name)
            continue
        for v in vis:
            o = v.find("origin")
            xyz = [_f(x) for x in (o.get("xyz").split() if o is not None and o.get("xyz") else [0, 0, 0])] if o is not None and o.get("xyz") else [0.0, 0.0, 0.0]
            rpy = [_f(x) for x in (o.get("rpy").split() if o is not None and o.get("rpy") else [0, 0, 0])] if o is not None and o.get("rpy") else [0.0, 0.0, 0.0]
            g = v.find("geometry").find("*")
            mm = v.find("material")
            mat = mm.get("name") if mm is not None else "default"
            if g.tag == "box":
                sx, sy, sz = [float(x) for x in g.get("size").split()]
                a, b, c = sx, sy, sz
                kind = "b"
            elif g.tag == "cylinder":
                a = float(g.get("radius"))
                b = float(g.get("length"))
                c = 0.0
                kind = "c"
            elif g.tag == "sphere":
                a = b = c = float(g.get("radius"))
                kind = "s"
            else:
                raise ValueError("link %s 用了 %s，viewer 画不了" % (name, g.tag))
            parts.append([FRAME[name], kind, xyz, rpy, (a, b, c), mat])
    if unknown:
        raise ValueError("这些 link 有 visual 但 viewer 没有对应帧，请补 FRAME: %s" % ", ".join(unknown))
    return parts, mats


def render(parts, mats):
    lines = [BEGIN,
             "// 由 scripts/gen_viewer_parts.py 从 artifacts/arm6.urdf 生成 —— 手改会被下次生成覆盖。",
             "// 每条 = [帧名, 形状, x,y,z, roll,pitch,yaw, a,b,c, 材料]；b=box(a=sx,sy,sz) c=cyl(a=r,b=L) s=sphere(a=r)",
             "const MAT = {"]
    for k in sorted(mats):
        r, g, b = mats[k]
        lines.append('  %s:[%s,%s,%s],' % (repr(k).replace("'", '"'), _num(r), _num(g), _num(b)))
    lines.append("};")
    lines.append("const PARTS = [")
    for fr, kind, xyz, rpy, abc, mat in parts:
        nums = ", ".join(_num(x) for x in list(xyz) + list(rpy) + list(abc))
        lines.append('  ["%s","%s",%s,"%s"],' % (fr, kind, nums, mat))
    lines.append("];")
    lines.append(END)
    return "\n".join(lines)


def patch_html(html_path, block, expect_parts):
    src = open(html_path, encoding="utf-8").read()
    if BEGIN not in src or END not in src:
        raise ValueError("%s 里找不到 %s / %s 标记" % (html_path, BEGIN, END))
    new = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END), block, src, flags=re.S)
    n = new.count('["f') + new.count('["g') + new.count('["t')
    if n != expect_parts:
        raise ValueError("注入后零件数 %d != URDF 里的 %d" % (n, expect_parts))
    return new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="artifacts/arm6.urdf")
    ap.add_argument("--html", default="arm6-viewer.html")
    ap.add_argument("--check", action="store_true", help="只比对，不写盘")
    a = ap.parse_args()
    for p in (a.urdf, a.html):
        if not os.path.exists(p):
            print("VIEWER-RESULT FAIL 文件不存在: %s" % p)
            return 2
    try:
        parts, mats = collect(a.urdf)
        block = render(parts, mats)
    except ValueError as e:
        print("VIEWER-RESULT FAIL %s" % e)
        return 1
    src = open(a.html, encoding="utf-8").read()
    cur = re.search(re.escape(BEGIN) + r".*?" + re.escape(END), src, flags=re.S)
    same = bool(cur) and cur.group(0) == block
    if a.check:
        if same:
            print("VIEWER-RESULT PASS (%d 个零件 / %d 种材料，与 URDF 一致)" % (len(parts), len(mats)))
            return 0
        print("VIEWER-RESULT FAIL viewer 里的零件清单与 URDF 不一致，请重跑本脚本（不带 --check）")
        return 1
    try:
        new = patch_html(a.html, block, len(parts))
    except ValueError as e:
        print("VIEWER-RESULT FAIL %s" % e)
        return 1
    open(a.html, "w", encoding="utf-8", newline="\n").write(new)
    print("VIEWER-RESULT PASS (%d 个零件 / %d 种材料 已注入 %s)" % (len(parts), len(mats), a.html))
    return 0


if __name__ == "__main__":
    sys.exit(main())
