#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/check_limits.sh   (在 WSL 里运行)
#  作用: 静态校验"限位/容差/目标值"三处定义是否自洽，一条命令给出 PASS/FAIL
#
#  为什么需要它（这三件事各自都会以"很难查"的方式坏掉）：
#    1) URDF <limit> 与 config/joint_limits.yaml 同时给速度上限时，
#       MoveIt 取**两者较小值**。yaml 写得比 URDF 大 = 那行配置完全无效，
#       而且没有任何告警。只有把"yaml <= URDF"变成断言才拦得住。
#    2) SRDF 的 <group_state> 与 config/waypoints.yaml 里的目标角，
#       只要有一个越出 URDF 限位，表现是"规划时才失败"：
#         - 越限 -> INVALID_GOAL / 无解
#         - 在限内但自碰撞 -> GOAL_IN_COLLISION(-12)
#       两种都要跑起 move_group 才看得见，而这类错误 100 % 能在静态阶段查出。
#    3) 手指是 prismatic，单位是 **m**；把它当 rad 写进 waypoints 是最容易
#       犯的错（0.040 写成 40.0 这种）。所以校验里对 gripper 组单独给单位提示。
#
#  用法: bash scripts/check_limits.sh
#  自检（判据不验证就等于没有 —— 故意喂一个越限目标，看它是否真的 FAIL）：
#      sed 's/joint2: -0.60/joint2: 3.50/' src/arm_demo/config/waypoints.yaml > /tmp/bad.yaml
#      WP_FILE=/tmp/bad.yaml bash scripts/check_limits.sh    # 期望 CHECK-RESULT FAIL
#  退出码: 0 = 全部自洽；1 = 有问题（问题逐条列出）
# =============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$WS/scripts/ros_env.sh"

CFG="$WS/src/arm_moveit_config/config"
XACRO="$WS/src/arm_description/urdf/arm6.urdf.xacro"
# 默认校验包内那份；自检时可用 WP_FILE 指向一份"故意写错"的副本
WP="${WP_FILE:-$WS/src/arm_demo/config/waypoints.yaml}"

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT
echo ">>> [1/2] xacro -> urdf"
xacro "$XACRO" -o "$TMP"

echo ">>> [2/2] 交叉校验 URDF / joint_limits.yaml / arm6.srdf / waypoints"
echo "        waypoints = $WP"
# 注意：这里用 heredoc 内嵌 python，是为了让"校验规则"和"调用它的方式"在同一处，
# 改规则不用翻两个文件。python 侧只依赖标准库 + PyYAML（ROS 环境自带）。
python3 - "$TMP" "$CFG" "$WP" <<'PY'
import sys
import xml.etree.ElementTree as ET

import yaml

urdf_path, cfg_dir, wp_path = sys.argv[1], sys.argv[2], sys.argv[3]

# ---------- URDF ----------
# 输入是本仓库 xacro 的产物 + 自己的配置（可信来源），标准库 ElementTree 足够。
# 若以后要校验第三方/下载来的 URDF，换成 defusedxml.parse —— XML 实体展开
# （billion laughs / 外部实体）只对不可信输入才是问题。
root = ET.parse(urdf_path).getroot()
lim = {}
for j in root.iter("joint"):
    name = j.get("name")
    L = j.find("limit")
    if L is None or L.get("velocity") is None:
        continue          # fixed joint 没有 limit，正常
    lim[name] = {
        "type": j.get("type"),
        "lower": float(L.get("lower")),
        "upper": float(L.get("upper")),
        "velocity": float(L.get("velocity")),
        "effort": float(L.get("effort")),
    }

# ---------- joint_limits.yaml ----------
with open(f"{cfg_dir}/joint_limits.yaml", encoding="utf-8") as fh:
    jl = yaml.safe_load(fh)["joint_limits"]

# ---------- SRDF group_state ----------
tree = ET.parse(f"{cfg_dir}/arm6.srdf").getroot()
states = {}
for gs in tree.iter("group_state"):
    states[gs.get("name")] = {
        j.get("name"): float(j.get("value")) for j in gs.iter("joint")
    }

# ---------- waypoints.yaml ----------
with open(wp_path, encoding="utf-8") as fh:
    wps = yaml.safe_load(fh)["waypoints"]

issues = []

# 规则 1：yaml 的速度必须 <= URDF 的速度，否则那行配置是死的
for name, v in jl.items():
    if name not in lim:
        issues.append(f"joint_limits.yaml 里的 {name} 在 URDF 中不存在（关节改名了？）")
        continue
    mv = v.get("max_velocity")
    if mv is not None and mv > lim[name]["velocity"] + 1e-9:
        issues.append(
            f"{name}: joint_limits.max_velocity={mv} > URDF limit.velocity="
            f"{lim[name]['velocity']} —— MoveIt 取小值，这行等于没写"
        )

# 规则 2：所有目标角必须落在 URDF 限位内
def check(label, group, joints):
    for name, val in joints.items():
        if name not in lim:
            issues.append(f"{label}: 未知关节 {name}")
            continue
        is_finger = name.startswith("finger")
        unit = "m" if is_finger else "rad"
        lo, hi = lim[name]["lower"], lim[name]["upper"]
        if not (lo - 1e-9 <= val <= hi + 1e-9):
            issues.append(
                f"{label}: {name}={val} 越出 URDF 限位 [{lo:.4f}, {hi:.4f}] ({unit})"
                f" —— 规划时会 INVALID_GOAL/无解"
            )
        if is_finger and abs(val) > 1.0:
            issues.append(
                f"{label}: {name}={val} 看起来把 rad 当成了 m"
                "（prismatic 手指行程只有 0.040 m）"
            )

for name, joints in states.items():
    check(f"SRDF group_state '{name}'", "", joints)
for i, wp in enumerate(wps, 1):
    check(f"waypoints[{i}] {wp.get('name')}", wp.get("group", ""), wp.get("joints", {}))

# 规则 3：waypoints 的 group 与关节集合要匹配（arm 组只该有 6 个臂关节）
ARM = {f"joint{n}" for n in range(1, 7)}
FIN = {"finger1_joint", "finger2_joint"}
for i, wp in enumerate(wps, 1):
    keys = set(wp.get("joints", {}))
    g = wp.get("group")
    if g == "arm" and not keys <= ARM:
        issues.append(f"waypoints[{i}] group=arm 但含非臂关节 {sorted(keys - ARM)}")
    if g == "gripper" and not keys <= FIN:
        issues.append(f"waypoints[{i}] group=gripper 但含非指关节 {sorted(keys - FIN)}")

print(f"    URDF 可控关节 {len(lim)} 个 / joint_limits.yaml {len(jl)} 个 / "
      f"SRDF 命名位形 {len(states)} 个 / waypoints {len(wps)} 条")
for name, v in sorted(lim.items()):
    print(f"    {name:<14} {v['type']:<10} "
          f"[{v['lower']:+.4f}, {v['upper']:+.4f}] v<={v['velocity']:.2f} e<={v['effort']:.1f}")

if issues:
    for it in issues:
        print(f"  ISSUE {it}")
    print(f"CHECK-RESULT FAIL ({len(issues)} issues)")
    sys.exit(1)
print("CHECK-RESULT PASS (速度取小值一致 / 所有目标在限位内 / group 与关节集合匹配)")
PY
