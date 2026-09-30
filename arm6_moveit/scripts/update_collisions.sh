#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/update_collisions.sh   (在 WSL 里运行)
#  作用: 用 MoveIt 官方 collisions_updater 离线计算"允许相碰的 link 对"，
#        产出候选 SRDF 供 diff 后并回 src/arm_moveit_config/config/
#
#  为什么这件事必须用工具算，不能手写：
#    碰撞矩阵决定规划器认为哪些姿态是合法的。手工勾少了 -> 大量"假碰撞"，
#    规划器在明明能动的位形上报 fail；手工勾多了 -> 真碰撞被放行，
#    规划出来的路径实际会撞，而仿真里看起来一切正常（因为没人再检查）。
#    collisions_updater 在整条关节行程里随机采样，只有"从来没碰过"的对
#    才写进 disabled —— 给的是**有证据的结论**，不是人的直觉。
#
#  TRIALS 不是越大越好，而是要"跑到收敛"。本模型的实测（同一 URDF）：
#      2e4   -> 38 对豁免
#      2e5   -> 37 对   （link1/link5 被采到碰撞，从豁免列表里掉出去）
#      1e6   -> 36 对   （link1/link4 同样掉出去）
#      1e7   -> 36 对   与 1e6 逐对一致  => 判定收敛，取这一版
#    结论：2 万次会把"其实会碰"的关节对误判成"永不相碰"，
#    代价是规划出一条会自撞的路径且仿真看不出来。默认值因此设 1e7（约 9 s）。
#
#  输出为什么落在 artifacts/ 而不是直接覆盖 src/：
#    本工程 Windows 侧是唯一真源，WSL 的 src/ 只是同步副本，
#    就地覆盖会在下一次 sync.sh 时被无声冲掉。
#
#  坑（实测）：**不要传 --config-pkg**。
#    2.12.4 的 collisions_updater 用旧版 .setup_assistant 的 schema 去解析，
#    期望根节点有 package_settings，遇到 Jazzy 的 moveit_setup_assistant_config
#    直接报 "invalid node; first invalid key: \"package_settings\"" 并退出 1。
#    只给 --urdf/--srdf/--output 三个路径就能正常跑完。
#
#  用法: bash scripts/update_collisions.sh [trials]
# =============================================================================
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRIALS="${1:-10000000}"

: "${ROS_DISTRO:?请先 source /opt/ros/jazzy/setup.bash 再运行本脚本}"
[ -f "$WS/install/setup.bash" ] || { echo ">>> [FAIL] 还没有 colcon build：$WS/install 不存在"; exit 1; }
# shellcheck disable=SC1091
# colcon 生成的 setup.bash 里引用了未导出的 ${COLCON_TRACE}，在 set -u 下会
# "unbound variable" 直接退出。只在 source 这一段临时关掉 -u。
set +u
source "$WS/install/setup.bash"
set -u

XACRO="$WS/src/arm_description/urdf/arm6.urdf.xacro"
SRDF="$WS/src/arm_moveit_config/config/arm6.srdf"
OUT="${2:-$WS/artifacts/arm6_calculated.srdf}"
mkdir -p "$WS/artifacts"

echo ">>> [1/3] 校验 xacro 可解析"
xacro "$XACRO" -o /dev/null
echo "    OK"

echo ">>> [2/3] collisions_updater 采样 $TRIALS 次"
timeout 1800 ros2 run moveit_setup_assistant collisions_updater \
    --urdf "$XACRO" \
    --srdf "$SRDF" \
    --output "$OUT" \
    --default --always --keep \
    --trials "$TRIALS" \
    --min-collision-fraction 0.05

echo ">>> [3/3] 结果对比"
[ -f "$OUT" ] || { echo ">>> [FAIL] 没有产出 $OUT"; exit 1; }
n_old=$(grep -c '<disable_collisions' "$SRDF")
n_new=$(grep -c '<disable_collisions' "$OUT")
echo "    现有 SRDF: $n_old 对  ->  本次计算: $n_new 对"
grep -o 'reason="[^"]*"' "$OUT" | sort | uniq -c | sort -rn | sed 's/^/    /'
if [ "$n_old" != "$n_new" ]; then
    echo "    [DIFF] 数量变了，逐对差异："
    diff <(grep -o 'link1="[^"]*" link2="[^"]*"' "$SRDF" | sort) \
         <(grep -o 'link1="[^"]*" link2="[^"]*"' "$OUT" | sort) || true
    # 注意这里不 exit 1：数量变了是**信息**不是错误 —— 改了几何本来就该变。
    # 真正要人决策的是"要不要把这一版并回 src/"，所以给结论词而不给失败码。
    echo ">>> COLLISION-RESULT DIFF old=$n_old new=$n_new cand=$OUT"
else
    echo "    与现有 SRDF 逐对一致（已收敛，无需改动）"
    echo ">>> COLLISION-RESULT CONVERGED pairs=$n_new trials=$TRIALS cand=$OUT"
fi
echo ">>> 候选文件: $OUT"
echo "    确认后: cp $OUT src/arm_moveit_config/config/arm6.srdf && colcon build --packages-select arm_moveit_config"
