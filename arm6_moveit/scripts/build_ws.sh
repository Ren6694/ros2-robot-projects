#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/build_ws.sh   (在 WSL 里跑)
#  作用: 用固定参数构建 ~/ros2_ws，并把完整输出留成一份可引用的日志
#
#  为什么把一条 colcon 命令包成脚本（三个都是真实踩过的点）：
#    * --event-handlers console_direct- console_cohesion+ 不是强迫症：
#      默认的 console_direct 会把三个包的输出交织打印，报错时看不出是哪个包炸的。
#    * 每次都记下 rc：`colcon build` 失败时返回非 0，但 shell 脚本里如果只
#      `tail` 一下就会把失败吞掉；本项目要求"报错要么自动修，要么给完整排查"，
#      所以 rc 必须显式落到日志和退出码上。
#    * 输出进 logs/colcon_build_<时间>.log：文档里可以引用"第 2 次构建用了多少秒、
#      哪几个包被重新编译"，而不是只写"构建成功"。
#
#  用法: bash scripts/build_ws.sh [额外的 colcon 参数...]
#        bash scripts/build_ws.sh --packages-select arm_demo     # 只重建一个包
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
cd "$WS"
mkdir -p logs
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="logs/colcon_build_${STAMP}.log"

# shellcheck disable=SC1091
source scripts/ros_env.sh

echo ">>> colcon build (日志: $LOG)"
T0=$(date +%s)
colcon build \
    --event-handlers console_direct- console_cohesion+ \
    --base-paths src \
    "$@" \
    > "$LOG" 2>&1
RC=$?
T1=$(date +%s)

echo ">>> rc=$RC  用时 $((T1 - T0))s"
# 只看每个包的结尾与所有告警/错误，避免把 200 行 CMake 输出糊到终端上
grep -E "^(Summary|  -|---)|Starting >>>|Finished <<<|Failed <<<" "$LOG" | tail -20
if [ "$RC" -ne 0 ]; then
    echo ">>> 构建失败，错误上下文："
    grep -nE -B3 -A12 "(^| )(Error|ERROR|CMake Error|error:|Traceback)" "$LOG" | head -60
fi

# 结果行：本脚本原来是唯一没有 `*-RESULT` 的脚本，而 docs/02 开头承诺
# "每个脚本都自带结果行，退出码即验收结论"。rc 其实已经能表达成败，但仍要
# 单独一行 BUILD-RESULT，原因是：本脚本的输出常被并进别的日志一起看，
# `>>> rc=0` 混在 colcon 的 Starting/Finished 里不好 grep，而文档引用验收
# 结论时希望有一个跨脚本同形的锚点（BUILD/CHECK/PROBE/DEMO/CAPTURE/COMPARE）。
# 退出码照旧等于 colcon 的 rc —— 结果行是给人看的补充，不是替代。
if [ "$RC" -ne 0 ]; then
    echo "BUILD-RESULT FAIL rc=$RC log=$LOG"
else
    echo "BUILD-RESULT PASS $(grep -oE 'Summary: .*' "$LOG" | tail -1) elapsed=$((T1 - T0))s log=$LOG"
fi
exit "$RC"
