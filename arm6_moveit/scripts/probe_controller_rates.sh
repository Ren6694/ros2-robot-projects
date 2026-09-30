#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/probe_controller_rates.sh   (在 WSL 里运行)
#  作用: 实测三个"频率类"参数的**真实生效值**，写成日志供文档引用
#
#  为什么要单独测这个（不是顺手，是被 yaml 骗过一次）：
#    ros2_controllers.yaml 里同时写了
#        controller_manager.update_rate: 100   （控制环周期）
#        arm6_controller.state_publish_rate: 50（名义上的状态发布节流）
#    但 waypoint_mover 的日志里 track_samples / exec_s 恒为 ~100.2 Hz
#    （9 个点位全部如此，见 logs/motion_*.jsonl）。
#    也就是说 ~/controller_state 这条误差信号**没有**被 state_publish_rate 节流。
#    这件事直接决定两件事，必须落到实测而不是猜：
#      1) 跟踪误差峰值的时间分辨率 —— 100 Hz ⇒ 10 ms 一格，
#         观测到的 Δt=0.44~0.92 ms 是"格内相位差"，不是"多个周期"；
#      2) 若按 yaml 以为只有 50 Hz，会低估采样密度、把丢拍判成正常。
#    判据：本脚本把 /joint_states、~/controller_state 的实测 Hz
#    与 update_rate 摆在一起，谁节流了谁没节流，一眼看清。
#
#  用法: bash scripts/probe_controller_rates.sh [ttl_s]
#  产物: logs/controller_rates_<stamp>.txt
# =============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TTL="${1:-150}"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$WS/logs/controller_rates_$STAMP.txt"
mkdir -p "$WS/logs"

# shellcheck disable=SC1091
source "$WS/scripts/ros_env.sh"

echo ">>> [1/3] 后台拉起 moveit（无 rviz），TTL=${TTL}s"
timeout "$TTL" ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false \
    > "$WS/logs/rates_bringup_$STAMP.log" 2>&1 &
BRINGUP=$!
cleanup() { kill "$BRINGUP" 2>/dev/null || true; wait "$BRINGUP" 2>/dev/null || true; }
trap cleanup EXIT

echo ">>> [2/3] 等 /arm6_controller/controller_state 出现（JTC 激活）"
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q '^/arm6_controller/controller_state$'; then
        echo "    就绪（${i}s）"; break
    fi
    sleep 1
    [ "$i" = 60 ] && { echo "    [FAIL] 60s 内没等到该 topic"; tail -30 "$WS/logs/rates_bringup_$STAMP.log"; exit 1; }
done
# 再多等 2s，让第一波消息稳定下来，否则 hz 的前几个统计窗口会被建链抖动污染
sleep 2

echo ">>> [3/3] 逐 topic 测频（每个 12s，window=100）"
# 注意：EXIT trap 只有一次生效，后面再 trap 会把"收掉后台 bringup"覆盖掉 —— 
# 所以临时文件并入 cleanup，不要另写一条 trap。
TMP="$(mktemp)"
cleanup() { rm -f "$TMP"; kill "$BRINGUP" 2>/dev/null || true; wait "$BRINGUP" 2>/dev/null || true; }
trap cleanup EXIT
{
    echo "# controller rates probe  $STAMP"
    echo "# update_rate(yaml)=100 Hz   state_publish_rate(yaml)=50 Hz"
    echo "# 采样窗口 12s / average window 100 —— 只保留最后一个统计窗口，避免逐行输出淹死日志"
    echo "# 注意：不能用 \`ros2 topic hz | tail || echo 无消息\` 判定 —— timeout 到点返回 124，"
    echo "#       会让\"明明有消息\"的 topic 也被打成\"无消息\"。必须先落盘再看有没有 average rate。"
    for T in /joint_states /arm6_controller/controller_state /arm6_controller/joint_trajectory; do
        timeout 12 ros2 topic hz "$T" --window 100 > "$TMP" 2>&1 || true
        echo
        echo "## $T"
        if grep -q 'average rate' "$TMP"; then
            tail -4 "$TMP"
            echo "RATE $T $(grep 'average rate' "$TMP" | tail -1 | awk '{print $3}')"
        else
            echo "    12s 内没有收到任何消息 —— 该 topic 只在有活动轨迹目标时才发布"
            grep -v '^\s*$' "$TMP" | tail -2 | sed 's/^/    /'
            echo "RATE $T IDLE-NO-PUB"
        fi
    done
} | tee "$OUT"

echo
echo ">>> 结论行（供文档直接引用）："
grep -E '^## |^RATE ' "$OUT" | sed 's/^/    /' || true
echo ">>> RATES-PROBE DONE out=$OUT"
