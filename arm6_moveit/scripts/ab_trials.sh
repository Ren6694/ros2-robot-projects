#!/usr/bin/env bash
# =============================================================================
#  环境 A/B 用的采样器：在**一套常驻栈**上连跑 N 轮点位演示。
#
#  为什么不用 scripts/run_waypoint_demo.sh 做 A/B（2026-09-17 的教训）：
#    那个脚本自带起栈+收栈。连着跑时上一场的进程还没退干净、下一场已经在抢
#    DDS 端口和 CPU，两轮互相污染 —— 第一批 A/B 里那两次 arr_err 0.77 rad 的
#    FAIL 就是这么来的，跟被测变量毫无关系。
#    调参期间栈必须常驻，只重启点位节点。
#
#  ★本脚本**不写任何缩放/额度默认值**。第一版写了 VSCALE:-0.5，结果"测默认值"
#    那一批量到的其实还是 0.5 —— 采样器和被测程序各存一份常数，就永远测不到
#    launch 里的默认值。现在没显式给环境变量就一个参数都不传，并在行首打
#    vscale=INHERITED 提醒这一批用的确实是默认值。
#
#  测什么：
#    ov     = 本轮期间 controller_manager 的 Overrun 次数（数**新增**行）
#    worst  = 本轮期间最坏一次 loop 用时（ms）。第一版是拿整份 blog 求最大值，
#             于是这个数字只随时间单调变大、分不出哪轮贡献的，等于没有。
#    其余   = MOTION-SUMMARY 原样落盘（plan_ok/exec_ok/replans/s 与路径长度）
#
#  用法：
#    BLOG=~/ros2_ws/logs/gui_bringup_<stamp>.log N=6 bash scripts/ab_trials.sh
#    ... N=6 VSCALE=0.5 ASCAL=0.5 bash scripts/ab_trials.sh     # 显式覆盖才传参
#  注意：本脚本**不**提权。提权是栈级前提，由 gui_bringup.sh 里的 rt_tune.sh 负责，
#        每轮重复提权会把新建线程的分布搅乱，反而看不出差异。
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
N="${N:-6}"
BLOG="${BLOG:?BLOG=常驻栈日志路径 必须给}"
TAG="${TAG:-ab}"

shown() { if [ -n "$1" ]; then printf '%s' "$1"; else printf 'INHERITED'; fi; }

ARGS=()
[ -n "${VSCALE:-}" ]  && ARGS+=("max_velocity_scaling_factor:=$VSCALE")
[ -n "${ASCAL:-}" ]   && ARGS+=("max_acceleration_scaling_factor:=$ASCAL")
[ -n "${GVSCALE:-}" ] && ARGS+=("gripper_velocity_scaling_factor:=$GVSCALE")
[ -n "${GASCAL:-}" ]  && ARGS+=("gripper_acceleration_scaling_factor:=$GASCAL")
[ -n "${REPLANS:-}" ] && ARGS+=("replan_retries:=$REPLANS")

cd "$WS"
mkdir -p logs
# shellcheck disable=SC1091
source scripts/ros_env.sh

[ -f "$BLOG" ] || { echo "AB-TRIALS ERROR blog_missing=$BLOG"; exit 1; }

echo "AB-TRIALS start tag=$TAG n=$N blog=$BLOG vscale=$(shown "${VSCALE:-}") gvscale=$(shown "${GVSCALE:-}") replans=$(shown "${REPLANS:-}")"
echo "AB-TRIALS baseline overruns_total=$(grep -c 'Overrun detected' "$BLOG")"

for i in $(seq 1 "$N"); do
    STAMP="$(date +%Y%m%d-%H%M%S)"
    RLOG="$WS/logs/${TAG}_run_${STAMP}.log"
    # 按字节偏移切窗口：本轮只统计这之后的新增日志，ov / worst 才真是"本轮的"。
    OFF0="$(wc -c < "$BLOG")"
    T0="$(date +%s.%N)"

    timeout 300 ros2 launch arm_demo waypoint_mover.launch.py \
        execute:=true \
        "${ARGS[@]}" \
        log_dir:="$WS/logs" \
        run_id:="${TAG}_${i}_${STAMP}" \
        > "$RLOG" 2>&1
    RC=$?

    T1="$(date +%s.%N)"
    SEG="$(tail -c +"$((OFF0 + 1))" "$BLOG" 2>/dev/null || true)"
    OV="$(printf '%s\n' "$SEG" | grep -c 'Overrun detected' || true)"
    WORST="$(printf '%s\n' "$SEG" | grep -o 'loop took [0-9.]* ms' | sed 's/[^0-9. ]//g' \
             | awk '{if($1>w)m=$1} END{printf "%.0f", m+0}')"
    WALL="$(awk -v a="$T0" -v b="$T1" 'BEGIN{printf "%.2f", b-a}')"
    SUM="$(grep -oE 'MOTION-SUMMARY .*' "$RLOG" | tail -1)"
    [ -n "$SUM" ] || SUM="MOTION-SUMMARY missing rc=$RC"

    printf 'AB-TRIALS i=%s rc=%s wall_s=%s ov=%s worst_loop_ms=%s %s\n' \
        "$i" "$RC" "$WALL" "$OV" "$WORST" "$SUM"
done

echo "AB-TRIALS done tag=$TAG overruns_total=$(grep -c 'Overrun detected' "$BLOG")"
