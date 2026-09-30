#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/rt_tune.sh
#
#  作用: 把 ros2_control_node 的所有线程提到 SCHED_FIFO（实时调度），
#        并打印一行机器可读的 RT-TUNE 判据，供日志/回归脚本判定。
#
#  为什么需要（2026-09-17 VirtualBox 实测）：
#    ros2_controllers.yaml 里 controller_manager 的 update_rate 是 100 Hz。
#    在普通 CFS 调度下，虚拟机里这个循环会被宿主随机摘下去几百毫秒，日志表现为：
#        [controller_manager]: Overrun detected! ... missed cycles : 30
#    一次超周期 ⇒ 硬件状态相对轨迹参考点滞后一拍。mock 硬件的 state = 上一拍的
#    command，而 JTC 比较的是"当前时刻的参考位置"，于是滞后被算成跟踪误差：
#        [tolerances]: Position Error: 0.057749, Position Tolerance: 0.050000
#        [arm6_controller]: Aborted due to state tolerance violation
#        PATH_TOLERANCE_VIOLATED  →  MoveIt ExecuteTrajectory = CONTROL_FAILED(-4)
#    提权到 SCHED_FIFO 后：超周期 0 次/60 s，/joint_states 从 48 Hz 回到 100 Hz，
#    与 WSL 上观测到的 98 Hz 对齐 —— 也就是说这条差异是**调度**问题，不是模型问题。
#
#  为什么用 chrt 而不是 launch 的事件处理器：
#    ros2_control_node 由 MoveIt 的 generate_demo_launch() 内部创建，我们的
#    moveit.launch.py 拿不到它的 Process 对象，挂不上 ProcessStarted 回调。
#    所以在拉起 bringup 之后按 /proc/<pid>/task 逐个线程设策略，幂等、可复跑。
#
#  用法:
#    bash scripts/rt_tune.sh              # 等进程出现，默认优先级 80
#    bash scripts/rt_tune.sh 80 60        # 自定义优先级 / 最长等待秒数
#    RT_TUNE_PRIO=90 bash scripts/rt_tune.sh
#
#  退出码: 0 = 已生效并通过校验；1 = 没等到进程或校验不通过（调用方决定要不要继续）
# =============================================================================
set -uo pipefail

PRIO="${1:-${RT_TUNE_PRIO:-80}}"
WAIT="${2:-${RT_TUNE_WAIT:-45}}"
SUDO="${RT_TUNE_SUDO:-sudo}"

deadline=$((SECONDS + WAIT))
PID=""
while [ "$SECONDS" -lt "$deadline" ]; do
    # comm 只有 15 个字符：ros2_control_node 在 /proc 里是 "ros2_control_no"，
    # 所以 pgrep -x ros2_control_node 恒为 0 命中（实测踩过：RT-TUNE FAIL reason=no_process）。
    # 先按截断名精确匹配，再用完整命令行兜底，两条都不中才算没等到。
    PID="$(pgrep -x ros2_control_no 2>/dev/null | head -1 || true)"
    [ -z "$PID" ] && PID="$(pgrep -x ros2_control_node 2>/dev/null | head -1 || true)"
    [ -z "$PID" ] && PID="$(pgrep -f 'lib/ros2_control_node/ros2_control_node' 2>/dev/null | head -1 || true)"
    [ -n "$PID" ] && break
    sleep 0.5
done

if [ -z "$PID" ]; then
    echo "RT-TUNE result=FAIL reason=no_process wait_s=${WAIT}"
    exit 1
fi

# 提权必须**多轮**：一次快照线程列表再逐个 chrt 会漏 —— ros2_control_node 的
# DDS / executor 线程在提权之后还在继续新建。实测第一轮后 22 个线程里只有 14 个
# 是 FF，剩下 8 个是快照之后才出现的，报出来的 PARTIAL 其实是竞态不是权限问题。
# 所以每轮都重新扫 /proc、对当前全部线程重复 chrt（幂等），最多 4 轮。
PASSES=0
APPLIED=0
FIFO=0
TOTAL=0
CLS_DIST=""
while [ "$PASSES" -lt 4 ]; do
    PASSES=$((PASSES + 1))
    TIDS=()
    for t in "/proc/$PID/task"/*; do
        [ -d "$t" ] && TIDS+=("$(basename "$t")")
    done
    [ "${#TIDS[@]}" -eq 0 ] && break

    # 不做"已经是 FF 就跳过"的预判：这台机器的 procps 不认 ps --tid
    # （实测报 unknown gnu long option），预判只会恒为空。重复 chrt 是幂等的。
    for t in "${TIDS[@]}"; do
        $SUDO chrt -f -p "$PRIO" "$t" >/dev/null 2>&1 && APPLIED=$((APPLIED + 1))
    done

    # 校验用 ps 的线程调度类别：FF = SCHED_FIFO，TS = SCHED_OTHER。
    # 两个坑都踩过，所以固定成"一次取全部线程"：
    #   1) /proc/<tid>/stat 的调度策略是第 43 列不是第 40 列（40 是 exit_signal）；
    #   2) 逐线程 ps --tid -o cls= 在部分 procps 版本上返回空，
    #      于是出现 dist=[ 22 ] 这种"有计数、没内容"的假摘要。
    #      这里 -T -p <pid> 一次取全部线程，--tid 只用于上面的单线程预判。
    CLS_LIST="$(LC_ALL=C ps -o cls= -T -p "$PID" 2>/dev/null | tr -d ' ' | grep -v '^$' || true)"
    TOTAL="$(printf '%s\n' "$CLS_LIST" | grep -c . || true)"
    FIFO="$(printf '%s\n' "$CLS_LIST" | grep -c '^FF$' || true)"
    CLS_DIST="$(printf '%s\n' "$CLS_LIST" | sort | uniq -c | tr '\n' ' ' | sed 's/  */ /g')"

    [ "$TOTAL" -gt 0 ] && [ "$FIFO" -eq "$TOTAL" ] && break
    sleep 0.5
done

if [ "$TOTAL" -gt 0 ] && [ "$FIFO" -eq "$TOTAL" ]; then
    echo "RT-TUNE result=OK pid=$PID threads=$FIFO policy=SCHED_FIFO prio=$PRIO passes=$PASSES chrt=$APPLIED dist=[$CLS_DIST]"
    exit 0
fi

echo "RT-TUNE result=PARTIAL pid=$PID total=$TOTAL fifo=$FIFO chrt_ok=$APPLIED prio=$PRIO passes=$PASSES dist=[$CLS_DIST]"
echo "RT-TUNE hint=已跑满 ${PASSES} 轮仍非全 FF：线程竞态已被排除，剩两种可能 —— sudo 非免密(执行 sudo -n true 验证)，或 RT 时间片被限制(cat /proc/sys/kernel/sched_rt_runtime_us)"
exit 1
