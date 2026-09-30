#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/run_waypoint_demo.sh   (在 WSL 里跑)
#  作用: 一条命令完成"起仿真栈 → 跑点位节点 → 收摊 → 打印可判定的结果行"
#
#  进程编排为什么放在 shell 里而不是写成一个大 launch：
#    bringup（move_group + ros2_control + 控制器）要一直活着，点位节点跑完就该退出。
#    用 launch 表达"一个跑完就把另一个带走"要写 RegisterEventHandler，
#    调试期看到的只是一堆嵌套的 process 名字；用 shell 的反而是三行清清楚楚：
#      后台 timeout 起栈 -> 前台跑节点(它自己会等就绪) -> kill 栈
#    节点的 wait_ready_s 会轮询 /move_action 与 /joint_states，
#    所以这里不需要 sleep 猜"move_group 起没起"——那是最容易出假故障的地方。
#
#  产物（全部落在 ~/ros2_ws/logs，由 fetch_artifacts.sh 拉回 Windows）：
#    bringup_<stamp>.log      整套栈的启动日志（控制器激活、模型载入）
#    motion_run_<stamp>.log   点位节点的 stdout（含 MOTION / MOTION-SUMMARY 机器行）
#    demo_<stamp>.log         本次运行的判据落盘（mode + MOTION-SUMMARY + DEMO-RESULT）
#    motion_<stamp>.jsonl     每条点位的完整记录（含轨迹点），画图的数据源
#    motion_<stamp>.csv       每条点位一行的标量指标
#
#  为什么要单独留一份 demo_<stamp>.log（2026-09-17 实测教训）：
#    logs/motion_run_20260917-005329.log 里写着
#        MOTION-SUMMARY n=9 plan_ok=9 exec_ok=9 … RESULT PASS (ok)
#    而那次其实是 plan-only —— 没执行任何轨迹。plan-only 时节点有意把 exec_ok
#    记成 1（否则总结会被判 FAIL，见 waypoint_mover.py:435），所以 **exec_ok 在
#    plan-only 下是空洞为真**，不能当"执行成功"的证据。
#    当时能区分模式的只有终端上的 `DEMO-RESULT … mode=plan-only` 一行，
#    终端会滚走，文件里没留 ⇒ 证据链断在评审打不开的地方。
#    判据行必须落盘，于是有了这个文件；也补了 DEMO-RESULT 的 FAIL 分支
#    （以前失败只打印中文提示，没有任何机器可读行 —— 和 build_ws.sh 当初同一个毛病）。
#
#  用法:
#    bash scripts/run_waypoint_demo.sh                 # 规划 + 执行（默认）
#    bash scripts/run_waypoint_demo.sh plan-only       # 只规划：快、适合回归
#    bash scripts/run_waypoint_demo.sh full 240        # 给 bringup 更长寿命(秒)
#    VSCALE=0.2 ASCAL=0.2 bash scripts/run_waypoint_demo.sh full 300
#                                                     # 虚拟机：降速换跟踪余量，见 VSCALE 注释
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
MODE="${1:-full}"
BRINGUP_TTL="${2:-240}"
# 速度/加速度缩放。跟踪误差 ≈ 关节速度 × 控制环实际滞后，所以直觉上"虚拟机里
# 宿主会随机把 vCPU 摘下去几十到几百毫秒 ⇒ 降速就能把容差余量买回来"。
# ★这个直觉被同一套常驻栈上的对照实验否掉了（2026-09-17，6 vCPU，各 6 轮）：
#     VSCALE=0.5 : 6/6 PASS, s/rad 中位 1.457, max_track_err 最大 0.0405, replans 16
#     VSCALE=1.0 : 6/6 PASS, s/rad 中位 1.027, max_track_err 最大 0.04398, replans 10
#   速度翻倍，误差只涨约 8%，却白付 42% 的时间 —— 说明主导项是**停拍时长**
#   （停多久就错多少），不是"速度 × 固定滞后"。降速压根没在治这个病。
# 默认因此回到 1.0。代价说清楚：容差余量变薄（最差一轮 0.04923 / 0.05 = 98%），
# 越界就是 PATH_TOLERANCE_VIOLATED。之所以敢承担，是因为下面 REPLANS=4 已经把
# 一次越界的代价变成"重新规划再来"而不是整场 FAIL —— 实测越界确实全被兜住了。
VSCALE="${VSCALE:-1.0}"
ASCAL="${ASCAL:-1.0}"
# 夹爪组单独的缩放：手指关节的容差比臂更紧 —— ros2_controllers.yaml 里
# finger*_joint 的 trajectory 是 0.01 rad（臂是 0.05），而 URDF 给的
# finger max_velocity 只有 0.10 rad/s。于是一次丢拍的容忍窗口
#   = trajectory / (max_velocity * GVSCALE) = 0.01 / (0.10 * GVSCALE) 秒
# GVSCALE=1.0 → 100 ms。
# 这个公式是对的，但别拿它当"必须降速"的理由：早先那句"实测最坏 212 ms 正好打断"
# 来自冷启动 + 上一场进程没退干净的污染样本；干净开机、rt_tune 跑过两轮之后，
# 13 轮里最坏一次 loop 只有 55 ms（见本轮 cpu6/v10 两批），100 ms 的窗口根本没被碰到。
# 所以默认保持 1.0 —— 夹爪整段路径只有 0.03-0.06 rad，降速省不下什么，
# 而一次 replan 的代价（replan_wait 0.5 s + 重规划 + 重跑余段）≈ 1 s 以上。
GVSCALE="${GVSCALE:-1.0}"
GASCAL="${GASCAL:-1.0}"
# 控制环丢拍后允许"重新规划再执行"的次数（0 = 严格模式：虚拟机里必随机 FAIL，
# 用它来确认抖动是否还在，见 arm_demo/waypoint_mover.py 的循环注释）。
# 2 -> 4：热栈连跑 12 轮，额度 2 只有 7/12 PASS（打断全在臂的长行程点）。
REPLANS="${REPLANS:-4}"

cd "$WS"
mkdir -p logs
STAMP="$(date +%Y%m%d-%H%M%S)"
BLOG="logs/bringup_${STAMP}.log"
MLOG="logs/motion_run_${STAMP}.log"
DLOG="logs/demo_${STAMP}.log"

# say：判据行同时进终端和 $DLOG。
# 用 tee 而不是 >> 追加，是为了让"我看到的"和"评审看到的"必须是同一串字节。
say() { printf '%s\n' "$*" | tee -a "$DLOG"; }

: > "$DLOG"
say "run_id=$STAMP mode=$MODE bringup_ttl=${BRINGUP_TTL}s vscale=$VSCALE ascale=$ASCAL gvscale=$GVSCALE gascal=$GASCAL replans=$REPLANS"

# shellcheck disable=SC1091
source scripts/ros_env.sh

EXECUTE=true
[ "$MODE" = "plan-only" ] && EXECUTE=false

echo ">>> [1/4] 后台拉起 move_group + ros2_control（use_rviz:=false，寿命 ${BRINGUP_TTL}s）"
timeout "$BRINGUP_TTL" ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false \
    > "$BLOG" 2>&1 &
BPID=$!
# 不 sleep 等栈：节点的 wait_ready 轮询会替我们等，这里只保证超时后能把后台收干净
trap 'kill "$BPID" 2>/dev/null || true' EXIT

# [2/4] 控制环提权。plan-only 不下发轨迹，跟踪容差用不上，所以跳过省时间。
#       判据行必须落盘：虚拟机里 6/9 FAIL 的根因就是 100 Hz 控制环被宿主摘下去，
#       没有这行 RT-TUNE，评审只能看到 PATH_TOLERANCE_VIOLATED 而看不到前提。
if [ "$MODE" = "plan-only" ]; then
    say "RT-TUNE result=SKIPPED reason=plan_only"
else
    echo ">>> [2/4] 给 ros2_control_node 提权到 SCHED_FIFO（虚拟机里跑真执行必需）"
    # 先分别取输出和退出码：写成 say "$(...)" || say WARN 的话，退出码是 say 的
    # （printf|tee 恒为 0），WARN 分支永远不触发 —— 和上面 MOTION 行同一个坑。
    RT_OUT="$(bash scripts/rt_tune.sh 80 45)"
    RT_RC=$?
    say "$RT_OUT"
    [ "$RT_RC" -ne 0 ] && say "RT-TUNE result=WARN rc=$RT_RC —— 提权未全部生效，执行阶段可能复现 Overrun / PATH_TOLERANCE_VIOLATED"
fi

echo ">>> [3/4] 跑点位节点（mode=$MODE）"
ros2 launch arm_demo waypoint_mover.launch.py \
    execute:="$EXECUTE" \
    max_velocity_scaling_factor:="$VSCALE" \
    max_acceleration_scaling_factor:="$ASCAL" \
    gripper_velocity_scaling_factor:="$GVSCALE" \
    gripper_acceleration_scaling_factor:="$GASCAL" \
    replan_retries:="$REPLANS" \
    log_dir:="$WS/logs" \
    run_id:="$STAMP" \
    > "$MLOG" 2>&1
NRC=$?

echo ">>> [4/4] 收摊"
kill "$BPID" 2>/dev/null || true
wait "$BPID" 2>/dev/null || true
trap - EXIT

echo "--- MOTION 机器行（来自 $MLOG）---"
# 注意别写成 `grep … | tee … || say …`：管道的退出码是 tee 的（恒 0），
# grep 没匹配上时那个 || 分支根本不会触发 —— 于是"缺判据行"这件事又被静默吞掉。
MLINES="$(grep -oE "MOTION .*" "$MLOG" || true)"
[ -n "$MLINES" ] && printf '%s\n' "$MLINES" | tee -a "$DLOG" || say "  (没有 MOTION 行：节点没跑起来，见下方日志尾部)"
echo "--- 结果行 ---"
SLINE="$(grep -oE "MOTION-SUMMARY .*" "$MLOG" || true)"
[ -n "$SLINE" ] && printf '%s\n' "$SLINE" | tee -a "$DLOG" || say "  (没有 MOTION-SUMMARY)"
LLINE="$(grep -oE "MOTION-LOG .*" "$MLOG" || true)"
[ -n "$LLINE" ] && printf '%s\n' "$LLINE" | tee -a "$DLOG"

if [ "$NRC" -ne 0 ] || ! grep -q "RESULT PASS" "$MLOG"; then
    say ">>> DEMO-RESULT FAIL mode=$MODE rc=$NRC log=$MLOG"
    echo
    echo ">>> 判定失败，节点日志尾部 40 行："
    tail -40 "$MLOG"
    echo ">>> bringup 日志里的 ERROR/WARN："
    grep -E "\[(ERROR|WARN)\]" "$BLOG" | tail -15
    exit 1
fi

say ">>> DEMO-RESULT PASS mode=$MODE jsonl=logs/motion_${STAMP}.jsonl csv=logs/motion_${STAMP}.csv"
