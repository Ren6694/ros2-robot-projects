#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/one_shot_demo.sh
#
#  作用: **给人在桌面上双击用的**一键演示。
#        起栈（带 RViz）→ 提权 → 跑一遍 9 个点位 → 收摊 → 打印结果行。
#        全程只需要一次点击；跑完 RViz 留着给人看，终端等 180 s（或按回车）后自动收摊。
#
#  和另外两个脚本的分工（为什么要三个，不是重复）：
#    run_waypoint_demo.sh  无头、自带起栈收栈，是**回归判据**用的（DEMO-RESULT）。
#    gui_bringup.sh        起一套**常驻**的带 RViz 栈后退出，用于手工调试；
#                          它收尾还带 list_controllers + topic hz 两个探测，
#                          白等 30~50 s —— 双击演示不该等这个。
#    one_shot_demo.sh      本脚本：把"常驻 + 手工再敲第二条"合成一次点击，
#                          并且**自己把栈收干净**，不留孤儿进程。
#
#  为什么这里要自己判"就绪"而不是靠节点的 wait_ready：
#    点位节点确实会轮询 /move_action 与 /joint_states（run_waypoint_demo.sh 头注
#    解释过），但**带 RViz 起栈时机器人模型先到、控制器后到**，
#    节点在"控制器还没 ACTIVE"的窗口里发起执行会得到一个跟被测内容无关的失败。
#    桌面双击的人没有第二只眼睛看日志，所以这里显式等 arm6_controller ACTIVE。
#
#  收摊为什么用 `kill -- -PID`（负号 = 整个进程组）：
#    栈是 `setsid nohup ros2 launch …` 起的，脚本里 job control 关着，
#    setsid 直接 exec ⇒ $! 就是新会话首进程的 pid ⇒ 负号一次带走
#    launch + 它 spawn 的 move_group / ros2_control_node / rviz2。
#    后面那几条 pkill 只是兜底，并且一律加方括号（[r]os2）——
#    `pkill -f ros2_control_node` 会把发起它的 SSH 会话一起杀掉（实测踩过）。
#
#  退出码: 0 = 演示 PASS；1 = 任一步没成（结果行里写清楚卡在哪一步）。
#  可调: STAY=1 跑完**不**收栈（想在 RViz 里接着手动拖点位时用）。
#        READ_TTL=<秒> 改收尾等待时长（默认 180；测试时给个小值即可验证自动收摊）。
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
STAY="${STAY:-0}"
cd "$WS" || { echo "ONESHOT-RESULT FAIL step=cd reason=no_ws"; exit 1; }
mkdir -p logs

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

# shellcheck disable=SC1091
source scripts/ros_env.sh gui

STAMP="$(date +%Y%m%d-%H%M%S)"
BLOG="logs/oneshot_bringup_${STAMP}.log"
MLOG="logs/oneshot_motion_${STAMP}.log"

echo "ONESHOT start run_id=$STAMP display=$DISPLAY log=$BLOG"

# ---- [1/5] 带 RViz 起栈（常驻到本脚本结束）------------------------------------
setsid nohup ros2 launch arm_moveit_config moveit.launch.py > "$BLOG" 2>&1 < /dev/null &
BPID=$!
echo "ONESHOT bringup pid=$BPID（日志实时写 $BLOG）"

teardown() {
    [ "$STAY" = "1" ] && { echo "ONESHOT STAY=1：栈保留，RViz 可以继续玩；收摊用 kill -- -$BPID"; return; }
    echo "ONESHOT teardown: 收栈"
    kill -TERM -- "-$BPID" 2>/dev/null || true
    sleep 3
    pkill -f '[r]os2 launch arm_moveit_config' 2>/dev/null || true
    pkill -x '[r]viz2' 2>/dev/null || true
    pkill -f '[r]viz2' 2>/dev/null || true
}
trap teardown EXIT

# ---- [2/5] 等控制器 ACTIVE --------------------------------------------------
# ★判据来源是 **bringup 日志里的 spawner 成功行**，不是 `ros2 control list_controllers`。
#   上一版用后者，实跑挂了：spawner 在起栈 4 s 内就打印了
#       [spawner-5] ... Configured and activated arm6_controller
#   而从栈外另起一个 ros2 CLI 去查同一个 service 却
#       [WARN] Could not contact service /controller_manager/list_controllers
#   —— 跨进程 DDS 发现（还要过 ros2cli daemon）在"脚本刚起、栈刚起"这个窗口里是不
#   稳定的，于是就绪判据永远不满足，桌面双击的人看到的是**卡住**。
#   教训：能用本地日志字符串判的就不要引入一次跨进程发现；后者会把"栈正常"误判成
#   "没就绪"，而这类误判在人的观感上等价于程序坏了。
READY=0
for i in $(seq 1 45); do
    if grep -q 'Configured and activated arm6_controller' "$BLOG" 2>/dev/null \
       && grep -q 'Configured and activated joint_state_broadcaster' "$BLOG" 2>/dev/null; then
        READY=1
        echo "ONESHOT ready waited_s≈$((i * 2)) gate=bringup_log/controllers_activated"
        break
    fi
    # 栈要是半路死了就别干等：早退，把日志尾巴交出来
    kill -0 "$BPID" 2>/dev/null || { echo "ONESHOT-RESULT FAIL step=bringup reason=process_died log=$BLOG"; tail -20 "$BLOG"; exit 1; }
    sleep 2
done
[ "$READY" = "1" ] || { echo "ONESHOT-RESULT FAIL step=controllers reason=not_active_90s log=$BLOG"; grep -E '\[(ERROR|WARN)\]' "$BLOG" | tail -10; exit 1; }

# ---- [3/5] 控制环提权（虚拟机里不提权必被宿主丢拍打断，见 rt_tune.sh 头注）----
RT_OUT="$(bash scripts/rt_tune.sh 80 90)"; RT_RC=$?
echo "$RT_OUT"
[ "$RT_RC" -ne 0 ] && echo "ONESHOT warn: 提权未全生效，执行阶段可能触发 PATH_TOLERANCE_VIOLATED（有 replan 兜底，不中止）"

# RViz 存活情况（只报状态、不中止）：看日志里 rviz2 那个子进程有没有提前退出。
# 这里刻意不用 pgrep —— 本轮排查时 rviz 明明在日志里活着、pgrep 却数到 0，
# 拿一个不可靠的探针当判据只会制造假故障（详见 [2/5] 那段教训）。
if grep -qE '\[rviz2-[0-9]+\]: process has (finished cleanly|died)' "$BLOG" 2>/dev/null; then
    echo "ONESHOT warn: RViz 已退出，桌面上看不到动画；点位仍会执行，结论照旧落在日志/CSV 里"
else
    echo "ONESHOT rviz=up（请看桌面上那个 RViz 窗口）"
fi

# ---- [4/5] 跑点位（默认参数 = 已验收的高速档 1.0/1.0/夹爪 1.0，replan 4）------
echo "ONESHOT moving: 9 个点位，执行约 11 s，请看 RViz 窗口"
ros2 launch arm_demo waypoint_mover.launch.py \
    execute:=true \
    log_dir:="$WS/logs" \
    run_id:="oneshot_${STAMP}" \
    > "$MLOG" 2>&1
NRC=$?

# ---- [5/5] 判定 + 把结论说人话 ------------------------------------------------
SUM="$(grep -oE 'MOTION-SUMMARY .*' "$MLOG" | tail -1)"
CSV="logs/motion_oneshot_${STAMP}.csv"
if [ "$NRC" -eq 0 ] && printf '%s\n' "$SUM" | grep -q 'RESULT PASS'; then
    echo "ONESHOT $SUM"
    echo "ONESHOT-RESULT PASS run_id=oneshot_${STAMP} csv=$CSV blog=$BLOG mlog=$MLOG"
    RC=0
else
    echo "ONESHOT ${SUM:-MOTION-SUMMARY missing rc=$NRC}"
    echo "ONESHOT-RESULT FAIL run_id=oneshot_${STAMP} rc=$NRC mlog=$MLOG"
    echo "--- motion 日志尾部 30 行 ---"; tail -30 "$MLOG"
    RC=1
fi

# 桌面双击时窗口会随脚本退出而关掉，所以留一步让人看结果；手工在终端跑时不打扰。
# ★为什么这个等待**必须带超时**（本轮实测复现）：RViz 是盖在终端上面的，
#   人常常直接关 RViz 或者走开，于是无限等待的 read 把整套栈一直挂在后台 ——
#   实测一次 PASS 之后 2 分钟，move_group / rviz2 / ros2_control_node 还在跑。
#   这台虚拟机上 xdotool 和 wmctrl 都没有，没法把终端提到前面，
#   所以只能"提示怎么切回来 + 180 s 后自动收摊"。
if [ -t 0 ]; then
    printf '\n=== 演示结束（%s）===\n' "$([ "$RC" = 0 ] && echo PASS || echo FAIL)"
    echo '结果行就在上面这几屏。RViz 窗口还开着，可以继续转着看。'
    echo '要看终端：点左侧 Dock 的终端图标，或 Alt+Tab 切回来。'
    read -t "${READ_TTL:-180}" -r _ || echo "ONESHOT note: ${READ_TTL:-180} s 无输入，自动收摊"
fi
exit "$RC"
