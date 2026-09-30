#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/probe_moveit_interfaces.sh   (在 WSL 里跑，也可从 Windows 侧 ssh)
#  作用: 把 move_group 真正暴露的 action / service / topic 抓成一份文本证据
#
#  为什么要这份脚本（这是本项目的方法论，不是多余的仪式感）：
#    MoveIt 2 的 Python 绑定了有三套互相不兼容的接口，能不能用完全取决于装了哪些 deb：
#      1) moveit_commander.MoveGroupCommander  —— 老接口，需 moveit_commander 包
#      2) moveit.planning.MoveItPy             —— 新接口，需 moveit 的 pybind11 组件
#      3) rclpy + moveit_msgs/action/MoveGroup —— 消息级接口，只要 move_group 在跑就一定有
#    本机实测：apt 索引(2026-09-13, 清华镜像)里查不到 ros-jazzy-moveit-commander，
#    dpkg 也确认 /opt/ros/jazzy/lib/python3.12/site-packages 下没有 moveit 目录，
#    于是 1) 2) 都 import 不了。与其为了一个 wrapper 去源码编译，不如直接用 3)：
#    它把"规划"和"执行"两个 action 显式写出来，日志里能拿到 plan 耗时、轨迹时长、
#    收敛误差，评审时能看懂 —— 而且零新增依赖。
#
#    但 3) 的前提是搞清楚 move_group 的 action 名字和类型。这个不能靠记忆：
#    Jazzy 上 MoveGroup 的 action 名字随 launch 参数变化（/move_group/MoveGroup vs
#    /move_group），execute 走的是另一个 action。所以本脚本把图抓下来再写节点。
#
#  用法: bash scripts/probe_moveit_interfaces.sh [等待秒数，默认 18]
#  产出: logs/moveit_interfaces_<stamp>.txt   （给文档引用的证据）
#        logs/probe_launch_<stamp>.log       （整套 bringup 的原始输出）
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
WAIT="${1:-18}"
cd "$WS"
mkdir -p logs

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="logs/moveit_interfaces_${STAMP}.txt"
PLOG="logs/probe_launch_${STAMP}.log"

# 环境准备（source ROS + 本工程 install、COLCON_TRACE 兜底、ROS_LOCALHOST_ONLY）见 ros_env.sh
# shellcheck disable=SC1091
source scripts/ros_env.sh

echo ">>> [1/4] 后台拉起整套仿真（use_rviz:=false，${WAIT}s 后探测，60s 自动收摊）"
timeout 60 ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false > "$PLOG" 2>&1 &
LPID=$!
sleep "$WAIT"

echo ">>> [2/4] 刷新 ros2 daemon 并抓图"
# daemon 有缓存，节点刚起就查会漏；stop/start 一次比 --no-daemon 反复扫全盘快
ros2 daemon stop > /dev/null 2>&1 || true
ros2 daemon start  > /dev/null 2>&1 || true
sleep 3

{
    echo "# move_group 接口快照  generated=$(date '+%F %T')  ros_distro=$ROS_DISTRO"
    echo "# 命令: ros2 node list / action list -t / service list -t / topic list -t"
    echo
    echo "## ros2 node list"
    ros2 node list 2>&1
    echo
    echo "## ros2 action list -t"
    ros2 action list -t 2>&1
    echo
    echo "## ros2 service list -t  (只保留 plan/execute/kinematics/状态相关)"
    ros2 service list -t 2>&1 | grep -Ei "plan|execute|kinemat|state|position|survey" || true
    echo
    echo "## ros2 topic list -t  (只保留 moveit/joint/robot 相关)"
    ros2 topic list -t 2>&1 | grep -Ei "moveit|joint|robot|planning|tf" || true
} > "$OUT" 2>&1

echo ">>> [3/4] 控制器与关节状态抽查"
{
    echo
    echo "## ros2 control list_controllers"
    # 注意：Jazzy 的 ros2controlcli 子命令是**下划线**。写成 list-controllers 会
    #   ros2 control: error: ... invalid choice: 'list-controllers'
    # 这个错误在 logs/moveit_interfaces_20260916-230333.txt:69-73 里留了原样证据。
    ros2 control list_controllers 2>&1 || true
    echo
    echo "## /joint_states 前 2 行（确认 8 个关节都在广播）"
    timeout 6 ros2 topic echo /joint_states --once 2>&1 | grep -E "^position|^name" -A9 | head -24 || true
} >> "$OUT" 2>&1

echo ">>> [4/4] 收摊"
kill "$LPID" 2>/dev/null || true
wait "$LPID" 2>/dev/null || true

echo "--- $OUT ---"
cat "$OUT"
echo "--- bringup 日志尾部 ---"
tail -12 "$PLOG"

# 结果行：抓到的条目数即"运行图是否完整"的判据。
# 任何一类为 0 都说明 daemon 缓存没刷新或 bringup 根本没起来 —— 那种情况下
# 文件照样会生成（cat 出来一片空），所以必须数一数，不能只看文件在不在。
NA=$(sed -n '/^## ros2 action list/,/^$/p' "$OUT" | grep -c '^/' || true)
NS=$(sed -n '/^## ros2 service list/,/^$/p' "$OUT" | grep -c '^/' || true)
NT=$(sed -n '/^## ros2 topic list/,/^$/p' "$OUT" | grep -c '^/' || true)
if [ "${NA:-0}" -ge 2 ] && [ "${NS:-0}" -ge 3 ] && [ "${NT:-0}" -ge 5 ]; then
    echo ">>> PROBE-RESULT PASS actions=$NA services=$NS topics=$NT out=$OUT"
else
    echo ">>> PROBE-RESULT FAIL actions=$NA services=$NS topics=$NT out=$OUT"
    echo "    先 ros2 daemon stop && ros2 daemon start，再确认 bringup 日志里没有 FATAL"
    exit 1
fi
