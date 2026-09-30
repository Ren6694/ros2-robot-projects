#!/usr/bin/env bash
# 在虚拟机桌面上跑一次**看得见**的 MoveIt2 演示：带 RViz 起栈 + 控制环提权。
# 与 scripts/run_waypoint_demo.sh 的区别只有两个：use_rviz 打开、点位节点另起，
# 因为要看窗口，栈必须一直活着，不能让 runner 在收摊时把它 kill 掉。
set -uo pipefail
WS="${WS:-$HOME/ros2_ws}"
cd "$WS"

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
# 复用 ros_env.sh：它已经处理过"`set -u` 下 source setup.bash 会因
# AMENT_TRACE_SETUP_FILES 未绑定而当场退出"这个坑（本脚本第一版就栽在这儿）。
# shellcheck disable=SC1091
source scripts/ros_env.sh gui

STAMP="$(date +%Y%m%d-%H%M%S)"
BLOG="$WS/logs/gui_bringup_${STAMP}.log"
echo "GUI-BRINGUP log=$BLOG display=$DISPLAY xauth=$XAUTHORITY"

setsid nohup ros2 launch arm_moveit_config moveit.launch.py > "$BLOG" 2>&1 < /dev/null &
echo "GUI-BRINGUP pid=$!"

# 等 ros2_control_node 出现 -> 提权（虚拟机里不提权必被丢拍打断，见 rt_tune.sh）
bash scripts/rt_tune.sh 80 90

# 等 RViz 进程出现。真正的"看得见"不靠这里判断 —— 那是 VBoxManage
# controlvm screenshotpng 截屏的事；这里只要确认进程活着、别误报成"没起来"。
for i in $(seq 1 60); do
    if pgrep -f "rviz2" >/dev/null 2>&1; then
        echo "GUI-RVIZ process=up waited_s=$((i * 2))"
        command -v xdotool >/dev/null 2>&1 && \
            echo "GUI-RVIZ windows=$(DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" xdotool search --name "" getwindowname %@ 2>/dev/null | tr '\n' '|')"
        break
    fi
    sleep 2
done
pgrep -f "rviz2" >/dev/null 2>&1 || echo "GUI-RVIZ process=MISSING —— 检查 $BLOG 里的 [rviz-4]"

echo "GUI-BRINGUP controllers:"
timeout 30 ros2 control list_controllers 2>/dev/null | sed 's/^/  /'
echo "GUI-BRINGUP js_hz:"
timeout 20 ros2 topic hz /joint_states --window 30 2>/dev/null | grep average | head -1 | sed 's/^/  /'
echo "GUI-BRINGUP ready"
