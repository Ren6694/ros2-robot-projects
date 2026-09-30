#!/usr/bin/env bash
# =============================================================================
#  文件: scripts/ros_env.sh   —— 本项目所有"在 WSL 里跑 ROS"的脚本共用的环境准备
#
#  为什么单独抽一个文件（三件事每件都真实踩过）：
#    1) source ROS 的 setup.bash 会引用未定义的 COLCON_TRACE，脚本开头写了
#       `set -u` 的话直接 "COLCON_TRACE: unbound variable" 退出。
#       → 只在 source 那两行临时 set +u，出来再 set -u，不牺牲脚本其余部分的严格性。
#    2) 从 Windows 侧 `ssh ros2vm` 进来的会话 DISPLAY 是空的，GUI（rviz2、joint
#       state publisher gui）会报 "cannot open display"。WSL 里 XFCE 桌面跑在 :0。
#       → 这里给 DISPLAY 兜底为 :0；无头脚本用不到，有头脚本不用再各自记得。
#    3) WSL2 的默认网卡是 NAT 虚拟网卡，FastDDS 有时会把发现流量绑到错误的接口上，
#       表现就是"同一个 ws 里 ros2 node list 看得见、跨终端看不见"。
#       → ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 强制走回环，单机多终端联调时
#         行为稳定且可复现。（Jazzy 前的写法是 ROS_LOCALHOST_ONLY=1，见下方注释）
#
#  用法: source scripts/ros_env.sh          （构建/无头脚本）
#        source scripts/ros_env.sh gui      （需要弹窗口的脚本）
# =============================================================================

WS="${WS:-$HOME/ros2_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
# shellcheck disable=SC1091
if [ -f "$WS/install/setup.bash" ]; then source "$WS/install/setup.bash"; fi
set -u

# Jazzy 起 ROS_LOCALHOST_ONLY 已弃用：仍然被 honoring，但**每个节点启动**都会打
#   [WARN] rcl: ROS_LOCALHOST_ONLY is deprecated ... Use ROS_AUTOMATIC_DISCOVERY_RANGE
# 早期日志里那一大片 WARN（如 logs/bringup_20260916-234501.log:8-13）就是它，
# 属于"噪声掩盖真问题"的典型：真正的 ERROR 被淹在 WARN 堆里。改用新变量。
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 回退到 Humble 时换成这一行（Humble 没有 AUTOMATIC_DISCOVERY_RANGE）：
#   export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# 关掉 rmw 的 INFO 噪声，日志里只留 WARN 以上，方便文档直接引用
export RCUTILS_LOGGING_SEVERITY="${RCUTILS_LOGGING_SEVERITY:-INFO}"

if [ "${1:-headless}" = "gui" ]; then
    export DISPLAY="${DISPLAY:-:0}"
fi

echo "ros_env: ROS_DISTRO=$ROS_DISTRO_NAME DISPLAY=${DISPLAY:-<none>} DOMAIN_ID=$ROS_DOMAIN_ID"
