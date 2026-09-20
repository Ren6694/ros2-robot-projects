#!/bin/bash
# ==============================================================
# ROS2 + Gazebo 仿真启动器（含环境修复）
#
# 封装两个必须的环境设置（否则 DDS 发现会失败）：
#   ROS_DOMAIN_ID=42        统一 DDS 域，发布者与订阅者必须一致
#   ROS_LOCALHOST_ONLY=1    强制 DDS 走 loopback，绕开 WSL 虚拟网卡
#                           （WSL2 下 DDS 默认走 10.255.255.254，多播发现会失败）
#
# 用法：
#   start_sim.sh                      启动完整仿真（Gazebo GUI + RViz）
#   start_sim.sh headless             无界面（省资源，适合后台建图）
#   start_sim.sh mapping              建图模式（仿真 + Cartographer）
#   start_sim.sh navigate             导航模式（仿真 + AMCL + Nav2）
#   start_sim.sh robot               仅机器人状态（不启 Gazebo，调 URDF 用）
#   start_sim.sh clean                仅清理环境
# ==============================================================

WS=$HOME/maze_ws

# ---------- 环境（必须在 source 之前导出）----------
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
# 软渲染优化（WSLg 无 GPU 图形加速）
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# ---------- 清理残留 ----------
clean() {
  local n
  n=$(ls /dev/shm/ 2>/dev/null | grep -cE "fastrtps|fastdds" | head -1 | tr -dc '0-9')
  [ -z "$n" ] && n=0
  if [ "$n" -gt 0 ]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastrtps* 2>/dev/null
    echo "[clean] 已清理 $n 个 FastRTPS 残留"
  fi
  # 杀掉上次残留的仿真进程
  for p in "[g]zserver" "[g]zclient" "[r]obot_state_publisher" \
           "[s]pawn_entity" "[r]viz2" "[c]artographer_node"; do
    pkill -f "$p" 2>/dev/null
  done
}

# ---------- 加载 ROS 环境 ----------
load_ros() {
  cd $WS                    # 必须先 cd 再 source
  source /opt/ros/humble/setup.bash
  if [ -f $WS/install/setup.bash ]; then
    source $WS/install/setup.bash
  else
    echo "[XX] 未找到 $WS/install/setup.bash —— 请先 colcon build"
    exit 1
  fi
}

MODE="${1:-sim}"

case "$MODE" in
  clean)
    clean
    echo "[done] 环境已清理"
    ;;
  robot)
    clean; load_ros
    echo "[run] 机器人状态模式（不启 Gazebo）"
    echo "      ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    exec ros2 launch maze_bot robot_state.launch.py
    ;;
  headless)
    clean; load_ros
    echo "[run] 完整仿真（无界面）"
    echo "      ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    exec ros2 launch maze_bot simulate.launch.py gui:=false use_rviz:=false
    ;;
  mapping)
    clean; load_ros
    echo "[run] 建图模式（仿真 + Cartographer）"
    exec ros2 launch maze_bot mapping.launch.py
    ;;
  navigate)
    clean; load_ros
    echo "[run] 导航模式（仿真 + AMCL + Nav2）"
    exec ros2 launch maze_bot navigate.launch.py
    ;;
  sim|*)
    clean; load_ros
    echo "[run] 完整仿真（Gazebo GUI + RViz）"
    echo "      ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    exec ros2 launch maze_bot simulate.launch.py
    ;;
esac