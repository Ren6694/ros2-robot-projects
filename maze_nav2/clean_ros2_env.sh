#!/bin/bash
# ==============================================================
# ROS2 环境清理 + 启动辅助
# 解决 FastRTPS 共享内存残留导致的
#   [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXXX:
#   open_and_lock_file failed
# ==============================================================
# 用法:
#   clean_ros2_env.sh              仅清理残留
#   clean_ros2_env.sh <命令...>    清理后执行命令
# ==============================================================

clean() {
  local n
  # 注意：grep -c 在某些情况下会输出多行（如文件列表异常），需取首行并保证是纯数字
  n=$(ls /dev/shm/ 2>/dev/null | grep -cE "fastrtps|fastdds|sem\.fastrtps" | head -1 | tr -dc '0-9')
  [ -z "$n" ] && n=0
  if [ "$n" -gt 0 ] 2>/dev/null; then
    rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastrtps* 2>/dev/null
    echo "[clean] 已清理 $n 个 FastRTPS 共享内存残留文件"
  else
    echo "[clean] /dev/shm 干净，无需清理"
  fi
}

if [ $# -eq 0 ]; then
  clean
  exit 0
fi

clean
echo "[run] 执行: $*"
exec "$@"