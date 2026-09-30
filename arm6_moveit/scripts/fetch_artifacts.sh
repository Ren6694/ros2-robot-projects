#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/fetch_artifacts.sh   (在 Windows 侧 Git-Bash 中运行)
#  作用: 把 WSL 里产生的 ROS 2 运行日志 / rviz 截图 / rosbag 回传到作品集目录
#
#  方向约定（与 sync.sh 成对，永远单向，避免两边互相覆盖）：
#    sync.sh            Windows -> WSL ：源码（Windows 侧是唯一真源）
#    fetch_artifacts.sh WSL -> Windows ：产物（WSL 侧是唯一真源）
#
#  用法: bash scripts/fetch_artifacts.sh                # 默认 logs + artifacts
#        bash scripts/fetch_artifacts.sh artifacts      # 只要截图/图
#
#  为什么不回传 install/ build/ log/：
#    colcon 的 build 目录里缓存了绝对路径与 CMake 配置，拿到 Windows 上不但没用，
#    还会让评审误以为那是源码；install/ 里的 share/ 又是 src/ 的副本，
#    两处同时存在时"哪份是真的"就成了问题。只回传不可再生的东西：日志、截图、bag。
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
REMOTE="${REMOTE:-ros2vm}"
SRC_ROOT="${SRC_ROOT:-~/ros2_ws}"
ITEMS=("$@")
if [ "${#ITEMS[@]}" -eq 0 ]; then ITEMS=(logs artifacts); fi

for d in "${ITEMS[@]}"; do
    echo ">>> 拉取 $REMOTE:$SRC_ROOT/$d -> ./$d"
    mkdir -p "$d"
    scp -q -r "$REMOTE:$SRC_ROOT/$d/"* "./$d/" 2>/dev/null || {
        echo "    (该目录为空或还不存在，跳过)"
    }
done

# 运行时的"最新日志"指针文件对评审没有价值，不入作品集
rm -f logs/latest_*.path

echo ">>> 本地现有产物："
for d in "${ITEMS[@]}"; do
    if [ -d "$d" ]; then
        echo "    $d: $(find "$d" -type f | wc -l) 个文件, $(du -sh "$d" | cut -f1)"
        find "$d" -type f \( -name '*.png' -o -name '*.md' -o -name '*.csv' -o -name '*.bag' \) \
            -printf '        %p (%s B)\n' 2>/dev/null | sort | tail -12
    fi
done
