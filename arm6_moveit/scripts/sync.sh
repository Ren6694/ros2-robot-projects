#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/sync.sh   (在 Windows 侧 Git-Bash 中运行)
#  作用: 把作品集里的 ROS 2 源码单向同步到 WSL 构建环境
#
#  工作流约定（与阶段一保持一致，便于简历里讲同一套工程流程）：
#    Windows  <工程目录>\20260916-ros2-moveit-arm  ← 源码唯一真源
#    WSL2     ~/ros2_ws                        ← 只做 colcon 构建与仿真运行
#    回传     logs/ artifacts/ 由 scripts/fetch_artifacts.sh 拉回 Windows
#
#  为什么是单向 + 手工回传，而不是双向 rsync：
#    双向同步在"远端会自己生成 build/install/log"的 workspace 上极易误删产物，
#    而且 colcon 的 build 目录里有绝对路径缓存，Windows 侧根本不该看到它。
#    单向（源码去、产物回）职责清楚，出问题只可能有一个方向。
#
#  依赖: ~/.ssh/config 中的 Host ros2vm（127.0.0.1:2222，公钥免密）
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
REMOTE="${REMOTE:-ros2vm}"
DEST="${DEST:-~/ros2_ws/}"

ITEMS=(src scripts)

# __pycache__/*.pyc 是本机解释器跑比对脚本时留下的编译缓存（目录名带 cpython-3xx，换版本即作废）：
# 远端用不上，还会混进下面那行"远端源码文件数"，让 16 个脚本看起来像 17 个。
# scp -r 没有可靠的排除语法，所以本机同步前、远端统计后各删一次。
find src scripts -type d -name '__pycache__' -prune -exec rm -rf {} +

echo ">>> 同步到 $REMOTE:$DEST"
scp -q -r "${ITEMS[@]}" "$REMOTE:$DEST"

# Windows 写的文本文件带 CRLF：
#   *.sh  -> bash 会把 \r 当成命令的一部分（": command not found"）
#   *.py  -> launch 文件里字符串常量末尾多个 \r，rviz/参数服务会静默不匹配
#   *.xacro/*.yaml -> XML/YAML 解析器多数能容忍，但 check_urdf 的报错信息会带上
#                    看不见的 ^M，排查时极难对齐行号
# 所以同步完立刻在远端就地规范化，而不是依赖编辑器设置。
ssh "$REMOTE" "cd ~/ros2_ws && \
  find src scripts -type f \( -name '*.sh' -o -name '*.py' -o -name '*.xml' \
       -o -name '*.yaml' -o -name '*.xacro' -o -name '*.urdf' -o -name '*.srdf' \
       -o -name '*.rviz' -o -name '*.txt' \) \
    -exec sed -i 's/\r\$//' {} + && \
  find src scripts -type d -name '__pycache__' -prune -exec rm -rf {} + ; \
  chmod +x scripts/*.sh 2>/dev/null; \
  echo \"    远端源码文件数: \$(find src scripts -type f | wc -l)\""
echo ">>> 完成（构建请在远端跑: cd ~/ros2_ws && colcon build）"
