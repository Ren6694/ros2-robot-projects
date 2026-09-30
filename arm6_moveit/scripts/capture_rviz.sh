#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/capture_rviz.sh   (在 WSL 里跑)
#  作用: 把 rviz2 的界面抓成 PNG 证据（空闲态 / 规划执行中 / 结束后）
#
#  ★ 为什么默认用 Xvfb 而不是 WSLg 的 :0 —— 这是实测出来的，不是想当然：
#    本机是 WSLg（存在 /mnt/wslg，X 套接字 /tmp/.X11-unix/X0 由 WSLg 的 Xwayland 提供）。
#    `DISPLAY=:0 scrot -o` 能成功返回 rc=0，但抓到的 root 窗口是 5120x1600 的**全黑图**：
#    WSLg 把每个 X 客户端窗口单独映射成一个 Wayland surface，Xwayland 的 root 窗口
#    本身没有合成内容，所以"截 root"必然是黑的 —— 而且它不报错，最容易骗人。
#    对策：起一个真正的内存 X 服务器 Xvfb，rviz2 用 Mesa 的软渲染(llvmpipe)画进去，
#    这时截 root 就是真实画面。这也是 ROS 官方 CI 截图的做法。
#    脚本保留 mode=wslg 分支，方便以后在真正的 XFCE 会话（startxfce4 + :0）下复用。
#
#  三张图分别证明三件事（只给一张"rviz 打开了"没有信息量）：
#    idle     : URDF 模型 + TF + MotionPlanning 面板加载正确，规划组/位姿下拉可见
#    moving   : 点位节点正在规划并执行，机器人真的在动（规划链路的最终可视化证据）
#    final    : 执行结束后停在最后一个点位（up_final），与日志里的到位误差互相印证
#
#  用法: bash scripts/capture_rviz.sh [xvfb|wslg] [分辨率 WxHxD]
#        bash scripts/capture_rviz.sh xvfb 1680x1050x24
#  产物: artifacts/rviz_<stamp>_{idle,moving,final}.png + logs/rviz_bringup_<stamp>.log
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
MODE="${1:-xvfb}"
RES="${2:-1680x1050x24}"
DISP_NUM="${DISP_NUM:-99}"

cd "$WS"
mkdir -p logs artifacts
STAMP="$(date +%Y%m%d-%H%M%S)"
BLOG="logs/rviz_bringup_${STAMP}.log"
DLOG="logs/rviz_mover_${STAMP}.log"

# shellcheck disable=SC1091
source scripts/ros_env.sh

XVFB_PID=""
cleanup() {
    [ -n "${STACK_PID:-}" ] && kill "$STACK_PID" 2>/dev/null || true
    [ -n "${MOVER_PID:-}" ] && kill "$MOVER_PID" 2>/dev/null || true
    [ -n "$XVFB_PID" ] && kill "$XVFB_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT

if [ "$MODE" = "xvfb" ]; then
    command -v Xvfb >/dev/null || { echo "缺 Xvfb：sudo apt-get install -y xvfb"; exit 1; }
    echo ">>> [1/6] 起 Xvfb :${DISP_NUM} (${RES})"
    Xvfb ":${DISP_NUM}" -screen 0 "$RES" -nolisten tcp > "logs/xvfb_${STAMP}.log" 2>&1 &
    XVFB_PID=$!
    export DISPLAY=":${DISP_NUM}"
    # 强制软件渲染：WSL 里 GPU 直通不一定可用，llvmpipe 慢但结果确定
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    sleep 2
else
    export DISPLAY="${DISPLAY:-:0}"
fi
echo ">>> 截图目标 DISPLAY=$DISPLAY mode=$MODE"

# --------------------------------------------------------------------------
# 抓一张并判断"是不是黑的"。
# 两个坑都踩过：
#   1) 判据不能用文件大小（全黑 PNG 压缩后也有几十 KB）。
#   2) matplotlib 读 PNG 得到的是 **float [0,1]** 数组，不是 0~255 ——
#      第一次写这张图时阈值用了 3.0，结果 std=0.44 的正常截图被判成"全黑"，
#      白等 90 s 还以为是 WSLg 的问题。现在统一先乘回 0~255 再判。
# ★ std 只是 liveness 检查（"有没有画面"），不是内容检查。实测三张图的视口 std 分别是
#   62.92 / 62.99 / 62.96 —— 手臂只占视口百分之十几的像素，全局统计量根本
#   分不出位形。"机器人真的动了 / 真的回到了起点"由第 6 步的 DIFF 负责。
# 取景：Xvfb 的屏幕比 rviz 窗口大很多，直接截 root 会有 90% 的黑边；
#       所以先问 xwininfo 要 "RViz" 顶层窗口的绝对坐标与尺寸，用 scrot -a 只截它。
#       拿不到窗口几何时自动退回截整屏（宁可丑，也不要因为裁图失败而没有证据）。
# --------------------------------------------------------------------------
rviz_geom() {
    local wid
    wid=$(xwininfo -root -tree 2>/dev/null | grep -m1 -E '"RViz"|"rviz2"' \
          | grep -oE '0x[0-9a-f]+' | head -1)
    [ -z "$wid" ] && return 1
    xwininfo -id "$wid" 2>/dev/null | awk '
        /Absolute upper-left X/ {x=$NF}
        /Absolute upper-left Y/ {y=$NF}
        /^  Width:/             {w=$NF}
        /^  Height:/            {h=$NF}
        END {if (w>10 && h>10) printf "%d,%d,%d,%d", x, y, w, h; else exit 1}'
}

shot() {
    local name="$1"
    local out="artifacts/rviz_${STAMP}_${name}.png"
    local geom
    if geom=$(rviz_geom); then
        scrot -a "$geom" -o "$out" 2>/dev/null || { echo "SHOT-FAIL $out (scrot -a $geom)"; return 1; }
        echo "    crop geom=$geom"
    else
        scrot -o "$out" 2>/dev/null || { echo "SHOT-FAIL $out (scrot 返回非 0)"; return 1; }
        echo "    crop 不可用，已截整屏"
    fi
    python3 - "$out" <<'PY'
import sys
import numpy as np
from matplotlib import image as mpimg
a = np.asarray(mpimg.imread(sys.argv[1])[:, :, :3], dtype=float)
if a.max() <= 1.0:      # matplotlib 给的是 [0,1] 浮点，统一换算回 0~255
    a *= 255.0
std = float(a.std())
print(f"SHOT {sys.argv[1]} shape={a.shape[:2]} std={std:.2f} "
      f"{'PASS' if std > 3.0 else 'FAIL(疑似全黑/空白)'}")
PY
}

echo ">>> [2/6] 带 rviz 拉起整套栈（use_rviz 默认为 true）"
timeout 240 ros2 launch arm_moveit_config moveit.launch.py > "$BLOG" 2>&1 &
STACK_PID=$!

echo ">>> [3/6] 等 rviz 真的画出内容（最多 90s）"
PROBE="logs/shot_probe.txt"
ok=""
for i in $(seq 1 30); do
    sleep 3
    if shot idle > "$PROBE" 2>&1; then
        grep -q "PASS" "$PROBE" && { ok=1; break; }
    fi
    echo "    第 $i 次尝试：$(grep -o 'std=[0-9.]*' "$PROBE" | head -1 || echo 'scrot 未产出图片')"
done
rm -f "$PROBE"
[ -z "$ok" ] && { echo "!!! rviz 画面始终接近全黑，见 $BLOG"; tail -20 "$BLOG"; exit 1; }
echo ">>> idle 截图就绪"

echo ">>> [4/6] 后台跑点位节点，抓执行中的画面"
timeout 200 ros2 launch arm_demo waypoint_mover.launch.py \
    log_dir:="$WS/logs" run_id:="rviz_${STAMP}" > "$DLOG" 2>&1 &
MOVER_PID=$!
# 等到第 3 条点位执行完再截 —— 此时正在跑第 4 条 pick(约 2.2s)，
# 那是一条明显弯臂的姿态，与 idle 的 up(全零、直上)差别最大，截图才有信息量。
# 早先只等 "MOTION idx=" 的第一条，而 idx=1 是 up(零位移，traj_pts=1)，
# 结果 moving 和 idle 两张图几乎一样，等于白截。
# 为什么不等 "MOTION idx=4"：MOTION 行是在**执行结束 + settle 之后**才印的，
# 等它出现时手臂已经停在 pick 上了 —— 那拍到的是"又一个静止位形"，不是运动过程。
for i in $(seq 1 60); do
    sleep 1
    grep -q "MOTION idx=3 " "$DLOG" && break
done
sleep 1.2
shot moving
wait "$MOVER_PID" 2>/dev/null || true
grep -oE "MOTION-SUMMARY .*" "$DLOG" || echo "    (点位节点没有输出总结行，见 $DLOG)"

echo ">>> [5/6] 收尾截图"
sleep 2
shot final

# --------------------------------------------------------------------------
# [6/6] 内容检查：三张图必须"不一样 + 又一样"
#   std 只能证明有画面（见 shot() 上方注释），证明不了位形。
#   这里只比对 3D 视口（左侧面板固定占约 700/1600 px，故取 x>0.44w），
#   统计"变化像素占比"（任一通道差 >25 记为变化，滤掉抗锯齿与 fps 计数器）。
#   阈值来自实测：修好 Loop Animation 之后 idle↔moving=0.52%、idle↔final=0.16%；
#   修之前 idle↔final 是 0.55%（地面上多了一条重播机器人）—— 所以
#     moving 必须 >0.20%（手臂真的动了）
#     final  必须 <0.35%（回到了起点：up 与 up_final 是同一个全零目标，
#                          这两张几乎相同**正是**重复定位精度的图像证据）
#   两个方向相反的判据一起用，才不会"随便截三张不同的图"也算过。
# --------------------------------------------------------------------------
echo ">>> [6/6] 比对三张图的内容差异"
# 判据抽到 scripts/check_rviz_frames.py：① 能对着归档图离线重跑（只换尺子不重截）；
# ② 分母从"视口面积"换成"手臂自己的投影面积"，取景一改判据就不跟着漂（推导与三批实测见该文件头）。
DIFF=$(python3 scripts/check_rviz_frames.py               "artifacts/rviz_${STAMP}_idle.png"               "artifacts/rviz_${STAMP}_moving.png"               "artifacts/rviz_${STAMP}_final.png" 2>&1)
RC_DIFF=$?
echo "$DIFF"
[ "$RC_DIFF" = 2 ] && echo "    (读图失败，不是判红 —— 见 check_rviz_frames.py 退出码约定)"

echo "--- 本次产物 ---"
ls -l "artifacts/rviz_${STAMP}_idle.png" "artifacts/rviz_${STAMP}_moving.png" "artifacts/rviz_${STAMP}_final.png" 2>/dev/null
grep -oE "MOTION .*" "$DLOG" | head -3
VERDICT=$(echo "$DIFF" | awk '/^FRAMES-RESULT/{print $2}')
echo "CAPTURE-RESULT ${VERDICT:-FAIL} mode=$MODE display=$DISPLAY stamp=$STAMP"
[ "${VERDICT:-FAIL}" = "PASS" ] || exit 1
