#!/usr/bin/env bash
# ============================================================================
# D12 三轮调参扫描（路线页原文：最大角速度、阈值、PD 三轮调参）
#
# 设计约束：
#   1) 用**基线世界**（无障碍）跑，否则每圈都会被障碍拦住，圈时不可比。
#   2) 只在 FOLLOW/有效帧上算 |e| —— 见 d12_metrics.py 的口径注释（D11 刚踩过）。
#   3) 视觉参数(v_hi/x_hi)变了才重启特征节点，省一半时间。
#   4) 每个配置前把车复位到起点，保证是同一条轨迹可比。
#   5) 结果边跑边追加到 TSV，中途断了也不用重跑全部。
#
# 用法（WSL，仿真已 headless 起好）：bash d12_sweep.sh [轮次]
#        轮次留空=全跑；1=角速度 2=阈值 3=PD
# ============================================================================
set +u
WS=~/mybot_ws
SHARE=$WS/install/mybot_description/share/mybot_description
BASE_WORLD=$SHARE/worlds/line_following.world
OUT=/tmp/d12
mkdir -p $OUT
TSV=$OUT/results.tsv
MET=$WS/src/mybot_control/scripts/d12_metrics.py

source /opt/ros/humble/setup.bash
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_SHM_PROVIDER=0
cd "$WS" || exit 1

if [ ! -f "$BASE_WORLD" ]; then echo "找不到基线世界 $BASE_WORLD"; exit 1; fi
# ★ 数进程别写 `pgrep -xc X || echo 0`：计数为 0 时 pgrep 自己已经输出了一行 "0"，
#   再 || echo 0 就变成两行，和 "0" 比较不相等 -> 误判"有进程在跑"直接退出。
#   正确写法：先 || true 吞掉退出码，再用 ${var:-0} 兜空。
NG=$(pgrep -xc gzserver 2>/dev/null || true); NG=${NG:-0}
NC=$(pgrep -xc gzclient 2>/dev/null || true); NC=${NC:-0}
if [ "$NG" != "1" ]; then
  echo "★ gzserver 必须是 1 个（现在是 $NG）。先跑：bash ros2_workbench.sh headless"
  exit 1
fi
if [ "$NC" != "0" ]; then
  echo "★ 检测到 $NC 个 gzclient 在跑。调参要在 headless 下测，否则 RTF 受渲染影响、圈时不可比。"
  echo "  先跑：bash ros2_workbench.sh headless"
  exit 1
fi
echo "前置检查通过：gzserver=$NG gzclient=$NC"

[ -f "$TSV" ] || printf 'round\tconfig\tframes\tgood\tlost\tlost_pct\te_mean_cm\te_max_cm\te_p95_cm\tover10cm_pct\tsat\tpath_m\twall_s\tend_gap\tlap\n' > "$TSV"

FEAT_VHI=""; FEAT_XHI=""

start_features() {           # $1=v_hi $2=x_hi
  pkill -9 -f "[l]ine_features_node" 2>/dev/null
  sleep 2
  local extra=""
  [ -n "$1" ] && extra="$extra --v-hi $1"
  [ -n "$2" ] && extra="$extra --x-hi $2"
  setsid nohup ros2 run mybot_control line_features_node.py --duration 3600 $extra \
      > "$OUT/features_$1_$2.log" 2>&1 < /dev/null &
  FEAT_VHI="$1"; FEAT_XHI="$2"
  sleep 16
}

reset_car() {
  timeout 30 ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: mybot, pose: {position: {x: 0.0, y: -0.9, z: 0.10}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0}}, reference_frame: world}}" \
    >/dev/null 2>&1
  sleep 2
}

# run_one <round> <config标签> <kp> <kd> <w_max> <v_hi> <x_hi>
run_one() {
  local rd="$1" tag="$2" kp="$3" kd="$4" wm="$5" vhi="$6" xhi="$7"
  local csv="$OUT/r${rd}_$(echo "$tag" | tr -d ' /:,=').csv"
  if [ "$vhi" != "$FEAT_VHI" ] || [ "$xhi" != "$FEAT_XHI" ]; then
    echo "  （重启特征节点 v_hi=$vhi x_hi=$xhi）"
    start_features "$vhi" "$xhi"
  fi
  reset_car
  printf '  跑 %-28s ' "$tag"
  timeout 220 ros2 run mybot_control line_follow_pd.py \
      --kp "$kp" --kd "$kd" --w-max "$wm" --duration 150 --csv "$csv" --quiet \
      > "$OUT/node_${rd}_${tag}.log" 2>&1
  local line
  line=$(python3 "$MET" "$csv" "$tag" 2>/dev/null | tail -1)
  echo "$line"
  printf '%s\t%s\n' "$rd" "$line" | sed 's/\t\t/\t/' >> "$TSV"
}

R="${1:-all}"
HDR="round\tconfig\tframes\tgood\tlost\tlost_pct\te_mean_cm\te_max_cm\te_p95_cm\tover10cm_pct\tsat\tpath_m\twall_s\tend_gap\tlap"
echo "======================================================================"
echo "D12 调参扫描   基线: kp=1.5 kd=0.8 w_max=1.5 v_hi=70 x_hi=0.60 v_max=0.30"
echo "======================================================================"
start_features 70 0.60

if [ "$R" = "all" ] || [ "$R" = "1" ]; then
  echo
  echo "--- 轮 1：最大角速度 w_max（其余取基线）---"
  printf '%s\n' "$HDR" | tr '\t' ' ' | cut -c1-100
  for wm in 0.6 1.0 1.5 2.2; do
    run_one 1 "w_max=$wm" 1.5 0.8 "$wm" 70 0.60
  done
fi

if [ "$R" = "all" ] || [ "$R" = "2" ]; then
  echo
  echo "--- 轮 2：视觉阈值 v_hi 与远端截断 x_hi（控制取基线）---"
  for vhi in 40 70 100; do
    run_one 2 "v_hi=$vhi" 1.5 0.8 1.5 "$vhi" 0.60
  done
  for xhi in 0.45 0.75; do
    run_one 2 "x_hi=$xhi" 1.5 0.8 1.5 70 "$xhi"
  done
fi

if [ "$R" = "all" ] || [ "$R" = "3" ]; then
  echo
  echo "--- 轮 3：PD 增益网格（视觉取基线）---"
  run_one 3 "kp1.0_kd0.4" 1.0 0.4 1.5 70 0.60
  run_one 3 "kp1.5_kd0.4" 1.5 0.4 1.5 70 0.60
  run_one 3 "kp2.0_kd0.8" 2.0 0.8 1.5 70 0.60
  run_one 3 "kp2.5_kd1.2" 2.5 1.2 1.5 70 0.60
fi

echo
echo "======================================================================"
echo "结果表：$TSV"
column -t -s $'\t' "$TSV" 2>/dev/null | head -30 || cat "$TSV"
