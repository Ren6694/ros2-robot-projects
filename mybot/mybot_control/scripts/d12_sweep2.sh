#!/usr/bin/env bash
# ============================================================================
# D12 调参扫描 · 第二版台架
#
# 为什么要重做：第一版用 set_entity_state 复位小车 + 固定 sleep 16s 等特征节点，
# 结果 v_hi=70（就是基线本身）跑出"3000 帧 100% 丢线、路径 0.02 m"。
# 查出来车停在 (1.331,-0.881) —— 右下圆弧上、车头直指赛道外，说明**复位没生效**，
# 而复位的服务返回值被我 >/dev/null 丢了，所以失败完全不可见。
#
# 本版两个改动：
#   1) 每个配置**完整重启 gzserver**，不再依赖 set_entity_state 复位；
#   2) 起完特征节点后**轮询 /line/pose 直到真的有数据**才起跑，不用固定 sleep。
#      （这是 D10 就记过的"半起的栈会让车一动不动，看上去像控制律写错"）
#
# 用法（WSL）：bash d12_sweep2.sh
# ============================================================================
set +u
WS=~/mybot_ws
SHARE=$WS/install/mybot_description/share/mybot_description
BASE_WORLD=$SHARE/worlds/line_following.world
OUT=/tmp/d12
LOGD=~/workbench_logs
MET=$WS/src/mybot_control/scripts/d12_metrics.py
TSV=$OUT/results2.tsv
mkdir -p "$OUT" "$LOGD"

source /opt/ros/humble/setup.bash
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_SHM_PROVIDER=0
cd "$WS" || exit 1

printf 'round\tconfig\tframes\tgood\tlost\tlost_pct\te_mean_cm\te_max_cm\te_p95_cm\tover10cm_pct\tsat\tpath_m\twall_s\tend_gap\tlap\n' > "$TSV"

start_world() {
  pkill -9 -x gzserver 2>/dev/null
  pkill -9 -f "[r]os2 launch" 2>/dev/null
  sleep 3
  local log="$LOGD/sw_$(date +%H%M%S).log"
  setsid nohup ros2 launch mybot_description line_follow.launch.py \
      gui:=false image_view:=false "world:=$BASE_WORLD" > "$log" 2>&1 < /dev/null &
  local i n
  for i in $(seq 1 40); do
    n=$(grep -c "Configured and activated" "$log" 2>/dev/null || true); n=${n:-0}
    [ "$n" -ge 2 ] && return 0
    sleep 2
  done
  echo "    ★ 世界没起稳"; return 1
}

wait_features() {          # 轮询直到 /line/pose 真的在发，最多 40 秒
  local i hz
  for i in $(seq 1 12); do
    hz=$(timeout 12 ros2 topic hz /line/pose 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
    [ -n "${hz:-}" ] && { echo "    /line/pose ${hz} Hz"; return 0; }
    sleep 3
  done
  echo "    ★ /line/pose 一直没数据"; return 1
}

start_features() {         # $1=v_hi $2=x_hi
  pkill -9 -f "[l]ine_features_node" 2>/dev/null
  sleep 2
  setsid nohup ros2 run mybot_control line_features_node.py --duration 1200 \
      --v-hi "$1" --x-hi "$2" > "$OUT/feat_$1_$2.log" 2>&1 < /dev/null &
  wait_features || return 1
}

run_one() {                # <round> <tag> <kp> <kd> <w_max> <v_hi> <x_hi>
  local rd="$1" tag="$2" kp="$3" kd="$4" wm="$5" vhi="$6" xhi="$7"
  local slug csv
  slug=$(echo "$tag" | tr -d ' /:,=')
  csv="$OUT/s2_${slug}.csv"
  rm -f "$csv"
  printf '  [%s] %s  ' "$rd" "$tag"
  start_world || { echo "$rd	$tag	0	0	0	0	nan	nan	nan	nan	0	0	0	-1	世界启动失败" >> "$TSV"; return 1; }
  start_features "$vhi" "$xhi" || { echo "$rd	$tag	0	0	0	0	nan	nan	nan	nan	0	0	0	-1	视觉节点失败" >> "$TSV"; return 1; }
  timeout 220 ros2 run mybot_control line_follow_pd.py \
      --kp "$kp" --kd "$kd" --w-max "$wm" --duration 150 --csv "$csv" --quiet \
      > "$OUT/n2_${slug}.log" 2>&1
  if [ ! -s "$csv" ]; then echo "    ★ 没写出 CSV"; return 1; fi
  python3 "$MET" "$csv" "$tag" 2>/dev/null | tail -1 | tee -a /dev/null | sed 's/^/    /'
  printf '%s\t%s\n' "$rd" "$(python3 "$MET" "$csv" "$tag" 2>/dev/null | tail -1)" >> "$TSV"
}

echo "======================================================================"
echo " D12 扫描 v2（每轮重启世界 + 轮询等 /line/pose）"
echo " 基线 kp=1.5 kd=0.8 w_max=1.5 v_max=0.30 v_hi=70 x_hi=0.60"
echo "======================================================================"

echo; echo "--- 交叉验证：用新台架重跑轮 1 的两个点，看是否与 v1 一致 ---"
run_one X "wmax1.5_复核" 1.5 0.8 1.5 70 0.60
run_one X "wmax2.2_复核" 1.5 0.8 2.2 70 0.60

echo; echo "--- 轮 2：视觉阈值 v_hi ---"
for vhi in 40 70 100; do run_one 2 "v_hi=$vhi" 1.5 0.8 1.5 "$vhi" 0.60; done
echo "--- 轮 2：远端截断 x_hi ---"
for xhi in 0.45 0.75; do run_one 2 "x_hi=$xhi" 1.5 0.8 1.5 70 "$xhi"; done

echo; echo "--- 轮 3：PD 增益网格 ---"
run_one 3 "kp1.0_kd0.4" 1.0 0.4 1.5 70 0.60
run_one 3 "kp1.5_kd0.4" 1.5 0.4 1.5 70 0.60
run_one 3 "kp2.0_kd0.8" 2.0 0.8 1.5 70 0.60
run_one 3 "kp2.5_kd1.2" 2.5 1.2 1.5 70 0.60

echo; echo "--- 追加：w_max 再往上找拐点 ---"
run_one 4 "w_max=3.0" 1.5 0.8 3.0 70 0.60
run_one 4 "wmax2.2_vmax0.35" 1.5 0.8 2.2 70 0.60

echo
echo "======================================================================"
echo "结果表 $TSV"
column -t -s $'\t' "$TSV" 2>/dev/null || cat "$TSV"
