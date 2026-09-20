#!/usr/bin/env bash
# ============================================================================
# D12 方案 A：6 个配置 × 3 次重复，取中位数
#   w_max=1.5 / w_max=2.2（基线增益）
#   PD 网格 (1.0,0.4) (1.5,0.4) (2.0,0.8) (2.5,1.2)
#
# 台架 v3 相对 v2 的三处修正：
#   1) 复位前先杀掉 PD 节点 —— v2 里复位失败很可能就是上一个节点的 cmd_vel
#      还在发（cmd_vel_timeout 是 2.0 秒仿真时间，慢仿真下能拖很久），
#      车被"传送回去 + 立刻又被开走"。
#   2) **复位后必须验证**车真的在起点、且 /line/pose 在发，才允许起跑。
#      验证不过 → 整世界重启再试一次；再不过 → 这一跑标成 HARNESS_FAIL 丢弃。
#      这一条把"台架故障"和"配置失败"分开，是这次最关键的改动。
#   3) 起跑后跑不出去（丢线>20% 或没完成一圈）**不丢弃**，照实记录 ——
#      "这个增益圆角过不去"本身就是结论（kp=1.0/kd=0.4 就是这么暴露的）。
#
# 用法（WSL）：bash d12_repeats.sh
# ============================================================================
set +u
WS=~/mybot_ws
SHARE=$WS/install/mybot_description/share/mybot_description
BASE_WORLD=$SHARE/worlds/line_following.world
OUT=${D12OUT:-$HOME/d12r}
# 结果拷出目录：默认落 Linux 侧持久区 ~/scratch；
# 想直接进 Windows 时先 export D12_WIN="/mnt/c/<你的目录>"（WSL 里用 /mnt 路径）。
WIN=${D12_WIN:-$HOME/scratch}
# ★ 为什么不放 /tmp：2026-09-19 那次 18 跑跑完、数字都抄进笔记之后，
#   WSL 虚拟机重启过一次，/tmp 里的 18 份 CSV + raw.tsv **全部消失**
#   （findmnt 显示 /tmp 并不是独立挂载，boot 时 systemd-tmpfiles-setup 跑过，
#    根因没坐实）。教训跟机制无关，规则是：**跑圈产出的唯一副本不许放在 /tmp**，
#   而且脚本收尾要主动拷一份到 Windows 的 scratch，别指望人记得手抄。
LOGD=~/workbench_logs
MET=$WS/src/mybot_control/scripts/d12_metrics.py
RAW=$OUT/raw.tsv
mkdir -p "$OUT" "$LOGD"

source /opt/ros/humble/setup.bash
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_SHM_PROVIDER=0
cd "$WS" || exit 1

printf 'config\trep\tframes\tgood\tlost\tlost_pct\te_mean\te_max\te_p95\tover10cm\tsat\tpath\twall\tend_gap\tlap\tverdict\n' > "$RAW"

kill_nodes() {
  pkill -9 -f "[l]ine_follow_pd" 2>/dev/null
  sleep 2
}

world_up() {
  local n=${1:-0}
  n=$(pgrep -xc gzserver 2>/dev/null || true); n=${n:-0}
  [ "$n" = "1" ] || return 1
  local hz
  hz=$(timeout 12 ros2 topic hz /camera/image_raw 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
  [ -n "${hz:-}" ] || return 1
  return 0
}

start_world() {
  pkill -9 -x gzserver 2>/dev/null
  pkill -9 -f "[r]os2 launch" 2>/dev/null
  sleep 3
  local log="$LOGD/rw_$(date +%H%M%S).log"
  setsid nohup ros2 launch mybot_description line_follow.launch.py \
      gui:=false image_view:=false "world:=$BASE_WORLD" > "$log" 2>&1 < /dev/null &
  local i n
  for i in $(seq 1 40); do
    n=$(grep -c "Configured and activated" "$log" 2>/dev/null || true); n=${n:-0}
    [ "$n" -ge 2 ] && { sleep 6; return 0; }
    sleep 2
  done
  return 1
}

start_features() {
  pkill -9 -f "[l]ine_features_node" 2>/dev/null
  sleep 2
  setsid nohup ros2 run mybot_control line_features_node.py --duration 3600 \
      --v-hi 70 --x-hi 0.60 > "$OUT/feat.log" 2>&1 < /dev/null &
  local i hz
  for i in $(seq 1 14); do
    hz=$(timeout 12 ros2 topic hz /line/pose 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
    [ -n "${hz:-}" ] && return 0
    sleep 3
  done
  return 1
}

reset_car() {
  timeout 30 ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: mybot, pose: {position: {x: 0.0, y: -0.9, z: 0.10}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0}}, reference_frame: world}}" \
    2>&1 | grep -oE "success=(True|False)" | head -1
}

# 复位后验证：车必须**还在起点那段直线上**（横向贴线 + 沿程没进第一个弯）。
# ★ 判据改过的原因（当天 18 跑里每一跑都被老判据误判，白重启世界约 12 分钟）：
#   set_entity_state 只传送位姿、不清轮子角动量，车被传回 (0,-0.9) 后还会顺着
#   原来的速度往前滑 0.28~0.30 m 才停 —— 它滑完仍然好好压在线上，是合法起跑点。
#   老判据用"离那个点 0.15 m 以内"卡，等于把"滑了 30 cm"当成"复位失败"。
#   要卡的是**横向**偏差（在不在线上）和**沿程**位置（有没有滑过第一个弯）。
verify_start() {
  local p ok
  p=$(timeout 25 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
        "{name: mybot, reference_frame: world}" 2>/dev/null | tr -d '\n')
  ok=$(python3 - "$p" <<'PY'
import re, sys
t = sys.argv[1]
x = re.search(r'position=.*?x=([-\d.eE+]+).*?y=([-\d.eE+]+)', t)
if not x:
    print('nopos'); sys.exit()
px, py = float(x.group(1)), float(x.group(2))
lat = abs(py + 0.9)
if lat < 0.12 and -0.35 < px < 0.90:
    print('ok')
else:
    print('bad %.2f,%.2f lat=%.2f' % (px, py, lat))
PY
)
  echo "$ok"
}

# 复位后先等车真停下再验位置（读 /odom 的实际线速度，不是傻 sleep）
wait_stopped() {
  local i vx
  for i in $(seq 1 15); do
    vx=$(timeout 8 ros2 topic echo /odom --field twist.twist.linear --once 2>/dev/null \
         | grep -oE 'x: [-0-9.eE+]+' | head -1 | cut -d: -f2 | tr -d ' ')
    if [ -n "${vx:-}" ] && python3 -c "import sys; sys.exit(0 if abs(float('$vx')) < 0.02 else 1)" 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# run_rep <config> <rep> <kp> <kd> <w_max>
run_rep() {
  local cfg="$1" rep="$2" kp="$3" kd="$4" wm="$5"
  local csv="$OUT/${cfg}_r${rep}.csv" slug line verdict
  rm -f "$csv"

  # 1) 先杀 PD 节点，再复位（顺序很重要，见文件头注释）
  kill_nodes
  local r; r=$(reset_car)
  sleep 2
  wait_stopped || echo "    ! 15 秒内没等到车速归零，照样验位置"
  local v; v=$(verify_start)
  if [ "$v" != "ok" ]; then
    echo "    复位验证失败($r/$v) -> 整世界重启"
    start_world || { echo "    ★ 世界重启失败，本跑作废"; return 1; }
    start_features || { echo "    ★ 视觉节点起不来，本跑作废"; return 1; }
    r=$(reset_car); sleep 2; wait_stopped; v=$(verify_start)
    if [ "$v" != "ok" ]; then echo "    ★★ 二次验证仍失败($v)，标 HARNESS_FAIL"
      printf '%s\t%s\tHARNESS_FAIL\n' "$cfg" "$rep" >> "$RAW"; return 1; fi
  fi

  timeout 220 ros2 run mybot_control line_follow_pd.py \
      --kp "$kp" --kd "$kd" --w-max "$wm" --duration 150 --csv "$csv" --quiet \
      > "$OUT/${cfg}_r${rep}.log" 2>&1
  [ -s "$csv" ] || { echo "    ★ 无 CSV"; printf '%s\t%s\tNO_CSV\n' "$cfg" "$rep" >> "$RAW"; return 1; }

  line=$(python3 "$MET" "$csv" "$cfg" 2>/dev/null | tail -1)
  # verdict：完成一圈且丢线<20% 记 PASS，否则记 OFF_TRACK（照实保留，是信号不是噪声）
  if echo "$line" | grep -q "一圈完成"; then verdict=PASS; else verdict=OFF_TRACK; fi
  # 注意列对齐：表头是 config,rep,frames,...，而 $line 的第一列是 metrics 脚本打的 tag，
  # 要把 tag 换成 "cfg + rep" 两列，否则整行右移一格（上一版就错在这里）。
  local body; body=$(printf '%s' "$line" | sed 's/^[^\t]*\t//')
  printf '%s\t%s\t%s\t%s\n' "$cfg" "$rep" "$body" "$verdict" >> "$RAW"
  # 显示用：|e|均值 / 一圈多否 / 路径 / 墙钟。字段序见 d12_metrics.py 的 print。
  printf '  %-14s rep%s  e=%.2fcm  %.2fm  %.1fs  %s\n' "$cfg" "$rep" \
      "$(echo "$line" | cut -f6)" "$(echo "$line" | cut -f11)" "$(echo "$line" | cut -f12)" "$verdict"
}

echo "======================================================================"
echo " D12 方案 A：6 配置 × 3 重复 = 18 跑"
echo "======================================================================"

ALL_SPECS="wmax1.5:1.5:0.8:1.5 wmax2.2:1.5:0.8:2.2 \
kp1.0kd0.4:1.0:0.4:1.5 kp1.5kd0.4:1.5:0.4:1.5 \
kp2.0kd0.8:2.0:0.8:1.5 kp2.5kd1.2:2.5:1.2:1.5"

# SMOKE=1 只跑第一个配置的 1 次，用来先验证"复位 -> 验证 -> 起跑"这条新链路真的通。
# 这个台架已经翻过两次车，15 分钟的全量之前必须先花 1 分钟证明链路是好的。
if [ "${SMOKE:-0}" = "1" ]; then
  SPECS=$(echo $ALL_SPECS | awk '{print $1}'); REPS=1
  echo "★★ 冒烟模式：只跑 1 次，验证台架链路"
else
  SPECS="$ALL_SPECS"; REPS=3
fi

# ★ 开场**无条件**重启世界。这里原来写的是 `world_up || start_world`，
#   结果冒烟测试就栽了：world_up 只看"gzserver 活着 + 相机在发"就判定世界可用，
#   但那是一小时前 aborted run 留下的世界，车停在轨道外 —— 特征节点处理了 4248 帧
#   全是"0 有效行"，/line/pose 永不发布，start_features 判失败直接 exit。
#   "节点活着"和"车在线上"是两件事：前者由 start_features 把关，
#   后者由 run_rep 里的 verify_start 把关，而世界新鲜度必须在开场就保证。
start_world || { echo "★ 世界起不来，退出"; exit 1; }
start_features || { echo "★ 视觉节点起不来，退出"; exit 1; }

for spec in $SPECS; do
  # 原来用 ${rest%%:*} 一层层剥，最后一步漏剥了一次，wm 拿到 "0.8:1.5"，
  # argparse 直接报错退出 -> 表现为 NO_CSV，看着像台架又坏了。用 IFS read 一次拆干净。
  IFS=: read -r cfg kp kd wm <<< "$spec"
  echo; echo "--- $cfg  (kp=$kp kd=$kd w_max=$wm) ---"
  for rep in $(seq 1 $REPS); do run_rep "$cfg" "$rep" "$kp" "$kd" "$wm"; done
done

echo
echo "======================================================================"
echo "原始数据 $RAW"
column -t -s $'\t' "$RAW" 2>/dev/null || cat "$RAW"

# ★ 收尾三件事：复算 -> 出中位数表 -> **立刻拷一份出 /tmp 之外的持久区**
SCRIPT_DIR=$(cd "$(dirname "$0")" 2>/dev/null && pwd)
[ -f "$SCRIPT_DIR/d12_recompute.py" ] || SCRIPT_DIR=$WS/src/mybot_control/scripts
python3 "$SCRIPT_DIR/d12_recompute.py" "$OUT" "$OUT/recomputed.tsv" 2>&1 | tail -3
python3 "$SCRIPT_DIR/d12_stats.py" "$OUT/recomputed.tsv" "$OUT/params.md" 2>&1 | tail -3
# ★ 拷出目录必须写完整路径，不写通配：`ls -d */ | head -1` 之类的挑法在目录一多
#   就可能挑错，cp 会把结果复制进非预期位置。这里固定用 $WIN（可用 D12_WIN 覆盖）。
mkdir -p ~/d12r_last 2>/dev/null
mkdir -p "$WIN" 2>/dev/null
if cp -r "$OUT" "$WIN/" 2>/dev/null; then
  echo "已拷贝整份结果到 $WIN/d12r（CSV + raw + recomputed + params.md）"
else
  echo "★ 拷出失败（$WIN 不可写？），结果只在 $OUT，请手动备份"
fi
echo "参数表： $OUT/params.md    复算表： $OUT/recomputed.tsv"
