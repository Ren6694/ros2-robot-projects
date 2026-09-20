#!/usr/bin/env bash
# ============================================================================
# D12 桌面实录：把今天参数表里三种真实结局各录一段 Gazebo 窗口视频（用户选的交付形式）
#
# 三条从探测轮学到的硬约束，全都写在这里：
#   1) 不能用 ffmpeg x11grab —— 抓 WSLg 窗口出来的是一片纯黑（已实测排除）。
#      可用的是 X 原生 dump：xwd -id <WID>（约 12.8 fps）/ import（约 5 fps）。
#   2) 录屏要在**有 gzclient** 的世界里跑，而调参扫描用的是 gui:=false。
#      所以这个脚本自己重启一次带 GUI 的世界，且必须在扫描跑完之后再做 ——
#      抓帧循环要吃 CPU，会把仿真拖慢，同时跑等于污染两边的量测。
#   3) 正因为要吃 CPU，这里先量一次"不录屏的一圈"当基线，再录同一配置，
#      两者之差就是录屏本身的开销，报出来而不是装作它不存在。
#
# 配置串格式： tag:kp:kd:w_max:v_max
#   默认"差"的那组只把 w_max 砍到 0.6 —— 其他增益一律不动，
#   这样画面里的差别**只有**"弯道转不过来"这一个变量，说服力最强。
#
# 用法（WSL）：bash d12_record.sh
#   可覆盖：BAD=... GOOD=... CAPMETH=xwd|import
# ============================================================================
set +u
WS=~/mybot_ws
SHARE=$WS/install/mybot_description/share/mybot_description
BASE_WORLD=$SHARE/worlds/line_following.world
OUT=${D12REC:-$HOME/d12rec}
# ★ 不放 /tmp 的原因：上一轮 18 跑的 CSV 就死在 /tmp —— WSL 虚拟机重启后 /tmp 被清空，
#   帧、CSV、raw.tsv 全没了（笔记里记着这件事）。而且 xwd 一帧 3~4 MB、
#   一段 35 秒实测能写 4.7 GB，放 /tmp 等于把最贵的东西放在最容易丢的地方。
LOGD=~/workbench_logs
WIN=/mnt/c/USER/Desktop/scratch
MET=$WS/src/mybot_control/scripts/d12_metrics.py
rm -rf "$OUT"; mkdir -p "$OUT" "$LOGD"

source /opt/ros/humble/setup.bash
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_SHM_PROVIDER=0
export DISPLAY=:0
cd "$WS" || exit 1

# 片段清单：tag:kp:kd:w_max:v_max:录屏?(1/0):时长秒:成品文件名后缀
# ★ 顺序有讲究：base 排在最前且**不录屏** —— 它和 tuned 参数完全相同，
#   两者墙钟之差就是"录屏本身让仿真慢了多少"，这个数必须主动报出来，
#   不能一边用抓帧污染计时一边装作没这回事。
#   后三段对应今天表里的三种真实结局：整定后 / 过冲蛇形(kp2.5kd1.2,|e|5.69) / 丢线停车(kd0.4,6/6全灭)。
CLIPS=${CLIPS:-"base:1.5:0.8:2.2:0.30:0:45:基线不录屏,tuned:1.5:0.8:2.2:0.30:1:45:整定后,weave:2.5:1.2:1.5:0.30:1:45:过冲蛇形,offtrack:1.5:0.4:1.5:0.30:1:35:丢线停车"}
CAPMETH=${CAPMETH:-xwd}
CAP_SLEEP=${CAP_SLEEP:-0.06}      # ~14 fps 上限；裸循环实测 38.7 fps、4.7 GB/段
SUMMARY=$OUT/summary.tsv
printf 'tag\tkp\tkd\tw_max\tv_max\tcaptured\tframes\tfps\te_mean_cm\tpath_m\twall_s\tverdict\n' > "$SUMMARY"

kill_all() {
  pkill -9 -f "[l]ine_follow_pd" 2>/dev/null
  pkill -9 -f "[x]wd -id" 2>/dev/null
  pkill -9 -f "[i]mport -window" 2>/dev/null
  sleep 2
}

start_world() {
  pkill -9 -x gzserver 2>/dev/null
  pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "[r]os2 launch" 2>/dev/null
  sleep 3
  local log="$LOGD/rec_world_$(date +%H%M%S).log"
  # ★ 与扫描唯一的区别：gui:=true，桌面必须看得见窗口
  setsid nohup ros2 launch mybot_description line_follow.launch.py \
      gui:=true image_view:=false "world:=$BASE_WORLD" > "$log" 2>&1 < /dev/null &
  local i n
  for i in $(seq 1 50); do
    n=$(grep -c "Configured and activated" "$log" 2>/dev/null || true); n=${n:-0}
    [ "$n" -ge 2 ] && { sleep 8; return 0; }
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

find_window() {
  local i wid
  for i in $(seq 1 20); do
    wid=$(xwininfo -root -tree 2>/dev/null \
          | grep -oE '0x[0-9a-f]+ "Gazebo".*[0-9]{3,}x[0-9]{3,}\+[0-9-]+\+[0-9-]+' \
          | head -1 | awk '{print $1}')
    [ -n "$wid" ] && { echo "$wid"; return 0; }
    sleep 2
  done
  return 1
}

reset_car() {
  timeout 30 ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: mybot, pose: {position: {x: 0.0, y: -0.9, z: 0.10}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0}}, reference_frame: world}}" \
    2>&1 | grep -oE "success=(True|False)" | head -1
}

# ★ 复位后必须等车**真的停下**再验位置。扫描台架里 6 次"复位验证失败"全是
#   同一个原因：set_entity_state 把位姿传送回去了，但上一跑的轮子还有角动量，
#   diff_drive 的 cmd_vel_timeout 是 2 秒**仿真时间**，慢仿真下要 3~5 秒墙钟，
#   这中间车又往前滑了 0.30 m —— 车其实没问题，是台架抢跑了。
#   所以这里读 /odom 的实际线速度，等它归零，而不是傻 sleep。
wait_stopped() {
  local i vx
  for i in $(seq 1 15); do
    vx=$(timeout 8 ros2 topic echo /odom --field twist.twist.linear --once 2>/dev/null \
         | grep -oE 'x: [-0-9.eE+]+' | head -1 | tr -d 'x: ')
    if [ -n "${vx:-}" ] && python3 -c "import sys; sys.exit(0 if abs(float('$vx')) < 0.02 else 1)" 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  return 1
}

verify_start() {
  local p
  p=$(timeout 25 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
        "{name: mybot, reference_frame: world}" 2>/dev/null | tr -d '\n')
  python3 - "$p" <<'PY'
import re, sys, math
t = sys.argv[1]
m = re.search(r'position=.*?x=([-\d.eE+]+).*?y=([-\d.eE+]+)', t)
if not m:
    print('nopos'); sys.exit()
px, py = float(m.group(1)), float(m.group(2))
# ★ 判据是"在起点那段直线上"，不是"必须在 (0,-0.9) 这个点"。
#   起点直线条沿 +x 方向、y=-0.9；车复位后还会带着轮子的角动量往前滑 0.3 m 左右，
#   滑完仍然稳稳压在线上 —— 这是合法起跑点，上一版拿 0.15 m 的**点**距离去卡，
#   把它误判成"复位失败"，白重启了 6 次世界（每次约 60 秒）。
#   x 上界 0.9 m 是"还没进第一个弯"，再往前的位置就不能算起跑了。
lat = abs(py + 0.9)
# x 上界从 0.90 放宽到 1.05：起点直道到 x=1.2 才进弯，实测复位后车最多滑到 0.93，
# 那时它横向只偏 0.01 m、稳稳压在线上 —— 是合法起跑点，不该判失败（第三轮录制就栽在这）。
if lat < 0.12 and -0.35 < px < 1.05:
    print('ok')
else:
    print('bad %.2f,%.2f lat=%.2f' % (px, py, lat))
PY
}

# 抓帧循环：一直拍到被杀为止，事后用 (帧数 / 实际秒数) 反推真实帧率。
# ★ CAP_SLEEP 不是可调项而是必需项：实测裸循环能跑到 **38.7 fps**（基准测出的 10.2 fps
#   是刚开机时窗口在重绘，不代表稳态），一帧 3.5 MB → 一段 35 秒就写 4.7 GB，
#   而 gzclient 自己也就渲染 30~60 fps，多抓的全是浪费。按 ~14 fps 抓，
#   回放速率用实测帧率反推，所以画面时间仍然是真实的。
capture_loop() {
  local d="$1" wid="$2" n=0
  mkdir -p "$d"
  while :; do
    n=$((n+1))
    if [ "$CAPMETH" = "import" ]; then
      import -window "$wid" "$d/$(printf '%04d' $n).png" 2>/dev/null
    else
      xwd -id "$wid" -silent -out "$d/$(printf '%04d' $n).xwd" 2>/dev/null
    fi
    printf '%s' "$n" > "$d/.count"
    sleep "$CAP_SLEEP"
  done
}

stop_capture() {
  # 先杀循环本身，再收拾可能还在跑的最后一帧 —— 反过来会留下一个孤儿子进程
  kill -9 "$CAPJID" 2>/dev/null
  pkill -9 -f "[x]wd -id" 2>/dev/null
  pkill -9 -f "[i]mport -window" 2>/dev/null
  sleep 1
}

# run_one <tag> <kp> <kd> <w_max> <v_max> <capture:1|0> [dur=45]
run_one() {
  local tag="$1" kp="$2" kd="$3" wm="$4" vmax="$5" cap="$6" dur="${7:-45}"
  local d="$OUT/$tag" csv="$OUT/$tag.csv" verdict line e_med path wall
  rm -rf "$d" "$csv"; mkdir -p "$d"
  kill_all
  reset_car >/dev/null; sleep 2
  wait_stopped || echo "  ! $tag 等了 15 秒车还没停稳，继续验位置"
  local v; v=$(verify_start)
  if [ "$v" != "ok" ]; then
    # 第一次验证不过，多半是"复位时车还在滑"（上一跑的车带着速度被传送回去，
    # 轮子角动量没清，会再滑 0.3~1.1 m）。这时候**不需要重启世界**，
    # 等它真停下、再传一次位姿就够了 —— 世界重启会把录屏节奏整个打断。
    echo "  ! $tag 首次验证失败($v) -> 再等停 + 再复位一次"
    wait_stopped; reset_car >/dev/null; sleep 2; wait_stopped; v=$(verify_start)
  fi
  if [ "$v" != "ok" ]; then
    echo "  ★ $tag 复位失败($v)，跳过"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t-\t-\t-\t-\t-\tSKIP_RESET\n' "$tag" "$kp" "$kd" "$wm" "$vmax" "$cap" >> "$SUMMARY"
    return 1
  fi

  local n0=0
  if [ "$cap" = "1" ]; then
    capture_loop "$d" "$WID" &
    CAPJID=$!
    sleep 2
  fi
  local t0=$SECONDS
  timeout 200 ros2 run mybot_control line_follow_pd.py \
      --kp "$kp" --kd "$kd" --w-max "$wm" --v-max "$vmax" \
      --duration "$dur" --csv "$csv" --quiet > "$OUT/$tag.log" 2>&1
  local elapsed=$((SECONDS - t0))

  local frames=0
  if [ "$cap" = "1" ]; then
    stop_capture            # 里面已经 kill 了循环 + 残留的 xwd
    frames=$(cat "$d/.count" 2>/dev/null); frames=${frames:-0}
  fi
  n0=$frames

  [ -s "$csv" ] || { echo "  ★ $tag 无 CSV"; return 1; }
  line=$(python3 "$MET" "$csv" "$tag" 2>/dev/null | tail -1)
  e_med=$(printf '%s' "$line" | cut -f6)
  path=$(printf '%s' "$line" | cut -f11)
  wall=$(printf '%s' "$line" | cut -f12)
  if echo "$line" | grep -q "一圈完成"; then verdict=PASS; else verdict=NO_LAP; fi
  local fps='-'
  [ "$n0" -gt 4 ] && fps=$(python3 -c "print('%.1f' % ($n0/max($elapsed,1)))")
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$tag" "$kp" "$kd" "$wm" "$vmax" "$cap" "$n0" "$fps" "$e_med" "$path" "$wall" "$verdict" >> "$SUMMARY"
  printf '  %-10s cap=%s 帧=%s (%s fps)  |e|=%s cm  路径=%s m  墙钟=%s s  %s\n' \
      "$tag" "$cap" "$n0" "$fps" "$e_med" "$path" "$wall" "$verdict"
  echo "$elapsed" > "$OUT/$tag.elapsed"
}

encode() {   # encode <tag> -> mp4
  local tag="$1" d="$OUT/$tag" el fps ext
  el=$(cat "$OUT/$tag.elapsed" 2>/dev/null || echo 1)
  [ -f "$d/.count" ] || return 1
  local n; n=$(cat "$d/.count")
  [ "$n" -gt 6 ] || { echo "  ★ $tag 帧数太少($n)"; return 1; }
  fps=$(python3 -c "print('%.2f' % max(4.0, min(25.0, $n/max($el,1))))")
  if [ "$CAPMETH" = "import" ]; then ext=png; else ext=xwd; fi
  # ★ 窗口实测 2548x1495（HiDPI），原样编码一段 35 s 就有几十 MB 且 hstack 出来宽 5096 px。
  #   统一降到 1280 宽、高度取偶数（-2 自动按比例且保证偶数，libx264 要求偶数）。
  timeout 300 ffmpeg -hide_banner -loglevel error -y -framerate "$fps" \
      -i "$d/%04d.$ext" -vf "scale=1280:-2" \
      -c:v libx264 -pix_fmt yuv420p -crf 22 "$OUT/$tag.mp4" 2>&1 | head -3
  local sz; sz=$(stat -c%s "$OUT/$tag.mp4" 2>/dev/null || echo 0)
  echo "  $tag.mp4 $n 帧 @ ${fps} fps -> $sz bytes"
  # 帧目录一段 4.7 GB，编码成功就立刻删（mp4 + CSV 才是留档的东西）
  [ "$sz" -gt 2000 ] && rm -rf "$d"
  [ "$sz" -gt 2000 ]
}

echo "======================================================================"
echo " D12 桌面实录（xwd 抓帧 -> ffmpeg 编码）"
echo "======================================================================"
start_world    || { echo "★ 带 GUI 的世界起不来"; exit 1; }
start_features || { echo "★ 视觉节点起不来"; exit 1; }
WID=$(find_window) || { echo "★ 找不到 Gazebo 窗口"; exit 1; }
echo "窗口 id = $WID"

# [0] 抓帧吞吐：各拍 8 帧，确认这一台机器上哪种更快（探测轮是 12.8 vs 5.2）
echo; echo "=== [0] 抓帧吞吐 ==="
for m in xwd import; do
  t0=$(date +%s.%N); i=0
  while [ $i -lt 8 ]; do
    i=$((i+1))
    if [ "$m" = "xwd" ]; then xwd -id "$WID" -silent -out "$OUT/bench_$i.xwd" 2>/dev/null
    else import -window "$WID" "$OUT/bench_$i.png" 2>/dev/null; fi
  done
  t1=$(date +%s.%N)
  python3 -c "print('  %-7s 8 帧 %.2fs -> %.1f fps' % ('$m', $t1-$t0, 8/($t1-$t0)))"
done
rm -f "$OUT"/bench_*.xwd "$OUT"/bench_*.png
# 内容非黑检查：拿当前方法拍一张，看均值（纯黑=0，有画面>0.01）
if [ "$CAPMETH" = "import" ]; then import -window "$WID" "$OUT/chk.png" 2>/dev/null
else xwd -id "$WID" -silent -out "$OUT/chk.xwd" 2>/dev/null; convert "$OUT/chk.xwd" "$OUT/chk.png" 2>/dev/null; fi
convert "$OUT/chk.png" -format "  抓帧内容检查 mean=%[fx:mean] size=%wx%h\n" info: 2>/dev/null

declare -a TAGS LABELS
i=0
IFS=',' read -ra _specs <<< "$CLIPS"
for spec in "${_specs[@]}"; do
  IFS=: read -r tag kp kd wm vmax cap dur label <<< "$spec"
  echo
  echo "=== [$((i+1))] $label  (kp=$kp kd=$kd w_max=$wm v_max=$vmax 录屏=$cap ${dur}s) ==="
  run_one "$tag" "$kp" "$kd" "$wm" "$vmax" "$cap" "$dur"
  TAGS[$i]="$tag"; LABELS[$i]="$label"
  i=$((i+1))
done

echo; echo "=== 编码 ==="
j=0
for tag in "${TAGS[@]}"; do
  if encode "$tag"; then
    cp -f "$OUT/$tag.mp4" "$WIN/d12_${LABELS[$j]}.mp4" 2>/dev/null \
      && echo "  -> scratch/d12_${LABELS[$j]}.mp4"
  fi
  j=$((j+1))
done
# 左右分屏：整定后 vs 过冲蛇形（两段都跑完一圈，放一起对比最直观）
if [ -f "$OUT/tuned.mp4" ] && [ -f "$OUT/weave.mp4" ]; then
  timeout 300 ffmpeg -hide_banner -loglevel error -y \
    -i "$OUT/weave.mp4" -i "$OUT/tuned.mp4" -filter_complex hstack -shortest \
    -c:v libx264 -pix_fmt yuv420p -crf 22 "$OUT/compare.mp4" 2>&1 | head -3
  cp -f "$OUT/compare.mp4" "$WIN/d12_左右对比_蛇形vs整定后.mp4" 2>/dev/null \
    && echo "  -> scratch/d12_左右对比_蛇形vs整定后.mp4（左：过冲蛇形 右：整定后）"
fi

echo
echo "======================================================================"
echo "汇总 $SUMMARY"
column -t -s $'\t' "$SUMMARY" 2>/dev/null || cat "$SUMMARY"
mkdir -p "$WIN/d12rec" 2>/dev/null && cp -f "$OUT"/*.csv "$OUT/summary.tsv" "$WIN/d12rec/" 2>/dev/null \
  && echo "CSV 与汇总已拷到 scratch/d12rec/（mp4 在 scratch/ 根目录）"
