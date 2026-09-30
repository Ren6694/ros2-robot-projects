#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/check_concurrency.sh   (在 WSL 里跑)
#  作用: 让阶段一的 QEMU 采集**真正压在**机械臂的执行段上，测两个工作空间互不干扰
#
#  ★ 这个脚本修的是"测试设计"缺陷，不是代码缺陷 ★
#    上一轮并发试跑（ROS run 005801 / embed run 005755）得出的结论是"并发没有影响"。
#    事后把两边的时间戳摆到同一根轴上才发现：QEMU 只在 ROS 栈 bringup 期间跑了 8 s，
#    机械臂真正运动的 22 s 里 CPU 是空的 —— 那次测的是"顺序执行"，
#    结论却被写成"并发执行"。错在哪？错在**用"我同时启动了"代替"两段确实重叠"**。
#    所以本脚本的第一判据不是"有没有出错"，而是"重叠够不够"：
#        overlap_s / exec_dur < REQ_OVERLAP  =>  CONCURRENCY-RESULT FAIL
#    一个没重叠的并发测试打出 PASS，比打出 FAIL 危险得多 —— 它会让人停止怀疑。
#    （与 docs/07 §3 第 1 条同源：先证伪判据，再相信结论。）
#
#  怎么保证重叠（不用 sleep 猜）：
#    点位节点是逐条 append 并 flush JSONL 的（motion_log.py:229-230），
#    所以"第 2 行落地"就是"idx=1 已经执行完"的确凿信号 —— 这一刻才起 QEMU，
#    采集负载必然落在 idx=2..9 的运动段里。
#    另外先预编译一次固件：否则第一次采集的前几秒跑的是 gcc 而不是 QEMU，
#    重叠的就是"编译负载"而不是"仿真负载"，那不是本测试要问的问题。
#
#  产物:
#    logs/concurrency_<stamp>.txt   时间轴 + OVERLAP 行 + 结果行（本脚本的 verdict 落盘，
#                                   理由同 demo_<stamp>.log：终端滚过去就不算证据）
#    logs/concurrency_demo_<stamp>.log  被调起的 run_waypoint_demo.sh 的完整 stdout
#    logs/embed_capture_<stamp>_<i>.log 阶段一 capture.sh 的 stdout
#      （为什么阶段一的日志放在 ros2_ws/logs：这次运行由 ROS 侧发起，日志跟着发起方走；
#        阶段一自己的 console/adc 产物仍落在 ~/embed_sim_ws 里，不新造数据格式。）
#
#  用法:
#    bash scripts/check_concurrency.sh 3 20260917-005248    # 双向都判：3 次采集 + 与指定基线比 ADC
#    bash scripts/check_concurrency.sh                       # 3 次采集，**不比对** ⇒ 必 FAIL（见下）
#    REQ_OVERLAP=0.5 bash scripts/check_concurrency.sh 3 …   # 放宽重叠下限（只用于看判据怎么fail）
#    TOL_MAE=0.05 / N_TOL=2 可放宽物理量容差（同样只用于自测判据，别拿它凑 PASS）
#
#  退出码: 0 = 重叠达标 且 ROS 侧 demo PASS 且 Δt 在控制周期内 且 每次采集的物理验收量与基线一致；
#          非 0 = 上述任一条不成立。**不给基线 ⇒ FAIL**：并发是双向命题，
#          只测 ROS→QEMU 一个方向就不算测过，"没测"不能让判据默认通过（docs/07 §3 第 1 条）。
#
#  ★ 结论的边界（写进文档时不要越过）★
#    本机 16 vCPU，QEMU 主要是单线程 + 一个 GDB，所以本测试能证明的是
#    "两栈在同机并发时互不破坏功能与精度"，**不能**推广成"1 核机器上也不会互相拖累"。
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
EMBED_WS="${EMBED_WS:-$HOME/embed_sim_ws}"
EMBED_RUNS="${1:-3}"
EMBED_BASELINE="${2:-}"          # 阶段一基线 adc_samples 的 stamp（空 = 不比对，直接判 FAIL）
REQ_OVERLAP="${REQ_OVERLAP:-0.7}"  # 执行段至少被覆盖 70% 才算"真并发"
DT_LIMIT_MS="${DT_LIMIT_MS:-10}"   # 跟踪误差折算时间 = err/v_max，一个控制周期(100Hz=10ms)以内算正常

cd "$WS"
mkdir -p logs
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="logs/concurrency_${STAMP}.txt"
DLOG="logs/concurrency_demo_${STAMP}.log"

say() { printf '%s\n' "$*" | tee -a "$OUT"; }
: > "$OUT"
say "run_id=$STAMP embed_runs=$EMBED_RUNS req_overlap=$REQ_OVERLAP dt_limit_ms=$DT_LIMIT_MS embed_baseline=${EMBED_BASELINE:-<none>}"

[ -x "$EMBED_WS/scripts/capture.sh" ] || { say ">>> CONCURRENCY-RESULT FAIL (找不到 $EMBED_WS/scripts/capture.sh)"; exit 2; }
source scripts/ros_env.sh

echo ">>> [1/5] 预编译阶段一固件（把编译挪出重叠窗口）"
"$EMBED_WS/scripts/build.sh" sense >/dev/null 2>&1 || { say ">>> CONCURRENCY-RESULT FAIL (embed build.sh 预编译失败)"; exit 2; }

echo ">>> [2/5] 后台起机械臂全量回归（规划+执行）"
T0="$(date +%s)"
bash scripts/run_waypoint_demo.sh full 300 > "$DLOG" 2>&1 &
DEMO_PID=$!
trap 'kill "$DEMO_PID" 2>/dev/null || true' EXIT

echo ">>> [3/5] 等执行段开始（轮询新 JSONL 的第 2 行，最多 90s）"
JSONL=""
for _ in $(seq 1 450); do
    JSONL="$(find logs -maxdepth 1 -name 'motion_*.jsonl' -newermt "@$T0" 2>/dev/null | head -1)"
    if [ -n "$JSONL" ] && [ "$(wc -l < "$JSONL")" -ge 2 ]; then break; fi
    sleep 0.2
done
if [ -z "$JSONL" ] || [ "$(wc -l < "$JSONL")" -lt 2 ]; then
    say ">>> CONCURRENCY-RESULT FAIL (90s 内没等到执行段，QEMU 一次都没起 —— 并发窗口根本没构成)"
    kill "$DEMO_PID" 2>/dev/null || true
    tail -30 "$DLOG"
    exit 1
fi
RUN_ID="$(basename "$JSONL" .jsonl | sed 's/^motion_//')"
say "ros_run_id=$RUN_ID jsonl=$JSONL exec_started_at=$(date -d "@$T0" +%H:%M:%S)+~$(awk "BEGIN{printf \"%.1f\", $(date +%s.%N)-$T0}")s"

echo ">>> [4/5] 执行段内跑 $EMBED_RUNS 次 QEMU 采集"
WINS=()
OK_RUNS=0
for i in $(seq 1 "$EMBED_RUNS"); do
    S="$(date +%s.%N)"
    ( cd "$EMBED_WS" && ./scripts/capture.sh sense ) > "logs/embed_capture_${STAMP}_${i}.log" 2>&1
    RC=$?
    E="$(date +%s.%N)"
    [ "$RC" -eq 0 ] && OK_RUNS=$((OK_RUNS + 1))
    say "EMBED i=$i rc=$RC start=$S end=$E log=logs/embed_capture_${STAMP}_${i}.log"
    WINS+=("$i:$S:$E:$RC")
done

echo ">>> [5/5] 收摊：等 ROS 回归结束并取它的结果行"
wait "$DEMO_PID"; DEMO_RC=$?
trap - EXIT
DEMO_VERDICT="$(grep -oE 'DEMO-RESULT [A-Z]+.*' "$DLOG" | tail -1)"
say "demo_rc=$DEMO_RC ${DEMO_VERDICT:-DEMO-RESULT MISSING}"

# ---------------------------------------------------------------------------
# 时间轴对齐 + 重叠计算。所有窗口都从**已落盘的证据**反读：
#   ROS 侧 = JSONL 里每条点位的 ts（墙钟，node 本机同一时区）
#   QEMU 侧 = 上面 say 出去的 start/end
# 不在这里判"精度有没有变差"（那归 compare_motion_runs.py），这里只判"重叠够不够"。
# ---------------------------------------------------------------------------
ANALYSIS="$(python3 - "$JSONL" "$REQ_OVERLAP" "${WINS[@]}" <<'PY'
import json, sys, datetime

jsonl, req, wins = sys.argv[1], float(sys.argv[2]), sys.argv[3:]
rows = []
for line in open(jsonl, encoding="utf-8"):
    if not line.strip():
        continue
    d = json.loads(line)
    if "idx" in d and "ts" in d:
        rows.append(d)
if not rows:
    print("ANALYSIS-ERROR JSONL 里没有带 idx/ts 的点位行"); sys.exit(3)

def epoch(ts):
    return datetime.datetime.fromisoformat(ts).timestamp()

# 第 i 条的 ts 是"这条执行完"的时刻，它自己的 plan+exec 发生在 ts 之前，
# 所以窗口左端从第一条往前推它自己的耗时 —— 宁可少算，不虚报覆盖率。
t_start = epoch(rows[0]["ts"]) - (rows[0].get("plan_s", 0) + rows[0].get("exec_s", 0))
t_end = epoch(rows[-1]["ts"])
dur = t_end - t_start

covered = 0.0
lines = [f"ROS exec_window t0={t_start:.3f} t1={t_end:.3f} dur_s={dur:.2f} waypoints={len(rows)}"]
for w in wins:
    idx, s, e, rc = w.split(":")
    s, e = float(s), float(e)
    ov = max(0.0, min(e, t_end) - max(s, t_start))
    covered += ov
    lines.append(f"OVERLAP i={idx} rc={rc} dur_s={e - s:.2f} overlap_exec_s={ov:.2f} "
                 f"frac_of_exec={ov / dur:.2f}")
frac = covered / dur if dur > 0 else 0.0
lines.append(f"COVERED exec_s={covered:.2f}/{dur:.2f} frac={frac:.2f} req={req:.2f}")
print("\n".join(lines))
sys.exit(0 if frac >= req else 1)
PY
)"
AN_RC=$?
printf '%s\n' "$ANALYSIS" | tee -a "$OUT"

# ★ 第一判据没过 ⇒ 这是一次**无效的测试**，不是被测系统的问题。
#   后面的 Δt / ADC 数字照样算（诊断有用），但必须先声明它们不能进结论：
#   重叠不足时给出的"误差正常"，正是本轮撤回 005801 那条结论的形状（docs/05 §6.1 ⚠ 段）。
if [ "$AN_RC" -ne 0 ]; then
    say "!! OVERLAP-GATE FAILED —— 本次测试无效（并发窗口重叠不足 req=${REQ_OVERLAP}）"
    say "!! 以下 Δt / ADC 数字**仅供参考**，不得写进任何回归结论"
fi

# Δt：跟踪误差折算成时间，直接量出"并发把控制环拖慢了几个采样周期"。
# plan-only 运行不再写进 --exclude：compare_motion_runs.py 现在自己认得出它们
# （名单只留一份真源，否则每加一次回归就要改两个脚本 —— docs/00 §7 规则 1）。
CMP_OUT="logs/compare_${STAMP}_concurrency.txt"
( cd logs && python3 ../scripts/compare_motion_runs.py --col track_err_rad \
    --exclude 234501,231533 \
    motion_2026*.csv motion_rviz_2026*.csv ) > "$CMP_OUT" 2>&1
CMP_RC=$?
say "compare_rc=$CMP_RC out=$CMP_OUT"
say "$(grep -E '^(Δt = |DT cols=|DT max |EXCLUDED |COMPARE-RESULT)' "$CMP_OUT" | tr '\n' '|' | sed 's/|$//')"
RUN_TAG="${RUN_ID#*-}"          # compare 打印的是被截短的 tag（如 60917-005801），按 时分秒 认列
DT_MAX="$(awk -v r="$RUN_TAG" '$1=="DT" && $2=="max" && index($3, r) { print $4 }' \
        "$CMP_OUT" | tail -1)"

# 阶段一方向：**每一次**并发采集都单独比对（旧版只比 `ls -t | head -1`，等于 3 次只测 1 次）。
#
# 为什么不拿 `analog_cells=0` 当关卡（这是本轮重测后改的，别"收紧"回去）：
#   ADC 模型有**两种离散时基对齐态**（阶段一 docs/03 §4-1 形态 b；实测 9 份归档的
#   `pot_raw[0]` 只有 `1`/`2` 两种取值，取 `1` 的两份彼此逐格相同、其中一份与并发毫无关系）。
#   落在另一态时整链平移约 0.2 % 量程，**与有没有负载无关** ⇒ 拿"逐格相同"当关卡
#   = 造一个会随机误报的判据。
# 关卡改用两态都不会动的量：`DO 边沿`、`低电平检出`、各面 `max|err|` 要求**精确相等**；
#   各面 MAE 容差 ${TOL_MAE} mm；样本数 ±${N_TOL}（窗口末端卡在 8000 ms 边界）。
#   逐格是否相同照样打印并存档，但标 `not-gated`。
TOL_MAE="${TOL_MAE:-0.02}"
N_TOL="${N_TOL:-1}"
PHYS_OK=0; SAME_K=0; CMP_RAN=0

# 从 plot_summary_*.md 取 6 个验收量：n edges detects mae_white mae_grey mae_black
# + 3 个 max|err|。两份文件都由同一个 plot_adc.py 生成 ⇒ 舍入口径一致，可比。
# 抽取规则放在单独文件里，和 scripts/selftest_embed_gate.sh 共用同一份
# （自测必须测真代码，不能测一份拷贝）。
if [ ! -f scripts/phys_from_summary.sh ]; then
    say "FATAL scripts/phys_from_summary.sh 不存在 —— 关卡的字段抽取没有真源，拒绝在降级状态下判 PASS"
    exit 2
fi
# shellcheck source=phys_from_summary.sh
. scripts/phys_from_summary.sh

if [ -n "$EMBED_BASELINE" ]; then
    BASE_CSV="$EMBED_WS/artifacts/adc_samples_${EMBED_BASELINE}.csv"
    BASE_MD="$EMBED_WS/artifacts/plot_summary_${EMBED_BASELINE}.md"
    if [ ! -f "$BASE_CSV" ] || [ ! -f "$BASE_MD" ]; then
        say "EMBED-CMP SKIPPED (基线 $EMBED_BASELINE 的 CSV/plot_summary 不存在 —— 这项没测，别在文档里引用它)"
        PHYS_OK=-1
    else
        BASE_PHYS="$(phys_from_summary "$BASE_MD")"
        say "EMBED-BASELINE $EMBED_BASELINE $BASE_PHYS"
        for i in $(seq 1 "$EMBED_RUNS"); do
            CLOG="logs/embed_capture_${STAMP}_${i}.log"
            CSV="$(grep -oE 'PLOT-ARTIFACT .*/adc_samples_[^ ]*\.csv' "$CLOG" 2>/dev/null | tail -1 | awk '{print $2}')"
            if [ -z "$CSV" ] || [ ! -f "$CSV" ]; then
                say "EMBED-CMP i=$i NO-ARTIFACT (采集日志里没有 PLOT-ARTIFACT —— 按不通过计)"
                PHYS_OK=$((PHYS_OK)); continue
            fi
            TAG="$(basename "$CSV" .csv)"; TAG="${TAG#adc_samples_}"
            MD="$EMBED_WS/artifacts/plot_summary_${TAG}.md"
            CI_OUT="logs/compare_${STAMP}_embed_${i}.txt"
            ( cd "$EMBED_WS" && python3 scripts/compare_csv_runs.py "$BASE_CSV" "$CSV" --fail-on-analog ) \
                > "$CI_OUT" 2>&1
            CMP_RAN=$((CMP_RAN + 1))
            if grep -q 'VERDICT SAME' "$CI_OUT"; then SAME_K=$((SAME_K + 1)); SAME=identical; else SAME="differs(not-gated)"; fi
            CUR_PHYS="$(phys_from_summary "$MD")"
            # 逐字段比：n 容差 N_TOL，edges/detects 与三个 max 精确，三个 MAE 差值 <= TOL_MAE
            # 表达式抽到 scripts/embed_gate.py（同一个文件也用于 negative self-test，
            # 免得"关卡长什么样"在两处各存一份）
            BAD="$(python3 scripts/embed_gate.py "$BASE_PHYS" "$CUR_PHYS" "$TOL_MAE" "$N_TOL")"
            say "EMBED-CMP i=$i run=$TAG cells=$SAME phys=$BAD  detail=$CI_OUT"
            [ "$BAD" = "OK" ] && PHYS_OK=$((PHYS_OK + 1))
        done
        say "EMBED-CMP 汇总: phys_ok=${PHYS_OK}/${EMBED_RUNS} cells_identical=${SAME_K}/${CMP_RAN} \
(逐格相同**不参与判定**，理由见上面注释；关卡=边沿/检出/max|err| 精确 + MAE<=${TOL_MAE}mm + n±${N_TOL})"
    fi
else
    say "EMBED-CMP NOT-RUN (第 2 个参数为空：阶段一方向**未测** ⇒ 本次判 FAIL。并发是双向命题)"
    PHYS_OK=-1
fi

PASS=true
[ "$AN_RC" -eq 0 ] || PASS=false
[ "$OK_RUNS" -eq "$EMBED_RUNS" ] || PASS=false
[ "$PHYS_OK" -eq "$EMBED_RUNS" ] || PASS=false      # 含 -1(未跑) 与 SKIPPED 的情况：没测就不算通过
echo "$DEMO_VERDICT" | grep -q "PASS" || PASS=false
[ -n "$DT_MAX" ] && awk "BEGIN{exit !($DT_MAX > $DT_LIMIT_MS)}" && PASS=false

say ">>> CONCURRENCY-RESULT $([ "$PASS" = true ] && echo PASS || echo FAIL) \
overlap_frac=$(printf '%s\n' "$ANALYSIS" | sed -n 's/^COVERED.*frac=\([0-9.]*\).*/\1/p') \
(req>=${REQ_OVERLAP}) dt_max_ms=${DT_MAX:-n/a}(req<${DT_LIMIT_MS}) \
embed_ok=${OK_RUNS}/${EMBED_RUNS} embed_phys=${PHYS_OK}/${EMBED_RUNS} \
embed_same=${SAME_K}/${CMP_RAN}(not-gated) demo=$(echo "$DEMO_VERDICT" | grep -oE 'PASS|FAIL' | head -1 || echo MISSING) \
verdict_log=${OUT}"
$PASS && exit 0
exit 1
