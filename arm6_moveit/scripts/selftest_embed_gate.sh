#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/selftest_embed_gate.sh   (在 WSL 里跑)
#  作用: 证明 `check_concurrency.sh` 的阶段一侧关卡**会 FAIL**
#
#  为什么需要它（这一步不能省）:
#    关卡 `embed_phys=N/N` 从写下到第一次真跑，只见过 OK。
#    一个从没见它 FAIL 过的判据，和一个恒真的判据，输出完全一样。
#    上一轮就栽在同源的地方：旧版印 `embed_ok=3/3`（其实只比了 1 次采集），
#    因为它从来没在"少比一次"的场景下 FAIL 过，所以没人发现。
#    所以这里做的是**正/负对照**：
#      正 —— 拿 6 份真实归档采集（两次并发测试的全部采集）逐份与基线比 ⇒ 必须全 OK；
#      负 —— 把基线那串字段人为改掉一个（边沿 -1 / max|err| +0.1 / MAE 越界 / n 越界），
#            关卡必须**点名**说出是哪一项，而不是含糊地"不通过"。
#    只喂字符串、不启动任何栈：这也是把表达式抽成 embed_gate.py 的全部理由 ——
#    内嵌在 shell 里时，做一次自测就得跑一整轮 33 s 的并发测试，于是自测永远不会做。
#
#  用法:
#    bash scripts/selftest_embed_gate.sh                      # 默认基线 20260917-005248
#    EMBED_WS=$HOME/embed_sim_ws bash scripts/selftest_embed_gate.sh
#
#  退出码: 0 = 正对照全 OK 且 4 个负对照全被点名；非 0 = 关卡不可信，别引用 CONCURRENCY-RESULT
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
EMBED_WS="${EMBED_WS:-$HOME/embed_sim_ws}"
BASELINE="${BASELINE:-20260917-005248}"
CAPTURES="${CAPTURES:-20260917-013005 20260917-013015 20260917-013025 20260917-083603 20260917-083613 20260917-083622}"
TOL_MAE="${TOL_MAE:-0.02}"
N_TOL="${N_TOL:-1}"

cd "$WS"
mkdir -p logs
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="logs/embed_gate_selftest_${STAMP}.txt"
say() { printf '%s\n' "$*" | tee -a "$OUT"; }
: > "$OUT"
say "run_id=$STAMP baseline=$BASELINE tol_mae=$TOL_MAE tol_n=$N_TOL captures=$(echo "$CAPTURES" | wc -w)"

[ -f scripts/phys_from_summary.sh ] || { say "FATAL 缺 scripts/phys_from_summary.sh"; exit 2; }
[ -f scripts/embed_gate.py ] || { say "FATAL 缺 scripts/embed_gate.py"; exit 2; }
# shellcheck source=phys_from_summary.sh
. scripts/phys_from_summary.sh

BASE_MD="$EMBED_WS/artifacts/plot_summary_${BASELINE}.md"
[ -f "$BASE_MD" ] || { say "FATAL 基线 summary 不存在: $BASE_MD"; exit 2; }
BASE_PHYS="$(phys_from_summary "$BASE_MD")"
say "BASELINE $BASELINE $BASE_PHYS"

fail=0

# ---- 正对照：真实归档采集必须全部 OK -------------------------------------
pos_ok=0
for tag in $CAPTURES; do
    md="$EMBED_WS/artifacts/plot_summary_${tag}.md"
    if [ ! -f "$md" ]; then
        say "POS $tag MISSING-SUMMARY  =>  FAIL"
        fail=1; continue
    fi
    cur="$(phys_from_summary "$md")"
    got="$(python3 scripts/embed_gate.py "$BASE_PHYS" "$cur" "$TOL_MAE" "$N_TOL")"
    if [ "$got" = "OK" ]; then
        pos_ok=$((pos_ok + 1)); say "POS $tag $cur => OK"
    else
        fail=1; say "POS $tag $cur => 越界(不该发生) $got"
    fi
done
say "POS 汇总: $pos_ok/$(echo "$CAPTURES" | wc -w) 份真实采集通过关卡"

# ---- 负对照：每一项越界都必须被点名 ---------------------------------------
# 用 sed 从 BASE_PHYS 派生"恰好错一项"的四串，逐个喂回同一个表达式。
neg_case() {
    local name="$1" mutated="$2"
    local got
    got="$(python3 scripts/embed_gate.py "$BASE_PHYS" "$mutated" "$TOL_MAE" "$N_TOL")"
    if [ "$got" = "OK" ]; then
        fail=1; say "NEG $name 关卡没抓到 => FAIL（关卡不可信）"
    else
        say "NEG $name 被抓到: $got"
    fi
}

# 派生"恰好错一项"的几串时用 python 而不是 sed/awk：要改的是**数值**，
# 用文本替换很容易把相邻字段一起改掉，那样一次负对照就不只测一条判据了。
M_EDGES="$(python3 -c 'import sys;s=sys.argv[1];print(" ".join(("edges=%d"%(int(t.split("=")[1])-1)) if t.startswith("edges=") else t for t in s.split()))' "$BASE_PHYS")"
M_MAXERR="$(python3 -c 'import sys;s=sys.argv[1];print(" ".join(("BLACK=%.2f/%.1f"%(float(t.split("=")[1].split("/")[0]),float(t.split("=")[1].split("/")[1])+0.1)) if t.startswith("BLACK=") else t for t in s.split()))' "$BASE_PHYS")"
M_MAE="$(python3 -c 'import sys;s=sys.argv[1];print(" ".join(("GREY=%.2f/%s"%(float(t.split("=")[1].split("/")[0])+1.0,t.split("=")[1].split("/")[1])) if t.startswith("GREY=") else t for t in s.split()))' "$BASE_PHYS")"
M_N="$(python3 -c 'import sys;s=sys.argv[1];print(" ".join(("n=%d"%(int(t.split("=")[1])+5)) if t.startswith("n=") else t for t in s.split()))' "$BASE_PHYS")"

neg_case "edges-1"      "$M_EDGES"
neg_case "max|err|+0.1" "$M_MAXERR"
neg_case "MAE+1.0mm"    "$M_MAE"
neg_case "n+5"          "$M_N"

# 缺字段（= 没测过）必须报 summary-missing，而不是 OK
M_MISS="$(python3 -c 'import sys;s=sys.argv[1];print(" ".join(t for t in s.split() if not t.startswith("n=")))' "$BASE_PHYS")"
got="$(python3 scripts/embed_gate.py "$BASE_PHYS" "$M_MISS" "$TOL_MAE" "$N_TOL")"
[ "$got" = "summary-missing" ] && say "NEG missing-n 被抓到: $got" || { fail=1; say "NEG missing-n 关卡没抓到 ($got) => FAIL"; }

if [ "$fail" -eq 0 ]; then
    say ">>> EMBED-GATE-SELFTEST PASS positive=${pos_ok}/$(echo "$CAPTURES" | wc -w) negative=5/5  (out=$OUT)"
else
    say ">>> EMBED-GATE-SELFTEST FAIL  (out=$OUT)"
fi
exit "$fail"
