#!/usr/bin/env bash
# =============================================================================
#  从阶段一的 `plot_summary_<stamp>.md` 里取出**验收量**，压成一行 key=value：
#      WHITE=0.06/0.9 GREY=0.04/0.1 BLACK=0.13/1.2 n=1599 edges=8 detects=4
#  （每面是 `MAE/max|err|`，单位 mm）
#
#  为什么单独一个文件：`check_concurrency.sh` 的关卡和
#  `selftest_embed_gate.sh` 的自测必须读**同一份**抽取规则，
#  否则自测通过只证明了一份拷贝是对的。
#
#  取法刻意的两点：
#   1. `grep -oE` 而不是"整行 sed" —— summary 是人读的 markdown，字段位置会变，
#      锚在字段名上比锚在列号上活得久。
#   2. 取不到就留 NA，由 `embed_gate.py` 判成 `summary-missing`（= 没测 = FAIL），
#      不在这里猜 0 —— 把"缺失"当成"0"是所有静默误判的源头。
# =============================================================================

phys_from_summary() {
    local f="$1"
    [ -f "$f" ] || { echo "MISSING"; return 1; }
    local n ed dtt
    n="$(grep -oE '样本数: [0-9]+' "$f" | grep -oE '[0-9]+' | head -1)"
    ed="$(grep -oE '边沿 [0-9]+ 次' "$f" | grep -oE '[0-9]+' | head -1)"
    dtt="$(grep -oE '低电平检出 [0-9]+ 次' "$f" | grep -oE '[0-9]+' | head -1)"
    awk -F'|' '/^\| (WHITE|GREY|BLACK) /{gsub(/ /,"",$2);gsub(/ /,"",$4);gsub(/ /,"",$5);printf "%s=%s/%s ",$2,$4,$5}
               END{printf "n=%s edges=%s detects=%s\n", "'"${n:-NA}"'","'"${ed:-NA}"'","'"${dtt:-NA}"'"}' "$f"
}
