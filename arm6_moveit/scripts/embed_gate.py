#!/usr/bin/env python3
"""
判据表达式：并发测试里"阶段一侧有没有被扰动"的那道关卡。

为什么单独成一个文件（而不是留在 check_concurrency.sh 的 heredoc 里）:
    关卡逻辑内嵌在 shell 里就**没法单独证伪** —— 想验证"它会 FAIL"，
    只能再跑一次完整的 33 s 并发测试（起 ROS、起 QEMU 采集），
    代价高，于是大家就不验了；一个从没见它 FAIL 过的判据和一个坏掉的判据
    长得很像。抽出来之后，喂两串字段就能测，negative self-test 才做得起。
    （同 05 §6.1 的教训：判据必须能被它自己打印的量喂回去。）

输入是两个 `phys_from_summary()` 的输出串（check_concurrency.sh:196），形如：
    WHITE=0.06/0.9 GREY=0.04/0.1 BLACK=0.13/1.2 n=1599 edges=8 detects=4
两份都由同一个 `plot_adc.py` 生成 ⇒ 舍入口径一致，可以直接比。

关卡用**两态时基对齐都不会动**的量（阶段一 docs/03 §4-1 形态 b：
模拟量列天生有 `pot_raw[0]` ∈ {1,2} 两种离散对齐态，换态时整链平移约 0.2 % 量程，
**与有没有并发负载无关**）：
    · `n`        允许 ±tol_n 帧（窗口末端可能卡在 8000 ms 边界上）
    · `edges`    必须精确相等
    · `detects`  必须精确相等
    · 三面 `max|err|` 必须精确相等
    · 三面 MAE      允许 ±tol_mae mm
"逐格是否相同"(`analog_cells=0`) **不在这里判** —— 那是随机误报的来源。

用法:
    python3 scripts/embed_gate.py <BASE_PHYS> <CUR_PHYS> [TOL_MAE] [TOL_N]
输出（stdout 一行）:
    OK                       —— 六项验收量都在关卡内
    summary-missing          —— 字段取不全（等于没测，调用方按 FAIL 计）
    "n 1599->1601 edges 8->7 ..."  —— 列出**具体哪几项**越界，不写"不通过"三个字
退出码恒为 0：判定由调用方读 stdout 做，避免 shell 的 `set -e` 与管道退出码歧义
（同一类坑见 05 §6.1 的 `timeout` 返回 124 那个 bug）。
"""
from __future__ import annotations

import sys


def parse(s: str) -> dict:
    d = {}
    for tok in s.split():
        k, _, v = tok.partition('=')
        d[k] = v
    return d


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print("usage: embed_gate.py <BASE_PHYS> <CUR_PHYS> [TOL_MAE] [TOL_N]")
        print("summary-missing")
        return 0
    b, c = parse(argv[1]), parse(argv[2])
    tol_mae = float(argv[3]) if len(argv) > 3 else 0.02
    tol_n = int(argv[4]) if len(argv) > 4 else 1

    bad: list[str] = []
    if b.get('n', 'NA') == 'NA' or c.get('n', 'NA') == 'NA':
        print('summary-missing')          # 没测出来过 = 不算通过
        return 0
    if abs(int(b['n']) - int(c['n'])) > tol_n:
        bad.append(f"n {b['n']}->{c['n']}")
    for k in ('edges', 'detects'):
        if b.get(k) != c.get(k):
            bad.append(f"{k} {b.get(k)}->{c.get(k)}")
    for s in ('WHITE', 'GREY', 'BLACK'):
        if s not in b or s not in c:
            bad.append(f"{s}:absent")
            continue
        mb, xb = (float(x) for x in b[s].split('/'))
        mc, xc = (float(x) for x in c[s].split('/'))
        if abs(mb - mc) > tol_mae + 1e-9:
            bad.append(f"{s} mae {mb}->{mc}")
        if xb != xc:
            bad.append(f"{s} max {xb}->{xc}")
    print(' '.join(bad) if bad else 'OK')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
