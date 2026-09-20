#!/usr/bin/env python3
"""D12 方案 A 汇总：把 raw.tsv（或 d12_recompute.py 的扩展表）压成"中位数 + 极差"的参数表。

为什么要中位数：单跑的噪声底实测 |e| ±0.23 cm、单圈 ±2.0 s，比 PD 组合之间的差距还大，
单次跑出来的名次没有意义。

★ 为什么主指标是**里程加权**的 |e|（e_dist），不是逐帧的 e_mean：
  逐帧平均会被"慢"污染 —— 20 Hz 采样下，车在弯里爬得越久，弯道帧占比越高，
  均值越偏向弯道的误差；而"改 w_max"恰好就是改弯道速度。两个配置比的是同一个
  逐帧指标，其实一半在比谁走得慢。
  实测对比（同配置三次）：逐帧极差 0.23 cm，里程加权极差 **0.03 cm** —— 噪声直接小一个数量级，
  所以判定"同档"用的容差也按 e_dist 给。

★ 口径：
  - 只用 PASS（真跑完一圈）的跑次算中位数；OFF_TRACK 单独计数并显示，
    但绝不参与平均 —— "这个增益跑不完一圈"本身就是结论。
  - 输入文件里缺哪一列就不报哪一列（扫描自带的 raw.tsv 没有 e_dist，仍可单独出表）。

用法：python3 d12_stats.py <raw.tsv> <out.md> [--tol-cm 0.05]
"""
import argparse
import csv
import math
import statistics
import sys

PRIMARY = ['e_dist', 'e_mean']          # 谁有值谁当排序与判档的主指标
EXTRA = ['e_p95', 'over10cm', 'lost_pct', 'wall', 'path', 'sat', 'w_obs', 'v_corner_med']


def load(path):
    groups = {}
    with open(path, newline='') as f:
        for r in csv.DictReader(f, delimiter='\t'):
            cfg = (r.get('config') or '').strip()
            if cfg:
                groups.setdefault(cfg, []).append(r)
    return groups


def nums(rows, key):
    out = []
    for r in rows:
        try:
            v = float(r[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(v):
            out.append(v)
    return out


def stats(rows, key):
    v = nums(rows, key)
    if not v:
        return None
    return {'med': statistics.median(v), 'lo': min(v), 'hi': max(v), 'n': len(v)}


def summarize(groups):
    table = []
    for cfg, rows in groups.items():
        ok = [r for r in rows if (r.get('verdict') or '') == 'PASS']
        d = {'config': cfg, 'n': len(rows), 'n_pass': len(ok)}
        for key in PRIMARY + EXTRA:
            d[key] = stats(ok, key)
        prim = next((k for k in PRIMARY if d.get(k)), 'e_mean')
        d['prim'] = prim
        table.append(d)
    table.sort(key=lambda x: (x[x['prim']] is None,
                              x[x['prim']]['med'] if x[x['prim']] else 9e9))
    return table


def cell(s, spec='%.2f'):
    if not s:
        return '-'
    return spec % s['med']


def spread(s, spec='%.2f ~ %.2f'):
    if not s:
        return '-'
    return spec % (s['lo'], s['hi'])


COLS = [('e_dist', '\\|e\\| 里程加权 (cm)'), ('e_mean', '\\|e\\| 逐帧 (cm)'),
        ('e_p95', 'p95 (cm)'), ('wall', '一圈 (s)'), ('path', '路径 (m)'),
        ('w_obs', 'max\\|ω\\| (rad/s)'), ('v_corner_med', '弯中速度 (m/s)'),
        ('over10cm', '>10cm (%)'), ('lost_pct', '丢线 (%)'), ('sat', '饱和帧')]


def to_markdown(table, tol):
    prim = table[0]['prim'] if table else 'e_mean'
    best = next((t[prim]['med'] for t in table if t[prim]), None)
    head = '| 配置 | 通过 | ' + ' | '.join(c[1] for c in COLS) + ' |'
    sep = '|---|---|' + '---|' * len(COLS)
    lines = [head, sep]
    for d in table:
        cells = []
        for key, _ in COLS:
            s = d.get(key)
            if not s:
                cells.append('-')
            elif key in ('e_dist', 'e_mean', 'wall'):
                cells.append('%s<br>_%s_' % (cell(s, '%.2f' if key != 'wall' else '%.1f'),
                                            spread(s, '%.2f ~ %.2f' if key != 'wall' else '%.1f ~ %.1f')))
            else:
                cells.append(cell(s, '%.0f' if key in ('sat',) else '%.2f'))
        note = ''
        if best is not None and d[prim]:
            delta = d[prim]['med'] - best
            if d[prim]['med'] == best:
                note = ' ★最优'
            elif abs(delta) <= tol:
                note = ' ※与最优同档'
            else:
                note = ' (%+.2f)' % delta
        lines.append('| `%s`%s | %d/%d | %s |' % (
            d['config'], note, d['n_pass'], d['n'], ' | '.join(cells)))
    lines.append('')
    lines.append('主指标 `%s`，每个配置 3 次重复取**中位数**，下面小字是**极差**。'
                 '差值 ≤ %.2f cm 判为同档（同配置重复跑的里程加权极差实测 0.01~0.03 cm，'
                 '逐帧极差 0.23 cm —— 别拿逐帧差 0.1 cm 说事）。'
                 'OFF_TRACK 不进平均，但 `通过` 列会把它暴露出来。'
                 % (prim, tol))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raw')
    ap.add_argument('out')
    ap.add_argument('--tol-cm', type=float, default=0.05)
    a = ap.parse_args()
    groups = load(a.raw)
    if not groups:
        print('EMPTY: no rows in %s' % a.raw)
        return 1
    table = summarize(groups)
    md = to_markdown(table, a.tol_cm)
    # 先打印纯 ASCII 摘要（Windows 控制台不背中文编码的锅），再写盘
    prim = table[0]['prim']
    for d in table:
        s = d[prim]
        print('%-14s pass=%d/%d  %s_med=%-7s spread=%s~%s' % (
            d['config'], d['n_pass'], d['n'], prim,
            '%.3f' % s['med'] if s else 'nan',
            '%.3f' % s['lo'] if s else 'nan', '%.3f' % s['hi'] if s else 'nan'))
    tbl = [l for l in md.split('\n') if l.startswith('|')]
    want = tbl[0].replace('\\|', '').count('|') if tbl else 0
    bad = [i for i, l in enumerate(tbl) if l.replace('\\|', '').count('|') != want]
    print('COLCHK lines=%d cols=%d bad=%s' % (len(tbl), want - 1, bad or 'none'))
    with open(a.out, 'w', encoding='utf-8', newline='') as f:
        f.write(md + '\n')
    print('WROTE %s (%d configs, %d runs)' % (
        a.out, len(table), sum(d['n'] for d in table)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
