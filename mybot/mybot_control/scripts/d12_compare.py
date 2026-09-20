#!/usr/bin/env python3
"""D12 复跑对照：把 09-19 那批与新一批的**中位数 + 极差**并排，量化"跨会话漂移"。

为什么基线是一个手抄的 TSV 而不是原始文件：09-19 那 18 跑的 CSV 只写在 /tmp，
一次 WSL 虚拟机重启就全没了（见《D12-参数表》第三节 #6）。
`d12_baseline_0919.tsv` 里的每个数当时都由 `d12_recompute.py` 打印过、并抄进笔记，
这里只是把那份打印结果结构化，好让"两批差多少"是算出来的而不是我眼估的。
**它是抄录件，不是原始件** —— 所以本脚本只用来做漂移体检，不用来复核单个跑次。

用法：python3 d12_compare.py <baseline.tsv> <new_recomputed.tsv> <out.md>
判读：主指标 e_dist 的中位数差 > 0.10 cm 或圈时差 > 1.5 s 的配置，标 "漂"。
"""
import csv
import statistics
import sys

KEY = 'e_dist'
LAP = 'wall'
DRIFT_E = 0.10      # cm：跨会话判定阈值（会话内极差实测 0.01~0.03 cm，见笔记 2b）
DRIFT_T = 1.5       # s


def load(path):
    g = {}
    with open(path, newline='') as f:
        for r in csv.DictReader(f, delimiter='\t'):
            cfg = (r.get('config') or '').strip()
            if cfg:
                g.setdefault(cfg, []).append(r)
    return g


def med(g, key, only_pass=True):
    v = []
    for r in g:
        if only_pass and (r.get('verdict') or '') != 'PASS':
            continue
        try:
            x = float(r[key])
        except (KeyError, TypeError, ValueError):
            continue
        v.append(x)
    if not v:
        return None
    return {'med': statistics.median(v), 'lo': min(v), 'hi': max(v), 'n': len(v)}


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    old, new, out = load(sys.argv[1]), load(sys.argv[2]), sys.argv[3]
    lines = ['| 配置 | 旧 n | 旧 e_dist 中位（极差） | 新 n | 新 e_dist 中位（极差） | Δ | '
             '旧圈时 | 新圈时 | 判定 |',
             '|---|---|---|---|---|---|---|---|---|']
    drift_n = 0
    for cfg in sorted(old):
        a, b = old.get(cfg, []), new.get(cfg, [])
        ae, be = med(a, KEY), med(b, KEY)
        at, bt = med(a, LAP), med(b, LAP)
        if not ae or not be:
            lines.append('| `%s` | %d | %s | %d | %s | - | - | - | 缺样本 |' % (
                cfg, len(a), '%.2f' % ae['med'] if ae else '-',
                len(b), '%.2f' % be['med'] if be else '-'))
            continue
        d = be['med'] - ae['med']
        dt = (bt['med'] - at['med']) if (at and bt) else float('nan')
        drift = abs(d) > DRIFT_E or (dt == dt and abs(dt) > DRIFT_T)
        if drift:
            drift_n += 1
        lines.append('| `%s` | %d | %.2f _(%.2f~%.2f)_ | %d | %.2f _(%.2f~%.2f)_ | %+.2f | %.1f | %.1f | %s |' % (
            cfg, ae['n'], ae['med'], ae['lo'], ae['hi'],
            be['n'], be['med'], be['lo'], be['hi'], d,
            at['med'] if at else float('nan'), bt['med'] if bt else float('nan'),
            '**漂**' if drift else '同档'))
    lines.append('')
    lines.append('判定阈值：主指标 \\|e\\| 中位差 > %.2f cm 或圈时差 > %.1f s 记为"漂"。'
                 '会话内重复极差只有 0.01~0.03 cm，所以这两个阈值已经比"能分辨"宽得多。'
                 % (DRIFT_E, DRIFT_T))
    with open(out, 'w', encoding='utf-8', newline='') as f:
        f.write('\n'.join(lines) + '\n')
    tbl = [l for l in lines if l.startswith('|')]
    want = tbl[0].replace('\\|', '').count('|')
    bad = [i for i, l in enumerate(tbl) if l.replace('\\|', '').count('|') != want]
    print('CONFIGS=%d DRIFT=%d COLCHK=%s' % (len(old), drift_n, 'ok' if not bad else bad))
    # ★ 控制台只打 ASCII：Windows 侧 GBK 代码页会把中文打成乱码（真内容在 out.md 里）
    for cfg in sorted(old):
        a, b = old.get(cfg, []), new.get(cfg, [])
        ae, be = med(a, KEY), med(b, KEY)
        at, bt = med(a, LAP), med(b, LAP)
        if not ae or not be:
            print('%-14s old=%s new=%s  NO_SAMPLE' % (
                cfg, '%.2f' % ae['med'] if ae else '-', '%.2f' % be['med'] if be else '-'))
            continue
        d = be['med'] - ae['med']
        dt = (bt['med'] - at['med']) if (at and bt) else float('nan')
        print('%-14s old=%.2f(n%d) new=%.2f(n%d) d=%+.2f dt=%+.1f %s' % (
            cfg, ae['med'], ae['n'], be['med'], be['n'], d, dt,
            'DRIFT' if (abs(d) > DRIFT_E or (dt == dt and abs(dt) > DRIFT_T)) else 'same'))
    print('WROTE %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
