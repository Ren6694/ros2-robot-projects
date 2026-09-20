#!/usr/bin/env python3
"""D12 复算：把 /tmp/d12r/*.csv 重新算成一张扩展表，**不采信扫描时写下的数字**。

为什么要有这个脚本（两个独立的理由，第二个才是主要的）：

1. 台架在跑的时候不能改 `d12_metrics.py` 的输出格式（列一动，raw.tsv 的表头就对不上），
   所以新指标只能由这个旁路脚本补算，跑完再合表。

2. ★ 复算时发现 `w_max` 这一轮扫的根本不是"执行器限幅"：
   所有跑次的 max|cmd_w| 都只有 0.57~0.66 rad/s，从来没接近过 1.5，更别提 2.2，
   所以 `reason='saturated'` 全程 0 帧 —— 那个 0 是**真的**，不是标记没落盘。
   而 `pd_line.compute()` 里 w_max 还出现在第二个地方：
       v = v_max * (1 - kv_speed * min(1, |w| / w_max))
   也就是说这套代码里"调 w_max"实际调的是**弯道减速的刻度**：w_max 越大，
   同样的 |w| 占比越小、车在弯里掉速越少。这解释了为什么 2.2 比 1.5 快 3.4 秒、
   而两个配置的限幅帧都是 0。
   顺带一个副作用：|e| 是**逐帧**平均，车走得越慢、弯道里 spent 的帧越多，
   均值被弯道的次数加权得越狠 —— 所以 r1_w_max0.6 的 5.44 cm 里有一部分是
   "它在弯里爬了很久"而不是"它偏得多"。为此这里多算一个**按里程加权**的 |e|，
   两个一起报，谁被采样方式骗了一眼就看出来。

用法：python3 d12_recompute.py <csv_dir> <out_tsv>
输出列 = 扫描 raw.tsv 的前 16 列（口径完全一致）+ e_dist / w_obs / kv_eff 三列。
"""
import csv
import glob
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d12_metrics import TRACK_PERIMETER, load, metrics   # noqa: E402  同一套口径，不要另起炉灶

HEADER = ['config', 'rep', 'frames', 'good', 'lost', 'lost_pct', 'e_mean', 'e_max',
          'e_p95', 'over10cm', 'sat', 'path', 'wall', 'end_gap', 'lap', 'verdict',
          'e_dist', 'w_obs', 'v_corner_med']


def dist_weighted(rows):
    """按里程加权的 |e|：每帧的误差乘以这一帧走到那一帧之间跑了多少米。

    逐帧平均会被"慢"污染：20 Hz 采样下，车在弯里爬 3 秒就是 60 帧，
    在直道上 3 秒只有同样的 60 帧但路程长得多 —— 弯道被隐形加权。
    """
    num = den = 0.0
    prev = None
    for r in rows:
        x, y, e = r['x'], r['y'], r['e']
        if prev is not None and math.isfinite(x) and math.isfinite(y):
            ds = math.hypot(x - prev[0], y - prev[1])
        else:
            ds = 0.0
        prev = (x, y) if math.isfinite(x) and math.isfinite(y) else prev
        if math.isfinite(e):
            num += abs(e) * ds
            den += ds
    return (num / den * 100) if den > 0.5 else float('nan')


def obs(rows, key='w'):
    v = [abs(r[key]) for r in rows if math.isfinite(r[key])]
    return max(v) if v else float('nan')


def corner_v(rows):
    """弯里（|w| 最大的那 10% 帧）的中位前进速度 —— 直接看减速刻度被调成了什么。"""
    ws = sorted((abs(r['w']) for r in rows if math.isfinite(r['w'])))
    if len(ws) < 20:
        return float('nan')
    cut = ws[int(len(ws) * 0.90)]
    vs = [r['v'] for r in rows if math.isfinite(r['v']) and abs(r['w']) >= cut]
    return statistics.median(vs) if vs else float('nan')


def one(path, default_rep='1'):
    name = os.path.basename(path)[:-4]
    if '_r' in name:
        cfg, rep = name.rsplit('_r', 1)
    else:
        # 轮 1 / 轮 2 的老 CSV 没有 _rN 后缀（那时还没有重复跑），
        # 一律当 n=1 处理，表里要标清楚"单跑"。
        cfg, rep = name, default_rep
    rows = load(path)
    m = metrics(rows)
    lap_done = m['lap_done']
    ed = dist_weighted(rows)
    vc = corner_v(rows)
    return [cfg, rep, m['frames'], m['good'], m['lost'],
            '%.2f' % m['lost_pct'], '%.2f' % m['e_mean_cm'], '%.1f' % m['e_max_cm'],
            '%.1f' % m['e_p95_cm'], '%.2f' % m['over10cm_pct'], m['sat'],
            '%.2f' % m['path_m'], '%.1f' % m.get('wall_s', 0),
            '%.2f' % m.get('end_gap_m', -1),
            '一圈完成' if lap_done else '未跑完一圈',
            'PASS' if lap_done else 'OFF_TRACK',
            '%.2f' % ed if math.isfinite(ed) else 'nan',
            '%.3f' % obs(rows, 'w'),
            '%.3f' % vc if math.isfinite(vc) else 'nan']


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else '/tmp/d12r'
    out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/d12r/recomputed.tsv'
    files = sorted(glob.glob(os.path.join(d, '*.csv')))
    if not files:
        print('NO_CSV in %s' % d)
        return 1
    rows = [r for r in (one(p) for p in files) if r]
    with open(out, 'w', newline='') as f:
        w = csv.writer(f, delimiter='\t', lineterminator='\n')
        w.writerow(HEADER)
        w.writerows(rows)
    # 纯 ASCII 摘要，Windows 控制台不背这个锅
    for r in sorted(rows, key=lambda x: (x[0], x[1])):
        print('%-14s r%s  e=%6s  e_dist=%6s  w_obs=%6s  v_corner=%6s  lap=%5s  %s' % (
            r[0], r[1], r[6], r[16], r[17], r[18], r[12], r[15]))
    print('WROTE %s  (%d runs, track perimeter %.2f m)' % (out, len(rows), TRACK_PERIMETER))
    return 0


if __name__ == '__main__':
    sys.exit(main())
