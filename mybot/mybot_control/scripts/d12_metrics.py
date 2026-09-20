#!/usr/bin/env python3
"""D12 调参指标：从巡线 CSV 算出可比较的几个数。

★ 口径（D11 学到的教训，别再犯）：
  |e| 只在**真的看到线**的帧上统计。丢线帧的 e 是 nan，而"停着不动"的帧 e≈0，
  两种都会把均值拉到好看得假。所以：
    - 分母 = 有效帧（e 有限）
    - 丢线帧单独计数，不进 |e|
  另外单圈耗时只在**真的跑完一圈**时才报，否则是"跑了多久被迫停"，不可比。
"""
import csv
import math
import sys

TRACK_PERIMETER = 9.42


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            def g(k):
                try:
                    return float(r[k])
                except (TypeError, ValueError, KeyError):
                    return float('nan')
            rows.append({'t': g('t_wall'), 'e': g('e_m'), 'w': g('cmd_w'), 'v': g('cmd_v'),
                         'x': g('odom_x'), 'y': g('odom_y'),
                         'reason': (r.get('reason') or '').strip(),
                         'state': (r.get('state') or '').strip()})
    return rows


def metrics(rows):
    # 有效帧：e 是有限数（丢线时节点写的是 nan）
    good = [r for r in rows if math.isfinite(r['e'])]
    lost = len(rows) - len(good)
    m = {'frames': len(rows), 'good': len(good), 'lost': lost,
         'lost_pct': (lost / len(rows) * 100) if rows else 0.0}

    if good:
        ae = [abs(r['e']) for r in good]
        m['e_mean_cm'] = sum(ae) / len(ae) * 100
        m['e_max_cm'] = max(ae) * 100
        m['e_p95_cm'] = sorted(ae)[int(len(ae) * 0.95)] * 100
        m['over10cm_pct'] = sum(1 for x in ae if x > 0.10) / len(ae) * 100
    else:
        m.update(e_mean_cm=float('nan'), e_max_cm=float('nan'),
                 e_p95_cm=float('nan'), over10cm_pct=float('nan'))

    m['sat'] = sum(1 for r in rows if 'saturated' in r['reason'])

    # 路径长度与闭合：从 odom 积分
    plen, prev = 0.0, None
    for r in rows:
        if not (math.isfinite(r['x']) and math.isfinite(r['y'])):
            continue
        if prev:
            plen += math.hypot(r['x'] - prev[0], r['y'] - prev[1])
        prev = (r['x'], r['y'])
    m['path_m'] = plen
    if rows:
        x0, y0 = rows[0]['x'], rows[0]['y']
        xN, yN = rows[-1]['x'], rows[-1]['y']
        if all(math.isfinite(v) for v in (x0, y0, xN, yN)):
            m['end_gap_m'] = math.hypot(xN - x0, yN - y0)
        m['wall_s'] = rows[-1]['t'] - rows[0]['t']
    # 跑完一圈 = 路径接近一圈且回到起点附近
    m['lap_done'] = (m.get('path_m', 0) > TRACK_PERIMETER * 0.85
                     and m.get('end_gap_m', 9) < 0.45)
    # 覆盖率：跑完一圈时，多少比例的有效帧是"贴线"的
    return m


def main():
    path = sys.argv[1]
    tag = sys.argv[2] if len(sys.argv) > 2 else path
    m = metrics(load(path))
    lap = '一圈完成' if m['lap_done'] else '未跑完一圈'
    print(f"{tag}\t{m['frames']}\t{m['good']}\t{m['lost']}\t{m['lost_pct']:.2f}\t"
          f"{m['e_mean_cm']:.2f}\t{m['e_max_cm']:.1f}\t{m['e_p95_cm']:.1f}\t"
          f"{m['over10cm_pct']:.2f}\t{m['sat']}\t{m['path_m']:.2f}\t"
          f"{m.get('wall_s', 0):.1f}\t{m.get('end_gap_m', -1):.2f}\t{lap}")


if __name__ == '__main__':
    main()
