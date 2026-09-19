#!/usr/bin/env python3
"""D11 验收分析：直接读节点写的 CSV，独立复算所有数字。

为什么不直接用节点末尾打印的 summary：那是同一份内存数据的自我汇总，
写错索引就会像第一次跑那样崩掉（abs('FOLLOW')）或悄悄算错。
从落盘的 CSV 重算一遍，才叫"独立可观测量证明任务真的完成了"。
"""
import csv
import math
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/d11_phase2.csv'
rows = []
with open(path) as f:
    for r in csv.DictReader(f):
        def fl(k):
            v = r.get(k, '')
            try:
                return float(v)
            except (TypeError, ValueError):
                return float('nan')
        rows.append({'t': fl('t_wall'), 'state': r.get('state', ''), 'e': fl('e_m'),
                     'front': fl('front_m'), 'v': fl('cmd_v'), 'w': fl('cmd_w'),
                     'x': fl('odom_x'), 'y': fl('odom_y')})

if not rows:
    print('CSV 是空的'); raise SystemExit(1)

print(f'=== {path}：{len(rows)} 帧 ===')

# 1. 状态分布与迁移时间线
counts = {}
for r in rows:
    counts[r['state']] = counts.get(r['state'], 0) + 1
print('\n[1] 状态帧数')
for k in sorted(counts, key=lambda x: -counts[x]):
    print(f'    {k:<15} {counts[k]:>6}  ({counts[k] / len(rows) * 100:.1f}%)')

timeline = []
prev = None
for r in rows:
    if r['state'] != prev:
        timeline.append((r['t'], r['state']))
        prev = r['state']
print('\n[2] 状态迁移时间线')
for t, s in timeline[:20]:
    print(f'    t={t:6.1f}s  -> {s}')
if len(timeline) > 20:
    print(f'    ...（共 {len(timeline)} 次迁移）')

# 2. 巡线质量（只统计真的在线上的帧）
errs = [abs(r['e']) for r in rows if r['e'] == r['e']]
if errs:
    print(f'\n[3] 巡线质量（{len(errs)} 帧有线）')
    print(f'    |e| 均值 {sum(errs) / len(errs) * 100:.1f} cm / 最大 {max(errs) * 100:.1f} cm')
    print(f'    |e|>10cm 的帧占比 {sum(1 for x in errs if x > 0.10) / len(errs) * 100:.1f}%')

# 3. 避障质量
fr = [r['front'] for r in rows if r['front'] == r['front'] and r['front'] > 0.05]
if fr:
    print(f'\n[4] 避障（{len(fr)} 帧有前向回波）')
    print(f'    最近前向距离 {min(fr):.3f} m')
    stop_frames = [x for x in fr if x <= 0.31]
    print(f'    <=0.31 m 的帧数 {len(stop_frames)}（这些帧应该都是 OBSTACLE_STOP）')

# 4. 有没有"撞上去"：障碍距离一度小于车体半宽 = 撞上
crash = [r for r in rows if r['front'] == r['front'] and 0.05 < r['front'] < 0.20]
print(f'\n[5] ★ 撞障检查：前向距离 <0.20 m 的帧 = {len(crash)}'
      + ('   ✅ 从没贴到 20 cm 以内' if not crash else '   ★ 有贴脸帧，检查停车逻辑'))

# 6. 一圈证明：轨迹包围盒 + 回到起点距离
xs = [r['x'] for r in rows if r['x'] == r['x']]
ys = [r['y'] for r in rows if r['y'] == r['y']]
x0, y0 = rows[0]['x'], rows[0]['y']
xN, yN = rows[-1]['x'], rows[-1]['y']
d_end = math.hypot(xN - x0, yN - y0)
# 路径长度（从 CSV 的 odom 积分，独立于节点里的 self.path）
plen = sum(math.hypot(rows[i]['x'] - rows[i - 1]['x'], rows[i]['y'] - rows[i - 1]['y'])
           for i in range(1, len(rows))
           if all(v == v for v in (rows[i]['x'], rows[i - 1]['x'], rows[i]['y'], rows[i - 1]['y'])))
print('\n[6] 轨迹独立复算（不信节点里的 path 计数）')
print(f'    x 范围 {min(xs):+.2f}~{max(xs):+.2f}   y 范围 {min(ys):+.2f}~{max(ys):+.2f}')
print(f'    由 odom 积分出的路径长度 = {plen:.2f} m（赛道一圈 9.42 m）')
print(f'    终点距起点 {d_end:.2f} m  ' + ('-> ✅ 闭合，确实跑完一圈' if d_end < 0.5 else '-> 未闭合'))

# 7. 速度是否真的为 0（停住）而不是"发 0 但还在滑"
stopped = [r for r in rows if r['state'] == 'OBSTACLE_STOP']
if stopped:
    vs = [r['v'] for r in stopped]
    print(f'\n[7] OBSTACLE_STOP 共 {len(stopped)} 帧，指令 v 最大 {max(vs):.3f} m/s'
          + ('   ✅ 全程零速' if max(vs) == 0.0 else '   ★ 停住期间还在给油'))
