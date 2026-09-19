#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D11：避障 + 丢线恢复的状态机（纯逻辑，不 import rclpy，所以能脱离 ROS 单测）。

放在 ROS 节点之下是有意的：状态机是全项目最容易"看起来对、实际会撞"的一层，
它必须能在 1 毫秒内被喂进任意合成的传感器序列（包括病态序列）并断言输出，
而不是只能靠在 Gazebo 里摆障碍碰运气。

═══════════════════════════════════════════════════════════════════════════
 三条从实测里拿来的硬约束（不是拍脑袋定的数）
═══════════════════════════════════════════════════════════════════════════
1) **前向扇区取 ±25°，不是全向。**
   实测（0.12 m 方块放在 0.64 m 处、车静止）：一帧 360 线里有 32 条回波落在
   0.123~0.307 m，角度全部在 |θ| ≥ 98° —— 那是激光打到**自己的车壳和轮子**。
   这些回波距离恒定、角度固定，如果按"全向最近值"判障碍，车会永远认为紧贴着
   一个 0.12 m 的障碍，一步都不肯走。±25° 既盖住了真实的前方接近（0.12 m 宽
   的障碍在 0.64 m 处只占 ±5.4°），又远离自我回波区。

2) **min_usable = 0.15 m 的下限。**
   激光 range_min=0.12（URDF 里写的），比它更近的东西本来就测不准；而实测自我回波
   最低到 0.123。所以前向扇区里 <0.15 m 的读数一律当噪声丢掉，**不能**当"障碍已贴脸"
   —— 否则一个假回波会让车永久停死。

3) **避障的分辨率上限是 /scan 的 5 Hz，不是控制环的 20 Hz。**
   0.30 m/s 下两帧激光之间车前进 0.30/5 = 0.06 m。所以 warn_dist 与 stop_dist 之间
   必须留出 > 0.06 m 的减速带，否则"发现"和"撞上"发生在同一帧里。
   默认 warn 0.55 / stop 0.30 → 0.25 m 减速带 ≈ 4 帧激光，够。

═══════════════════════════════════════════════════════════════════════════
 状态优先级（这条顺序就是安全性来源）
═══════════════════════════════════════════════════════════════════════════
   障碍距离说了算，**先判障碍再判线**：

   front <= stop_dist          -> OBSTACLE_STOP  (v=0, w=0；即使线还看得见也停)
   front <= warn_dist          -> DECEL          (照常转向，速度按比例压下去)
   线丢了 且 前方是干净的       -> SEARCH         (原地转着找线)
   线丢了 且 前方不干净          -> OBSTACLE_STOP  (★ 不许在障碍跟前转身找线)
   搜索超时                    -> SAFE_STOP      (终态，等人来接管)
   其余                        -> FOLLOW         (交给 D10 的 PD)

   为什么"障碍优先"必须排在最前：D11 的真实场景就是**方块挡住黑线** ——
   相机看不见线、激光看见方块。如果先判丢线，车会进入 SEARCH 原地旋转，
   把一个正对着它的障碍物当成"线丢了"来处理，转着转着就撞上去。
   这一条是 D10 遗留的"丢线只会停不会找"接进避障时最容易写反的地方。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

from .pd_line import Cmd, PDGains, compute

# ────────────────────────────── 状态名 ──────────────────────────────
FOLLOW = 'FOLLOW'
DECEL = 'DECEL'
OBSTACLE_STOP = 'OBSTACLE_STOP'
SEARCH = 'SEARCH'
SAFE_STOP = 'SAFE_STOP'

STATES = (FOLLOW, DECEL, OBSTACLE_STOP, SEARCH, SAFE_STOP)


@dataclass
class AvoidParams:
    """所有阈值。改这里而不是改节点，是为了让单测能覆盖到同一套数。"""
    # 激光扇区
    front_half_angle_deg: float = 25.0   # 见模块注释 1)
    min_usable_m: float = 0.15           # 见模块注释 2)
    max_consider_m: float = 2.0          # 再远的回波与避障无关，省得被 8 m 外的墙干扰

    # 距离门限（都是"激光到障碍近面"的距离，不是到车头的）
    stop_dist_m: float = 0.30
    warn_dist_m: float = 0.55
    # ★ 迟滞：从 OBSTACLE_STOP 脱出要求前向距离回到 stop + release_margin 之外。
    #   实测必要性：车停在障碍前时，激光噪声让前向距离在 0.278~0.300 m 之间摆（±23 mm），
    #   正好压在 0.300 这条线上 -> 单阈值会不停在 OBSTACLE_STOP/DECEL 之间来回跳
    #   （D11 首轮 200 秒里出现 3 次）。车停着的时候只是难看，
    #   但一旦是"贴着障碍缓慢爬行"的场景，这个抖动会变成一步步往前蹭 —— 所以现在就修。
    release_margin_m: float = 0.10

    # 丢线 / 搜索
    lost_timeout_s: float = 1.0          # 与 D10 一致：超过这么久没线才算"丢"
    search_w: float = 0.35               # 原地找线的角速度(rad/s)，故意比巡线慢
    search_flip_s: float = 1.5           # 这么久没找到就反向扫（单向转可能正好背离线）
    search_max_s: float = 6.0            # 总搜索时长上限，超了就 SAFE_STOP

    def __post_init__(self):
        # stop 必须真小于 warn，否则减速带宽为 0，"发现即撞上"
        if self.stop_dist_m >= self.warn_dist_m:
            raise ValueError(f"stop_dist({self.stop_dist_m}) 必须小于 warn_dist({self.warn_dist_m})")
        # 前向扇区不能宽到把自我回波（实测 |θ|>=98°）卷进来
        if self.front_half_angle_deg >= 90.0:
            raise ValueError("front_half_angle_deg >= 90 会把车体自我回波当成障碍")


@dataclass
class Decision:
    cmd: Cmd
    state: str
    front: Optional[float]      # 本帧前向最近障碍距离（None = 扇区里没有有效回波）
    line_ok: bool


def front_obstacle_distance(ranges: Sequence[float], angle_min: float, angle_incr: float,
                            p: AvoidParams) -> Optional[float]:
    """从一维 ranges 数组里取前向扇区最近的有效回波。

    参数刻意用 ranges/angle_min/angle_incr 而不是 LaserScan 对象：这样纯函数可以
    直接喂 list 做单测，不需要 ROS 运行时。节点那边传 scan.ranges 即可。
    返回 None 表示扇区内没有任何有效回波 = 前方干净。
    """
    half = math.radians(p.front_half_angle_deg)
    best: Optional[float] = None
    for i, r in enumerate(ranges):
        ang = angle_min + i * angle_incr
        if abs(ang) > half:
            continue
        # inf / nan / 负值 / 盲区内的自我回波，全部丢弃（理由见模块注释 2)）
        if r is None or not math.isfinite(r) or r <= p.min_usable_m or r > p.max_consider_m:
            continue
        if best is None or r < best:
            best = r
    return best


@dataclass
class AvoidFSM:
    """带记忆的状态机。时间一律由调用方传进来（now），这样单测可以精确控制步长。

    注意 SAFE_STOP 是**吸收态**：一旦进去就不再自己出来。理由是走到 SAFE_STOP
    意味着"线丢了、转着找了 6 秒还没找到"，这时候继续乱动的风险大于收益，
    正确处置是人来接管或重启，不是让车自己再试一轮。
    """
    p: AvoidParams = field(default_factory=AvoidParams)
    state: str = FOLLOW
    # 用 None 而不是 0.0 当"还没收到过线"的哨兵：
    # 单测里第一帧就常常是 now=0.0，若用 0.0 做哨兵，`if self.last_line_time` 判假，
    # lost_for 会永远算成 0，SEARCH 分支直接变死代码（这个 bug 就是被单测抓出来的）。
    last_line_time: Optional[float] = field(default=None, init=False)
    search_start: float = field(default=0.0, init=False)
    search_dir: float = field(default=1.0, init=False)
    n_transitions: int = field(default=0, init=False)
    prev_state: str = field(default=FOLLOW, init=False)

    def step(self, now: float, line_ok: bool, e: float, theta: float,
             front: Optional[float], gains: PDGains) -> Decision:
        prev = self.state
        reason = ''

        if line_ok:
            self.last_line_time = now
            self.search_start = 0.0          # 线回来了，搜索计时清零，下次丢线重新计满额
        # 还没收到过线时给宽限期（lost_for=0）：开机第一帧不该立刻判"丢线"
        lost_for = 0.0 if self.last_line_time is None else (now - self.last_line_time)
        # 迟滞判障：已经停在 OBSTACLE_STOP 时，用更宽的 release 门限，
        # 避免激光噪声在 stop 线上来回推（见 AvoidParams.release_margin_m 注释）
        thresh = self.p.stop_dist_m
        if self.state == OBSTACLE_STOP:
            thresh = self.p.stop_dist_m + self.p.release_margin_m
        blocked = front is not None and front <= thresh

        if self.state == SAFE_STOP:
            # 吸收态：不再评估任何输入，等人接管
            reason = 'terminal'
            cmd = Cmd(0.0, 0.0, 'safe_stop')
            self.state = SAFE_STOP
        elif blocked:
            # ★ 障碍优先：哪怕线还看得见、哪怕正在搜索，只要前方贴脸就立刻停死
            self.state = OBSTACLE_STOP
            cmd = Cmd(0.0, 0.0, 'obstacle')
            reason = f'front={front:.3f}'
        elif self.state == OBSTACLE_STOP:
            # 刚从 OBSTACLE_STOP 出来：障碍移开了。线可能还在（继续 FOLLOW），
            # 也可能被障碍遮挡期间丢了（进 SEARCH）—— 两种都由下面的分支处理。
            self.state = FOLLOW
            reason = 'cleared'
            # 不 return，让它继续往下走一遍 FOLLOW/DECEL/SEARCH 判定
            # 这样"障碍移开"到"恢复动作"之间不需要再等一帧
            if not line_ok and lost_for >= self.p.lost_timeout_s:
                self.state = SEARCH
                self.search_start = now
                self.search_dir = 1.0 if e >= 0 else -1.0
                cmd = Cmd(0.0, self.p.search_w * self.search_dir, 'search')
                reason = 'cleared->search'
            else:
                cmd = compute(e, theta, gains)
                if front is not None and front <= self.p.warn_dist_m:
                    self.state = DECEL
                    cmd = self._decelerate(cmd, front)
                    reason = f'warn front={front:.3f}'
        elif line_ok:
            cmd = compute(e, theta, gains)
            if front is not None and front <= self.p.warn_dist_m:
                self.state = DECEL
                cmd = self._decelerate(cmd, front)
                reason = f'decel front={front:.3f}'
            else:
                self.state = FOLLOW
                reason = 'follow'
        elif lost_for < self.p.lost_timeout_s:
            # 丢线还没到超时：沿用 D10 的行为，先停车等一等（瞬时丢检不该触发搜索）
            self.state = FOLLOW
            cmd = Cmd(0.0, 0.0, 'lost_brief')
            reason = 'lost_brief'
        else:
            # 线真丢了，且前方干净 -> 原地找线
            if self.state != SEARCH:
                self.state = SEARCH
                self.search_start = now
                # 朝上一次线所在的那一侧先找：e>0 线在左 -> 左转
                self.search_dir = 1.0 if e >= 0 else -1.0
            elapsed = now - self.search_start
            if elapsed >= self.p.search_max_s:
                self.state = SAFE_STOP
                cmd = Cmd(0.0, 0.0, 'search_timeout')
                reason = f'search_timeout {elapsed:.1f}s'
            else:
                # 每 search_flip_s 反向一次：单向转有可能正好背着线走
                if int(elapsed // self.p.search_flip_s) % 2 == 1:
                    wdir = -self.search_dir
                else:
                    wdir = self.search_dir
                cmd = Cmd(0.0, self.p.search_w * wdir, 'search')
                reason = f'search {elapsed:.1f}s'

        if self.state != prev:
            self.n_transitions += 1
            self.prev_state = prev
        return Decision(cmd=cmd, state=self.state, front=front, line_ok=line_ok)

    def _decelerate(self, cmd: Cmd, front: float) -> Cmd:
        """减速带：把速度按 (front-stop)/(warn-stop) 线性压下去，**转向不动**。

        转向必须原样保留 —— 减速期间车还在沿线上，砍掉转向等于把车甩出线外，
        那会让"避障"变成"制造丢线"。
        """
        span = self.p.warn_dist_m - self.p.stop_dist_m
        k = max(0.0, min(1.0, (front - self.p.stop_dist_m) / span)) if span > 0 else 0.0
        return Cmd(cmd.v * k, cmd.w, cmd.reason + '+decel')
