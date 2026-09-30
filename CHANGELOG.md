# 发布说明

## v0.2 —— 2026-09-30

新增第三个项目 **`arm6_moveit/`**：六轴机械臂 + 二指夹爪的 MoveIt 2 点位闭环。
本版本对 `mybot/` 与 `maze_nav2/` **内容零改动**（唯一的仓库级改动是根 README 从"两个项目"改成"三个项目"）。

提交 `7212cdf` · 105 个文件 · +12 956 / −6 行 · 仓库总大小 4 133 KB

### 这个项目在做什么

一串"张开 → 到位 → 夹住 → 搬走 → 松开 → 归位"的点位，走完整条链路：
手写 URDF/xacro → SRDF 碰撞矩阵（高 trials 采样算到收敛）→ `ros2_control` mock 硬件 + JTC
→ 自研 `waypoint_mover` 直连 `MoveGroup` action → 逐点位 JSONL 运动日志 → 曲线图 + RViz 截图。

交付批次（2026-09-30，ROS 2 Jazzy / MoveIt 2.12.4）实测：

| 指标 | 数值 | 出处 |
|---|---|---|
| 点位规划 / 执行 | **9/9 · 9/9**，`retries=0` | `MOTION-SUMMARY … RESULT PASS` |
| 到位误差 | **1.0e-4 rad**（= 容差的 3%） | 同上，逐点位 `MOTION` 行 |
| 轨迹总长 | 11.4345 rad / 执行 16.13 s | 同上 |
| 截图内容判据 | `mv_norm=23.9% fn_norm=6.6%` → **PASS** | `check_rviz_frames.py` |
| 静态自洽核对 | URDF 限位 / `joint_limits` / SRDF 位形 / 点位表四处一致 → **PASS** | `check_limits.sh` |

### 同时进来的"外观改造"（2026-09-30 当天）

每个 link 的 `<visual>` 从 1 个几何体扩成一份零件清单（**73 个零件 / 6 种材料**）：
关节毂、肩/腕的颊板夹毂、臂体薄腹板 + 两侧盖板、腕部法兰 + 螺栓圈、夹爪指座/指骨/橡胶内垫/指尖。

关键是**没有碰物理**，而且这件事由机器判据守住而不是"我小心了"：

| 新锚点 | 谁印的 | 判什么 | 本版本实测 |
|---|---|---|---|
| `BASELINE-RESULT` | `check_visual_envelope.py` | collision 与改造前基线逐项一致 | 12 link / 10 几何体，**0 差异** |
| `ENVELOPE-RESULT` | 同上 | visual 相对 collision 的外扩（侧向 ≤ 6 mm、端面 link ±z ≤ 6 mm） | 侧向最大 **5.00 mm** |
| `SELFTEST-RESULT` | 同上 `--selftest` | 判据自身：3 个该红的红 + 1 个该绿的绿 + 2 个基线样 | **7/7** |
| `VIEWER-RESULT` | `gen_viewer_parts.py` | 浏览器预览页的零件清单与 URDF 一致 | **73 零件 / 6 材料** |

所以既有全部规划与计时数字**继续有效**，无需重跑回归。

### 一处判据修正（值得单独讲）

RViz 取景从 `Distance 3.2` 拉近到 `1.62` 后，原来的截图内容判据（变化像素占**视口**的比例）
判成 `idle↔final 0.69% > 0.35%` FAIL。先排除了两个假因（鬼影机器人、手臂没归位），
量出来真因是**分母**：手臂投影面积从 20 016 px 涨到 78 364 px（3.9 倍），分子当然跟着涨。

于是把分母换成"同一张图里手臂自身的投影面积"，两条判据变成无量纲比值，
并抽成可离线重跑的 `scripts/check_rviz_frames.py`（"只换尺子不重截"）。
新阈值拿**一次真实缺陷的归档图**当红样验过：

| 批次 | `mv_norm` | `fn_norm` | 判定 |
|---|---|---|---|
| 09-16 `Loop Animation` 鬼影（该红） | 33.5% | 26.8% | **FAIL** ✓ |
| 09-17 修好后 | 24.9% | 5.6% | PASS |
| 09-30 新取景 | 21.8% | 7.5% | PASS |

### 预览页

`arm6-viewer.html` 是零依赖的浏览器 3D 预览（Canvas 手写投影，双击即开，不需要 ROS）。
本版本把它从"手抄几何体"改成**由 URDF 生成零件清单**，并修掉三处导致它此前**从未真正渲染过**的缺陷
（重复声明的语法错、未定义的 `REACH`、`toCam` 与投影/剔除两套不一致的坐标约定）。详见 `arm6_moveit/docs/06` §11。

### 怎么自己复核这些数字

```bash
colcon build --base-paths src && source install/setup.bash
bash scripts/check_limits.sh                       # CHECK-RESULT
python3 scripts/check_visual_envelope.py artifacts/arm6.urdf \
        --collision-baseline artifacts/arm6.urdf.pre-visual   # ENVELOPE / BASELINE
python3 scripts/gen_viewer_parts.py --check        # VIEWER-RESULT
ros2 launch arm_moveit_config moveit.launch.py     # 另开终端
ros2 launch arm_demo waypoint_mover.launch.py      # MOTION-SUMMARY
```

### 已知限制

- 硬件接口是 `mock_components/GenericSystem`（命令位置直接回写成状态位置）——
  证明的是**规划链路正确且可回归**，不证明动力学保真；接真机只需换 `<plugin>`，上层不动。
- 运动学求解用 KDL；未接笛卡尔路径约束、感知与场景障碍物。
- 未接 Gazebo，没有重力/力矩层面的验证。
- 零位约定是"全 0 = 直臂朝天"（唯一保证各 link 互不相碰的姿态），与真机常见的"手臂水平前伸"零位不同。

### 脱敏与门禁

6 处命中 / 3 个文件 / 4 行，全部就地中性化（**只改仓库副本，开发源不动**）；
`scan` 与 `scan --history` 均 **0 命中**，`preflight OK (24 checklist items checked)`。
排除 `logs/`（168 个运行期日志）——每条结论仍可由 `*-RESULT` 判据行或 `artifacts/` 图证独立复核。
明细见 `工具与清单/脱敏清单.md` §五。

---

## v0.1 —— 2026-09-20

首次发布，一仓双项目：

- **`mybot/`** —— 视觉巡线 + 避障小车。一圈 30.8 s、里程加权 |e| 1.07 cm、舵量饱和 0 帧；
  避障在障碍前 29.6 cm 停死、删障后自行恢复跑完；ESP32 差速混控层 33 条断言。
  保留 D1→D13 逐日提交历史。
- **`maze_nav2/`** —— 程序生成迷宫里的 Cartographer 建图 + Nav2 自主导航。
  导航 SUCCEEDED、0 次恢复行为、末距 0.246 m（容差 0.25 m）。

环境：WSL2 · Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic。演示 GIF 全部来自真实运行录制。
