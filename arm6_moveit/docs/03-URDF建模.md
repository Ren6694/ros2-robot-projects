# 03 URDF 建模

> 包：`src/arm_description/`
> 文件：`urdf/arm6.urdf.xacro`（主）、`urdf/inertial_macros.xacro`（惯量宏）、
> `urdf/arm6.ros2_control.xacro`（硬件接口契约）、`launch/display.launch.py`（单模型显示）
> 展开后的完整 URDF 存档：`artifacts/arm6.urdf`

## 1. 拓扑与自由度选择

```
world ─(fixed)→ base_link ─joint1(z)→ link1 ─joint2(y)→ link2 ─joint3(y)→ link3
              ─joint4(z)→ link4 ─joint5(y)→ link5 ─joint6(z)→ link6
              ─(fixed)→ gripper_base ─┬─finger1_joint(+y)→ finger1_link
                                      ├─finger2_joint(-y)→ finger2_link
                                      └─(fixed)→ tool0
```

12 个 link（含 `world` 与 `tool0`）、11 个 joint，其中**可控关节 8 个**
（`joint1..6` revolute + `finger1/2_joint` prismatic）。

| 设计决定 | 理由 |
|---|---|
| 6 自由度而不是 7 或 4 | 6 是"末端任意位姿无约束"的最小自由度（3 位置 + 3 姿态），也是工业六轴关节机器人的通用构型，能覆盖 MoveIt2 的 IK/碰撞/规划全部难点。4 自由度会退化出大量奇异；7 自由度引入冗余解歧义，反而看不清规划器本身的行为 |
| 夹爪用 2 个独立 prismatic，而不是 1 个 + `<mimic>` | `mimic` 在 ros2_control 的 mock 硬件 + JTC 路径下支持不完整（要额外插件）。多花 2 维状态，换"整条链路零特例" |
| 规划末端用 `tool0` 而不是 `link6` | 抓取规划关心"夹取点"。`tool0` 定在两指指尖平面（`xyz = 0 0 gb_h+f_h`），`set_pose_target` 给的目标就是物体要到的位置，不需要业务代码再补 75 mm 偏置 —— 这类偏置是现场 bug 高发区 |
| `tool0` 只给 visual 不给 collision | 它是纯坐标系。给了碰撞体会在 SRDF 里多出一堆"小球与指片相碰"的假阳性，还得逐条 disable |

## 2. 坐标系约定（改模型前必读）

- 根 link `base_link` 原点在地面（z=0），底座几何向 +z 伸出。
- 每个运动 link 的**原点与该关节的轴重合**，几何体向自身 +z 伸出。
  因此惯量必须用 `com_z = L/2` + 平行轴定理平移到 link 原点（见 §4）。
- 关节轴向：1/4/6 绕 z（腰、腕滚、腕摆），2/3/5 绕 y（肩、肘、腕俯仰）。
- **零位 = 全 0 = 手臂竖直向上伸直**。这是 SRDF 的 `up` 位形，也是唯一保证各
  link 互不相碰的姿态，所以所有 URDF `<limit>` 的中点都取 0（对称限位）。
  这条约定直接决定了 `waypoints.yaml` 里第一个点位必须是全 0。

## 3. 几何与限位（实测值，`bash scripts/check_limits.sh` 会打印同一张表）

| 关节 | 类型 | 限位 (rad) | 限位 (°) | v_max (rad/s) | effort (N·m) | 质量 (kg) |
|---|---|---|---|---|---|---|
| joint1 | revolute z | ±2.7925 | ±160 | 3.14 | 60 | 0.95 (link1) |
| joint2 | revolute y | ±1.7453 | ±100 | 3.14 | 60 | 0.70 (link2) |
| joint3 | revolute y | ±2.0944 | ±120 | 3.14 | 30 | 0.45 (link3) |
| joint4 | revolute z | ±2.9671 | ±170 | 4.19 | 15 | 0.25 (link4) |
| joint5 | revolute y | ±2.0944 | ±120 | 4.19 | 15 | 0.18 (link5) |
| joint6 | revolute z | ±2.9671 | ±170 | 4.19 | 8 | 0.12 (link6) |
| finger1_joint | prismatic +y | [0, 0.0400] m | — | 0.10 m/s | 20 N | 0.045 |
| finger2_joint | prismatic −y | [0, 0.0400] m | — | 0.10 m/s | 20 N | 0.045 |

长度链：`base_h 0.035 → l1 0.200 → l2 0.240 → l3 0.200 → l4 0.080 → l5 0.060 →
l6 0.030 → 夹爪座 gb_h 0.020 → 指片 f_h 0.055`，零位时 `tool0` 高度 ≈ 0.92 m。
全臂 11 个 link 合计 **4.646 kg**（底座 1.80 kg 占 39 %，前四段占 81 %）——
越靠末端越轻是刻意的：腕部电机小、惯量小，才敢给 joint4..6 更高的角速度。

单指行程 40 mm ⇒ 开口增量 40 mm（两指反向等速，靠 `waypoints.yaml` 里给相同值实现，
不是靠 mimic）。

## 4. 惯量：为什么必须有、为什么必须是宏

URDF 里每个 link 都必须带 `<inertial>`，否则：

- MoveIt 的碰撞检测/质心计算报 `link has no inertial`；
- `robot_state_publisher` 干脆不发该 link 的 TF。

8 个运动 link 手抄 3×3 惯量矩阵，改一次尺寸要改 8 处，所以用宏。
`inertial_macros.xacro` 提供 `box_inertial` / `cylinder_inertial` / `sphere_inertial`，
公式是均匀密度刚体绕质心、主轴对齐自身坐标系：

```
长方体        Ixx = m/12 (y² + z²)   Iyy = m/12 (x² + z²)   Izz = m/12 (x² + y²)
圆柱(轴=自身z) Ixx = Iyy = m(3r² + L²)/12                    Izz = m r² / 2
球            I = 2/5 m r²
```

关键细节：URDF 的 `<inertia>` 定义在 **link 自身坐标系**，而 `<inertial><origin>`
描述"质心相对 link 原点"。本模型里 link 原点 = 关节轴，几何体向 +z 伸出，
所以宏接受 `com_z` 参数并用**平行轴定理**把 I 从质心平移到 link 原点：

```xml
<xacro:cylinder_inertial radius="${r1}" length="${l1}" mass="0.95" com_z="${l1/2}"/>
```

漏掉这一步的后果不是"数值差一点"，而是惯量主轴错位 → KDL/MoveIt 的重力与
力矩估算偏掉，`robot_state_publisher` 的 TF 仍然对（它不看惯量），
所以**肉眼完全看不出来**，只有动力学相关行为会怪。

## 5. `world` dummy 根：一个必须踩一次才知道的坑

现象：不给 `base_link` 加 `<inertial>` 时 MoveIt 报根 link 惯量问题；给了以后
KDL 又抱怨根 link 带惯量。

> **KDL 不允许根 link 带惯量**，而 `base_link` 是底座、必须有质量
> （MoveIt 的质心/力矩估算要用）。

标准解法（本模型采用）：加一个无几何、无惯量的 dummy 根 `world`，用 fixed joint 连下去。

```xml
<link name="world"/>
<joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/>...
```

附带收益：TF 树里天然有 `world` 帧，规划场景放桌面/料盒时直接用它当参考。

★ **连带条件（改任何一边都要同时改另一边）**：SRDF 里**不能**再写
`<virtual_joint parent_frame="world" child_link="base_link"/>`。
`world → base_link` 的 TF 已由 `robot_state_publisher` 从 URDF 发布，
再声明一次就有两个发布者，TF 直接报 `multiple parent publishers`。
这条写在 `arm6.srdf` 的头部注释里。

## 6. ros2_control 接口声明（仿真与真机的分界线）

`arm6.ros2_control.xacro` 不是"模型"，而是一份**接口契约**：

```xml
<ros2_control name="arm6_mock_system" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">false</param>
    <param name="state_following_offset">0.0</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position">
      <param name="min">${-pi*160/180}</param><param name="max">${pi*160/180}</param>
    </command_interface>
    <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
    <state_interface name="velocity"/>
  </joint>
  ...
</ros2_control>
```

| 决定 | 理由 |
|---|---|
| 用 `mock_components/GenericSystem` | 它把"命令位置"直接回写成"状态位置"，因此**不需要物理引擎**就能跑完整条 规划 → 执行 → 到位校验 链路。这是本项目的验证目标：证明规划链路正确且可回归，不证明动力学保真 |
| 只声明 `position` 命令接口 | JTC 配 `position` 即可；加 velocity 命令接口会让 JTC 变成需要 `use_feedforward` 的另一套行为 |
| `command_interface` 上重复一遍 min/max | 硬件侧也守限位。URDF 的 `<limit>` 只管仿真，真机限位在驱动器里 —— 两层都写才不会出现"仿真能过、真机拒绝" |
| `mock_sensor_commands=false` | 没有力/扭矩传感，省掉 controller_manager 的传感器命令通道噪声 |
| `state_following_offset=0.0` | 显式写出"我要零偏置"。**跟踪误差分析（05 §5）的前提就是这个 0**：非零偏置会被当成稳态误差 |

**这一层是"仿真与真机共用同一套控制栈"的技术依据**：把 `<plugin>` 换成实机驱动
（串口/CAN/EtherCAT 的 ros2_control hardware），上层的 MoveIt + JTC 一行都不用改。

## 7. 验证

```bash
# (a) xacro 能否解析（update_collisions.sh 的第一步也做这个）
xacro src/arm_description/urdf/arm6.urdf.xacro -o /dev/null

# (b) 三处定义是否自洽：URDF 限位 / joint_limits.yaml / SRDF 位形 / waypoints
bash scripts/check_limits.sh
#   CHECK-RESULT PASS (速度取小值一致 / 所有目标在限位内 / group 与关节集合匹配)

# (c) 只看模型与 TF（不需要 MoveIt）
ros2 launch arm_description display.launch.py
```

`check_limits.sh` 拦的是三类"跑起来才炸、而且报错很难对上原因"的错误：

1. `joint_limits.yaml` 的速度写得比 URDF 大 ⇒ MoveIt 取小值，那行配置**完全无效且无告警**；
2. SRDF `group_state` 或 `waypoints.yaml` 的目标越出 URDF 限位 ⇒ 表现为规划期
   `INVALID_GOAL`/无解；在限内但自碰撞则表现为 `GOAL_IN_COLLISION(-12)`；
3. 手指 prismatic 的单位（m）被当成 rad（`0.040` 写成 `40.0`）。

判据本身也要验证 —— 脚本头部写了自检命令，实测：

```bash
sed 's/joint2: -0.60/joint2: 3.50/; s/finger1_joint: 0.040/finger1_joint: 40.0/' \
    src/arm_demo/config/waypoints.yaml > /tmp/bad_wp.yaml
WP_FILE=/tmp/bad_wp.yaml bash scripts/check_limits.sh
#   ISSUE waypoints[2] grip_open: finger1_joint=40.0 越出 URDF 限位 [0.0000, 0.0400] (m)
#   ISSUE waypoints[2] grip_open: finger1_joint=40.0 看起来把 rad 当成了 m
#   ISSUE waypoints[3] home: joint2=3.5 越出 URDF 限位 [-1.7453, 1.7453] (rad)
#   CHECK-RESULT FAIL (3 issues)
```

## 8. 外观（`<visual>`）与碰撞（`<collision>`）的分工

2026-09-30 这一轮把每个 link 的 visual 从"一个几何体"改成"一份零件清单"
（73 个零件 / 6 种材料），目的是让 RViz 里看得出这是工业六轴而不是一摞方盒：
关节毂（绕 y 的短圆柱盘）、肩/腕的**颊板夹毂**、臂体的薄腹板 + 两侧盖板、
腕部法兰 + 螺栓圈、夹爪的指座/指骨/内侧橡胶垫/指尖。

**碰撞体一个字节都没改**，所以 SRDF 碰撞矩阵、`update_collisions.sh`、
`check_limits.sh` 和 08 篇里所有计时/到位数字继续有效。这条边界不是靠"我小心了"
守的，是两条机器判据（`scripts/check_visual_envelope.py`，纯离线、不需要 ROS）：

| 结果行 | 判什么 | 本轮实测 |
|---|---|---|
| `BASELINE-RESULT PASS` | 与改造前基线 URDF 比，每个 link 的 collision 几何体（位姿 + 类型 + 尺寸）逐项一致 | 12 个 link / 10 个几何体，**0 处差异** |
| `ENVELOPE-RESULT PASS` | 每个 link 的 visual 并集 AABB 相对 collision 并集 AABB 的外扩量 | 侧向最大 **5.00 mm**；端面 link 最大 **5.00 mm** |

外扩规则三条，写在脚本头部，也写在 xacro 的宏注释里：

1. **侧向 (x/y) ≤ 6 mm** —— 侧向才是"看着穿进桌子、规划却说不碰"的方向；
2. **端面朝向工件的 link**（`link4` / `link6` / `gripper_base` / 两指 / `tool0`）
   的 ±z ≤ 6 mm —— 这里多几毫米会直接变成"规划到了但没夹住"；
3. **其余 link 的 ±z 只报数不判红**，上限 40 mm —— 俯仰毂是**绕着自己的轴**画的，
   往轴后面凸出一个毂半径是几何必然。实测 link1 20 mm / link2 35 mm / link5 27 mm，
   都正好等于各自的毂半径，不是失控。

> 判据自己也过了 7 项自测（`--selftest`），其中两条值得留下：
> **红样 1~3 第一版全部"通过"** —— 因为 `str.replace` 默认全替换，我把 visual 和
> collision 一起改大了，外扩量恒为 0。一个永远不会红的判据比没有判据更糟，
> 所以自测里必须同时有"该红的红"和**"该绿的绿"**：`link6` 沿 z 凸 20 mm 判红、
> 同名几何放在非端面 link 上判绿，这一对因果对照钉住的就是规则 3 的边界。

造型侧的可维护性：零件坐标全部走 `vz_cyl` / `vy_cyl` / `v_box` / `v_bolt` 四个宏，
尺寸一律从 `l2/w2/r4…` 这些既有参数推导（如毂半径 = `w2/2 + 0.005`），
所以 §3 那张长度链表改一个数，外观会跟着整体缩放，不会出现"改了尺寸忘了改壳"。

一处**顺手纠正**：`arm6-viewer.html` 的 HUD 原来手抄"理论最大前伸 0.705 m"，
用同一套正运动学现算 extended 位形的 tool0 水平距离，实际是 **0.685 m**
（差 20 mm = 夹爪座高度 `gb_h` 被多算了一次）。现在这个数由 FK 现算，见 06 §11。

### 双击入口与 FAIL 分支实测

`看外观改造验收.bat`（项目根目录，纯离线）依次跑
`ENVELOPE-RESULT` → `BASELINE-RESULT` → `SELFTEST-RESULT` → `VIEWER-RESULT`，
四段输出另存 `logs/visual_check_last.txt`。**判据见过 FAIL 才算数**，两条人工红样：

| 红样 | 怎么造的 | 结果 |
|---|---|---|
| visual 侧移 | 把 link2 一片侧盖板的 `y` 从 0.025 改成 0.05 | `ENVELOPE-RESULT FAIL (max 侧向 25.00 mm …)` + `ISSUE link2: 侧向 25.00 mm > 6.0`，退出码 **1** |
| collision 被改 | 把 link2 的 collision 盒 `0.24` 改成 `0.235` | `BASELINE-RESULT FAIL 以下 link 的 collision 与基线不同: link2`，并把基线/现值两行都印出来 |

> 造红样时踩到一个 Windows 侧的坑，记下来省下次的时间：**Git-Bash 的 `/tmp` 与
> Windows 版 `python` 眼里的 `/tmp` 不是同一个目录**（前者在 `%TEMP%`，本机是
> `D:\Relo\Temp`），用 heredoc 里的 `/tmp/xxx` 喂给 `python` 写文件会直接
> `FileNotFoundError`。红样一律写到工程目录或会话目录里。
> 另一条：heredoc 里的字面量 `nul` 会被 MSYS 改写成 `/dev/null`，
> 所以 BAT 里的 `>nul` 必须由脚本拼接出来，不能直接写。

## 9. 下一步

规划侧配置（SRDF、运动学、控制器、碰撞矩阵）见
[04-MoveIt 配置与碰撞矩阵](04-MoveIt配置与碰撞矩阵.md)。
