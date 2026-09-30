# 04 MoveIt 配置与碰撞矩阵

> 包：`src/arm_moveit_config/`
> 这个包不含一行 C++/Python —— 它的全部内容是 6 份配置 + 7 个 launch，
> 由 `MoveItConfigsBuilder` 在运行期组装成 move_group 的参数。

## 1. 文件清单：谁读它、读来干什么

| 文件 | 读者 | 作用 |
|---|---|---|
| `config/arm6.srdf` | move_group（`robot_description_semantic`） | 规划组、末端执行器、命名位形、**碰撞豁免矩阵** |
| `config/joint_limits.yaml` | move_group | 速度/加速度/jerk 上限，喂给时间参数化(TOTG) |
| `config/kinematics.yaml` | move_group | IK 插件与超时 |
| `config/moveit_controllers.yaml` | move_group | 规划出来的轨迹交给哪个 action、执行监控容差 |
| `config/ros2_controllers.yaml` | `controller_manager` | 加载哪些控制器、JTC 的关节/接口/容差 |
| `config/sensors_3d.yaml` | move_group | 空（无点云），保留文件避免 Octomap 监控报缺配置 |
| `config/moveit.rviz` | rviz2 | MotionPlanning 面板；另有两个会影响证据的开关：`TF: false`（避免坐标轴糊满截图）与 `Planned Path/Loop Animation: false`（避免截图里出现第二只手臂）。相机参数与这两个值的来龙去脉见 06 §5/§6 |
| `launch/moveit.launch.py` | — | 一键 demo（含 `use_rviz:=false` 无头模式） |
| `launch/move_group.launch.py`、`rsp.launch.py`、`spawn_controllers.launch.py`、`moveit_rviz.launch.py`、`warehouse_db.launch.py`、`setup_assistant.launch.py` | — | 官方 `generate_*_launch()` 的薄封装 |
| `launch/gdb_settings.gdb` | `debug:=true` 时的 move_group | 必须存在，否则 debug 模式启动即失败（见 02 §5） |

组装入口只有一行链式调用，但它是版本敏感的：

```python
moveit_config = (
    MoveItConfigsBuilder("arm6", package_name="arm_moveit_config")
    .planning_pipelines(pipelines=["ompl"])   # 只装 ompl；加 chomp/stomp 会拖慢启动
    .to_moveit_configs()
)
return generate_demo_launch(moveit_config)
```

`generate_demo_launch()` 按依赖顺序拉起 6 件事：
`static_virtual_joint_tfs → rsp → move_group → rviz(可选) → ros2_control_node(mock) → spawn_controllers`。
其中 `ros2_control_node` 的参数只有 `ros2_controllers.yaml`，它要用的 URDF 通过
remap（`/controller_manager/robot_description ← /robot_description`）从 rsp 拿 ——
**这就是 rsp 必须先起来的原因**。

## 2. SRDF：规划语义

```xml
<group name="arm">      <chain base_link="base_link" tip_link="tool0"/> </group>
<group name="gripper">  finger1_link finger2_link gripper_base + 两个 prismatic 关节 </group>
<end_effector name="gripper" parent_link="tool0" group="gripper" parent_group="arm"/>
<group_state name="up|home|extended" group="arm"/>
<group_state name="open|closed" group="gripper"/>
```

- **规划组是 MoveIt 的基本单位**：`set_start_state`、目标约束、IK 全部按组生效。
  tip 选 `tool0` 而不是 `link6`（理由见 03 §1）。
- `gripper` 单独成组才能被命名位形一句话控制，也才能声明成 `end_effector`
  挂到 `arm` 上 —— rviz 的 MotionPlanning 面板因此才给出"抓取"子面板，
  且抓取时夹爪碰撞体会自动纳入 arm 的规划场景。
- `group_state` 的数值必须落在 URDF `<limit>` 内。SRDF 解析**不报错**，
  失败要等到规划时才出现。`bash scripts/check_limits.sh` 就是提前把这条拦住。
- 世界坐标系不在 SRDF 里声明（不写 `<virtual_joint>`）：URDF 已有 dummy 根 `world`，
  再声明会出现同一帧两个 TF 发布者 → `multiple parent publishers`。见 03 §5。

## 3. ★ 碰撞矩阵：用工具算，并且算到收敛

### 3.1 为什么不能手写

碰撞矩阵决定规划器认为哪些姿态合法。

- 手工勾**少**了 ⇒ 大量"假碰撞"，规划器在明明能动的位形上报 fail；
- 手工勾**多**了 ⇒ 真碰撞被放行，规划出的路径实际会撞，而仿真里看起来一切正常
  （因为没人再检查）。

`collisions_updater` 在整条关节行程里随机采样，只有"从来没碰过"的对才写进
disabled —— 给的是**有证据的结论**，不是人的直觉。

### 3.2 命令

```bash
bash scripts/update_collisions.sh 10000000
```

实际执行：

```bash
xacro arm6.urdf.xacro -o /dev/null          # 先确认能解析，否则后面全是白跑
ros2 run moveit_setup_assistant collisions_updater \
    --urdf <xacro> --srdf <srdf> --output <artifacts/arm6_calculated.srdf> \
    --default --always --keep \
    --trials 10000000 --min-collision-fraction 0.05
```

> ⚠ **不要传 `--config-pkg`**。MoveIt 2.12.4 的 `collisions_updater` 用旧版
> `.setup_assistant` 的 schema 去解析，期望根节点有 `package_settings`，
> 遇到 Jazzy 生成的 `moveit_setup_assistant_config` 直接报
> `invalid node; first invalid key: "package_settings"` 并退出 1。
> 只给 `--urdf/--srdf/--output` 三个路径就能跑完。

输出落在 `artifacts/` 而不是直接覆盖 `src/`：Windows 侧是唯一真源，
WSL 的 `src/` 只是同步副本，就地覆盖会在下一次 `sync.sh` 时被无声冲掉。

### 3.3 TRIALS 不是越大越好，而是要"跑到收敛"

同一份 URDF，只改采样次数（`artifacts/` 里留下了这几份候选）：

| trials | 被豁免的对数 | 相对上一次的差异 | 文件 |
|---|---|---|---|
| 20 000 | 38 | — | `arm6_20k.srdf` |
| 200 000 | 37 | **−link1/link5**（这次采到了碰撞） | `arm6_200k.srdf` |
| 1 000 000 | 36 | **−link1/link4**（又采到一次） | `arm6_1m.srdf` |
| 10 000 000 | 36 | 与 1M **逐对一致** ⇒ 判定收敛 | `arm6_10m.srdf` = 采用版 |

结论：**2 万次会把"其实会碰"的关节对误判成"永不相碰"**，
后果是规划出一条实际会自撞的路径，而仿真里看不出问题 —— 这类错误不会以
"报错"的形式出现，只会以"现场撞机"的形式出现。默认值因此设 1e7（约 9 s）。

### 3.4 最终矩阵构成（36 条豁免 / 13 对仍在检查）

候选文件都留在 `artifacts/` 里，对数可以直接数：
`grep -c '<disable_collisions' artifacts/arm6*.srdf`

| 文件 | 对数 | 说明 |
|---|---|---|
| `artifacts/arm6_20k.srdf` | 38 | 20 k trials 的候选（多豁免了 `link1↔link5`、`link1↔link4`） |
| `artifacts/arm6_200k.srdf` | 37 | 200 k：`link1↔link5` 被采到碰撞，掉出去 |
| `artifacts/arm6_1m.srdf` | 36 | 1 M：`link1↔link4` 又掉出去 |
| `artifacts/arm6_10m.srdf` | 36 | 10 M：与 1 M **逐对 diff 为空** ⇒ 判定收敛 |
| `config/arm6.srdf` | 36 | **= 在用版本**（23 `Never` + 9 `Adjacent` + 4 `tool0 has no geometry`），与 `arm6_10m.srdf` 逐对一致 |
| `artifacts/arm6_calculated.srdf` | 36 | `update_collisions.sh` 最近一次的输出，与在用版本一致 |

> 20 k 那份原本躺在 `artifacts/` 下叫 `arm6.srdf`，与在用的 `config/arm6.srdf`
> 只差一个目录名 —— 数对数时极容易把它当成 canonical（本轮就误读了一次，
> 得到"在用的 SRDF 有 38 对、比收敛版多豁免 2 对"的错误结论，而那是往
> "漏检真碰撞"的危险方向偏的）。已改名 `arm6_20k.srdf`：**候选文件名里必须带
> 采样次数**，因为这份文件的语义就是"某个 trials 下的中间结论"。

"还剩多少对在真正做碰撞检查"是有算法的，不是感觉：

```
URDF 里带 <collision> 的 link = 10 个（tool0 只有 visual，world 无几何）
两两组合                    = C(10,2) = 45 对
36 条豁免里命中这 45 对的     = 32 条（另 4 条涉及 tool0，属"写了也不起作用"的说明性条目）
仍在检查                     = 45 − 32   = 13 对
```

```
base_link <-> link4, link5, link6, gripper_base, finger1_link, finger2_link
link1     <-> link4, link5, link6, gripper_base, finger1_link, finger2_link
link4     <-> link6
```

前两组就是"满伸展时大臂/小臂压向基座方向"的真实风险项 ——
`extended`（joint2=90°）这个点位存在的意义正是把臂推到最容易自碰的构型上，
规划能过就说明矩阵没勾错。

验收：`bash scripts/update_collisions.sh 10000000` 最后一行
`COLLISION-RESULT CONVERGED pairs=36 trials=10000000` 表示与现有 SRDF 逐对一致。

## 4. 运动学与时间参数化

### 4.1 `kinematics.yaml`

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

- 用 KDL 而不是 `bio_ik`/`trac_ik`：KDL 是 MoveIt 自带的解析-数值混合求解
  （NR 迭代 + 零空间投影），零外部依赖、结果确定，适合做回归基线。
  `bio_ik`（LM 优化）在 6 轴上成功率更高，但每次重启可能给出不同解，
  会把"规划器行为"混进"求解器随机性"。要换只改这一行。
- `timeout=0.05 s`：KDL 对 6 轴臂一次成功求解通常 < 5 ms，50 ms 已是 10 倍余量；
  再放大只会拖慢"目标不可达"的失败判定，而 rviz 里拖 goal 是实时交互。
- `gripper` 组不写：两个 prismatic 关节由命名位形/关节目标直接给值，没有末端位姿概念。

### 4.2 `joint_limits.yaml`：为什么 URDF 有 velocity 还不够

URDF 的 `<limit>` 只有 `velocity` 与 `effort`，**没有加速度和 jerk**。
而 MoveIt 的时间参数化是 TOTG（Time-Optimal Path Parameterization），
约束集必须是 (v, a, jerk)。缺 a 时 TOTG 退化成"只按速度限速"，
拐角处加速度理论上无穷大 —— 日志里那句
`Joint limit for acceleration not defined` 就是这个，
表现出来是"轨迹能跑但电机指令是方波"。真机上这一步不设就是给硬件挖坑。

| 关节组 | v_max | a_max | max_jerk | 依据 |
|---|---|---|---|---|
| joint1..3 | 3.14 rad/s | 1.875 rad/s² | 200 | 承担主要重力矩，保守 |
| joint4..6 | 4.19 rad/s | 2.5 rad/s² | 200 | 腕部惯量小（0.12~0.25 kg） |
| finger1/2 | 0.10 m/s | 0.5 m/s² | 100 | 行程只有 40 mm |

MoveIt 取 **URDF 与本文件两者的较小值**，所以这里的数值一律 ≤ URDF。
一致性由 `bash scripts/check_limits.sh` 断言（规则 1）。
`max_jerk` 限制加加速度，让 JTC 下发的指令连续 —— 真机上这直接决定振动与噪声。

### 4.3 规划器名从哪里查

`config/ompl_planning.yaml` 里**没有**规划器清单（它只有 plugins/adapters）。
可用名字在安装介质里：

```bash
grep -oE '^\s+[A-Za-z]+' /opt/ros/jazzy/share/moveit_configs_utils/default_configs/ompl_defaults.yaml
# AnytimePathShortening / BLPIECE / BKPIECE / EST / KPIECE / LBKPIECE / PRM / RRT / RRTConnect / ...
```

本项目写死 `RRTConnect`：它在 6 轴臂上稳定、快，且**写死名字才能做跨次比较**
（见 05 §6）。`planner_id` 通过 ROS 参数传入，不硬编码在代码里。

## 5. 控制器接缝，以及"拆成两个 JTC"的改动清单

MoveIt 与 ros2_control 之间是两份文件对接：

```
moveit_controllers.yaml (move_group 读)          ros2_controllers.yaml (controller_manager 读)
  moveit_controller_manager:                       controller_manager.update_rate: 100
    moveit_simple_controller_manager               arm6_controller:
  moveit_simple_controller_manager:                  joints: [joint1..6, finger1/2_joint]
    controller_names: [arm6_controller]              command_interfaces: [position]
    arm6_controller:                                 state_publish_rate: 50
      type: FollowJointTrajectory                    allow_partial_joints_goal: true   ★
      action_ns: follow_joint_trajectory             constraints: {trajectory: 0.05, goal: 0.003}
      joints: 8 个全列                            stopped_velocity_tolerance: 0.02
  trajectory_execution:                            allow_nonzero_velocity_at_trajectory_end: true
    allowed_start_tolerance: 0.01
    allowed_execution_duration_scaling: 1.2
```

### 5.1 ★ 成对约束：`allow_partial_joints_goal`

本项目把 8 个关节合成**一个** JTC（时序天然同步，适合验证规划链路）。
而 `arm_demo` 是**按 planning group 逐条下发**的 —— `group=arm` 的轨迹只带 6 个关节、
`group=gripper` 只带 2 个。两者相遇必须满足：

```yaml
allow_partial_joints_goal: true
```

否则控制器在收到目标的一瞬间就 REJECTED，表现为**"规划成功但执行失败"**，
MoveIt 侧只报 `CONTROL_FAILED(-4)`，极易误判成规划器的问题。
改任何一边都要同时改另一边 —— 这条同时写在两份 yaml 的头部注释里。

### 5.2 换成两个 JTC 的改动清单（真机形态：夹爪常走另一路总线）

| # | 文件 | 改动 |
|---|---|---|
| 1 | `ros2_controllers.yaml` | 新增 `gripper_controller`（joints 只列两个指关节），从 `arm6_controller` 的 joints 里删掉它们；`allow_partial_joints_goal` 改回 **false** |
| 2 | `moveit_controllers.yaml` | `controller_names` 加 `gripper_controller`，两个块各列自己的 joints；`trajectory_execution` 不变 |
| 3 | `spawn_controllers.launch.py` | 多 spawn 一个控制器（或改用 `spawner` 的多参数形式） |
| 4 | `arm6.ros2_control.xacro` | 不用改 —— 硬件接口声明与控制器划分无关，这正是 ros2_control 抽象的价值 |
| 5 | `arm_demo/config/waypoints.yaml` | 不用改 —— 节点按 group 下发，本来就没假设"一条轨迹带 8 个关节" |
| 6 | 代价 | 两条轨迹之间**没有同步**，"手臂到位后夹爪才动"要靠节点侧的顺序保证（本项目节点本来就是串行的，所以行为不变） |

### 5.3 执行监控（TEM）的两个旋钮

- `allowed_start_tolerance: 0.01` —— 执行开始时"当前状态与轨迹首点的最大偏差"。
  mock 硬件下恒为 0；真机上这是最常见的"`execute()` 直接 ABORTED"原因，
  写出来并注释清楚，是为了换硬件时一眼能看到旋钮。
- `allowed_execution_duration_scaling: 1.2` —— 实际耗时 > 规划时长 ×1.2 判失败，
  是 TEM 唯一的看门狗，**别关掉**。

## 6. 下一步

点位控制节点如何消费以上配置：[05-点位控制节点与运动日志](05-点位控制节点与运动日志.md)
