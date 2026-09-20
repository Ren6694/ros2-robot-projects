# mybot —— Gazebo 仿真里的差速巡线小车

> ROS 2 Humble · Gazebo 11 · 从一个手写 URDF 开始，一路做到**自主巡线 + 避障 + 参数化调优 + 固件侧混控层**。
> 两个包：`mybot_description`（车长什么样）与 `mybot_control`（车怎么看、怎么动）。
> 姊妹项目 [`maze_nav2/`](../maze_nav2/)：另一台车的 SLAM 建图 + Nav2 导航，见[仓库根 README](../README.md)。

## P1–P5 脉络

| 阶段 | 覆盖 | 内容 | 状态 |
|---|---|---|---|
| **P1** | D1–D6 | xacro 参数化建模、caster 万向轮与惯性矩阵、Gazebo 11 接入、`ros2_control` 栈、激光 360 线 + 摄像头 320×240 | ✅ |
| **P2** | D7–D13 | 参数化巡线赛道、HSV 阈值分割、逆透视米制线特征（e, θ）+ RANSAC、PD 巡线、避障状态机、可复现参数台架、ESP32 差速混控层 | ✅ |
| **P3–P5** | — | 真机移植、建图与导航（见姊妹项目）、micro-ROS | 计划中 |

## 两包架构

```
mybot_description                             mybot_control
┌──────────────────────────────┐              ┌────────────────────────────────────┐
│ urdf/   核心模型 + 3 种驱动外壳│   /scan      │ 感知  line_mask.py    HSV 阈值分割   │
│ worlds/ 巡线赛道 + 障碍变体    │   /camera/   │      line_features_node.py          │
│ launch/ display/gazebo/       │   image_raw  │      （逆透视 → 米制 e,θ + RANSAC）  │
│         control/line_follow   │ ───────────► │ 控制  line_follow_pd.py   PD 20 Hz   │
│ config/ 控制器参数            │   /cmd_vel   │      line_follow_avoid.py 避障 FSM   │
│ rviz/   预设                  │ ◄─────────── │ 固件  firmware/mixer.h  纯函数混控   │
└──────────────────────────────┘              └────────────────────────────────────┘
              Gazebo 11 · /odom · /camera/camera_info
```

两包**只通过话题契约耦合**：`/camera/image_raw`、`/camera/camera_info`、`/scan`、`/cmd_vel`、`/odom`。
包内部再加 `/line/mask`、`/line/debug`、`/line/pose`、`/line/centers`、`/avoid/state` 作调试与验收用。

## 成果数字

| 指标 | 数值 | 出处 |
|---|---|---|
| 完整一圈用时 | **30.8 s**（两批各 3 次，中位数差 ≤0.05 cm） | D12 台架 18 跑 |
| 巡线精度（里程加权 \|e\|） | **1.07 cm**（逐帧 1.11 / p95 3.00） | `d12_recompute.py` 从 CSV 独立复算 |
| 舵量饱和帧 | **0**（全程未触发限幅） | 同上 |
| 避障停距（障碍前停死） | **29.6 cm** → 运行时删障 → 自己恢复并跑完一圈 | D11 验收 |
| 避障下的巡线质量 | \|e\| 均值 1.2 cm / 最大 4.6 cm（与纯巡线同级，**避障没牺牲精度**） | D11 CSV 独立复算 |
| 状态占比 | FOLLOW 83.8% / OBSTACLE_STOP 10.6% / DECEL 5.6% | 同上 |
| 固件混控断言 | **33 条**（符号 / 饱和等比缩 / NaN→刹车 / 死区 / 单位往返 / 工况自洽性） | `test_mixer.cpp` |
| 参数台架 | 6 配置 × 3 重复 = 18 圈，复位→等停→验起跑点三段把关 | D12 |

## 设计决策与踩坑精选

1. **为什么是 HSV 而不是灰度阈值**（D8）：赛道里故意放了一块暗红"地板污渍"当干扰。它的灰度亮度 ≈44，低于任何能抓住黑线的阈值；但 HSV 的 `V = max(R,G,B)` ≈110，与黑线的 26 拉开距离。四组对照（gray / +高斯 / hsv / +开运算）里只有 `hsv+blur5+open3` 做到**连通域 = 1 且最大域占比 = 100%**——判据是这两个数，不是"看着像"。

2. **e 绝不用斜率外推到车轴**（D9）：画面最近只能看到车前 ≈0.29 m，外推等于把斜率误差乘 0.3~0.5 灌进偏差，实测 12° 夹角时单这一项就制造 **68 mm 假偏差**。PD 的输入取"最近可用行的实测值"，要轴上值才显式调用 `e_at(0.0)`。

3. **`cmd_vel_timeout` 从 0.5 s 调到 2.0 s，以及为什么不是 5.0**（D7/D9）：它按**仿真时间**计时，低 RTF 下被拉成几秒墙上时间，DDS 成簇投递一叠加就把最新命令判成过期，车会每隔几秒自顿一次。2.0 s 实测最多再滑 0.4 m 且无顿挫；5.0 s 时松手还能跑 ≈1 m，遥控场景不安全。**真机必须回 0.5 s**，那是安全超时。

4. **录屏开销主动上报，顺带查出真凶是时钟**（D12）：桌面实录会吃 CPU 拖慢仿真，所以先量一次"不录屏的一圈"当基线，把两者之差报出来而不是装作不存在。连挂三轮后查出实时性真凶 = WSL 里 Hyper-V 对时与 systemd-timesyncd 互踩，墙上钟每 5 秒前跳 6.7 秒再弹回，把 RTF 从 1.0 打到 0.07。

5. **先判障碍、再判丢线**（D11）：真实场景就是"方块挡住黑线"——相机看不见线、激光看得见方块。若先判丢线，车会原地旋转找线，把正对它的障碍当成"线丢了"处理，转着转着就撞上。避障的分辨率上限是 `/scan` 的 5 Hz 而不是控制环的 20 Hz：0.30 m/s 下每帧激光之间车前进 6 cm，所有距离门限都按这个留余量。

## 快速开始

```bash
# 依赖：Ubuntu 22.04 / ROS 2 Humble / Gazebo 11（gazebo_ros_pkgs）/ ros2_control
mkdir -p ~/ws/src && cd ~/ws/src && git clone <仓库地址> && cd ..
colcon build && source install/setup.bash

# 1) 只看模型（RViz + 关节滑条）
ros2 launch mybot_description display.launch.py

# 2) 起巡线赛道（WSLg 下建议先 headless 起服务，再单独拉窗口）
ros2 launch mybot_description line_follow.launch.py gui:=false image_view:=false

# 3) 完整自主巡线（另开终端）
ros2 run mybot_control line_features_node.py            # 感知：e / θ
ros2 run mybot_control line_follow_pd.py --csv /tmp/lap.csv   # 控制：PD 20 Hz

# 4) 带避障（换成带障碍的世界，运行时可用 /delete_entity 移开）
ros2 launch mybot_description line_follow.launch.py \
    world:="$(ros2 pkg prefix mybot_description)/share/mybot_description/worlds/line_following_obstacle.world" \
    gui:=false image_view:=false
ros2 run mybot_control line_follow_avoid.py

# 5) 纯逻辑单测（不依赖仿真，毫秒级）
ros2 run mybot_control test_line_features.py    # 解析真值 12/12
ros2 run mybot_control test_pd_line.py          # 符号/限幅/NaN + 参数自洽性
ros2 run mybot_control test_avoid.py            # 4 条安全性质 + 实测噪声零翻转

# 6) 固件混控层宿主机断言（同一份 mixer.h，未接硬件即可跑）
cd mybot_control/firmware && g++ -std=c++17 -I. test_mixer.cpp -o /tmp/test_mixer && /tmp/test_mixer
```

参数与开关：`camera_rate:=30`（摄像头步进）、`camera_pitch:=0.6`（俯角）、`world:=<path>`（换世界）、
PD 侧 `--kp --kd --w-max --v-max`、避障侧 `--stop-dist --warn-dist` 等，`--help` 齐全。

## 阶段索引（提交对照）

| 阶段 | 提交 | 标题 |
|---|---|---|
| D1–D6 | `6e84300` | P1 D1-D6: mybot_description first release |
| D7 | `43f79c4` | P2 D7: line-following track world + camera chain |
| D8 | `ccb0d6b` | P2 D8: mybot_control vision package + track distractor; repo moved to monorepo |
| D8 | `4ee19c5` | D8 docs: tune mode unusable under WSLg (no RAIL window) - compare is the acceptance path |
| D8 | `78939fc` | D8 perf recheck: camera_rate arg + corrected frequency findings |
| D9 | `9cb27e4` | P2 D9: metric line features (e, theta) via inverse projection + RANSAC |
| D9+ | `0a53b19` | D9+: user-facing controls + RTF split; cmd_vel_timeout 5.0 -> 2.0 |
| D10 | `23743da` | P2 D10: PD line following - first autonomous lap |
| D10+ | `3ff23ac` | Add rtf_probe.py; correct the RTF constants the docs were carrying |
| D11 | `d382899` | P2 D11: obstacle avoidance state machine - line following and avoidance coexist |
| D12 | `5ba81ab` | P2 D12: reproducible tuning bench - what w_max actually tunes Turns "it drives a lap" into "we know which number to set and why". Two of the three findings were measurement artefacts that had to be dug out first, so they are documented as prominently as the parameters themselves. |
| D12+ | `4246a99` | D12 follow-up: re-ran all 18 laps on the repaired clock, added the drift check |
| D13 | `aa04f3a` | D13: ESP32 toolchain without GitHub, plus the differential mixer layer |

各阶段的完整过程记录（含被实验推翻的中间结论）在每日提交信息与包内 README 里：
[`mybot_description`](mybot_description/README.md)（P1/D1–D7）· [`mybot_control`](mybot_control/README.md)（P2/D8–D13）。
