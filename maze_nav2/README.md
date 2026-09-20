# maze_nav2 —— Gazebo 迷宫里的 Cartographer + Nav2 自主导航

> ROS 2 Humble · Gazebo Classic · Nav2 · Cartographer：程序生成的 8×6 m 迷宫（走廊净宽 0.88 m），
> 一台 0.22 m 差速小车跑通 **SLAM 建图** 与 **点到点自主导航** 两条链路。单包 `maze_bot`。
> 姊妹项目 [`mybot/`](../mybot/)：巡线 + 避障小车；仓库总览见[根 README](../README.md)。

![迷宫导航实录（RViz 俯视，2.4× 播放）](docs/assets/maze_nav.gif)

## 项目是什么

迷宫为程序生成：8 m × 6 m、8×6 格、格子 1 m、走廊净宽 0.88 m、墙厚 0.12 m / 高 0.8 m，
起点 (-3.5, -2.5)、终点 (3.5, 2.5)——完整生成档案（含全部墙体真值，生成种子 20260917）在 `maze_info.json`。
小车是 0.22 m 的差速底盘（车体半径 0.11 m，URDF 在 `urdf/maze_bot.urdf.xacro`，`diff_drive` 插件发布 `/odom`），
传感器是一台 2D 激光：180 线 / 360° / 8 m 量程 / 12 Hz。

工程提供两条自主链路，都用同一套仿真：

- **建图**：Cartographer 2D SLAM（`mapping` 模式；另附一份 slam_toolbox 替代配置）；
- **导航**：Nav2 全套——AMCL 定位 + 全局/局部代价地图 + Regulated Pure Pursuit 控制器（`navigate` 模式）。

## 环境

| 项 | 值 |
|---|---|
| 平台 | WSL2 · Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 仿真 | Gazebo Classic（`gazebo_ros_pkgs`） |
| 导航 / 建图 | Nav2（`nav2_bringup`）· Cartographer（`cartographer_ros`）· slam_toolbox（可选替代） |

本工程实测所用依赖（ROS 2 Humble，与 `package.xml` 对应）：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup \
                 ros-humble-cartographer-ros ros-humble-slam-toolbox
```

启动脚本已封装 WSL2 下必须的 DDS 环境（`ROS_DOMAIN_ID=42` + `ROS_LOCALHOST_ONLY=1`，
否则多播发现会失败）。

## 快速开始

```bash
# 1) 取工程（只需要迷宫项目这一份包）
mkdir -p ~/maze_ws/src && cd ~/maze_ws
git clone <仓库地址> _clone
cp -r _clone/maze_nav2/src/maze_bot src/
cp _clone/maze_nav2/start_sim.sh .
rm -rf _clone

# 2) 编译并跑
colcon build

bash start_sim.sh            # 默认：Gazebo GUI + RViz 完整仿真
bash start_sim.sh navigate   # 导航：仿真 + AMCL + Nav2（RViz 由 launch 单独拉起）
bash start_sim.sh mapping    # 建图：仿真 + Cartographer
```

其余模式：`headless`（无界面，省资源）· `robot`（只发机器人状态，调 URDF 用）· `clean`（仅清理环境）。

注意：`start_sim.sh` 里工作区路径写死为 `WS=$HOME/maze_ws`——克隆到别处时改这一行即可。

## 结构导览

```
maze_nav2/
├── start_sim.sh          # 一键启动器：sim(默认)/headless/mapping/navigate/robot/clean 六模式
├── clean_ros2_env.sh     # 清 FastRTPS 共享内存残留（DDS 起不来时急救）
├── nav2_lite.rviz        # 精简版 Nav2 RViz 视图（录制演示时在此配置上改为固定俯视）
├── maze_info.json        # 迷宫生成档案：种子、8×6 网格、走廊宽 0.88、起终点、墙体真值
├── src/maze_bot/
│   ├── launch/           # simulate 仿真基座 · navigate Nav2 · mapping Cartographer · slam_toolbox 替代建图 · robot_state 调 URDF
│   ├── config/           # nav2_params.yaml · maze_2d.lua（Cartographer）· maze_slam_toolbox.yaml · sim_view.rviz / mapping_view.rviz
│   ├── maps/             # maze_map.* 真值地图（9×7 m @0.05 m/px）· maze_scan.* 一次建图产物（@0.03 m/px，yaml 里 image 仍是保存时的 /tmp 绝对路径）
│   ├── urdf/             # maze_bot.urdf.xacro：0.22 m 差速小车（车体 + 双轮 + 万向轮 + 2D 激光 + 驱动插件）
│   ├── worlds/           # maze.world：迷宫墙体 + 物理参数（200 Hz 步长）+ 相机预设
│   └── scripts/          # scan_time_fix.py：激光时间戳修正 relay（/scan → /scan_fixed）
└── docs/assets/          # maze_nav.gif（导航实录）· tf_tree.png（运行时 TF 树）
```

## Nav2 配置要点（`config/nav2_params.yaml`，数字照抄该文件）

- **局部代价地图 3 m × 3 m**（`width/height: 3`，`resolution: 0.05`，滚动窗口跟随机器人）。
  迷宫单格 1 m、走廊净宽 0.88 m，3 m 窗口能同时看到当前走廊与前后交叉口，避免"只看眼前"决策。
- **膨胀半径 `inflation_radius: 0.25`**（局部/全局相同，`robot_radius: 0.15`）。
  两侧共 0.50 m：0.88 − 0.50 = 0.38 m，仍容得下按 0.15 m 半径（直径 0.30 m）建模的车；
  若沿用官方默认 0.55（两侧 1.10 m > 0.88 m），所有走廊会被判成死区——这是全文件最关键的改动。
- **控制器 20 Hz**（`controller_frequency: 20.0`），FollowPath 从 DWB 换成 Regulated Pure Pursuit
  （`desired_linear_vel: 0.45`，转弯自动降速 `use_regulated_linear_velocity_scaling: true`）；
  最终下发还经 velocity_smoother 限幅到 0.26 m/s（`max_velocity: [0.26, 0.0, 1.0]`）。
- **全局规划器 NavFn + A\***（`use_astar: true`，`tolerance: 0.5`）——窄走廊梯度地形下比 Dijkstra 出路径更稳。
- **AMCL 订阅 `/scan_fixed`** 并显式设置初始位姿 (-3.5, -2.5, 0)——起点在墙角，
  不给初值粒子会全错、定位发散（`set_initial_pose: true` + `initial_pose`）。
- **全局代价地图只用静态层 + 膨胀层**（不挂激光 obstacle 层）：已知静态迷宫，
  激光 clearing 会把静态墙清掉一大半，规划器"看不见墙"→ 远距离规划失败。

## 结果与已知限制

一次完整导航实录（2026-09-20，`navigate` 模式）：

| 指标 | 实测值 | 出处 |
|---|---|---|
| 目标状态 | **SUCCEEDED** | `navigate_to_pose` action 最终结果 |
| 恢复行为次数 | **0** | `number_of_recoveries` |
| 导航耗时 | **55.6 s**（仿真时间） | `navigation_time` |
| 到达时距目标 | **0.246 m**（容差 0.25 m） | `distance_remaining` |
| 演示 GIF | 21 s / 10 fps / 760×582 | `docs/assets/maze_nav.gif`（3.3 MB，整段按 2.4× 播放） |

已知限制（按实况）：

- **导航用的是真值地图**（`maps/maze_map.yaml`，与场景完全一致）——目的是考察完整 Nav2 栈的行为。
  现场建图 → 导航的端到端闭环未在本仓库验证；`navigate.launch.py` 的 `map:=` 参数已支持任意地图 yaml。
- **WSL2 时钟跳变**：Hyper-V 对时与 systemd-timesyncd 互踩时，墙上钟会"每 ~5 s 前跳数秒再弹回"，
  把实时因子（RTF）从 1.0 打到 0.07，录制与计时类实验必须先做时钟自检（同类现场记录见 [`mybot`](../mybot/)）。
- **激光时间戳滞后半个采样周期**（12 Hz → 41.7 ms，`scripts/scan_time_fix.py` 的用途）：
  对"直接拿 TF 投影激光点"的消费者（AMCL、代价地图）修正有效——里程计投影基准下
  R@±3 从 91.2% 提升到 98.4%，Nav2 链路因此统一订阅修正后的 `/scan_fixed`；
  但对做 scan matching 的 Cartographer **实测反而有害**（17.4% → 14.9%、自由空间 IoU 0.359 → 0.295，
  会破坏 scan↔odom 时间模型），因此建图模式默认仍订阅原始 `/scan`
  （relay 照常启动，`carto_scan_topic:=scan_fixed` 可切换）。
- **WSLg 软渲染的取舍**：`start_sim.sh` 显式导出 `LIBGL_ALWAYS_SOFTWARE=1` / `GALLIUM_DRIVER=llvmpipe`，
  用帧率换确定性（WSLg 下 GPU 路径曾出现窗口白屏/卡死；`nav2_bringup` 内嵌的 RViz 甚至"进程活着但窗口不出"，
  所以 `navigate.launch.py` 把 RViz 拆出来单独拉）。同样的约束也反映在 `worlds/maze.world`：
  物理步长放宽到 0.005 s / 200 Hz——注释记录 1 kHz 步长时软渲染下 RTF 只有 0.10，跑一遍要 ~50 分钟墙钟。
  若你的 WSLg GPU 直通正常（`glxinfo` 显示硬件渲染），可以去掉这两行导出提速。
- 小瑕疵：`robot` 模式默认引用的 `config/robot_view.rviz` 未随工程保存（不写它 RViz 起默认视角）。

## 素材

- `docs/assets/maze_nav.gif` —— 一次完整导航的 RViz 俯视实录（21 s / 10 fps / 760×582 / 3.3 MB）：
  红色 = 全局规划路径，蓝色 = 局部规划，红点 = 激光扫描，半透明色块 = 代价地图（跟随机器人的蓝青色块即局部代价地图）。
  用 RViz 俯视而非 Gazebo 视角：0.8 m 墙在低角相机下会把小车遮挡掉约 3/4 的帧，俯视则全程无遮挡。
- `docs/assets/tf_tree.png` —— 运行时 TF 树渲染（`odom → base_footprint → base_link → laser_frame / wheel_*`）。
