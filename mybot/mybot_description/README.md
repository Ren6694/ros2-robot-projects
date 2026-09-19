# mybot_description

> ROS 2 Humble · Gazebo 11 · 从零搭一台仿真差速小车。
> 「ROS2 嵌入式与建模 30 天培养计划」**P1（D1–D6）+ P2 起步（D7）**的交付物。

一个**教学向的**、**分层的** URDF/xacro 包，同一份核心模型可以挂两种驱动外壳、支持两种传感器路径，用来对照 Gazebo Classic 传统插件与 `ros2_control` 生态。作者学习路径的每一步都留下了 commit-style 的注释与文档。

---

## 目录结构

```
mybot_description/
├── urdf/
│   ├── mybot_core.urdf.xacro         # 几何 + 惯性 + 摩擦 + 万向轮（D2）
│   ├── mybot.urdf.xacro              # 经典 Gazebo diff_drive 插件外壳（D3）
│   ├── mybot.ros2c.urdf.xacro        # ros2_control + gazebo_ros2_control 外壳（D4）
│   ├── mybot_sensors.urdf.xacro      # 激光 360 线 + 摄像头 320×240（D5）
│   └── mybot.urdf                    # D1 手写 URDF，仅存档不作运行时用
├── config/
│   ├── mybot_controllers.yaml        # controller_manager + diff_drive_controller 参数
│   └── mybot_controllers_slowsim.yaml# D7：只覆盖 cmd_vel_timeout，应对慢仿真
├── launch/
│   ├── display.launch.py             # D1：RViz + joint_state_publisher_gui 看模型
│   ├── gazebo.launch.py              # D3：经典 diff_drive 插件跑起来
│   ├── gazebo_control.launch.py      # D4+D5：ros2_control 栈 + 传感器
│   ├── line_follow.launch.py         # D7：巡线赛道世界 + 相机下倾 + rqt_image_view
│   └── view_sensors.launch.py        # D5：独立 RViz 预设，与 gazebo 并行使用
├── rviz/
│   ├── mybot.rviz                    # 模型/TF/关节三件套
│   └── mybot_sensors.rviz            # 追加 LaserScan + Image 显示
├── scripts/
│   ├── make_line_world.py            # D7：参数化赛道 .world 生成器
│   ├── snap_camera.py                # D7：抓一帧相机图 + 黑线量化统计（D9 最小原型）
│   └── drive_check.py                # D7：按仿真时间开车并打轨迹表
├── worlds/
│   ├── mybot_world.world             # 空世界 + 3 障碍 + 1 前墙（D5 起）
│   └── line_following.world          # D7：9.42 m 巡线环线（生成物，勿手改）
├── CMakeLists.txt                    # ament_cmake，install 上述目录 + scripts 可执行
└── package.xml
```

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo 11（`gazebo`） + `gazebo_ros_pkgs`（`ros-humble-gazebo-ros-pkgs`）
- `ros2_control` + `ros2_controllers`（含 `diff_drive_controller`）
- `gazebo_ros2_control`（`ros-humble-gazebo-ros2-control`，本机 0.4.10）
- `xacro`、`robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`、`controller_manager`

## 构建

```bash
# 工作区约定：~/mybot_ws/src/mybot_description
source /opt/ros/humble/setup.bash
cd ~/mybot_ws
colcon build --packages-select mybot_description
source install/setup.bash
```

## 使用

四条 launch 覆盖学习路径的每一步。**任何一条**运行前都要求 shell 已 source 过本包的 `install/setup.bash`。

| 目的 | 命令 | 你会看到什么 |
|---|---|---|
| **D1/D2** 只看模型 | `ros2 launch mybot_description display.launch.py` | RViz 中三连杆 + 轮子；关节滑条面板 |
| **D3** 经典插件驱动 | `ros2 launch mybot_description gazebo.launch.py` | Gazebo 世界；`ros2 topic pub /cmd_vel ...` 让车动起来 |
| **D4/D5** ros2_control 栈 | `ros2 launch mybot_description gazebo_control.launch.py` | Gazebo 里同一台车 + 激光 / 相机；控制器 `joint_state_broadcaster` + `diff_drive_controller` 双 active |
| **D5** 传感器可视化 | `ros2 launch mybot_description view_sensors.launch.py` | 独立 RViz 显示 `/scan` + `/camera/image_raw` |
| **D7** 巡线赛道 | `ros2 launch mybot_description line_follow.launch.py` | 9.42 m 黑线环线 + 相机下倾画面（rqt_image_view 自动弹出） |

D7 常用开关：`camera_pitch:=0.6`（相机俯仰，正角向下）、`gui:=false`（不起 gzclient）、`image_view:=false`（不起 rqt）、`extra_params_file:=<path>`（换控制器参数文件）。

**WSL2 上让 GUI 稳定显示的启动顺序**（否则窗口会落进 `[WARN:COPY MODE]`、X 里有画面但 Windows 不重绘）：

```bash
# 1) 先把服务端起稳（不开任何 GUI）
ros2 launch mybot_description line_follow.launch.py gui:=false image_view:=false &
# 2) 等日志出现两条 "Configured and activated" 后，再分别单独拉窗口
gzclient &
ros2 run rqt_image_view rqt_image_view /camera/image_raw &
```

推荐验证顺序（D4 + D5 联跑）：终端 A 起 `gazebo_control.launch.py`，终端 B 起 `view_sensors.launch.py`。

## 关键接口

| 话题 / 服务 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令。ros2_control 下由 `diff_drive_controller` 消费（`use_stamped_vel:=false`，控制器内部话题 `cmd_vel_unstamped` 已 remap） |
| `/odom` | `nav_msgs/Odometry` | 里程计，`odom → base_link` TF 亦由控制器发布 |
| `/scan` | `sensor_msgs/LaserScan` | 360 线、5 Hz、`laser_link` 坐标系 |
| `/camera/image_raw` | `sensor_msgs/Image` | 320×240、RGB8、`camera_link_optical`（REP-103） |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 对应焦距 `fy ≈ 120.50` |
| `/gazebo/set_entity_state` | `gazebo_msgs/srv/SetEntityState` | 复位小车（**Humble 新 API**，取代 ROS1 的 `set_model_state`） |

## 复位与调试

```bash
# 把 mybot 瞬移回 (1.0, 0, 0.10)，朝向 0。
# 注意请求体是**嵌套的 state 字段**（Humble 的 gazebo_msgs/srv/SetEntityState 只有一个
# `EntityState state`，不是把 name/pose 平铺；平铺会报 "no attribute 'header'" 之类）。
# odom 不会跟着归零——diff_drive_controller 按关节积分自己的位姿估计。
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: mybot,
            pose: {position: {x: 1.0, y: 0.0, z: 0.10},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}},
            reference_frame: world}}"
```

## 三种驱动方案的教学对比

| 维度 | D3 经典 `libgazebo_ros_diff_drive.so` | D4 `ros2_control` + `diff_drive_controller` |
|---|---|---|
| 速度指令接口 | 插件内直连关节 | 通过 `GazeboSystem` hardware interface |
| 指令超时 | **无**——发布器消失后保持末速（真机级安全缺陷） | `cmd_vel_timeout: 0.5`——0.5 s 内无新命令自动清零 |
| 与真机切换 | 需要整套替换 | 保持 `controller_manager` API，只需换 `system_interface` 到真实硬件 |
| 学习价值 | 直观、上手快 | 现代 ROS 2 机器人栈的通用范式 |

D3 版本保留在 `mybot.urdf.xacro` 与 `gazebo.launch.py` 里，随时可 `ros2 launch mybot_description gazebo.launch.py` 复现"末速保持"缺陷；D4 版本通过 `mybot.ros2c.urdf.xacro` 用同一份核心模型（`mybot_core.urdf.xacro`）演示正确做法。

## P2 / D7：巡线赛道世界

`worlds/line_following.world` 由 `scripts/make_line_world.py` 生成（**改赛道改参数重跑生成器，不要手改 XML**）：

| 参数 | 值 | 说明 |
|---|---|---|
| 环线周长 | 9.42 m | 外轮廓 3.30 × 1.80 m |
| 直道半长 / 赛道半宽 / 圆角半径 | 1.2 / 0.9 / 0.45 m | 四个 90° 圆角 |
| 黑线宽 / 厚 | 50 mm / 3 mm | `Gazebo/FlatBlack`(0.1) vs 地面 `Gazebo/Grey`(0.7)，**7:1 亮度比**留给 D8 阈值 |
| 圆弧切分 | 15° | 24 段弦 + 4 段直 = 28 个盒子，接缝靠"两端各延长半线宽"重叠 |
| 黑线 collision | **无** | 只画不碰，3 mm 台阶不会污染驱动轮接触与 `/odom` |
| 起点 | `(0, -0.9, yaw 0)` | 底部长直道中点，车头朝 +x；旁边 12.5 cm 处有黄色起跑块作视觉参考 |
| `floor_stain`（D8 加） | 黑线左 18 cm，暗红 0.45/0.05/0.05 | **故意放的干扰色块**：灰度亮度≈44 会被误判成线，HSV 的 V≈110 能排除。用来回答"D8 为什么选 HSV 不选灰度"。生成器 `--no-stain` 可关 |

相机几何（`mybot_core.urdf.xacro` 的 `camera_joint`）：挂点从车壳内 `(0.18, 0, 0.045)` 移到车壳外前方 `(0.21, 0, 0.055)`，离地 **0.154 m**；`camera_pitch` 抽成 xacro arg，巡线取 **+0.6 rad**。

推导：`hfov=1.2 rad`、320×240 → `vfov/2 = atan(tan(0.6)·240/320) = 0.474 rad`。要让**整幅画面**都落在地面上，俯角必须大于 0.474 rad；取 0.6 rad 时可见地面为车前 **0.084 ~ 1.22 m**，50 mm 线宽在 0.3 m 处约占 44 px、1.2 m 处约占 11 px。

一键复现验收：

```bash
ros2 launch mybot_description line_follow.launch.py gui:=false image_view:=false   # 起服务
ros2 run mybot_description snap_camera.py --out /tmp/d7.png                        # 抓帧 + 黑线统计
ros2 run mybot_description drive_check.py --speed 0.2 --duration 8 --rate 50       # 开车 + 轨迹表
```

## 已验证行为

- **D4 超时停车**：2.5 s 的 `x=0.30 m/s` 指令结束后，`/odom.pose.pose.position.x` 在 kill+0.5 s 起冻结、`/odom.twist.twist.linear.x` 稳定 ≈0，直到 kill+6 s 无继续滑移。（2026-09-19 复测）
- **D5 激光**：`/scan` 360 线、有效距离 0.12–3.18 m，最小值命中 `box_ahead`；`laser_link` TF 正确。
- **D5 相机**：`/camera/image_raw` 320×240 RGB8；`camera_link_optical` 遵循 REP-103（`rpy = -π/2, 0, -π/2`）。
- **模型解析**：`xacro` + `check_urdf` 通过；`base_link` 5 子 link（4 wheel + caster）+ 2 传感器挂载点 + `camera_link_optical`。
- **D7 相机看得见赛道**：起点 `pitch=+0.6` 抓帧，画面 6 个采样行**全部命中黑线**，线宽随透视收敛 76→30 px，中心列恒为 159（画面中线 160），横向偏差 **−0.01**；黑像素占比 13.5%。
- **D7 赛道能跑**：`cmd_vel = 0.20 m/s` 连续 8 仿真秒，每 1 s 位移 0.199~0.200 m、`v_x` 全程无掉零（放宽 `cmd_vel_timeout` 后）。
- **D7 弯道信号**：车置于右下圆弧 45° 处沿切线，线在 0.7 m 内拐出画面左边界、偏差饱和到 −0.89 —— 这就是 D10 PD 要消掉的量。
- **D7 复位接口**：`/gazebo/set_entity_state` 返回 `success=True`（请求体必须是嵌套 `state`，见上）。

## 已知限制

1. **WSL2 上 Gazebo RTF 通常 ~0.16**（2.5 s 实际位移 0.12 m，理论 0.75 m）。这是 GPU/驱动限制，不影响控制逻辑；D4 停车实验只看 `pose` 是否冻结即可。
2. **`libgazebo_ros_api_plugin.so` 在 Humble 已不存在**。世界文件里的 `/gazebo/set_model_state` 走的是**新 API `/gazebo/set_entity_state`**（消息类型 `SetEntityState`，字段 `name + type` 取代 `model_name`）。旧资料如按 ROS1 名字调用会拿到 `!rclpy.ok()` 或 `Fault` 类错误。
3. **`pkill` 自杀陷阱**：清理 ROS 进程时命令本身不能包含与目标进程命令行重叠的裸字面量。所有 `pgrep/pkill` 一律用括号正则，如 `pkill -9 "[g]zserver"`。参考：本包所有测试脚本都遵守此约定。
4. **ROS_DOMAIN_ID / SHM 铁律**：`~/maze_ws` 项目沿用 `ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_SHM_PROVIDER=0`；本包在任意域下都能跑，但复用同一台机器上的 maze_ws 时请保持域一致。
5. **慢仿真的隐藏杀手：`cmd_vel_timeout` 按仿真时间计时**（D7 实测，D9 收尾量化）。**RTF 分档实测**：只跑 gzserver ≈ **1**（相机 15 Hz 就出 14.95 Hz、`/odom` 50 Hz）；**加上 gzclient + rqt_image_view 后掉到 ≈ 0.15**。0.5 s 仿真超时在 0.15 RTF 下 = 墙上 3.3 s，DDS 成簇投递一叠加就把命令判成过期 → 车每隔几秒自顿一次。把发布频率从 10 Hz 提到 200 Hz **不能**解决（瓶颈不在发布端）。对策：`config/mybot_controllers_slowsim.yaml` 覆盖 `cmd_vel_timeout: 2.0`，由 `line_follow.launch.py` 作为第二个 `<parameters>` 传入。**为什么不是 5.0**：5.0 时松手后车还会跑约 1 m，遥控场景不安全；2.0 实测最多再滑 0.4 m 且无顿挫。**真机（P3 之后）必须回 0.5 s，那是安全超时。**
6. **`gazebo_ros2_control` 的多参数文件写法**：要**多个 `<parameters>` 标签**（后者覆盖前者）。写成空格分隔的单个标签会被当成**一个**路径，报 `Error opening YAML file`，且 gzserver 仍会起来、只有控制器加载失败，容易误判。
7. **URDF 绕 y 轴正角 = 低头**（x 前 / y 左 / z 上的右手系里 `+y` 旋转把 `+x` 推向 `−z`）。相机俯角写成 `-0.6` 实际是抬头看天，而 Gazebo 天空背景色是 `0.7,0.7,0.7` = **178**，于是 `/camera/image_raw` 出一张 **std 严格为 0** 的纯色图 —— 极易误判成"WSL2 传感器渲染坏了"。快速判别法：`camera_pitch:=0.0` 跑一次，能看见地面就说明世界与传感器都好，问题在姿态符号。
8. **WSLg COPY MODE**：Qt/GL 类窗口（gzclient、rqt、RViz）可能落进 `[WARN:COPY MODE] <标题>`——X 服务器里画面完全正常（`DISPLAY=:0 import -window <Xid> out.png` 可直接抓出来），但 Windows 侧不重绘。处置见上文"启动顺序"；必要时 `wsl --shutdown` 重来。纯 X11 小程序（xeyes）不受影响，所以"xeyes 能显示"不代表 GUI 已修好。
9. **两个 spawner 并发加载偶发 `Failed to configure controller`**（慢仿真下的竞态）。`ros2 control list_controllers` 显示两个控制器其实都是 `active`，属假报错，不必重启整套。

## 里程碑

| Day | 交付 |
|---|---|
| D1 | 三连杆 URDF + RViz 显示 |
| D2 | xacro 参数化、caster 万向轮、惯性矩阵齐、`check_urdf` 与 TF 实测一致、无手填魔数 |
| D3 | 接 Gazebo 11 + 经典 diff_drive 插件；0.37 m / 2 s 位移与理论吻合 |
| D4 | 拆分为核心 + 双外壳；ros2_control + diff_drive_controller；`cmd_vel_timeout` 生效 |
| D5 | 激光 360 线 + 摄像头 320×240；world 加障碍；RViz 预设与 `view_sensors.launch.py` |
| D6 | README + `git init` + tag `v0.1`；D3 遗留 `/set_model_state` 排查 → **Humble API 改名为 `/set_entity_state`** |
| D7 | 巡线赛道世界（参数化生成器 + 28 段黑线）；相机挂点/俯角修正并参数化；`snap_camera.py` 出图量化验收；`drive_check.py` 无停顿跑通；慢仿真 `cmd_vel_timeout` 对策 |
| D8 | （视觉部分在姊妹包 [`mybot_control`](../mybot_control/README.md)）本仓库只配合改动：赛道加 `floor_stain` 干扰色块，用于证明灰度阈值会误检 |

## License

Apache-2.0（见 `LICENSE`）。
