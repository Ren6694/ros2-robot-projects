# mybot_description

> ROS 2 Humble · Gazebo 11 · 从零搭一台仿真差速小车。
> 「ROS2 嵌入式与建模 30 天培养计划」**P1 阶段的交付物（D1–D6）**。

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
│   └── mybot_controllers.yaml        # controller_manager + diff_drive_controller 参数
├── launch/
│   ├── display.launch.py             # D1：RViz + joint_state_publisher_gui 看模型
│   ├── gazebo.launch.py              # D3：经典 diff_drive 插件跑起来
│   ├── gazebo_control.launch.py      # D4+D5：ros2_control 栈 + 传感器
│   └── view_sensors.launch.py        # D5：独立 RViz 预设，与 gazebo 并行使用
├── rviz/
│   ├── mybot.rviz                    # 模型/TF/关节三件套
│   └── mybot_sensors.rviz            # 追加 LaserScan + Image 显示
├── worlds/
│   └── mybot_world.world             # 空世界 + 3 障碍 + 1 前墙（D5 起）
├── CMakeLists.txt                    # ament_cmake，只 install 上述目录
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
# 把 mybot 瞬移回 (1.0, 0, 0.035)，朝向 0：
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{name: 'mybot', type: 'model', \
    pose: {position: {x: 1.0, y: 0.0, z: 0.035}, orientation: {w: 1.0}}, \
    reference_frame: 'world'}"
```

## 三种驱动方案的教学对比

| 维度 | D3 经典 `libgazebo_ros_diff_drive.so` | D4 `ros2_control` + `diff_drive_controller` |
|---|---|---|
| 速度指令接口 | 插件内直连关节 | 通过 `GazeboSystem` hardware interface |
| 指令超时 | **无**——发布器消失后保持末速（真机级安全缺陷） | `cmd_vel_timeout: 0.5`——0.5 s 内无新命令自动清零 |
| 与真机切换 | 需要整套替换 | 保持 `controller_manager` API，只需换 `system_interface` 到真实硬件 |
| 学习价值 | 直观、上手快 | 现代 ROS 2 机器人栈的通用范式 |

D3 版本保留在 `mybot.urdf.xacro` 与 `gazebo.launch.py` 里，随时可 `ros2 launch mybot_description gazebo.launch.py` 复现"末速保持"缺陷；D4 版本通过 `mybot.ros2c.urdf.xacro` 用同一份核心模型（`mybot_core.urdf.xacro`）演示正确做法。

## 已验证行为

- **D4 超时停车**：2.5 s 的 `x=0.30 m/s` 指令结束后，`/odom.pose.pose.position.x` 在 kill+0.5 s 起冻结、`/odom.twist.twist.linear.x` 稳定 ≈0，直到 kill+6 s 无继续滑移。（2026-09-19 复测）
- **D5 激光**：`/scan` 360 线、有效距离 0.12–3.18 m，最小值命中 `box_ahead`；`laser_link` TF 正确。
- **D5 相机**：`/camera/image_raw` 320×240 RGB8；`camera_link_optical` 遵循 REP-103（`rpy = -π/2, 0, -π/2`）。
- **模型解析**：`xacro` + `check_urdf` 通过；`base_link` 5 子 link（4 wheel + caster）+ 2 传感器挂载点 + `camera_link_optical`。

## 已知限制

1. **WSL2 上 Gazebo RTF 通常 ~0.16**（2.5 s 实际位移 0.12 m，理论 0.75 m）。这是 GPU/驱动限制，不影响控制逻辑；D4 停车实验只看 `pose` 是否冻结即可。
2. **`libgazebo_ros_api_plugin.so` 在 Humble 已不存在**。世界文件里的 `/gazebo/set_model_state` 走的是**新 API `/gazebo/set_entity_state`**（消息类型 `SetEntityState`，字段 `name + type` 取代 `model_name`）。旧资料如按 ROS1 名字调用会拿到 `!rclpy.ok()` 或 `Fault` 类错误。
3. **`pkill` 自杀陷阱**：清理 ROS 进程时命令本身不能包含与目标进程命令行重叠的裸字面量。所有 `pgrep/pkill` 一律用括号正则，如 `pkill -9 "[g]zserver"`。参考：本包所有测试脚本都遵守此约定。
4. **ROS_DOMAIN_ID / SHM 铁律**：`~/maze_ws` 项目沿用 `ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_FASTRTPS_SHM_PROVIDER=0`；本包在任意域下都能跑，但复用同一台机器上的 maze_ws 时请保持域一致。

## 里程碑

| Day | 交付 |
|---|---|
| D1 | 三连杆 URDF + RViz 显示 |
| D2 | xacro 参数化、caster 万向轮、惯性矩阵齐、`check_urdf` 与 TF 实测一致、无手填魔数 |
| D3 | 接 Gazebo 11 + 经典 diff_drive 插件；0.37 m / 2 s 位移与理论吻合 |
| D4 | 拆分为核心 + 双外壳；ros2_control + diff_drive_controller；`cmd_vel_timeout` 生效 |
| D5 | 激光 360 线 + 摄像头 320×240；world 加障碍；RViz 预设与 `view_sensors.launch.py` |
| D6 | README + `git init` + tag `v0.1`；D3 遗留 `/set_model_state` 排查 → **Humble API 改名为 `/set_entity_state`** |

## License

Apache-2.0（见 `LICENSE`）。
