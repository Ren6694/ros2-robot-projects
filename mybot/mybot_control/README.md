# mybot_control

> ROS 2 Humble · P2 视觉与控制包。**描述包 `mybot_description` 只管"车长什么样"，这个包管"车怎么看、怎么动"**。
> D8 交付：cv_bridge 桥接 + 灰度/高斯/HSV 阈值实验。

## 为什么单独一个包

巡线要的是"感知 → 决策 → 控制"闭环，和 URDF 的迭代节奏完全不同：
调一次阈值不该重新构建一次模型。所以按 ROS 2 惯例切开——
`mybot_description`（模型/世界/launch）与 `mybot_control`（视觉节点/控制节点/参数），
两者只通过话题契约耦合：`/camera/image_raw`、`/scan`、`/cmd_vel`、`/odom`。

## `line_mask.py` 的管线

```
/camera/image_raw (BGR8)
   └─ cv_bridge → numpy
        ├─ method=gray : BGR2GRAY → [GaussianBlur] → threshold(<T, INV_BINARY)
        └─ method=hsv  : BGR2HSV → inRange(H 0-179, S s_lo..s_hi, V ≤ v_hi)
                          → [GaussianBlur + 二次二值化]
                          → [MORPH_OPEN 椭圆核]        ← 吃掉孤立噪点与色块边缘
        ├─ /line/mask  (mono8，线=255)
        ├─ /line/debug (bgr8，mask 半透明绿叠回原图)
        └─ 指标：覆盖率 / 连通域数 / 最大域占比 / 底部中心偏移
```

四个指标直接对应 D9/D10 的成败：

| 指标 | 含义 | 健康范围 |
|---|---|---|
| 覆盖率 % | 白像素占比 | 直道经验 8~20%；偏大=阈值太松，偏小=线丢了 |
| 连通域数 | 白区被切成几块 | 理想 1；>3 说明有噪点或反光切断 |
| 最大域占比 % | 最大域 / 全部白像素 | 应 >95%，否则"主线"不完整，e 信号会被假块带偏 |
| 底部中心偏移 | 画面最下 1/5 行白区中心，−1 左 ~ +1 右 | 这就是 D10 PD 的输入 `e` |

## D8 实验：为什么要 HSV，还要再加开运算

赛道里**故意放了一块暗红"地板污渍"**（`make_line_world.py` 的 `floor_stain`，在黑线左侧 18 cm、车压不到）。
它的灰度亮度约 44（低于阈值 80 → 会被当成线），但 HSV 的 `V = max(R,G,B)` 约 110（远高于黑线的 26 → 能被排除）。

同一帧、四种配置（2026-09-19，起点直道）：

| 配置 | 覆盖率% | 连通域 | 最大域占比% | 底部中心偏移 | 结论 |
|---|---|---|---|---|---|
| `gray` | 16.9 | 2 | 80.1 | −0.00 | 污渍整块误检，**20% 的白像素是假的** |
| `gray+gauss` | 16.7 | 2 | 80.0 | −0.00 | 高斯平滑对"颜色误检"完全无用（它只治高频噪声） |
| `hsv` | 13.8 | 2 | 98.1 | −0.00 | 污渍本体消失，但抗锯齿边缘残留一圈细线 → 仍是 2 个域 |
| `hsv+gauss+open` | **13.5** | **1** | **100.0** | −0.00 | 开运算清掉残留边缘，mask 干净可用 |

**选定基线：`method=hsv, blur=5, open=3`**，D9/D10 直接沿用。
判据不是"看着像"，而是**连通域=1 且最大域占比=100%**——这条在 D11 加障碍、D12 调参时可以复跑回归。

## 用法

```bash
# 前置：先起仿真（Windows 侧双击 启动ROS2仿真.bat，或 ros2 launch mybot_description line_follow.launch.py）

# 1) 对照实验（D8 验收，一条命令出表 + 五联图）
ros2 run mybot_control line_mask.py --mode compare --outdir /tmp/d8_masks

# 2) 持续发布 mask，供 RViz / rqt_image_view 订阅；顺带存 3 帧
ros2 run mybot_control line_mask.py --save 3 --outdir /tmp/d8
rqt_image_view /line/mask

# 3) 滑条实时调参（需要 GUI）：hsv/gray 切换、两个阈值、blur/open 核大小；s 存图 q 退出
ros2 run mybot_control line_mask.py --mode tune

# 4) 换参数直接试
ros2 run mybot_control line_mask.py --method hsv --v-hi 90 --blur 5 --open 5
```

工作台里也有一个 `跑D8阈值实验.bat`，双击即跑第 1 条并把图拷到
`scratch\d8\`。

## 实测频率（2026-09-19 复测，更正先前"6.8 Hz、需要换 C++"的错误结论）

先说结论：**Python 不是瓶颈，20 Hz 的 PD 环现在就能跑**。之前那个 6.8 Hz 是**测量污染**。

| 项 | 实测 | 说明 |
|---|---|---|
| `make_mask` + `measure` 单帧耗时 | **0.40 ms**（hsv+blur5+open3，320×240） | 单核理论上限 ≈2500 Hz，换 C++ 毫无收益 |
| `camera_rate:=15` | 14.95 Hz，抖动 2.7 ms | 单 server、`/robot_description` 已核对 |
| `camera_rate:=60` | 59.93 Hz，抖动 1.5 ms | 同上 |
| `camera_rate:=30` | **59.93 Hz**（URDF 里确实写着 30） | 这个档位**不线性**，30 会掉到渲染帧率 |
| 加 gzclient | 仍 59.96 Hz | 客户端渲染不吃传感器带宽 |
| `/line/mask` | 与相机同频（59.76 Hz） | 管线不引入额外降频 |

给 D10 的设定：**`camera_rate:=30` 起步（实测约 60 Hz），控制环 20 Hz、看门狗按仿真时间算**。
`update_rate` 这个旋钮在本机不单调，所以**别拿它当保证**，用 `ros2 topic hz` 实测。

**为什么先前会量出 5.3~8.7 Hz**：工作台脚本的判活函数用了 `pgrep -f "[g]zserv"`，
它会漏报 → `do_start` 以为没在跑、跳过 `do_stop` → 攒出 2~3 个 gzserver 同时发 `/camera/image_raw`，
频率既会叠加（15 被读成 60）也会因抢占而抖动（读到 5~8）。已改成 `pgrep -x gzserver` 精确匹配。
**教训：测频率之前先 `pgrep -xc gzserver` 数清楚有几个 server**，否则任何数字都没意义。

## 已知限制

1. **`--mode tune` 在这台机器上看不见窗口**（2026-09-19 实测）：进程不崩、X 服务器里确实建了 640×384 的
   OpenCV 窗口，但 WSLg 没给它注册 RAIL 窗口，Windows 侧完全看不到。所以 D8 的验收走
   `--mode compare` 出静态图这条路；要实时看效果就 `rqt_image_view /line/debug`，要调参就改
   `--v-hi / --gray / --blur / --open` 重跑 compare（一次 20 秒）。tune 代码保留，换到
   原生 Linux 或修好 WSLg 后可直接用。
2. 指标里的"连通域"用 8 邻域。线在画面顶部因透视变得极细（1~2 px）时可能被切成多段——
   这是 D9 要用"逐行中心 + 连续性检查"解决的问题，不是 mask 的错。

## D9：米制线特征 `/line/pose`

把画面变成**米和弧度**（不是像素），D10 的 PD 直接可用。

```
/camera/image_raw + /camera/camera_info
   → masking.make_mask(FEATURE)            # hsv + 高斯，不做开运算
   → line_features.row_centers             # 逐行最长连续段中心；歧义行/贴边行丢弃
   → CamGeom.pixel_to_ground               # 逆透视：像素 -> 地面米
   → fit_line_ransac (tol=8mm)             # 全局鲁棒直线拟合 + 内点精修
   → /line/pose (PoseStamped)  /line/centers(可视化)  /line/mask
```

定义与符号：`y = b·x + a`（x 前向、y 左侧，车体系）→ **θ = atan(b)**、
**e = a + b·x_ref**（`x_ref` 默认取最近可用行，**不外推到车轴**；要轴上值用 `e_at(0.0)`）。
`e>0` = 线在车左 → `cmd_vel.angular.z` 取正。

为什么 e 不外推到 x=0：画面最近只能看到 x≈0.29 m，外推等于把斜率误差乘 0.3~0.5 灌进 e，
实测 12° 夹角时单这一项就贡献 **68 mm 假偏差**。

### 验证（两条独立路径）

**解析真值单元测试** `ros2 run mybot_control test_line_features.py` —— 给定
`y = e0 + tan(θ0)·x` 用相机模型画成图，再反解比对，覆盖居中/横偏/夹角/干扰块/断线/
"线出画面必须拒绝输出"/投影闭环，**12/12 通过**，端到端用例 e 差 4 mm、θ 差 1.5°。

**实机交叉验证**（Gazebo 真实渲染，带污渍带夹角）：

| 车位姿（`set_entity_state`） | 真值 | 节点实测 | 误差 |
|---|---|---|---|
| (0,−0.90) yaw 0 正对 | e=0 | −0.35 mm / θ +0.2° | 亚毫米 |
| (0,−0.95) yaw 0 横移 5 cm | +5.00 cm | +5.1 cm @x=0.29，内点 170/170 | 1 mm |
| (0,−0.95) yaw −10° | +11.4 cm / +10.1° | +11.5 cm @x=0.36 / +10.1°，内点 104/104 | 1 mm / 0.0° |

细节与**两次错误归因的记录**（先怪开运算、再用顺序门控，都被单变量对照推翻，
最终解法是 RANSAC）见 `workspace\projects\机器人与仿真\20260919-ROS2学习笔记\D9-线中心提取与偏差.md`。

### 给 D10 的用法

```python
from mybot_control.line_features import CamGeom, extract
from mybot_control.masking import FEATURE, make_mask
geom = CamGeom.from_camera_info(info_msg, cam_x=0.21, cam_h=0.154, pitch=0.6)
lf = extract(make_mask(img, **FEATURE), geom)
if lf.ok:
    e, theta = lf.e, lf.theta
```
节点在**没有有效线特征时不发布**（不是发上一次的值），这样 D10 的看门狗能区分
"线在正中"和"线丢了"。

## D10：PD 巡线 `/cmd_vel`

```
/line/pose ──→ pd_line.compute(e, θ) ──→ /cmd_vel
             ω = Kp·e + Kd·θ（限幅 ±w_max）
             v = clamp(v_max·(1 − kv·|ω|/w_max), v_min, v_max)
```

**θ 就是"D"**：它是航向误差，等价于 e 的变化率，所以不对 e 求导（15 Hz 图像求导只会把噪声
放大成方向盘抖动）。纯 P 控制必然在圆角处滞后 → 冲出去 → 回摆 → 振荡。
速度随 |ω| 自动降（满舵降到 30%），`v_min` 兜底防止轮速进静摩擦区。

节点三条硬规矩：**① 控制环自己定时 20 Hz**（不绑图像回调，否则控制周期随负载抖）；
**② 丢线看门狗发零速停车**，不沿用上次方向（D9 无效时刻意不发布就是为配合这里）；
**③ 里程 >6 m 且回到起点 0.35 m 内自动判定一圈**。

### 速度扫描（headless，RTF≈1）

| v_max | 结果 | 里程 | 用时 | \|e\| 均值/最大 | 丢线帧 |
|---|---|---|---|---|---|
| 0.25 | 一圈 ✓ | 8.21 m | 39 s | 3.6 / 7.8 cm | 0/785 |
| 0.35 | 一圈 ✓ | 8.48 m | 32 s | **1.3 / 2.3 cm** | 0/630 |
| 0.50 | 圆角入口丢线→停车 | 1.50 m | — | — | 2873/2944 |

稳定上限在 0.35~0.50 之间，**默认取 0.30**。反直觉的一点：0.35 比 0.25 跟得更紧——
速度越高同样偏差对应的误差变化率越大，θ 项更早介入，车是"贴线"而不是"漂出去再拽回来"。

### 验收（`跑D10巡线.bat` 实跑）

```
v_max=0.30  墙上 35 s  里程 8.36 m  圈数 1
|e| 均值 1.3 cm / 最大 4.7 cm   丢线 4/697（圆角瞬时，停 0.2 s 后自恢复）  饱和 0
```
"跑完一圈"用轨迹独立证明（不只看判据）：`odom_x` 跨 −1.50~1.69、`odom_y` 跨 0~1.81
（赛道外轮廓 3.30×1.80），四个角区各有 53/48/32/27 帧经过，终点距起点 0.34 m，
`|e|>10cm` 帧占比 0.0%。轨迹 CSV：`scratch\d10_*.csv`。

### 单元测试 `ros2 run mybot_control test_pd_line.py`

符号 / 限幅 / NaN→lost / 线性之外，还有**参数自洽性**检查：
`过 0.45 m 圆角需要 ω=v_max/r=0.78 < w_max`、`该工况速度 0.185 > v_min`、
`丢线超时 1.0 s ≫ 控制周期 0.05 s`。这几条不写进测试，就会变成 D12 现场"怎么调都冲出去"。

### 遗留

0.5 m/s 那次是**安全地失败**（停住而非乱跑），但停车后不会自己找线——D11 的状态机
要补"丢线→倒车/原地找线"的恢复行为，正好把 `/scan` 避障一起接上。

## D11：避障状态机 —— 巡线与避障共存

```
/line/pose (PoseStamped) ┐
                         ├→ avoid.AvoidFSM.step() → /cmd_vel (Twist)
/scan (LaserScan)        ┘                        → /avoid/state (String)
```

**控制律一行没改**（还是 D10 的 `pd_line.compute`），这一层只在外面套了"什么时候允许动、
往哪儿动"的仲裁。状态逻辑全在 `mybot_control/avoid.py`（纯函数、不 import rclpy），
节点 `scripts/line_follow_avoid.py` 只做 ROS 胶水 —— 因为状态机是全项目最容易
"看起来对、实际会撞"的一层，必须能在毫秒级被喂进任意病态传感器序列并断言输出。

### 状态与优先级

```
front <= stop_dist(0.30)      -> OBSTACLE_STOP   v=0 w=0
stop < front <= warn(0.55)    -> DECEL           速度按比例压，转向原样保留
线丢了 且 前方干净             -> SEARCH          原地找线，1.5s 反向，6s 超时
线丢了 且 前方不干净           -> OBSTACLE_STOP   ★ 不许在障碍跟前转身找线
搜索超时                      -> SAFE_STOP       吸收态，等人接管
其余                          -> FOLLOW
```

★ **必须先判障碍、再判丢线**。D11 的真实场景就是"方块挡住黑线"：相机看不见线、激光看得见
方块。若先判丢线，车会进 SEARCH 原地旋转，把正对着它的障碍当成"线丢了"处理，转着转着就撞上。

### `/scan` 的两条实测构成（决定了参数，不是拍脑袋）

- **自我回波**：一帧 360 线里有 32 条落在 0.123~0.307 m，角度**全部 \|θ\| ≥ 98°**（车壳与轮子）。
  按"全向最近值"判障碍会让车永远认为贴着障碍、一步不走 → 前向扇区取 **±25°**。
- **盲区** `range_min=0.12`：比它更近的读数一律丢弃，否则一个假回波会让车永久停死。
- **避障的分辨率上限是 `/scan` 的 5 Hz，不是控制环的 20 Hz**：0.30 m/s 下每帧激光之间车前进
  6 cm，所有距离门限按这个数留余量（`test_avoid.py` 第 [5] 组把这条写成断言）。

### 迟滞（防阈值抖振）

车停在障碍前时激光噪声让前向距离在 0.278~0.300 m 之间摆（极差 23 mm），正好压在 0.300 门限上，
首轮 200 秒出现 3 次 `OBSTACLE_STOP ↔ DECEL` 来回跳。→ 脱出 OBSTACLE_STOP 要求
`front > stop_dist + release_margin(0.10)`。单测直接喂 ±11 mm 实测噪声 200 帧断言**零次翻转**，
并保留"障碍又逼近仍判停"的用例（迟滞不牺牲安全）。

### 失效安全：激光断流 = 停车，不是"前方干净"

"没数据"和"没障碍"在数据上长得一模一样。节点里 `/scan` 超过 `--scan-timeout`(1 s) 没更新
就返回哨兵距离 0.0 → 走 OBSTACLE_STOP。首轮数据里确实抓到一次相机+激光同时断 1.16 秒，
行为是"停 1.2 秒 → 传感器恢复 → 自己继续跑完"，而不是闭眼往前开。

### 用法

```bash
# 带障碍的世界（世界文件由生成器产出，见 mybot_description）
ros2 launch mybot_description line_follow.launch.py \
    world:=<share>/worlds/line_following_obstacle.world gui:=false image_view:=false
ros2 run mybot_control line_features_node.py
ros2 run mybot_control line_follow_avoid.py --csv /tmp/d11.csv
ros2 run mybot_control test_avoid.py            # 不需要开仿真

# 运行时移开障碍（验证"恢复"那半段，不用重启仿真）
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: obstacle_00}"
```

### D11 验收（一键 `跑D11避障.bat`）

```
/line/pose 14.986 Hz → 车驶向障碍 → 1.2s DECEL → 3.4s OBSTACLE_STOP（零速、最近 0.296 m）
→ 运行时删障碍 success=True → 6.2s 恢复 FOLLOW → 40s 跑完一圈
状态帧数 FOLLOW 661(83.8%) / OBSTACLE_STOP 84(10.6%) / DECEL 44(5.6%)
|e| 均值 1.2 cm / 最大 4.6 cm（与 D10 的 1.3 cm 同量级 → 避障没牺牲巡线）
<0.20 m 的帧 = 0；轨迹 x −1.52~+1.72、y 0~1.83、积分路径 8.35 m、终点距起点 0.34 m（闭合）
```

数字由 `scripts/analyze_avoid_log.py` **从 CSV 独立复算**，不采信节点自己的汇总
（第一版 summary 就因为多插一列 `state` 导致索引错位而崩过）。

⚠ 统计口径教训：全帧 `|e|` 算出来 0.2 cm 好得可疑——85% 的帧是"停在障碍前没动、e≈0"
把均值稀释了。只统计 FOLLOW 帧才是真数。**凡是"加了个状态机之后精度暴涨"，先怀疑口径。**

### 遗留

不会绕行（挡住就等，绕行走 P4/Nav2）；SEARCH 只在前方干净时允许，被完全堵死不会自己绕开
——有意的安全取舍；长会话偶发传感器断流未查根因，已被失效安全兜住。

## 里程碑

| Day | 交付 |
|---|---|
| D8 | cv_bridge 管线 + 灰度/高斯/HSV/开运算四组对照实验，选定 `hsv+blur+open` 基线；`/line/mask`、`/line/debug` 发布；四项质量指标 |
| D9 | `line_features.py` 逆透视出**米制 e 与 θ** + RANSAC 鲁棒拟合；解析真值单元测试 12/12；实机三组位姿交叉验证误差 ≤1 mm；`/line/pose`(PoseStamped) 与 `/line/centers` |
| D10 | `pd_line.py` 控制律 + `line_follow_pd.py` 节点（20 Hz 定时、丢线看门狗、自动判圈）；**0.30 m/s 35 秒跑完整圈**，\|e\| 均值 1.3 cm；速度扫描定出稳定上限 0.35~0.50；`test_pd_line.py` 含参数自洽性检查 |
| D11 | `avoid.py` 避障状态机（FOLLOW/DECEL/OBSTACLE_STOP/SEARCH/SAFE_STOP，含迟滞与激光断流失效安全）+ `line_follow_avoid.py` 节点 + `analyze_avoid_log.py` 独立复算；**障碍前停死 29.6 cm → 运行时删障 → 自己恢复并跑完一圈**，\|e\| 1.2 cm（与 D10 同级）；`test_avoid.py` 含 4 条★安全性质与实测噪声零翻转断言 |
