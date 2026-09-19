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

## 里程碑

| Day | 交付 |
|---|---|
| D8 | cv_bridge 管线 + 灰度/高斯/HSV/开运算四组对照实验，选定 `hsv+blur+open` 基线；`/line/mask`、`/line/debug` 发布；四项质量指标 |
