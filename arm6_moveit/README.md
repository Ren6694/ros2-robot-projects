# arm6 —— 6 轴机械臂 + 二指夹爪的 MoveIt2 闭环

> 手写 URDF/xacro → SRDF 碰撞矩阵算到收敛 → `ros2_control` + JTC → 自研点位节点直连 `MoveGroup`
> → 逐点位运动日志 → 曲线图与 RViz 三张截图证据。**13 个脚本各自带机器可读结论行，退出码即验收结论。**

一圈"抓取-放置"节拍跑完：**9/9 点位规划成功、9/9 执行成功、到位误差 `max_arr_err = 1.0e-4 rad`**
（只有容差的 3%），11 个关节的轨迹总长 11.43 rad。

## 它证明什么

不是"RViz 里能动一下"，而是**整条规划链路可回归**：同一套栈、同一份点位表，
跨次运行的关节路径长度差 <0.01 rad、规划耗时中位差 <20 ms；换到 VirtualBox 虚拟机里
（`VSCALE 0.5 → 1.0`）13/13 全过，并在那里量出"跟踪误差由控制环停拍时长主导、
不由关节速度决定"这条与直觉相反的结论（降速 42% 时间换不来精度余量，已撤回过一次错误机理）。

## 规格

| 项 | 值 |
|---|---|
| 可控关节 | 8（`joint1..6` revolute + `finger1/2_joint` prismatic） |
| link / joint | 12 / 11（含 dummy 根 `world` 与纯坐标系 `tool0`） |
| 零位 `tool0` 高度 | ≈ 0.92 m；理论最大前伸 **0.685 m**（由正运动学现算，非手抄） |
| 全臂质量 | 4.646 kg（底座配重占 39%，前四段占 81%） |
| 关节限位 | ±160° / ±100° / ±120° / ±170° / ±120° / ±170°，腕部 `v_max` 4.19 rad/s |
| 视觉零件 | 73 个 `<visual>` 零件 / 6 种材料（碰撞体 10 个，与外观改造前逐项一致） |

## 环境与怎么跑

ROS 2 **Jazzy** · MoveIt 2.12.4 · ros2_control 4.48.0 / JTC 4.42.1 ·
Ubuntu 24.04（WSL2 或 VirtualBox 均可，无 GUI 依赖）。

```bash
colcon build --base-paths src
source install/setup.bash

ros2 launch arm_moveit_config moveit.launch.py          # 栈 + RViz
ros2 launch arm_demo waypoint_mover.launch.py           # 跑点位表，打印 MOTION-SUMMARY
bash scripts/check_limits.sh                            # 不启 ROS 的静态自洽核对
python3 scripts/gen_viewer_parts.py                     # 让 arm6-viewer.html 跟上 URDF
```

零依赖的浏览器预览：直接双击 `arm6-viewer.html`（零件清单由 URDF 生成，正运动学与 RViz 同一套）。

## 结构

```
arm6_moveit/
├── src/arm_description/     # URDF/xacro：几何、惯量宏、ros2_control 接口契约
├── src/arm_moveit_config/   # SRDF / 关节限位 / 运动学 / 控制器 / launch
├── src/arm_demo/            # waypoint_mover 节点 + 点位表 + 运动日志(JSONL)
├── scripts/                 # 13 个带结果行的验收/诊断脚本 + 3 个判据自测
├── docs/                    # 8 篇：环境 → 构建 → URDF → MoveIt → 控制 → 可视化 → 排错 → 虚拟机对照
└── artifacts/               # 展开后的 URDF/SRDF 存档、运动曲线图、RViz 截图、改造前基线
```

## 值得看的几个决定

- **6 自由度而不是 7 或 4**：6 是"末端任意位姿无约束"的最小自由度；4 会退化出大量奇异，
  7 的冗余解歧义反而看不清规划器本身的行为。
- **夹爪用两个真 prismatic，不用 `<mimic>`**：mimic 在 `mock_components` + JTC 路径下支持不完整
  （要额外插件）。多花 2 维状态，换"整条链路零特例"。
- **`world` dummy 根**：KDL 不允许根 link 带惯量，而 `base_link` 必须有质量（MoveIt 估质心要用）。
  连带条件：SRDF 里就**不能**再写 `virtual_joint`，否则 `world→base_link` 有两个发布者。
- **规划末端用 `tool0` 而不是 `link6`**：`tool0` 定在两指指尖平面，`set_pose_target` 给的目标
  就是物体要到的位置，不需要业务代码再补一个 75 mm 偏置——这类偏置是现场 bug 高发区。
- **零位 = 全 0 = 直臂朝天**：唯一保证各 link 互不相碰的姿态，所以所有 `<limit>` 中点取 0；
  代价是它和真机的"手臂水平前伸"零位不同，这条在文档里显式标为已知差异。

## 工程方法（为什么这些数字可信）

- **判据必须能证伪**：`check_limits.sh` 头部写着注入式红样命令（把 `joint2` 改成 3.5 rad、
  把指关节的 `0.040 m` 写成 `40.0`），实测三条 ISSUE 全中；截图内容判据拿一次
  **真实缺陷归档**（RViz `Loop Animation` 造成的"地上多一条重播机器人"）当红样验过。
- **只换尺子不重跑**：判据能从归档产物离线重判。2026-09-30 把 RViz 取景从 3.2 m 拉近到 1.62 m
  时，旧判据（变化像素占**视口**的比例）判红，量出来是分母随取景涨了 3.9 倍——
  于是把分母换成"手臂自身的投影面积"，两批读数立刻落到同一量级，红样仍然判红。
- **外观与物理分离**：视觉零件从每 link 1 个几何体扩到 73 个，靠两条机器判据保证
  没碰物理——`BASELINE-RESULT`（collision 与改造前基线逐项一致）+
  `ENVELOPE-RESULT`（visual 相对 collision 的侧向外扩 ≤ 6 mm）。所以规划与计时数字全部继续有效。
- **口径先于结论**：跟踪误差、到位误差、`s/rad` 三类量各有明确定义与出处，
  文档里不允许出现"实测表明"而后继没有任何日志/图片/行号支撑的句子。

## 已知限制

- 硬件是 `mock_components/GenericSystem`（命令位置直接回写成状态位置），
  **证明的是规划链路正确且可回归，不证明动力学保真**；接真机只需换 `<plugin>`，上层 MoveIt + JTC 不动。
- 运动学求解用 KDL；未接笛卡尔路径约束、感知与障碍物碰撞体（规划场景是空的）。
- 未接 Gazebo：没有重力/力矩层面的验证，腕部 `effort` 限值只参与时间参数化（TOTG）。

## License

Apache-2.0，见仓库根 [LICENSE](../LICENSE)。
