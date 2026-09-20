-- ==============================================================
-- Cartographer 2D SLAM 配置 —— 迷宫仿真专用
-- 场景：8m x 6m 迷宫（48 格），走廊净宽 0.88m，墙高 0.8m
-- 传感器：2D 激光雷达（180 线，360°，8m 量程，8Hz）
-- 载体：差速小车，Gazebo diff_drive 提供 /odom
--
-- ⚠️⚠️ 开头两行 include 不能少！
--    它们负责创建 MAP_BUILDER / TRAJECTORY_BUILDER 两个全局表。
--    漏掉会启动即崩：
--      attempt to index global 'TRAJECTORY_BUILDER_2D' (a nil value)
--    结尾的 `return options` 同样不能少。
-- ==============================================================
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ============ 坐标系 ============
  -- 实际 TF 链: odom -> base_footprint -> base_link -> laser_frame
  --   tracking_frame  = 传感器所在坐标系
  --   published_frame = Cartographer 发布 map-> 的目标帧
  --     ⚠️ 必须用 "odom"！若用 "base_link"，TF 里会同时出现
  --        odom->base_footprint->base_link （外部提供）
  --        map ->base_link                  （Cartographer 发布）
  --     base_link 有两个父节点 → TF 树成环，rviz/nav2 全报错。
  --     这与 TurtleBot3 官方配置一致。
  map_frame = "map",
  tracking_frame = "laser_frame",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  -- ============ 传感器开关 ============
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- ============ 时序 ============
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- ============ 采样率（1.0 = 全用，精度优先）============
  -- ⚠️⚠️ options 表的键集合会被 Cartographer 严格校验：
  --     多一个键或漏一个键都会崩，报
  --       "Key 'xxx' was used the wrong number of times"
  --     因此下面的键必须与官方 backpack_2d.lua 完全一致（25 个）。
  --     新版 Cartographer 的 max_laser_scans_per_subdivision /
  --     min_point_clouds_per_subdivision 等键在 Humble 里【不存在】，
  --     抄过来会直接崩 —— 这是实测踩到的坑。
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ============ 轨迹构建器（前端）============
TRAJECTORY_BUILDER_2D.min_range = 0.12      -- 与 URDF 里雷达 min 一致
TRAJECTORY_BUILDER_2D.max_range = 8.0       -- 与 URDF 里雷达 max 一致
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
-- 本小车无 IMU，必须显式关闭（默认是 true，会等 IMU 数据）
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ⚠️ Ceres 扫描匹配的两个权重（本仿真最关键的一处调优）
--
-- 这两个权重决定「多大程度上相信初值（来自里程计外推） vs 扫描匹配结果」。
-- 权重越大 → 越信任里程计。
--
-- 背景：本仿真里 gazebo_ros_diff_drive 在 odometry_source=WORLD 模式下发布的是
--       【仿真真值位姿】—— 无漂移、无累积误差。所以先验比扫描匹配更可信。
--       而迷宫走廊**高度自相似**（全是 0.88m 宽的直线通道），
--       默认权重（10./40.）下扫描匹配容易沿走廊方向"滑动"到错误位置，
--       反而把本来就准确的位姿优化坏。
--
-- 实测对比（同一轨迹，统一评估口径）：
--       纯里程计直接投影（位姿=真值）   R@±3 = 98.4%   占用 2172 格
--       Cartographer 默认权重           R@±3 = 19.3%   占用  330 格
--   → 覆盖率差 5 倍，瓶颈明确在 scan matching。
--
-- 调到 1e4 后基本等于"让优化器以里程计为准、扫描匹配只做微调"。
-- （真实机器人上 odom 有漂移，不应这么设；本仿真 odom 是真值，故可如此。
--   要还原真实场景就把这两项改回官方默认 10. / 40.，并给 odom 加漂移。）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e4
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e4

-- ============ 栅格分辨率（本项目调优的关键）============
-- ⚠️ 0.05m（默认）在本场景下偏粗：
--    迷宫墙厚仅 0.12m，在 5cm 栅格上只占 2.4 格。
--    位姿只要有 ±2.5cm 抖动，同一面墙的命中就会**分散到相邻格子**，
--    导致没有任何一格累积到 0.65 的占用阈值 ——
--    实测墙几乎全落在"未知"区间（占用格仅占地图 1.2%）。
--    改 0.03m 后墙占 4 格，命中更集中，墙更易"立起来"。
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03

-- 命中 / 穿过 的概率权重（决定墙多快"立起来"）
--
--   数学关系：每次命中累加 log(p_hit/(1-p_hit))，达到 p>0.65 需 0.619。
--       hit=0.55 → +0.201 → 需 4 次净命中
--       hit=0.58 → +0.323 → 需 2 次净命中
--       hit=0.65 → +0.619 → 需 1~2 次净命中   ← 当前
--
--   诊断依据（2026-09-18 实测）：原先 hit=0.58 时，submap 里 44% 的格子
--   停在灰度 128（p=0.498）这个"中性未决"值上，墙的概率压根没超过 0.5，
--   输出地图里占用格只占 1%。这正是"墙看着有、但被判为未知"的根因。
--
--   miss=0.45 与 hit=0.65 对称，即"穿过"的抵消力度同等。
--   一个被命中 1 次 + 穿过 1 次的格子净值为 0 → 保持未知（正确，不产生假墙）。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

-- ============ Ceres 扫描匹配权重见上方「轨迹构建器」段（唯一定义处）============

-- ============ 运动滤波（小车差速，速度不快）============
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- ============ 回环检测（后端）============
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- ============ 优化器 ============
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10

-- ============ 全局定位 ============
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

-- ⚠️ 不要设置 TRAJECTORY_BUILDER.pure_localization_trimmer.*
--    该字段仅在 TRAJECTORY_BUILDER.pure_localization = true 时才存在，
--    否则为 nil，赋值会崩：
--      attempt to index field 'pure_localization_trimmer' (a nil value)
--    （官方 trajectory_builder.lua 里这段是注释掉的）
--    本项目是建图模式，不需要纯定位裁剪。

-- ⚠️ 结尾必须 return options，否则 Cartographer 读不到配置
return options
