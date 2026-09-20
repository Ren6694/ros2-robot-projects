cmake_minimum_required(VERSION 3.8)
project(maze_bot)

find_package(ament_cmake REQUIRED)

# 安装资源目录（launch / urdf / worlds / maps / config）
install(DIRECTORY launch urdf worlds maps config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)


# 激光时间戳修正 relay
# Gazebo ray sensor 的 /scan header.stamp 比实际采集时刻晚半个采样周期
# （12Hz → 41.7ms），会让 Cartographer 用错位姿投影激光点。
install(PROGRAMS scripts/scan_time_fix.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
