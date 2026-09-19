cmake_minimum_required(VERSION 3.8)
project(mybot_description)

find_package(ament_cmake REQUIRED)

# D1 仅需分发模型与启动文件；D2 引入 xacro 后目录不变
# D7 起 scripts/ 里放工具脚本：make_line_world.py（赛道生成器）、snap_camera.py（抓帧统计）
install(
  DIRECTORY urdf launch rviz worlds config scripts
  DESTINATION share/${PROJECT_NAME}
)

# 装到 lib/${PROJECT_NAME} 才能 `ros2 run mybot_description <脚本>`；install(PROGRAMS) 会带上可执行位
install(PROGRAMS
  scripts/make_line_world.py
  scripts/snap_camera.py
  scripts/drive_check.py
  scripts/rtf_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
