cmake_minimum_required(VERSION 3.8)
project(mybot_control)

find_package(ament_cmake REQUIRED)

# D8 只有 Python 节点；脚本走 lib/${PROJECT_NAME} 才能 ros2 run。
# launch/ 与 config/ 等 D9/D10 有了内容再补 install（空目录 git 也不跟踪）。
install(PROGRAMS
  scripts/line_mask.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
