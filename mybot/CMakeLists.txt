cmake_minimum_required(VERSION 3.8)
project(mybot_description)

find_package(ament_cmake REQUIRED)

# D1 仅需分发模型与启动文件；D2 引入 xacro 后目录不变
install(
  DIRECTORY urdf launch rviz worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
