# =============================================================================
#  CMakeLists.txt —— arm_moveit_config（纯配置包）
#
#  安装清单里有两样东西容易被漏掉，都是踩过坑之后显式写上的：
#   1) .setup_assistant —— MoveItConfigsBuilder 靠它反查"URDF 在哪个包的哪个路径"。
#      漏了它不会报错，只会打一行黄色 warning 然后去找 config/arm6.urdf，
#      接着在 launch 阶段以 "robot_description 为空" 的形式炸掉，非常难联想到
#      是这个隐藏文件没装。install(FILES) 对点开头文件是有效的，必须显式列。
#   2) launch/gdb_settings.gdb —— generate_move_group_launch() 无条件把这个路径
#      传给 --xargs 的 gdb commands_file。debug:=false 时不会用到，
#      但 debug:=true 而文件不存在会让 move_group 在 gdb 里直接起不来。
# =============================================================================
cmake_minimum_required(VERSION 3.16)
project(arm_moveit_config VERSION 1.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# 注意没有 rviz/ 目录：generate_moveit_rviz_launch() 把配置文件路径写死成
# <config_pkg>/config/moveit.rviz，所以 rviz 配置跟 SRDF/YAML 放一起。
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
