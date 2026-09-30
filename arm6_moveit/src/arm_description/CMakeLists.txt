# =============================================================================
#  CMakeLists.txt —— arm_description
#
#  这个包没有一行 C++/Python 代码，装的全是"资源文件"（xacro / launch / rviz）。
#  ROS 2 的 ament_cmake 提供了 install(DIRECTORY ...)，会把目录原样
#  拷进 <install_share>/arm_description/，之后别处就能用
#      $(find arm_description)/urdf/arm6.urdf.xacro
#  这种包相对路径引用 —— 这是 ROS 包里共享文件的唯一标准方式。
#
#  注意：本包**不**在构建阶段调用 xacro。URDF 的预处理快照由 scripts/make_urdf.sh
#  显式生成（xacro arm6.urdf.xacro -o artifacts/arm6.urdf）。原因是 launch 文件里
#  本来就会调 xacro，构建期再生成一份只会带来"两份快照谁更新"的歧义；
#  把生成动作放进脚本，回归时还能单独检查 xacro 是否报错。
#
#  LANGUAGES NONE：只安装、不编译，省掉编译器探测。
# =============================================================================
cmake_minimum_required(VERSION 3.16)
project(arm_description VERSION 1.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
