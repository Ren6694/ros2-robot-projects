cmake_minimum_required(VERSION 3.8)
project(mybot_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# D9：算法放进可导入的 Python 包，脚本只做 CLI/节点外壳。
# 这样单元测试（test_line_features.py）能直接 import mybot_control.line_features，
# 不用为了复用代码去 sys.path 里翻 scripts/，也不用把同一份算法抄两遍。
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/line_mask.py
  scripts/line_features_node.py
  scripts/test_line_features.py
  scripts/line_follow_pd.py
  scripts/test_pd_line.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
