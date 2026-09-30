# =============================================================================
#  setup.py —— arm_demo (ament_python)
#
#  和 ament_cmake 的区别：Python 包没有"编译"这一步，install() 就是把
#  源码 + 资源文件拷进 install/<ws>/lib/python3.12/site-packages 和
#  install/<ws>/share/arm_demo/。所以：
#    * 改了 .py 也必须重新 colcon build（除非用 --symlink-install）
#    * data_files 里的相对路径就是 share/arm_demo/ 下的目录结构
#    * console_scripts 决定 `ros2 run arm_demo waypoint_mover` 这个命令名
# =============================================================================
from setuptools import find_packages, setup

package_name = "arm_demo"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 点位表与 launch 一起进 share/，这样别处能用 (get_package_share_directory("arm_demo"), "config", ...)
        ("share/" + package_name + "/config", ["config/waypoints.yaml"]),
        ("share/" + package_name + "/launch", ["launch/waypoint_mover.launch.py"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=False,
    author="Ren",
    author_email="ren@example.com",
    maintainer="Ren",
    maintainer_email="ren@example.com",
    description="MoveIt 点位规划/执行演示节点与可复算运动日志",
    license="MIT",
    entry_points={
        "console_scripts": [
            "waypoint_mover = arm_demo.waypoint_mover:main",
        ],
    },
)
