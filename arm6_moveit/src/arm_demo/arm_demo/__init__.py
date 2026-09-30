"""arm_demo —— MoveIt 点位控制演示包。

包内分成两个模块，分界线是"能不能在没有 ROS 的终端里跑单测"：

* ``motion_log``     纯 Python：误差计算、峰值跟踪、JSONL/CSV 落盘、PASS 判据。
                     任何 ROS 消息都先在这里被转成普通 dict/list，所以这块可以
                     脱离仿真单独复算（阶段一的 plot_adc.py 用的是同一个思路）。
* ``waypoint_mover`` ROS 节点：ActionClient、等待就绪、按时序调用上面那套。
"""

__version__ = "1.0.0"
