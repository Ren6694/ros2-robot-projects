"""D7：巡线赛道世界启动 —— line_following.world + 相机下倾 + ros2_control 驱动栈。

用法：
  ros2 launch mybot_description line_follow.launch.py                 # 完整（gzclient + rqt_image_view）
  ros2 launch mybot_description line_follow.launch.py gui:=false image_view:=false   # headless，只留数据链路
  ros2 launch mybot_description line_follow.launch.py camera_pitch:=-0.45             # 改相机俯角做对比

与 D5 的差别只有一处：camera_pitch 从 0 改成 -0.6 rad，让整幅画面落在车前 0.08~1.2 m 的地面上
（推导见 mybot_core.urdf.xacro 注释）。赛道由 scripts/make_line_world.py 生成。

跑起来后另开终端验证 image topic 调试链路：
  ros2 topic hz /camera/image_raw                     # 期望 ~15 Hz（WSL2 上会低）
  ros2 run mybot_description snap_camera.py --out /tmp/d7.png    # 抓一帧存盘 + 打印黑线统计
  ros2 run teleop_twist_keyboard teleop_twist_keyboard # 键盘绕场一圈
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('mybot_description')
    world = os.path.join(pkg, 'worlds', 'line_following.world')
    xacro = os.path.join(pkg, 'urdf', 'mybot.ros2c.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'mybot_controllers.yaml')
    slowsim = os.path.join(pkg, 'config', 'mybot_controllers_slowsim.yaml')

    # 起点：底部长直道中点 (0, -0.9)，车头朝 +x；z=0.10 让轮子刚好落地（底盘中心离地 0.099）
    # camera_pitch 符号（D7 踩坑）：URDF 是 x 前 / y 左 / z 上，绕 y **正角 = 低头**
    #   （+x 转向 −z）。写成 -0.6 会让相机抬头看天，Gazebo 天空背景 = 0.7,0.7,0.7 = 178，
    #   于是 /camera/image_raw 出一张 std=0 的纯色图，看起来像"传感器坏了"。
    args = [
        DeclareLaunchArgument('camera_pitch', default_value='0.6',
                              description='相机俯仰角(rad)，正值向下（URDF 绕 y 右手法则）'),
        DeclareLaunchArgument('camera_rate', default_value='15.0',
                              description='相机 update_rate(Hz，按仿真时间)。墙上频率=本值×RTF'),
        DeclareLaunchArgument('extra_params_file', default_value=slowsim,
                              description='第二个控制器参数文件；填 mybot_controllers.yaml 可恢复 0.5s 超时'),
        DeclareLaunchArgument('gui', default_value='true', description='是否启动 gzclient'),
        DeclareLaunchArgument('image_view', default_value='true',
                              description='是否启动 rqt_image_view'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-0.9'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose-sdf', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen',
                              condition=IfCondition(LaunchConfiguration('gui')))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro,
                     ' ros2c_params_file:=', controllers,
                     ' extra_params_file:=', LaunchConfiguration('extra_params_file'),
                     ' camera_pitch:=', LaunchConfiguration('camera_pitch'),
                     ' camera_rate:=', LaunchConfiguration('camera_rate')]),
            value_type=str)}],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mybot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.10', '-Y', LaunchConfiguration('yaw'),
                   '-timeout', '120'],
        output='screen',
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    image_view = Node(
        package='rqt_image_view', executable='rqt_image_view',
        arguments=['/camera/image_raw'], output='screen',
        condition=IfCondition(LaunchConfiguration('image_view')),
    )

    return LaunchDescription(args + [
        gzserver, gzclient, robot_state_publisher, spawn,
        joint_state_broadcaster_spawner, diff_drive_spawner, image_view,
    ])
