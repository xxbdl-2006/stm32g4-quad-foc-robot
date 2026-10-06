#!/usr/bin/env python3
# ============================================================================
#  bringup.launch.py
#  整车启动：CAN 桥 + 底盘控制器 + 相机同步 + 可选遥操作/RViz。
#
#  典型用法
#  --------
#    # 第一次跑之前先把 can0 拉起来（需要 root 或 sudo）
#    ros2 launch hw_bringup bringup.launch.py
#
#    # 只跑硬件，不带 RViz 和遥操作（车上部署用）
#    ros2 launch hw_bringup bringup.launch.py use_rviz:=false use_teleop:=false
#
#    # 用 ros2_control 路径（此时 can_bridge 自动关闭指令下发）
#    ros2 launch hw_bringup bringup.launch.py use_ros2_control:=true
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = get_package_share_directory('hw_bringup')

    # ---------------------------------------------------------------- 参数
    declared = [
        DeclareLaunchArgument('can_interface', default_value='can0',
                              description='SocketCAN 接口名'),
        DeclareLaunchArgument('can_bitrate', default_value='1000000',
                              description='CAN 波特率（bps）'),
        DeclareLaunchArgument('auto_setup_can', default_value='true',
                              description='启动时自动 ip link set can0 up'),
        DeclareLaunchArgument('config',
                              default_value=os.path.join(pkg_bringup, 'config', 'hardware.yaml'),
                              description='参数文件'),
        DeclareLaunchArgument('use_ros2_control', default_value='false',
                              description='true=走 ros2_control 硬件接口路径'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('publish_robot_description', default_value='true'),
        DeclareLaunchArgument('use_task', default_value='false',
                              description='是否同时拉起取放任务层（hw_task）'),
        DeclareLaunchArgument('task_autostart', default_value='false',
                              description='任务层是否在启动后自动开始（试跑用）'),
    ]

    can_interface = LaunchConfiguration('can_interface')
    can_bitrate = LaunchConfiguration('can_bitrate')
    config = LaunchConfiguration('config')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # ---------------------------------------------------------------- CAN 接口拉起
    # 为什么放在 launch 里而不是让用户手动敲：
    # 车上断电重启后很多人会忘记这一步，然后在别处排查半小时"为什么没数据"。
    # 失败也不致命 —— 后面的节点会各自报出清晰的连接错误。
    setup_can = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            # 已经 up 就跳过，避免重复配置把总线上正在跑的设备打断
            f'if ip -details link show {can_interface} 2>/dev/null | grep -q "state UP"; then '
            f'  echo "[hw_bringup] {can_interface} 已经是 UP，跳过配置"; '
            f'else '
            f'  echo "[hw_bringup] 配置 {can_interface} @ {can_bitrate} bps"; '
            f'  sudo -n ip link set {can_interface} down 2>/dev/null || true; '
            f'  sudo -n ip link set {can_interface} type can bitrate {can_bitrate} 2>/dev/null || '
            f'    echo "[hw_bringup] 提示：需要 sudo 权限或已配置 udev 规则，请手动执行 setup_can.sh"; '
            f'  sudo -n ip link set {can_interface} up 2>/dev/null || true; '
            f'fi',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_setup_can')),
    )

    # ---------------------------------------------------------------- 机器描述
    robot_description = Command([
        'xacro ', PathJoinSubstitution([pkg_bringup, 'urdf', 'hwb_robot.urdf.xacro']),
        ' can_interface:=', can_interface,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        condition=IfCondition(LaunchConfiguration('publish_robot_description')),
    )

    # ---------------------------------------------------------------- CAN 桥
    can_bridge = Node(
        package='hw_can',
        executable='can_bridge_node',
        name='can_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            config,
            {
                'can_interface': can_interface,
                # 走 ros2_control 时让出指令下发权
                'motion_output_enabled': True,
            },
        ],
        condition=UnlessCondition(use_ros2_control),
    )

    # ros2_control 路径下，桥只做反馈聚合与诊断，不下发指令
    can_bridge_feedback_only = Node(
        package='hw_can',
        executable='can_bridge_node',
        name='can_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            config,
            {
                'can_interface': can_interface,
                'motion_output_enabled': False,
            },
        ],
        condition=IfCondition(use_ros2_control),
    )

    # ---------------------------------------------------------------- 底盘控制器
    motor_control = Node(
        package='motor_control',
        executable='motor_control_node',
        name='motor_control',
        output='screen',
        parameters=[config],
        condition=UnlessCondition(use_ros2_control),
    )

    # ---------------------------------------------------------------- 相机同步
    camera_trigger = Node(
        package='camera_trigger',
        executable='camera_trigger_node',
        name='camera_trigger',
        output='screen',
        parameters=[config, {'image_topic': '/image_raw'}],
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # ---------------------------------------------------------------- ros2_control
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'ros2_control.launch.py')),
        launch_arguments={
            'config': config,
            'can_interface': can_interface,
        }.items(),
        condition=IfCondition(use_ros2_control),
    )

    # ---------------------------------------------------------------- 取放任务层
    # 【为什么默认关闭】任务层会主动驱动底盘移动。默认关掉，让"跑起来看看"
    # 和"开始干活"是两个显式的动作 —— 现场调试时最怕的就是起个 launch
    # 车突然自己动了。
    task_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hw_task'), 'launch', 'task.launch.py')),
        launch_arguments={
            'image_topic': '/image_raw',
            'autostart': LaunchConfiguration('task_autostart'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_task')),
    )

    # ---------------------------------------------------------------- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'bringup.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ---------------------------------------------------------------- 遥操作
    # 用 teleop_twist_keyboard 打键盘控制；没装就跳过而不是让整个 launch 挂掉
    teleop = ExecuteProcess(
        cmd=['bash', '-lc',
             'if ros2 pkg prefix teleop_twist_keyboard >/dev/null 2>&1; then '
             '  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard '
             '       --ros-args -r cmd_vel:=/cmd_vel; '
             'else '
             '  echo "[hw_bringup] 未安装 teleop_twist_keyboard，跳过遥操作节点"; '
             'fi'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    # ---------------------------------------------------------------- 启动时序
    # CAN 桥必须在 3 秒内看到驱动板心跳，否则大声报警 —— 这是现场最常见的故障，
    # 让它在启动日志里就暴露出来，而不是等到第一次发指令发现车不动。
    health_check = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[hw_bringup] 等待驱动板心跳…（若下面出现 bus 超时告警，'
                        '请检查 120Ω 终端电阻与 CAN_H/CAN_L 是否接反）'),
            ExecuteProcess(
                cmd=['bash', '-lc',
                     'sleep 2; ros2 topic hz /hw/motor_states --window 20 2>&1 | '
                     'head -3 || true'],
                output='screen'),
        ],
    )

    return LaunchDescription(
        declared + [
            setup_can,
            robot_state_publisher,
            can_bridge,
            can_bridge_feedback_only,
            motor_control,
            camera_trigger,
            ros2_control,
            task_layer,
            rviz,
            teleop,
            health_check,
        ]
    )
