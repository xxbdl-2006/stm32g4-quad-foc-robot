#!/usr/bin/env python3
# ============================================================================
#  task.launch.py
#  取放任务的三个节点：物块识别 + 任务执行 + 可选的调试视图。
#
#  前提：硬件的 bringup 已经在跑（bringup.launch.py）。
#  这个 launch 只负责"任务层"，不碰驱动。
#
#  典型用法
#   -------
#    # 1. 先起硬件
#    ros2 launch hw_bringup bringup.launch.py use_teleop:=false use_rviz:=true
#
#    # 2. 再起任务层
#    ros2 launch hw_task task.launch.py
#
#    # 3. 标定相机内参（一次性）
#    ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
#        image:=/image_raw camera:=/camera
#
#    # 4. 用一个已知位置的物块做一次投影校正（可选，但建议做）
#    ros2 service call /task/calibrate_mapping hw_msgs/srv/CalibrateMapping \
#        "{block_color: 1, pixel_x: 318.0, pixel_y: 356.0, world_x: 0.0, world_y: 0.45}"
#
#    # 5. 启动任务
#    ros2 service call /task/start hw_msgs/srv/StartTask \
#        "{max_blocks: 5, require_stable: true, return_home_after: true}"
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_task = get_package_share_directory('hw_task')

    declared = [
        DeclareLaunchArgument('task_config',
                              default_value=os.path.join(pkg_task, 'config', 'task.yaml'),
                              description='任务参数文件'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw',
                              description='图像输入话题'),
        DeclareLaunchArgument('start_detector', default_value='true',
                              description='是否启动物块识别节点'),
        DeclareLaunchArgument('use_camera_driver', default_value='false',
                              description='是否顺带拉起 usb_cam'),
        DeclareLaunchArgument('debug_view', default_value='false',
                              description='物块识别弹出 OpenCV 调试窗口'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true 则在启动 5 秒后自动开始任务（试跑用）'),
        DeclareLaunchArgument('max_blocks', default_value='5'),
    ]

    task_config = LaunchConfiguration('task_config')
    image_topic = LaunchConfiguration('image_topic')

    # ---------------------------------------------------------------- 相机驱动
    # 比赛现场相机型号不固定，所以默认不代管；需要时用 use_camera_driver:=true。
    # 要用 hw_bringup 里已经配好的那一路也行 —— 只要 /image_raw 有数据就行。
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv2rgb',
            'framerate': 30.0,
            'camera_name': 'camera',
        }],
        remappings=[('image_raw', image_topic)],
        condition=IfCondition(LaunchConfiguration('use_camera_driver')),
    )

    # ---------------------------------------------------------------- 物块识别
    detector = Node(
        package='hw_task',
        executable='block_detector_node',
        name='block_detector',
        output='screen',
        emulate_tty=True,
        parameters=[
            task_config,
            {
                'image_topic': image_topic,
                'debug_view': LaunchConfiguration('debug_view'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('start_detector')),
    )

    # ---------------------------------------------------------------- 任务执行
    # 稍微延后启动：让 TF 缓冲先填上几帧再开始查位姿，
    # 否则启动瞬间的 lookupTransform 会因为"目标时间早于缓冲最早时间"而失败，
    # 日志里会出现一串吓人的 TF 警告（虽然不影响功能）。
    executor = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='hw_task',
                executable='task_executor_node',
                name='task_executor',
                output='screen',
                emulate_tty=True,
                parameters=[task_config],
            ),
        ],
    )

    # ---------------------------------------------------------------- 提示
    hint = TimerAction(
        period=3.5,
        actions=[
            LogInfo(msg=[
                '\n================================================================\n',
                '  取放任务层已就绪。下一步：\n',
                '  1) 确认视觉： ros2 topic hz /detected_blocks      (应为相机帧率)\n',
                '  2) 确认投影： ros2 topic echo /grasp_targets --once\n',
                '                在 RViz 里看 "/grasp_targets" 的箭头是否落在物块上。\n',
                '                如果整体偏移固定 -> 用 calibrate_mapping 校正；\n',
                '                如果随距离变化 -> 相机内参错了，去重新标定。\n',
                '  3) 启动任务： ros2 service call /task/start hw_msgs/srv/StartTask \\\n',
                '                 "{max_blocks: 5, require_stable: true}"\n',
                '  4) 中止任务： ros2 service call /task/abort hw_msgs/srv/AbortTask \\\n',
                '                 "{release_payload: true, return_home: false}"\n',
                '================================================================\n',
            ]),
        ],
    )

    # ---------------------------------------------------------------- 可选自动启动
    autostart = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rclcpp_components',
                executable='component_container',
                name='autostart_stub',
                output='log',
                condition=IfCondition(LaunchConfiguration('autostart')),
                # 这里不用 Node 调服务，而是让用户自己敲 —— 自动开始运动是危险操作，
                # 保留一个显式的人为动作。autostart 只打印提示。
            ),
            LogInfo(msg='[hw_task] autostart=true：如需自动开始，请手动调用 /task/start',
                    condition=IfCondition(LaunchConfiguration('autostart'))),
        ],
    )

    return LaunchDescription(
        declared + [camera, detector, executor, hint, autostart]
    )
