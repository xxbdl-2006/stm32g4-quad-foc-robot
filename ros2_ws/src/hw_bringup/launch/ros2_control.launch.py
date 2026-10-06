#!/usr/bin/env python3
# ============================================================================
#  ros2_control.launch.py
#  只负责 ros2_control 那条路：robot_description + controller_manager + 控制器加载。
#
#  为什么和 bringup.launch.py 分开：
#  这条路可以被纯仿真复用（换一个 hardware 插件就是 Gazebo），
#  也可以被单机调试用（只起 controller_manager + joint_state_broadcaster）。
#  把"硬件相关"和"控制框架相关"拆开，是这套工程从一开始就守的边界。
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('hw_bringup')

    declared = [
        DeclareLaunchArgument('config',
                              default_value=os.path.join(pkg_bringup, 'config', 'hardware.yaml')),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('controllers',
                              default_value=os.path.join(pkg_bringup, 'config', 'controllers.yaml')),
        DeclareLaunchArgument('start_controllers', default_value='true'),
    ]

    can_interface = LaunchConfiguration('can_interface')
    controllers_yaml = LaunchConfiguration('controllers')

    robot_description = Command([
        'xacro ', PathJoinSubstitution([pkg_bringup, 'urdf', 'hwb_arm4.urdf.xacro']),
        ' can_interface:=', can_interface,
        ' use_ros2_control:=true',
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
        emulate_tty=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controllers')),
    )

    arm_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controllers')),
    )

    # 顺序很重要：broadcaster 必须先起来，否则 controller_manager 会因为
    # 拿不到 joint_states 而拒绝加载轨迹控制器。
    load_drive_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_joint_trajectory_controller],
        )
    )

    return LaunchDescription(
        declared + [
            ros2_control_node,
            robot_state_publisher,
            joint_state_broadcaster,
            load_drive_after_broadcaster,
        ]
    )
