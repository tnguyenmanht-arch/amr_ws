#!/usr/bin/env python3
"""
Launch toàn bộ hệ thống AMR: hardware driver, description, LiDAR, control.
Dùng cho bringup thực tế trên Jetson Orin Nano.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ─── Arguments ───────────────────────────────────────────────────
    use_slam = LaunchConfiguration('use_slam')
    use_nav  = LaunchConfiguration('use_nav')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_slam = DeclareLaunchArgument(
        'use_slam', default_value='false',
        description='Bật SLAM Toolbox')
    declare_use_nav = DeclareLaunchArgument(
        'use_nav', default_value='false',
        description='Bật Nav2 navigation stack')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Bật RViz2 visualization')

    # ─── Sub-launches ────────────────────────────────────────────────
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('amr_description'), '/launch/description.launch.py'
        ])
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('amr_hardware'), '/launch/hardware.launch.py'
        ])
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('sllidar_ros2'), '/launch/sllidar_a1_launch.py'
        ])
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('amr_control'), '/launch/control.launch.py'
        ])
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('amr_slam'), '/launch/slam.launch.py'
        ]),
        condition=IfCondition(use_slam)
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('amr_navigation'), '/launch/navigation.launch.py'
        ]),
        condition=IfCondition(use_nav)
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('amr_bringup'), '/launch/rviz.launch.py'
        ]),
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_slam,
        declare_use_nav,
        declare_use_rviz,
        description_launch,
        hardware_launch,
        lidar_launch,
        control_launch,
        # Delay nhỏ để hardware node khởi động trước khi SLAM/Nav bắt đầu
        TimerAction(period=3.0, actions=[slam_launch]),
        TimerAction(period=5.0, actions=[nav_launch]),
        rviz_launch,
    ])
