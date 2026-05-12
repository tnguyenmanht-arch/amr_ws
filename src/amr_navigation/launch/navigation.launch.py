#!/usr/bin/env python3
"""
Nav2 navigation stack launch.
Yêu cầu: bản đồ đã có (từ SLAM), AMCL localization, global + local planner.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    amr_nav_dir      = get_package_share_directory('amr_navigation')

    map_yaml    = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim     = LaunchConfiguration('use_sim_time')

    default_params = os.path.join(amr_nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map',
            default_value='',
            description='Đường dẫn đến file map .yaml (để trống nếu dùng SLAM online)'),
        DeclareLaunchArgument('params_file',
            default_value=default_params,
            description='Nav2 params file'),
        DeclareLaunchArgument('use_sim_time',
            default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'map':            map_yaml,
                'params_file':    params_file,
                'use_sim_time':   use_sim,
                'autostart':      'true',
            }.items(),
        ),
    ])
