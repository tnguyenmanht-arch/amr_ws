#!/usr/bin/env python3
"""
SLAM Toolbox launch — chạy async SLAM với RPLidar A1M8.
Dùng khi muốn vẽ bản đồ mới (mode: mapping).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params  = LaunchConfiguration('slam_params_file')

    default_params = os.path.join(
        get_package_share_directory('amr_slam'),
        'config', 'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',      default_value='false'),
        DeclareLaunchArgument('slam_params_file',  default_value=default_params),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
    ])
