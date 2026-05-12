#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_debug', default_value='true',
                              description='Publish /lane_debug_image để xem trên RViz'),

        # LiDAR filter — nên chạy trước SLAM/Nav để giảm nhiễu
        Node(
            package='amr_perception',
            executable='lidar_filter_node',
            name='lidar_filter_node',
            output='screen',
            parameters=[{
                'min_range': 0.15,
                'max_range': 10.0,
            }],
        ),

        # Lane detection — chỉ bật khi cần lane keeping
        Node(
            package='amr_perception',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[{
                # CẦN ĐIỀU CHỈNH theo điều kiện ánh sáng và làn đường thực tế
                'canny_low':     50,
                'canny_high':    150,
                'hough_threshold': 30,
                'roi_top_ratio': 0.55,
                'publish_debug': LaunchConfiguration('publish_debug'),
            }],
        ),
    ])
