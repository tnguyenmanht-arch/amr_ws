#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_control',
            executable='ackermann_controller_node',
            name='ackermann_controller_node',
            output='screen',
            parameters=[{
                # CẦN ĐIỀU CHỈNH theo kích thước xe thực tế (đơn vị: mét)
                'wheelbase_m':      0.22,
                'track_width_m':    0.18,
                'max_steering_deg': 30.0,  # kiểm tra giới hạn servo HTS-20H
            }],
        ),
    ])
