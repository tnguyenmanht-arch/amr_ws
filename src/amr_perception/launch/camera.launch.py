#!/usr/bin/env python3
"""
Camera driver — tạm dùng webcam USB (v4l2) thay cho IMX-219 (chưa về hàng).
Publish /camera/image_raw để lane_detection_node.py tiêu thụ.

CẦN ĐIỀU CHỈNH khi đổi sang IMX-219: thay node/package này bằng driver CSI
(vd nvarguscamerasrc qua gstreamer), giữ nguyên topic /camera/image_raw.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Thiết bị webcam — kiểm tra bằng ls /dev/video*'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device':   video_device,
                'image_width':    640,
                'image_height':   480,
                'framerate':      30.0,
                'pixel_format':   'yuyv',
                'camera_name':    'webcam',
                'camera_frame_id': 'camera_frame',
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ],
        ),
    ])
