#!/usr/bin/env python3
"""
SLAM launch — vẽ bản đồ với RPLidar A1M8 + STM32 odometry.
Khởi động 4 node:
  1. serial_driver_node  — giao tiếp STM32 qua /dev/stm32
  2. sllidar_node        — LiDAR A1M8 qua /dev/ttyUSB0
  3. slam_toolbox        — async SLAM, output /map
  4. static_tf           — base_link → laser_frame (x=0.15, z=0.10)
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
    stm32_port   = LaunchConfiguration('stm32_port')
    lidar_port   = LaunchConfiguration('lidar_port')

    default_params = os.path.join(
        get_package_share_directory('amr_slam'),
        'config', 'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time',     default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        DeclareLaunchArgument(
            'stm32_port', default_value='/dev/stm32',
            description='Serial port kết nối STM32 — kiểm tra: ls /dev/tty*'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='Serial port kết nối RPLidar A1M8'),

        # ── Node 1: STM32 serial driver ───────────────────────────
        # Nhận cmd_vel → gửi lệnh STM32; nhận encoder → publish /odom
        Node(
            package='amr_hardware',
            executable='serial_driver_node',
            name='serial_driver_node',
            output='screen',
            parameters=[{
                'serial_port':     stm32_port,
                'baud_rate':       115200,
                'wheel_radius':    0.10,
                'wheel_base':      0.21,
                'encoder_ppr':     11,
                'gear_ratio':      90.0,
                'publish_rate_hz': 20.0,
                'use_sim_time':    use_sim_time,
            }],
        ),

        # ── Node 2: RPLidar A1M8 ─────────────────────────────────
        # Publish /scan với frame_id=laser_frame
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type':     'serial',
                'serial_port':      lidar_port,
                'serial_baudrate':  115200,
                'frame_id':         'laser_frame',
                'inverted':         False,
                'angle_compensate': True,
                'scan_mode':        'Sensitivity',
                'use_sim_time':     use_sim_time,
            }],
        ),

        # ── Node 3: SLAM Toolbox (async) ──────────────────────────
        # Input: /scan + /odom + /tf → Output: /map + pose estimate
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),

        # ── Node 4: Static TF — base_link → laser_frame ───────────
        # LiDAR đặt trước tâm robot 15 cm, cao 10 cm
        # Cần đo lại khi lắp ráp xong (x=0.15 z=0.10 là tạm thời)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                '--x',        '0.15',
                '--y',        '0.0',
                '--z',        '0.10',
                '--roll',     '0.0',
                '--pitch',    '0.0',
                '--yaw',      '0.0',
                '--frame-id',       'base_link',
                '--child-frame-id', 'laser_frame',
            ],
        ),
    ])
