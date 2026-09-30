#!/usr/bin/env python3
"""
Nav2 — điều hướng tự động trên bản đồ đã lưu từ SLAM.

Bật 6 thứ:
  1. hardware.launch.py  — serial_driver_node (INCLUDE, KHÔNG khai báo lại tham số)
  2. sllidar_node        — LiDAR A1M8 -> /scan
  3. static_tf           — base_link -> laser_frame
  4. localization_launch — map_server (nạp bản đồ) + AMCL
  5. navigation_launch   — planner (Smac Hybrid-A*) + controller (RPP) + costmap + BT
  6. (RViz chạy riêng, xem README)

⚠️ 2026-09-09: bản trước của file này CHỈ include `navigation_launch.py` và truyền
`map` vào đó — nhưng navigation_launch.py KHÔNG hề nạp bản đồ (map_server + AMCL nằm
trong `localization_launch.py`). Tức tham số `map` bị nuốt mất, và hệ sẽ chạy KHÔNG
có `map -> odom`, AMCL không tồn tại, Nav2 treo chờ TF vĩnh viễn. Nó cũng không bật
serial_driver/LiDAR/static_tf nên chẳng điều khiển được xe thật.

⚠️ Cùng bài học với `slam.launch.py`: KHÔNG khai báo lại tham số của serial_driver ở
đây. Mọi hằng số hiệu chuẩn (wheel_radius, ticks_per_rev, track_width, encoder sign)
nằm DUY NHẤT trong `hardware.launch.py`.
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
    hardware_launch  = os.path.join(
        get_package_share_directory('amr_hardware'), 'launch', 'hardware.launch.py')

    map_yaml    = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim     = LaunchConfiguration('use_sim_time')
    stm32_port  = LaunchConfiguration('stm32_port')
    lidar_port  = LaunchConfiguration('lidar_port')

    default_params = os.path.join(amr_nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/amr_ws/maps/map_20260909_0000.yaml'),
            description='Bản đồ .yaml lưu từ SLAM'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # ⚠️ Cả STM32 lẫn LiDAR đều là CP210x (10c4:ea60) báo serial "0001" giống hệt
        # nhau nên thứ tự ttyUSB0/1 KHÔNG đảm bảo. Dùng scripts/start_nav.sh để tự
        # nhận diện, hoặc truyền tay.
        DeclareLaunchArgument('stm32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),

        # ── 1. STM32 serial driver ────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={'serial_port': stm32_port}.items(),
        ),

        # ── 2. RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2', executable='sllidar_node',
            name='sllidar_node', output='screen',
            parameters=[{
                'channel_type':     'serial',
                'serial_port':      lidar_port,
                'serial_baudrate':  115200,
                'frame_id':         'laser_frame',
                'inverted':         False,
                'angle_compensate': True,
                'scan_mode':        'Sensitivity',
                'use_sim_time':     use_sim,
            }],
        ),

        # ── 3. Static TF — base_link -> laser_frame ───────────────
        # Đo bằng thước + dữ liệu 2026-09-07/08 (xem CLAUDE.md Giai đoạn 4):
        #   x=0.205 (tâm trục sau -> tâm LiDAR), z=0.135 (sàn -> khe laser)
        #   yaw=π  — LiDAR LẮP QUAY NGƯỢC RA SAU, xác định bằng 3 bằng chứng độc lập.
        # PHẢI GIỮ KHỚP với slam.launch.py — lệch là bản đồ và định vị đá nhau.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_laser_tf', output='screen',
            arguments=[
                '--x', '0.205', '--y', '0.0', '--z', '0.135',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
        ),

        # ── 4. map_server + AMCL ─────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map':          map_yaml,
                'params_file':  params_file,
                'use_sim_time': use_sim,
                'autostart':    'true',
            }.items(),
        ),

        # ── 5. Nav2 (planner + controller + costmap + BT) ────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file':  params_file,
                'use_sim_time': use_sim,
                'autostart':    'true',
            }.items(),
        ),
    ])
