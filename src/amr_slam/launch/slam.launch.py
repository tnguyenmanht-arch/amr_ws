#!/usr/bin/env python3
"""
SLAM launch — vẽ bản đồ với RPLidar A1M8 + STM32 odometry.
Khởi động 4 thành phần:
  1. hardware.launch.py  — serial_driver_node (INCLUDE, không copy tham số)
  2. sllidar_node        — LiDAR A1M8
  3. slam_toolbox        — async SLAM, output /map
  4. static_tf           — base_link -> laser_frame

⚠️ 2026-09-07: node serial_driver trước đây được KHAI BÁO LẠI trong file này
với bộ tham số riêng, và bộ đó đã lỗi thời từ 2026-09-05 (wheel_radius=0.10 =
lỗi đường-kính-nhầm-bán-kính, encoder_ppr=11 thay vì 44, THIẾU hẳn track_width
và left/right_encoder_sign). Chạy SLAM bằng file này sẽ vứt bỏ toàn bộ hiệu
chuẩn: /odom sai quãng đường ~8 lần VÀ đứng yên hoàn toàn (không có encoder
sign thì d=(dl+dr)/2 triệt tiêu về 0).
=> Nay INCLUDE thẳng hardware.launch.py. Hiệu chuẩn chỉ tồn tại ở MỘT chỗ
   duy nhất, không thể lệch pha giữa 2 file được nữa.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    hardware_launch = os.path.join(
        get_package_share_directory('amr_hardware'),
        'launch', 'hardware.launch.py'
    )

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time',     default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        # ⚠️ CẢ HAI thiết bị đều là cầu USB-UART Silicon Labs CP210x
        # (10c4:ea60) nên thứ tự ttyUSB0/ttyUSB1 KHÔNG đảm bảo qua các lần
        # cắm. Kiểm tra trước mỗi lần chạy:
        #     udevadm info -q property -n /dev/ttyUSB0 | grep ID_PATH
        # hoặc truyền tay: stm32_port:=/dev/ttyUSBx lidar_port:=/dev/ttyUSBy
        DeclareLaunchArgument(
            'stm32_port', default_value='/dev/ttyUSB0',
            description='Serial port kết nối STM32 (CP2102)'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB1',
            description='Serial port kết nối RPLidar A1M8'),

        # ── 1. STM32 serial driver ────────────────────────────────
        # INCLUDE, KHÔNG khai báo lại tham số — mọi hằng số hiệu chuẩn
        # (wheel_radius, ticks_per_rev, track_width, encoder sign, trim)
        # nằm DUY NHẤT trong hardware.launch.py.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={'serial_port': stm32_port}.items(),
        ),

        # ── 2. RPLidar A1M8 ──────────────────────────────────────
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

        # ── 3. SLAM Toolbox (async) ───────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),

        # ── 4. Static TF — base_link -> laser_frame ───────────────
        # ✅ ĐO BẰNG THƯỚC 2026-09-07 (thay số ước lượng cũ x=0.15, z=0.10):
        #   x = 0.205 — từ TÂM TRỤC SAU tới tâm LiDAR, theo chiều dọc xe.
        #               (user đo lại 2026-09-07, thay số 0.19 đo lần đầu)
        #               Kiểm chứng chéo: user đo thêm LiDAR cách ĐUÔI xe
        #               0.25m -> phần đuôi thò ra sau trục sau = 25-20.5 =
        #               4.5cm, khớp giả định 4.7cm suy từ (dài 304 - trục cơ
        #               sở 210)/2. Hai phép đo độc lập nhau nên số này chắc.
        #               base_link của hệ đang chạy CHÍNH LÀ tâm trục sau, vì
        #               serial_driver_node tích phân chuyển động của trục sau
        #               (d=(dl+dr)/2, dtheta=(dr-dl)/track_width).
        #   y = 0.0   — user xác nhận LiDAR nằm đúng giữa trái-phải.
        #   z = 0.135 — từ mặt sàn lên khe laser. Với SLAM 2D thì z không ảnh
        #               hưởng gì, chỉ để hiển thị trong RViz cho đúng.
        #   yaw = 3.1416 (180°) — LiDAR LẮP QUAY NGƯỢC RA SAU. Đo bằng dữ
        #               liệu, KHÔNG đoán theo mũi tên trên vỏ: đặt vật mốc ở 2
        #               vị trí khác nhau, mỗi lần dự đoán trước góc sẽ thấy rồi
        #               so với thực đo (2026-09-07).
        #                 vật trước mũi xe 50cm  -> dự đoán 180.0° | đo 175-180°
        #                 vật bên trái, tâm 56cm -> dự đoán -81.4° | đo   -78°
        #               Cả hai khớp. Bằng chứng thứ 3 độc lập: thân xe (thứ duy
        #               nhất chắc chắn nằm SAU LiDAR) hiện ra quanh 0°, tức
        #               0° của LiDAR đang chỉ về phía sau xe.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                '--x',        '0.205',
                '--y',        '0.0',
                '--z',        '0.135',
                '--roll',     '0.0',
                '--pitch',    '0.0',
                '--yaw',      '3.14159',
                '--frame-id',       'base_link',
                '--child-frame-id', 'laser_frame',
            ],
        ),
    ])
