#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # CẦN ĐIỀU CHỈNH serial_port theo thiết bị thực tế
    # Kiểm tra bằng: ls /dev/ttyUSB* /dev/ttyACM*
    serial_port = LaunchConfiguration('serial_port')
    baud_rate   = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Port serial kết nối STM32'),
        DeclareLaunchArgument('baud_rate',   default_value='115200',
                              description='Baud rate — phải khớp với STM32 firmware'),
        Node(
            package='amr_hardware',
            executable='serial_driver_node',
            name='serial_driver_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate':   baud_rate,
                'wheel_radius':    0.10,
                'wheel_base':      0.21,
                # encoder_ppr đã HIỆU CHUẨN THỰC NGHIỆM (2026-07-03), KHÔNG còn
                # là giá trị datasheet (11). Đo: xe chạy thẳng ~0.426m thực tế
                # (thước dây) trong khi /odom tính ra ~3.361m -> ticks_per_rev
                # thật ~7810 (thay vì 990 lý thuyết = 11*90). Chỉ 1 lần đo, độ
                # lệch hướng y không chắc chắn -> CẦN đo lại thêm vài lần, quãng
                # đường dài hơn để tăng độ chính xác trước khi tin tưởng hoàn toàn.
                'encoder_ppr':     86.78,
                'gear_ratio':      90.0,
                'publish_rate_hz': 20.0,
            }],
        ),
    ])
