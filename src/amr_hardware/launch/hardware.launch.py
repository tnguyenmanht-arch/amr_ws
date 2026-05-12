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
                # CẦN ĐIỀU CHỈNH theo phần cứng thực tế
                'wheel_radius':    0.04,
                'wheel_base':      0.22,
                'encoder_ppr':     11,
                'gear_ratio':      90.0,
                'publish_rate_hz': 20.0,
            }],
        ),
    ])
