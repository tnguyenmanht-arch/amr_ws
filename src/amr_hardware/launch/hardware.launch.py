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
                # là giá trị datasheet (11). Đo 3 lần độc lập trên khung xe thật
                # (bánh chạm đất, quãng ~0.34-0.5m/lần, đo bằng thước dây so với
                # /odom): ticks_per_rev thật ~7830/8036/7827 -> kết hợp có trọng
                # số = ~7885 (độ lệch chuẩn giữa 3 lần chỉ ~1.24%, đáng tin cậy).
                # Thay cho 990 lý thuyết (11 PPR x gear 90).
                'encoder_ppr':     87.61,
                'gear_ratio':      90.0,
                'publish_rate_hz': 20.0,
                # Bù lệch tâm servo lái — HIỆU CHUẨN THỰC NGHIỆM (2026-07-03).
                # Xe đi thẳng (angular.z=0) bị lệch trái ~6.3cm/m khi chưa bù.
                # Dò bằng thực nghiệm (test 0, +0.08, -0.15, -0.10 rad/s), điểm
                # tin cậy nhất là quãng dài 1.465m @ trim=-0.10 (lệch phải
                # 6.5cm = 4.4%/m) kết hợp baseline -> nội suy trim=-0.06.
                # CẦN đo lại xác nhận khi có không gian dài hơn (>2m) — số liệu
                # hiện tại còn nhiễu ở quãng ngắn (<1m).
                'steering_trim_angular_z': -0.06,
                # Dấu encoder — ĐO THỰC NGHIỆM trên Jetson (2026-09-05) với
                # wiring DRV8871 hiện tại: gửi lệnh tiến, quan sát $ODO thấy
                # enc_l chạy ÂM (0 -> -4543) còn enc_r chạy DƯƠNG (0 -> +4518),
                # độ lớn khớp nhau -> bánh trái phải nhân -1 để "tick tăng =
                # lăn tiến". Nếu KHÔNG bù, d=(dl+dr)/2 triệt tiêu về ~0 và
                # /odom đứng yên dù xe chạy thật.
                # ⚠️ ĐO LẠI sau MỖI lần đấu lại dây motor/encoder — quy ước dấu
                # không cố định qua các lần rewire (bài học lặp lại nhiều lần).
                'left_encoder_sign':  -1.0,
                'right_encoder_sign':  1.0,
            }],
        ),
    ])
