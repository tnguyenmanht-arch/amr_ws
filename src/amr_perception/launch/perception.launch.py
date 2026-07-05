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
                # CẦN ĐIỀU CHỈNH theo điều kiện ánh sáng và làn đường thực tế.
                # canny_low/high hạ từ 50/150 -> 20/60 (2026-07-05): ở ngưỡng
                # cũ, Canny hoàn toàn không bắt được cạnh vạch trái trong 1
                # frame test thật (0 edge pixel trong ROI) dù vạch rõ ràng
                # trong ảnh gốc -> thiếu tương phản dưới ánh sáng hiện tại.
                # Ngưỡng 20/60 bắt được cả 2 vạch (11 trái/9 phải trên cùng
                # frame test). CẦN kiểm tra lại có gây nhiễu vân sàn gỗ nhiều
                # hơn không khi test dài hơi.
                'canny_low':     20,
                'canny_high':    60,
                'hough_threshold': 30,
                # roi_top_ratio=0.35 khớp vị trí camera hiện tại (cao 22.5cm,
                # nghiêng ~7.5°, xem camera.xacro) — đo thực nghiệm 2026-07-05.
                # Giá trị cũ 0.55 khiến ROI không bao trọn vạch làn (vạch nằm
                # ngoài/phía trên ROI) -> mất dấu liên tục, KHÔNG phải do ánh
                # sáng như nghi ngờ ban đầu. Verify: /lane_center_error publish
                # ổn định 26.8Hz (khớp camera) sau khi sửa, so với 4.7-10.9Hz
                # thất thường trước đó. CẦN đo lại nếu đổi vị trí/góc camera.
                'roi_top_ratio': 0.35,
                'publish_debug': LaunchConfiguration('publish_debug'),
            }],
        ),
    ])
