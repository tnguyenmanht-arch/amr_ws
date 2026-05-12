#!/usr/bin/env python3
"""
LiDAR Filter Node — lọc điểm nhiễu và giới hạn vùng quan tâm của RPLidar A1M8.
Input:  /scan          (sensor_msgs/LaserScan) — raw từ LiDAR
Output: /scan_filtered (sensor_msgs/LaserScan) — sau khi lọc
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')

        # ─── Parameters ─────────────────────────────────────────
        # CẦN ĐIỀU CHỈNH theo môi trường và vị trí gắn LiDAR
        self.declare_parameter('min_range',     0.15)   # mét — RPLidar A1M8 noise zone
        self.declare_parameter('max_range',     10.0)   # mét — max reliable range
        self.declare_parameter('angle_min_deg', -180.0)
        self.declare_parameter('angle_max_deg',  180.0)

        self.min_range   = self.get_parameter('min_range').value
        self.max_range   = self.get_parameter('max_range').value
        self.angle_min   = np.radians(self.get_parameter('angle_min_deg').value)
        self.angle_max   = np.radians(self.get_parameter('angle_max_deg').value)

        # ─── Pub / Sub ───────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10)

        self.scan_pub = self.create_publisher(
            LaserScan, '/scan_filtered', 10)

        self.get_logger().info(
            f'LiDAR filter: range [{self.min_range}, {self.max_range}]m, '
            f'angle [{np.degrees(self.angle_min):.0f}, {np.degrees(self.angle_max):.0f}]deg'
        )

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header          = msg.header
        filtered.angle_min       = msg.angle_min
        filtered.angle_max       = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment  = msg.time_increment
        filtered.scan_time       = msg.scan_time
        filtered.range_min       = self.min_range
        filtered.range_max       = self.max_range

        ranges      = np.array(msg.ranges,      dtype=np.float32)
        intensities = np.array(msg.intensities, dtype=np.float32) if msg.intensities else None

        # Tính góc của từng điểm
        n      = len(ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        # Tạo mask lọc
        valid = (
            (ranges >= self.min_range) &
            (ranges <= self.max_range) &
            (angles >= self.angle_min) &
            (angles <= self.angle_max) &
            np.isfinite(ranges)
        )

        # Thay điểm không hợp lệ bằng inf (ROS convention)
        filtered_ranges = np.where(valid, ranges, np.inf)
        filtered.ranges = filtered_ranges.tolist()

        if intensities is not None and len(intensities) == n:
            filtered_intensities = np.where(valid, intensities, 0.0)
            filtered.intensities = filtered_intensities.tolist()

        self.scan_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
