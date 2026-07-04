#!/usr/bin/env python3
"""
Lane Follow Controller — P controller đơn giản
Input:  /lane_center_error (std_msgs/Float32) — dương: robot lệch phải, âm: lệch trái
Output: /cmd_vel (geometry_msgs/Twist)

Quy ước đã XÁC MINH THỰC NGHIỆM (2026-07-03, test tĩnh không cho xe chạy):
đặt xe lệch trái tâm làn thật -> error đo được = âm, khớp docstring gốc.
angular.z = +Kp * error (dương=quay trái theo quy ước ackermann.h) là ĐÚNG dấu:
error âm (lệch trái) -> angular.z âm (quay phải) -> xe tự sửa về tâm làn.

An toàn: watchdog dừng xe (cmd_vel=0) nếu không nhận /lane_center_error mới
trong error_timeout giây (lane detection mất dấu / node chết / camera rớt).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('lane_follow_node')

        # ─── Parameters — CẦN TUNE thực nghiệm ─────────────────────
        # Tốc độ RẤT THẤP cho lần test đầu tiên (chưa có safety-stop LiDAR
        # ổn định, người test đi cạnh xe cầm dây theo dõi).
        self.declare_parameter('linear_speed',   0.12)
        self.declare_parameter('kp',             0.6)
        self.declare_parameter('max_angular_z',  0.3)
        self.declare_parameter('error_timeout',  0.5)   # giây

        self.linear_speed  = self.get_parameter('linear_speed').value
        self.kp            = self.get_parameter('kp').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.error_timeout = self.get_parameter('error_timeout').value

        self.last_error_time = None
        self.stopped_by_watchdog = False

        self.error_sub = self.create_subscription(
            Float32, '/lane_center_error',
            self.error_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Watchdog 10Hz: dừng xe nếu error quá cũ hoặc chưa từng nhận
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f'Lane follow node khởi động: linear_speed={self.linear_speed} '
            f'kp={self.kp} max_angular_z={self.max_angular_z}')

    def error_callback(self, msg: Float32):
        self.last_error_time = self.get_clock().now()
        self.stopped_by_watchdog = False

        angular_z = self.kp * msg.data
        angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

        twist = Twist()
        twist.linear.x  = self.linear_speed
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def watchdog_callback(self):
        """Dừng xe nếu chưa từng nhận error, hoặc error đã quá cũ (mất dấu làn)."""
        if self.last_error_time is None:
            return  # Chưa nhận error nào -> chưa gửi lệnh gì, không cần dừng

        elapsed = (self.get_clock().now() - self.last_error_time).nanoseconds / 1e9
        if elapsed > self.error_timeout:
            self.cmd_vel_pub.publish(Twist())  # Tất cả = 0 -> dừng xe
            if not self.stopped_by_watchdog:
                self.get_logger().warn(
                    f'Mất /lane_center_error > {self.error_timeout}s -> dừng xe')
                self.stopped_by_watchdog = True


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())  # Dừng xe khi thoát
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
