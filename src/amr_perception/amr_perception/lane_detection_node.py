#!/usr/bin/env python3
"""
Lane Detection Node — IMX-219 camera + OpenCV Canny/Hough
Input:  /camera/image_raw  (sensor_msgs/Image)
Output: /lane_center_error (std_msgs/Float32)  — dương: robot lệch phải, âm: lệch trái
        /lane_debug_image  (sensor_msgs/Image)  — ảnh debug với lane overlay
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import numpy as np


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # ─── Parameters ─────────────────────────────────────────
        # CẦN ĐIỀU CHỈNH các threshold theo điều kiện ánh sáng thực tế
        self.declare_parameter('canny_low',    50)
        self.declare_parameter('canny_high',   150)
        self.declare_parameter('hough_threshold', 30)
        self.declare_parameter('roi_top_ratio', 0.55)   # ROI từ 55% chiều cao trở xuống
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)

        self.canny_low     = self.get_parameter('canny_low').value
        self.canny_high    = self.get_parameter('canny_high').value
        self.hough_thresh  = self.get_parameter('hough_threshold').value
        self.roi_top       = self.get_parameter('roi_top_ratio').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.img_w         = self.get_parameter('image_width').value
        self.img_h         = self.get_parameter('image_height').value

        self.bridge = CvBridge()

        # ─── Pub / Sub ───────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10)

        self.error_pub = self.create_publisher(Float32, '/lane_center_error', 10)

        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/lane_debug_image', 10)

        self.get_logger().info('Lane detection node khởi động')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (self.img_w, self.img_h))

        error, debug_frame = self._detect_lane(frame)

        # Publish lỗi tâm làn
        err_msg = Float32()
        err_msg.data = float(error)
        self.error_pub.publish(err_msg)

        if self.publish_debug and debug_frame is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _detect_lane(self, frame: np.ndarray):
        """Phát hiện làn đường bằng Canny + Hough transform."""
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top)

        # ─── Tiền xử lý ─────────────────────────────────────────
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, self.canny_low, self.canny_high)

        # ─── ROI mask (hình thang) ───────────────────────────────
        mask    = np.zeros_like(edges)
        roi_pts = np.array([[
            (int(w * 0.05), h),
            (int(w * 0.95), h),
            (int(w * 0.75), roi_y),
            (int(w * 0.25), roi_y),
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi_pts, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # ─── Hough transform ────────────────────────────────────
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1, theta=np.pi / 180,
            threshold=self.hough_thresh,
            minLineLength=40, maxLineGap=20
        )

        left_lines, right_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                # Lọc theo độ dốc: |slope| > 0.3 tránh đường ngang nhiễu
                if abs(slope) < 0.3:
                    continue
                if slope < 0:
                    left_lines.append(line[0])
                else:
                    right_lines.append(line[0])

        # ─── Tính tâm làn ────────────────────────────────────────
        cx_w = w / 2.0  # mặc định: không phát hiện được làn
        debug_frame = frame.copy()

        left_x  = self._avg_x(left_lines,  h)
        right_x = self._avg_x(right_lines, h)

        if left_x is not None and right_x is not None:
            cx_w = (left_x + right_x) / 2.0
        elif left_x is not None:
            cx_w = left_x + w * 0.25  # ước tính tâm nếu chỉ thấy làn trái
        elif right_x is not None:
            cx_w = right_x - w * 0.25

        # error dương = lệch phải, âm = lệch trái (tính theo pixel, chuẩn hoá)
        error = (cx_w - w / 2.0) / (w / 2.0)

        # ─── Vẽ debug ────────────────────────────────────────────
        cv2.polylines(debug_frame, roi_pts, True, (0, 255, 255), 2)
        if left_x is not None:
            cv2.circle(debug_frame, (int(left_x), h - 20), 8, (0, 255, 0), -1)
        if right_x is not None:
            cv2.circle(debug_frame, (int(right_x), h - 20), 8, (0, 0, 255), -1)
        cv2.circle(debug_frame, (int(cx_w), h - 20), 10, (255, 0, 0), -1)
        cv2.putText(debug_frame, f'error: {error:.3f}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return error, debug_frame

    def _avg_x(self, lines, img_h):
        """Tính x trung bình ở đáy ảnh từ list các line."""
        if not lines:
            return None
        xs = []
        for x1, y1, x2, y2 in lines:
            if y2 == y1:
                continue
            # Extrapolate đến đáy ảnh
            slope     = (x2 - x1) / (y2 - y1 + 1e-6)
            x_bottom  = x1 + slope * (img_h - y1)
            xs.append(x_bottom)
        return float(np.mean(xs)) if xs else None


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
