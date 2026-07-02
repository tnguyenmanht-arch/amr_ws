#!/usr/bin/env python3
"""
teleop_relay — điều khiển AMR bằng bàn phím, đọc phím ở RAW MODE.

Khác với teleop_twist_keyboard: script này tự set raw mode bằng tty.setraw
và publish geometry_msgs/Twist THẲNG lên /cmd_vel ngay khi nhấn phím
(không cần Enter). Dùng khi teleop_twist_keyboard bị kẹt ở cooked mode.

Bản đồ phím:
    i = tiến          (linear.x = +0.5)
    , = lùi           (linear.x = -0.5)
    j = tiến + rẽ trái  (linear.x = 0.3, angular.z = +0.5)
    l = tiến + rẽ phải  (linear.x = 0.3, angular.z = -0.5)
    k = dừng          (0, 0)
    q / Ctrl-C = thoát

Lưu ý: /cmd_vel phải được relay sang
       /ackermann_steering_controller/reference_unstamped
(node cmd_vel_relay trong gazebo.launch.py) thì robot mới chạy.
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Bản đồ phím → (linear.x, angular.z)
KEY_MAP = {
    'i': (0.5, 0.0),
    ',': (-0.5, 0.0),
    'j': (0.3, 0.5),
    'l': (0.3, -0.5),
    'k': (0.0, 0.0),
}

HELP = __doc__


class TeleopRelay(Node):
    def __init__(self):
        super().__init__('teleop_relay')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)


def main(args=None):
    if not sys.stdin.isatty():
        # Bắt buộc phải chạy trong terminal thật (có tty) mới đọc raw được
        print('[teleop_relay] LỖI: stdin không phải tty. '
              'Hãy chạy trực tiếp trong terminal VS Code, không qua pipe.')
        return

    rclpy.init(args=args)
    node = TeleopRelay()
    print(HELP)
    print('Sẵn sàng — nhấn phím để điều khiển...')

    # ── Raw mode được set MỘT LẦN, giữ SUỐT vòng lặp ──────────────────
    # BUG cũ: setraw()+restore mỗi lần đọc phím → terminal nhảy về cooked
    # mode giữa các lần nhấn, mất raw mode sau vài phím. Đúng pattern:
    # setraw() một lần trước while, restore một lần duy nhất ở finally.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)            # vào raw mode 1 lần
        while rclpy.ok():
            ch = sys.stdin.read(1)         # đọc 1 ký tự, không cần Enter
            if ch in ('q', '\x03'):        # 'q' hoặc Ctrl-C
                break
            if ch in KEY_MAP:
                lin, ang = KEY_MAP[ch]
                node.send(lin, ang)
                # '\r' đưa con trỏ về đầu dòng vì raw mode không tự xuống dòng
                print(f'\r[teleop_relay] {ch!r} → linear.x={lin:+.2f} '
                      f'angular.z={ang:+.2f}   ', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        # Khôi phục terminal về cooked mode MỘT LẦN khi thoát
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.send(0.0, 0.0)        # dừng robot khi thoát
        print('\n[teleop_relay] Đã dừng robot, thoát.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
