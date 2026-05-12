# CLAUDE.md — AMR Ackermann Robot | HUST Automotive Engineering

> Dự án môn học — Chương trình Kỹ sư chuyên sâu Ô tô số (Automotive Digital Engineering)
> Đại học Bách Khoa Hà Nội (HUST) · Việt Nam

---

## 1. Tổng quan dự án

Robot AMR 4 bánh dẫn động Ackermann có khả năng tự định vị, lập bản đồ và điều hướng thông minh trong môi trường trong nhà. Mục tiêu cuối: bám làn, phân làn và đỗ xe đơn giản.

**Kiến trúc hệ thống:**

```
[Jetson Orin Nano Super 8GB] ←→ UART/USB ←→ [STM32F446RE Nucleo-64]
        (ROS2 Master)                              (Low-level Slave)
        ↑            ↑                              ↑           ↑
  IMX-219 Camera  RPLidar A1M8              JGB37-520 Motors  HTS-20H Servo
  (lane detect)   (SLAM/Nav)               (drive wheels)    (steering)
```

---

## 2. Hardware — Thông số kỹ thuật

### Master: Jetson Orin Nano Super Dev Kit 8GB
- OS: Ubuntu 22.04 LTS (JetPack 6.x)
- ROS2 Distribution: Humble Hawksbill
- GPU: Ampere, 1024 CUDA cores (dùng cho vision inference nếu cần)
- Giao tiếp với slave: UART hoặc USB-Serial
- Thư viện NVIDIA có thể dùng: Isaac ROS, DeepStream, TensorRT (nếu phần cứng đủ điều kiện)

### Slave: STM32F446RE Nucleo-64
- Firmware: STM32CubeIDE / PlatformIO
- Nhiệm vụ: nhận cmd_vel → điều khiển motor PWM, đọc encoder, điều khiển servo qua Serial Bus
- Giao thức với master: rosserial / micro-ROS / custom UART protocol
- Timer dùng cho encoder: TIM2, TIM3 (quadrature mode)
- PWM output cho motor: TIM1_CH1, TIM1_CH2

### Cơ cấu chấp hành
| Thiết bị | Model | Giao tiếp | Ghi chú |
|---|---|---|---|
| Drive Motor (×2) | JGB37-520 DC w/ Encoder | PWM + DIR + Encoder | 12V, đọc encoder 11 PPR × gear ratio: 90:1 |
| Steering Servo | HTS-20H Serial Bus Servo | Serial Bus (UART-based) | Góc lái Ackermann, không phải servo thường |
| Motor Driver | SA8339 | GPIO + PWM | Xác nhận lại theo mạch thực tế |

### Cảm biến
| Thiết bị | Model | Giao tiếp | Topic ROS2 |
|---|---|---|---|
| Camera | IMX-219 (CSI) | MIPI CSI-2 | `/camera/image_raw` |
| LiDAR | RPLidar A1M8 | USB-Serial | `/scan` |
| Encoder (×2) | Tích hợp trong JGB37-520 | Đọc qua STM32 | `/odom` (tính toán từ STM32) |

### Tương lai (chưa tích hợp)
- **reSpeaker 4-mic Array** (SeeedStudio): voice control, wake word detection
- Topic dự kiến: `/speech_command`, `/wake_word`

---

## 3. Software Stack

### ROS2 Package Structure (dự kiến)

```
amr_ws/
├── src/
│   ├── amr_bringup/          # launch files, config tổng
│   ├── amr_hardware/         # driver STM32, serial comm
│   │   ├── motor_driver/     # node điều khiển JGB37-520
│   │   └── steering_driver/  # node HTS-20H serial bus servo
│   ├── amr_description/      # URDF/Xacro mô tả robot
│   ├── amr_navigation/       # Nav2 config, costmaps
│   ├── amr_slam/             # SLAM Toolbox config
│   ├── amr_perception/       # lane detection, object detect
│   │   ├── lane_detection/   # IMX-219 + OpenCV/TensorRT
│   │   └── lidar_filter/     # lọc điểm LiDAR
│   └── amr_control/          # Ackermann controller, cmd_vel → steering
├── CLAUDE.md
└── README.md
```

### Các topic ROS2 chính

```
/scan                    → sensor_msgs/LaserScan       (LiDAR A1M8)
/camera/image_raw        → sensor_msgs/Image           (IMX-219)
/camera/camera_info      → sensor_msgs/CameraInfo
/cmd_vel                 → geometry_msgs/Twist         (Nav2 output)
/odom                    → nav_msgs/Odometry           (từ encoder STM32)
/tf, /tf_static          → tf2 transforms
/map                     → nav_msgs/OccupancyGrid      (SLAM output)
/amcl_pose               → geometry_msgs/PoseWithCovarianceStamped
/lane_center_error       → std_msgs/Float32            (lane keeping output)
/steering_angle          → std_msgs/Float32            (góc lái actual)
```

### Frames TF quan trọng

```
map → odom → base_link → laser_frame
                       → camera_frame
                       → base_footprint
```

---

## 4. Thuật toán & Tính năng mục tiêu

### 4.1 Ackermann Steering Geometry
- Robot có 2 bánh dẫn động phía sau, steering phía trước
- Tỉ lệ góc lái trái/phải theo công thức Ackermann
- cmd_vel nhận linear.x và angular.z → tính góc servo và tốc độ 2 bánh
- Chú ý: cmd_vel standard là Twist, cần convert sang Ackermann command

### 4.2 SLAM — Tự vẽ bản đồ
- Tool: **slam_toolbox** (khuyến nghị, async mode)
- Input: `/scan` + `/odom` + `/tf`
- Output: `/map` + pose estimate
- Config file: `amr_slam/config/slam_toolbox_params.yaml`

### 4.3 Navigation — Nav2
- Global Planner: NavFn hoặc SmacPlanner
- Local Planner: DWB (Dynamic Window Approach) hoặc RPP (Regulated Pure Pursuit)
- Costmaps: inflation layer, obstacle layer (LiDAR + camera optional)
- Config files trong `amr_navigation/config/`

### 4.4 Lane Keeping (perception)
- Input: IMX-219 camera (CSI, 1080p → resize 640×480 để xử lý)
- Phương pháp: Canny edge + Hough transform (cơ bản) → sau đó có thể upgrade lên DNN
- Output: `/lane_center_error` → PID controller → angular.z trong cmd_vel
- Nếu GPU đủ: dùng TensorRT với model lane detection đã train sẵn

### 4.5 Đỗ xe đơn giản
- Phát hiện vị trí đỗ bằng LiDAR (khoảng trống đủ lớn)
- State machine: SEARCHING → APPROACHING → PARKING → DONE

---

## 5. Quy ước code

### Ngôn ngữ
- **Python**: nodes ROS2 logic cao (perception, navigation client, state machine)
- **C++**: nodes real-time, driver phần cứng, serial comm với STM32
- **C (STM32)**: firmware slave, HAL-based

### Naming convention
- ROS2 nodes: `snake_case` (ví dụ: `lane_detection_node`)
- Topics: `/snake_case` (ví dụ: `/lane_center_error`)
- Parameters: `snake_case` trong YAML
- C++: `camelCase` cho hàm, `UPPER_CASE` cho macro
- STM32: prefix `APP_` cho application layer, `DRV_` cho driver

### Cấu trúc node ROS2 (Python template)
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodeName(Node):
    def __init__(self):
        super().__init__('node_name')
        self.declare_parameter('param_name', default_value)
        # publishers, subscribers, timers...

def main(args=None):
    rclpy.init(args=args)
    node = NodeName()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 6. Lệnh hay dùng

### Build & Run

```bash
# Build workspace
cd ~/amr_ws && colcon build --symlink-install
source install/setup.bash

# Bringup toàn bộ hệ thống
ros2 launch amr_bringup amr_bringup.launch.py

# Chỉ LiDAR
ros2 launch sllidar_ros2 sllidar_a1_launch.py

# Chỉ SLAM
ros2 launch amr_slam slam.launch.py

# Navigation
ros2 launch amr_navigation navigation.launch.py map:=/path/to/map.yaml

# RViz
ros2 launch amr_bringup rviz.launch.py
```

### Debug nhanh

```bash
# Xem topic đang publish
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /camera/image_raw

# Kiểm tra TF
ros2 run tf2_tools view_frames

# Teleop manual
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Xem log
ros2 run rqt_console rqt_console
```

### STM32 / Serial

```bash
# Kiểm tra port
ls /dev/ttyUSB* /dev/ttyACM*

# Monitor serial (debug)
screen /dev/ttyUSB0 115200
# hoặc
minicom -D /dev/ttyUSB0 -b 115200
```

---

## 7. Hướng dẫn cho Claude — Quy tắc làm việc

### Ưu tiên tiếp cận
1. **Luôn giải thích ngắn gọn lý do** trước khi viết code — đặc biệt với thuật toán mới
2. **Ghi chú rõ phần cần sinh viên tự điều chỉnh** (giá trị PID, thông số cơ học, địa chỉ port)
3. **Ưu tiên giải pháp đơn giản, hoạt động được** hơn giải pháp tối ưu phức tạp
4. Code phải có comment bằng tiếng Việt cho những phần logic quan trọng

### KHÔNG tự động làm
- Không thay đổi frame_id trong URDF nếu chưa hỏi
- Không thay đổi baud rate serial mà không xác nhận
- Không sửa PID gains — luôn comment rõ "cần tune thực nghiệm"
- Không assume motor driver model nếu chưa được xác nhận

### Khi gặp lỗi ROS2
Luôn hỏi: "Bạn đang dùng ROS2 distro gì?" nếu chưa rõ → mặc định là **Humble**

### Khi compact context, giữ lại
- Danh sách file đã tạo/chỉnh sửa
- Thông số đã xác nhận (port, baud rate, gear ratio, PID values nếu có)
- Lỗi đã gặp và cách giải quyết
- Milestone đã hoàn thành

---

## 8. Trạng thái dự án (cập nhật thủ công)

### Đã hoàn thành
- [ ] Thiết kế cơ khí và lắp ráp khung xe
- [ ] Cài đặt Ubuntu + ROS2 Humble trên Jetson Orin Nano
- [ ] Flash firmware STM32 cơ bản
- [ ] Driver LiDAR A1M8 hoạt động
- [ ] Driver Camera IMX-219 hoạt động
- [ ] Giao tiếp STM32 ↔ Jetson
- [ ] URDF mô tả robot
- [ ] SLAM hoạt động
- [ ] Nav2 hoạt động
- [ ] Lane detection cơ bản
- [ ] Lane keeping controller
- [ ] Parking logic

### Vấn đề đang gặp
_(cập nhật khi có)_

### Quyết định kỹ thuật đã chốt
_(cập nhật khi có)_

---

## 9. Tài liệu tham khảo

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Nav2 Docs](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [sllidar_ros2 (RPLidar A1)](https://github.com/Slamtec/sllidar_ros2)
- [Isaac ROS (NVIDIA)](https://github.com/NVIDIA-ISAAC-ROS)
- [micro-ROS for STM32](https://micro.ros.org/)
- [HTS-20H Servo Datasheet](https://www.hiwonder.com/) ← xác nhận link chính xác
- [JGB37-520 Specs](https://www.aliexpress.com/) ← điền gear ratio thực tế
- Demo tham khảo: https://www.youtube.com/watch?v=0s0URZ9IyTs
