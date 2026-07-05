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
        ↑            ↑                              ↑                    ↑
  IMX-219 Camera  RPLidar A1M8        [Hiwonder 4-Ch Motor Driver]  USART1 (single-wire)
  (lane detect)   (SLAM/Nav)              I2C ↓                     qua điện trở nối tiếp ↓
                                     JGB37-520 Motors              HTS-20H Servo
                                      (drive wheels)                (steering)
```
> Servo lái nối **thẳng** vào STM32 (không qua TTL Bus Servo Debugging Board — board này đã hỏng, xem mục 8).

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
- Nhiệm vụ: nhận cmd_vel → điều khiển motor (I2C), đọc encoder, điều khiển servo qua single-wire trực tiếp
- Giao thức với master: custom UART protocol qua USART2

**Pin assignments đã xác nhận:**
| Peripheral | Pins | Kết nối |
|---|---|---|
| I2C1 | PB8 (SCL), PB9 (SDA) | Hiwonder 4-Ch Motor Driver |
| USART2 | PA2 (TX), PA3 (RX) | Jetson Orin Nano, 115200 baud |
| USART1 | PA9 (TX), PA10 (RX) | PA9 qua điện trở ~1kΩ + PA10 nối thẳng → dây SIG của HTS-20H (single-wire half-duplex, không qua debug board) |

**Motor Driver (I2C) — Registers quan trọng:**
| Register | Addr | Giá trị | Mô tả |
|---|---|---|---|
| MOTOR_TYPE | `0x14` | `3` (JGB37-520R90) | Loại motor |
| ENCODER_POLARITY | `0x15` | `0` | Chiều encoder |
| FIXED_SPEED | `0x33` | `-100 ~ 100` | Closed-loop speed |
| ENCODER_TOTAL | `0x3C` | đọc pulse / ghi `0` reset | Odometry |

### Cơ cấu chấp hành
| Thiết bị | Model | Giao tiếp | Ghi chú |
|---|---|---|---|
| Drive Motor (×2) | JGB37-520 DC w/ Encoder | PWM + DIR + Encoder | 12V, đọc encoder 11 PPR × gear ratio: 90:1 |
| Steering Servo | HTS-20H Serial Bus Servo | Serial Bus (TTL, single-wire) | Góc lái Ackermann; STM32 nối **thẳng** qua điện trở nối tiếp (xem mục 8) |
| Motor Driver | 4-Ch Encoder Motor Driver Hiwonder | I2C | Mạch module sẵn của Hiwonder |
| ~~TTL Bus Servo Board~~ | ~~Hiwonder TTL Bus Servo Debugging Board~~ | — | **ĐÃ HỎNG, không dùng nữa** — thay bằng đấu trực tiếp STM32↔servo |

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

### Cấu trúc repo (monorepo: ROS2 + firmware trong 1 repo, branch `main`)

```
amr_ws/
├── src/                    ← ROS2 packages (chạy trên Jetson)
│   ├── amr_bringup/
│   ├── amr_description/    ← HOÀN THÀNH
│   ├── amr_hardware/       ← CHƯA VIẾT
│   ├── amr_control/        ← CHƯA VIẾT
│   ├── amr_slam/           ← CHƯA
│   ├── amr_navigation/     ← CHƯA
│   └── amr_perception/     ← CHƯA
├── firmware/               ← STM32 firmware (build trên Windows)
│   ├── Core/Src/           ← Application code
│   ├── Core/Inc/
│   ├── Drivers/
│   └── amr_stm32.ioc
├── CLAUDE.md
└── README.md
```

> **Git:** repo chỉ còn **1 branch `main`** — branch `stm32-firmware` đã bị xóa.
> Firmware nằm tại `amr_ws/firmware/` (không còn repo riêng).

### Workflow 2 máy (Windows ↔ Jetson)

- **Windows**: STM32CubeIDE build/flash firmware từ `amr_ws/firmware/`
- **Windows**: Claude Code viết `.c`/`.h` trong `amr_ws/firmware/Core/`
- **Jetson**: Claude Code viết ROS2 nodes trong `amr_ws/src/`
- **Sync**: `git push`/`git pull` trên cả 2 máy (cùng branch `main`)

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

### ✅ Giai đoạn 1 — Nền tảng & Môi trường: HOÀN THÀNH
- [x] GitHub repo tạo xong, toàn bộ code push lên
- [x] SSH Windows → Jetson hoạt động (IP `192.168.22.105`, user `nmt`)
- [x] Ubuntu 22.04 + ROS2 Humble cài trên Jetson Orin Nano
- [x] `colcon build` → 7 packages finished, 0 errors
- [x] URDF mô tả robot (amr_description)
- [x] Kích thước xe thực tế đã đo và cập nhật vào code

### 🔧 Giai đoạn 2 — STM32 Firmware: ĐANG TRIỂN KHAI
- [x] **Firmware skeleton: HOÀN THÀNH** (motor_driver, servo_buslinker, jetson_comm)
  - Tạo project STM32CubeIDE, config I2C1 + USART1 + USART2
  - Driver I2C → Hiwonder Motor Driver (FIXED_SPEED, ENCODER_TOTAL) — `firmware/Core/Src/motor_driver.c`
  - Driver USART1 → HTS-20H, protocol BusLinker (LEN=7) — `firmware/Core/Src/servo_buslinker.c` (nối trực tiếp qua điện trở, xem bài học bên dưới)
  - Parser USART2: nhận lệnh từ Jetson, gửi encoder về — `firmware/Core/Src/jetson_comm.c`
- [x] **Test thực tế encoder: XONG** — encoder 2 bánh đếm đều, khớp nhau, đã verify qua bit-bang I2C (xem Hardware lessons)
- [x] **Protocol Jetson↔STM32: ASCII `$VEL` / `$ODO`** — đã viết lại xong, verify end-to-end (motor + encoder + servo cùng lúc, 0 lỗi)

**Hardware đã xác nhận thực tế:**
- USART1: PA9(TX)/PA10(RX) — không phải PB6/PB7 như thiết kế ban đầu
- I2C1: cần GPIO_PULLUP (không phải GPIO_NOPULL mặc định của CubeMX)
- Motor Driver: CH1=bánh trái, CH2=bánh phải (CH3/CH4 không dùng)
- Chiều motor: đã đảo dấu speed để tiến đúng hướng
- BusLinker protocol: LEN=7 (tính cả chính byte LEN), không phải LEN=6

**Hardware lessons (bài học phần cứng — tránh hỏng/treo bus):**
- **Motor driver EEPROM**: KHÔNG ghi quá 1 byte/register cho `0x14` (MOTOR_TYPE) và `0x15` (ENCODER_POLARITY) — ghi nhiều byte làm hỏng cấu hình EEPROM
- **Motor driver timeout**: driver tự dừng motor nếu >2s không nhận I2C command → cần gửi FIXED_SPEED định kỳ (watchdog)
- **Power cycle order**: GND trước → signal → 5V → 12V (sai thứ tự dễ chập/ngược dòng)
- **I2C BUSY**: xảy ra khi debugger ngắt giữa transaction → bus kẹt, phải power cycle để giải phóng

**⚠️ UART HAL Lock Deadlock (bài học firmware — RX chết âm thầm):**
- KHÔNG trộn `HAL_UART_Transmit` (blocking) với `HAL_UART_Receive_IT` trên cùng 1 UART
- Blocking TX giữ `huart->Lock` suốt thời gian gửi; nếu RX interrupt cần re-arm (`Receive_IT`) đúng lúc đó → nhận `HAL_BUSY` → RX không re-arm → **RX chết vĩnh viễn** (TX vẫn chạy bình thường, rất khó nhận ra)
- Triệu chứng thực tế: `$ODO` vẫn gửi đều nhưng `$VEL` không bao giờ được xử lý
- Giải pháp: TX dùng **raw register access** (`USART->DR` + poll `TXE`/`TC`), KHÔNG qua `HAL_UART_Transmit`
- Luôn có watchdog re-arm RX trong main loop: `if (huart->RxState == HAL_UART_STATE_READY)` → re-arm `Receive_IT`
- `HAL_UART_ErrorCallback` phải clear cờ ORE/FE (đọc SR rồi DR) + re-arm RX

**⚠️ I2C Bit-bang với Hiwonder Motor Driver (đọc encoder):**
- HW I2C (STM32 hardware peripheral) chỉ **GHI** được, **KHÔNG ĐỌC** được: slave NAK (AF, ec=0x04) mọi giao dịch read dù write OK — thử repeated-START, STOP-separated, hạ clock 20kHz đều fail. Nguyên nhân chưa rõ hẳn (nghi slave từ chối repeated-START / timing strict của HAL).
- **Giải pháp: bit-bang I2C thủ công qua GPIO** trên PB8(SCL)/PB9(SDA), với 4 điều kiện BẮT BUỘC:
  + **Open-drain thật** (`GPIO_MODE_OUTPUT_OD` + pull-up) — KHÔNG push-pull
  + **`HAL_I2C_DeInit(&hi2c1)`** trước khi chiếm GPIO làm bit-bang (nhả AF I2C1)
  + **Clock-stretch**: sau khi nhả SCL, poll chờ SCL thực sự lên HIGH mới tính là 1 xung
  + **`GPIO_SPEED_FREQ_LOW`** ⭐ QUAN TRỌNG NHẤT — `VERY_HIGH` gây ringing/overshoot trên dây jumper → slave loại cạnh → **NAK toàn bộ giao dịch**. Hạ `LOW` → cạnh sạch → hết NAK.
- KHÔNG cần tắt ngắt (thử rồi vô tác dụng — slew-rate mới là gốc; I2C cho phép master kéo dài clock khi ISR chen vào, nên để ngắt chạy bình thường cho UART RX).
- Đã verify end-to-end: `$VEL→$ODO` 100% tin cậy, encoder 2 bánh tăng đều khớp nhau, đọc voltage reg 0x00 = 11.6V.

**⚠️ Thay thế TTL Bus Servo Debugging Board bị hỏng (2026-06-xx):**
- Board debug (cầu nối TX/RX riêng của STM32 thành 1 dây SIG single-wire cho HTS-20H) bị hỏng, không mua kịp linh kiện thay thế.
- **Giải pháp: đấu thẳng STM32 ↔ servo qua 1 điện trở nối tiếp**, không cần sửa firmware:
  + `PA9(TX) --[R≈1kΩ]--> SIG servo` ; `PA10(RX) --(nối thẳng)--> SIG servo`
  + UART TX vốn idle ở mức HIGH (mark) → tự đóng vai trò "pull-up" cho single-wire, không cần điện trở kéo lên riêng
  + Điện trở chỉ để giới hạn dòng khi servo tự kéo SIG xuống thấp lúc TX đang ở mức cao (tránh đoản mạch)
  + VIN servo lấy trực tiếp từ cổng nguồn 12V của Motor Driver (cổng 2-pin, cùng rail với cổng 3-pin cắm pin) — an toàn vì HTS-20H rated 9.6–12.6V (datasheet), khớp đúng dải 12V hệ thống
- Đã verify: gửi `$VEL` với các `angular_z` khác nhau → servo xoay đúng theo `steer` tính toán trong `$ODO`, không cần sửa `servo_buslinker.c`/`usart.c`
- **Lưu ý nếu tái tạo mạch này**: R quá lớn (>~4.7kΩ) có thể làm tín hiệu yếu ở baud 115200; nếu servo không phản hồi, thử giảm R hoặc hạ `GPIO_SPEED_FREQ_VERY_HIGH` → `LOW` cho PA9/PA10 (bài học từ vụ I2C bit-bang ở trên)
- **Test tổ hợp (2026-07-02)**: gửi `$VEL` với `linear_x` VÀ `angular_z` khác 0 cùng lúc (motor chạy + servo đánh lái đồng thời) → encoder tăng đều 2 bánh khớp nhau, servo giữ đúng góc, **0 glitch** — xác nhận bit-bang I2C và UART1 servo không tranh chấp nhau trong cùng vòng loop 10ms. Firmware layer coi như đã chốt xong.

### 🔧 Giai đoạn 3 — ROS2 Hardware Nodes: ĐANG TRIỂN KHAI
- [x] `serial_driver_node` (`amr_hardware`) đã có sẵn khung ROS2 tốt: sub `/cmd_vel`, pub `/odom` + TF `odom→base_link`, công thức odometry differential-drive đúng, tham số khớp xe thật (`wheel_radius=0.10`, `wheel_base=0.21`, `ticks_per_rev=990`)
- [x] **`SerialDriver` đã viết lại sang ASCII line-based** (`serial_driver.hpp`/`stm32_comm.cpp`), khớp firmware `$VEL`/`$ODO`:
  + `sendCmdVel`: build `"$VEL,%.2f,%.2f\n"` rồi `write()`
  + `readOdom`: gom byte tới `'\n'`, `sscanf("$ODO,%ld,%ld,%f")`, bỏ state machine header/CRC nhị phân cũ
  + `OdomData.steering_pos` (uint16_t cũ) → `steer_deg` (float, độ, có dấu)
  + Thêm publisher `/steering_angle` (`std_msgs/Float32`, đơn vị độ) — topic đã khai trong mục 3 nhưng trước đó chưa implement
- [x] **Verify thực tế trên Jetson + STM32 thật (2026-07-02)**: `/odom` publish ổn định 20Hz, TF `odom→base_link` đúng, `/steering_angle` phản hồi đúng ngay lập tức khi đánh lái trái/phải/giữa (test bằng `ros2 topic pub` steering-only, `linear.x=0`), encoder tăng đúng khi motor chạy thật (test `linear.x=0.1` ~1s, bánh nhấc khỏi mặt đất)
- [ ] Driver Camera IMX-219 hoạt động (`/camera/image_raw`)
- [x] Driver LiDAR A1M8 hoạt động (`/scan`)

**Bug đã phát hiện + fix trong lúc verify (bài học cho lần sau):**
- `readOdom()` ban đầu chỉ đọc **1 dòng `$ODO` hợp lệ đầu tiên rồi return** mỗi lần gọi. Firmware gửi `$ODO` ở 100Hz nhưng node chỉ gọi `readOdom()` ở `publish_rate_hz` (mặc định 20Hz) → mỗi lần gọi có ~5 dòng tồn đọng trong kernel buffer, code chỉ tiêu thụ 1 dòng → backlog dồn lại, dữ liệu publish ra ngày càng trễ so với lệnh cmd_vel thực tế (quan sát được: đánh lái không phản hồi ngay, giá trị "đứng" ở lệnh vài bước trước).
- **Fix**: đọc HẾT byte đang có trong buffer mỗi lần gọi `readOdom()`, cập nhật `data` với dòng hợp lệ MỚI NHẤT tìm được (không return sớm ở dòng đầu tiên). Đã verify lại: phản hồi tức thời, không còn trễ.
- Bug này tồn tại y hệt trong cả code binary cũ (cũng return sớm ở frame CRC đầu tiên) — chỉ chưa bị phát hiện vì protocol cũ không tương thích firmware nên chưa từng chạy được để lộ lỗi.

**Bug thứ 2 phát hiện khi lắp lên khung xe thật (2026-07-03) + hiệu chuẩn odometry:**
- Khi khởi động node lần đầu trên khung xe đã lắp (bánh chạm đất), `/odom` nhảy vọt ngay `x=-669, y=-1057` thay vì `(0,0)`. Nguyên nhân: thanh ghi `ENCODER_TOTAL` (0x3C) trên mạch Hiwonder là bộ đếm **không tự reset** khi node ROS2 khởi động lại — giữ nguyên tổng tick tích lũy từ các lần test trước. Code cũ giả định tick ban đầu = 0 (`prev_left_/prev_right_ = 0`) nên delta lần đọc đầu tiên = tick tích lũy khổng lồ.
- **Fix**: `serial_driver_node.cpp` thêm cờ `got_first_odom_` — lần đọc `$ODO` hợp lệ đầu tiên chỉ dùng để set `prev_left_/prev_right_` làm baseline (không tính delta/publish), từ lần thứ 2 trở đi mới tính odometry bình thường. Đã verify: node khởi động lại nhiều lần đều cho `/odom` bắt đầu đúng `(0,0)`.
- **Hiệu chuẩn `ticks_per_rev` thực nghiệm**: xe chạy thẳng `linear.x=0.1` ~1s trên khung xe thật (bánh chạm đất), đo bằng thước dây thực tế **42.5cm**, trong khi `/odom` (dùng `ticks_per_rev=990` lý thuyết = 11 PPR × gear 90) tính ra **~3.36m** — sai lệch ~7.9 lần. Tính lại `ticks_per_rev` thật ≈ **7810** (→ `encoder_ppr` hiệu chỉnh = 7810/90 ≈ **86.78**, giữ `gear_ratio=90` cố định). Đã cập nhật `src/amr_hardware/launch/hardware.launch.py`.
- **Verify lại sau hiệu chuẩn**: lặp lại đúng test (`linear.x=0.1` ~1s) → `/odom` báo `x=0.427m` so với đo thật `42.5cm` — sai lệch chỉ ~0.5% (nằm trong sai số đo tay). Lệch trục y (~1cm sau 42cm) nghi do servo lái chưa canh giữa tuyệt đối (offset cơ khí), không phải lỗi công thức odometry.
- ⚠️ **Lưu ý**: mới hiệu chuẩn dựa trên **1 lần đo** ~42cm — nên đo lại thêm vài lần ở quãng đường dài hơn (1-2m) để tăng độ tin cậy trước khi dùng cho SLAM/Nav2 thật. `encoder_ppr=86.78` không còn phản ánh PPR vật lý thật của encoder (datasheet ghi 11) — đây là giá trị hiệu chỉnh thực nghiệm cho khớp hành vi thật của hệ (có thể do driver Hiwonder đếm quadrature khác giả định, hoặc gear ratio thật khác 90:1).

**Hiệu chuẩn `ticks_per_rev` — làm thêm 3 lần đo độc lập (2026-07-03, phiên sau)**: quãng ~0.34-0.5m/lần trên khung xe thật, so thước dây với `/odom`. `ticks_per_rev` thật tính ra ~7830/8036/7827 mỗi lần — độ lệch chuẩn giữa 3 lần chỉ ~1.24% (đáng tin cậy). Kết hợp có trọng số theo khoảng cách đo → **`ticks_per_rev` cuối = ~7885** (`encoder_ppr=87.61`, đã cập nhật `hardware.launch.py`). Verify lần cuối ở quãng ~1.465m: `/odom` báo 1.486m so với đo thật 1.465m — sai lệch ~1.4%, xác nhận calibration giữ vững cả ở quãng dài.

**Phát hiện quan trọng khi hiệu chuẩn servo**: công thức odometry hiện tại (kiểu differential-drive, dùng chênh lệch tick 2 bánh sau) **KHÔNG hề dùng góc lái servo** để tính hướng đi — vì `CALC_Ackermann` trong firmware luôn đặt tốc độ 2 bánh sau bằng nhau bất kể góc lái. Do đó độ lệch hướng thật (do servo lệch tâm) không phản ánh trong `y` của `/odom` — phải đo lệch hướng bằng mắt/thước, không dùng `/odom` làm căn cứ. Đây là gap kiến trúc cần vá sau này nếu muốn odometry chính xác khi vào cua (nằm ngoài phạm vi bug fix, ghi nhận để cân nhắc sau).

**Hiệu chuẩn servo lái (bù lệch tâm) — thực nghiệm (2026-07-03)**: xe lệch trái ~6.3cm/m khi `angular.z=0`. Thêm tham số `steering_trim_angular_z` (rad/s, cộng vào `angular.z` trước khi gửi `$VEL`) trong `serial_driver_node.cpp` — sửa ở phía Jetson, KHÔNG cần build lại firmware STM32. Dò bằng thực nghiệm nhiều điểm (`0` → lệch trái 6.3%/m; `+0.08` → lệch trái nhiều hơn; `-0.15` → lệch phải ~3%/m; `-0.10` → lệch phải ~4.4-4.7%/m, nhất quán ở cả quãng trung bình 52.8cm và quãng dài 146.5cm). Nội suy từ baseline + điểm đo dài đáng tin cậy nhất (146.5cm) → **`steering_trim_angular_z = -0.06`** (đã cập nhật `hardware.launch.py`). ⚠️ Chưa verify trực tiếp giá trị `-0.06` (hết không gian test 3m) — cần chạy xác nhận lại ở phiên sau, lý tưởng là quãng đường >2m để giảm nhiễu đo đạc (số liệu ở quãng <1m dao động khá nhiều, ví dụ cùng `trim=-0.10` cho 1.5%, 4.7%, 4.4% tùy quãng đo).

**Việc tiếp theo:** Verify trực tiếp `steering_trim_angular_z=-0.06` khi có không gian dài hơn (>2m). Cân nhắc vá odometry để phản ánh góc lái servo khi tính hướng đi (hiện chỉ đúng khi đi thẳng). Cân nhắc bù vi sai tốc độ trái/phải khi vào cua (hiện `CALC_Ackermann` trong firmware dùng tốc độ 2 bánh sau bằng nhau — ghi rõ trong `ackermann.h` là mô hình đơn giản hóa).

### 🔧 Giai đoạn 6 — Lane Detection: BẮT ĐẦU TRIỂN KHAI (2026-07-03)
- **Camera**: IMX-219 chưa về hàng → tạm dùng **webcam USB** (`/dev/video0`, encoding `yuv422_yuy2`). Cài `ros-humble-usb-cam`, thêm `src/amr_perception/launch/camera.launch.py` (node `usb_cam_node_exe`, remap `/image_raw` → `/camera/image_raw`, 640×480 @ 30fps). Vị trí lắp thật đã cập nhật vào `camera.xacro` (xem Giai đoạn 3).
- **`lane_detection_node.py`** (Canny+Hough, đã có sẵn từ lúc scaffold workspace, chưa từng chạy thử): verify end-to-end lần đầu — camera → node → `/lane_center_error` + `/lane_debug_image`, cả 2 topic publish đúng, không lỗi/crash. Test trong nhà (không có vạch làn thật) nên thuật toán bắt nhầm đường vân sàn gỗ làm "vạch trái" — đúng hành vi dự kiến, không phải bug, chỉ chưa có môi trường test thật.
- Thêm `usb_cam` vào `exec_depend` trong `amr_perception/package.xml`.
- **Tham khảo kiến trúc từ repo bạn** (`github.com/DangTinhPat/Ackerman_minCell`, nhánh `Lane_detect`): dùng model DNN (ONNX) thay Canny/Hough, RANSAC polyfit, quy đổi pixel→mét qua `transformer.py` (`e_y`, `e_psi`, `kappa`), Stanley controller + blend κ_vision/κ_odom qua EKF (`robot_localization`, encoder+IMU). Kiến trúc này xác nhận hướng thêm MPU6050+EKF là đúng, nhưng phức tạp hơn nhiều so với mục tiêu hiện tại — để tham khảo dần, không copy nguyên.

**Test với vạch làn thật (băng dính đen, 46cm × 3m) — 2026-07-04:**
- **Xác nhận dấu `/lane_center_error` bằng thực nghiệm tĩnh** (không cho xe chạy): đặt xe lệch trái tâm làn thật → đo được `error` âm — khớp docstring gốc (`âm=lệch trái`). Công thức `angular.z = +Kp * error` là ĐÚNG dấu (đã verify, không cần đảo dấu).
- **Viết `lane_follow_node.py`** (P controller đơn giản, `amr_perception/amr_perception/lane_follow_node.py`): sub `/lane_center_error` → pub `/cmd_vel`. Có watchdog dừng xe nếu mất `/lane_center_error` > `error_timeout` (0.5s).
- **Sửa `lane_detection_node.py`**: chỉ publish `/lane_center_error` khi **thực sự phát hiện được ≥1 vạch** (`detected` flag mới) — trước đó luôn publish kể cả khi không thấy gì (mặc định `error=0`), khiến xe chạy thẳng "mù quáng" khi mất dấu làn mà watchdog không phát hiện được (vì message vẫn đều đặn tới). Fix này để watchdog trong `lane_follow_node` hoạt động đúng.
- **Camera lắp lại lần 2** (2026-07-04, sau khi phát hiện lần 1 vạch không lọt khung hình vì làn 46cm rộng hơn FOV): cao **22.5cm**, lùi vào **1.5cm** so mép trước chassis, nghiêng xuống **~7.5°** (ước lượng, chưa đo chính xác). Đã cập nhật `camera.xacro`.
- **Phát hiện: đứng sát camera che khung hình** — người test đứng cạnh xe cầm dây theo dõi vô tình che gần hết 1 nửa khung hình camera, gây nhận diện làn chập chờn (log ghi nhận watchdog "mất dấu" kích hoạt 11 lần trong ~3 phút test). Cần đứng xa/lệch sang bên khi test, không đứng ngay trước/sát ống kính.
- **🔴 Phát hiện bug an toàn firmware nghiêm trọng khi test thật** — xem chi tiết đầy đủ ở mục "Vấn đề đang gặp" bên dưới (watchdog `$VEL` timeout, đã sửa code `main.c` nhưng CHƯA build/nạp).

**✅ Firmware watchdog: build/nạp + verify xong (2026-07-05)** — xe tự dừng chỉ 85ms sau khi mất kết nối phần mềm (đo bằng script Python theo dõi `/odom` chính xác theo thời gian). An toàn để tiếp tục test lái tự động.

**✅ Fix ROI mismatch — nguyên nhân thật của "mất dấu làn chập chờn" (2026-07-05):** ban đầu nghi do ánh sáng vàng, nhưng xem ảnh debug phát hiện **cả 2 vạch băng dính nằm ngoài/phía trên vùng ROI** (`roi_top_ratio=0.55` cũ không còn khớp sau khi camera đổi vị trí lần 2). Đổi `roi_top_ratio` → **0.35** trong `perception.launch.py`: `/lane_center_error` từ publish thất thường 4.7-10.9Hz (có khoảng trống tới 0.9s) → **ổn định 26.8Hz**, khớp gần đúng tốc độ camera. Không phải do ánh sáng hay che khung hình như nghi ban đầu — bài học: luôn xem ảnh debug (`/lane_debug_image`) để xác nhận ROI có bao trọn vạch làn trước khi đổ lỗi cho ánh sáng/thuật toán.

**Việc tiếp theo (Lane Detection):** Test `lane_follow_node` thật với ROI đã sửa (kỳ vọng ổn định hơn nhiều). Tune thêm `canny_low/high`, `hough_threshold` nếu cần. Khi IMX-219 về: đổi driver camera sang CSI thật, đo lại vị trí lắp (và `roi_top_ratio` theo đó), giữ nguyên topic `/camera/image_raw`.

### 🔧 Giai đoạn 4 — SLAM: ĐANG TRIỂN KHAI
- [x] Cài `ros-humble-slam-toolbox` (apt, 2026-05-14)
- [x] `amr_slam/config/slam_toolbox_params.yaml` — async mode, resolution 5cm, Ceres solver
- [x] `amr_slam/launch/slam.launch.py` — 4 node: serial_driver, sllidar, slam_toolbox, static_tf
  - LiDAR offset tạm thời: `base_link → laser_frame` x=0.15m, z=0.10m (cần đo lại sau khi lắp)
- [x] `colcon build --packages-select amr_slam` — 0 errors
- [ ] Chạy thực tế: `ros2 launch amr_slam slam.launch.py` và kiểm tra `/map`
- [ ] Lưu bản đồ `.yaml` + `.pgm` (`ros2 run nav2_map_server map_saver_cli`)

### ⏳ Giai đoạn 5 — Nav2: CHƯA BẮT ĐẦU
- [ ] AMCL localization trên bản đồ có sẵn
- [ ] Điều hướng tự động A → B

### Quyết định kỹ thuật đã chốt
- Motor driver: Hiwonder 4-Ch Encoder Motor Driver, giao tiếp I2C1 (PB8/PB9), GPIO_PULLUP
- Servo lái: HTS-20H, USART1 PA9(TX)/PA10(RX), 115200 baud — nối **trực tiếp** qua điện trở nối tiếp ~1kΩ (không qua debug board, đã hỏng)
- Jetson ↔ STM32: custom UART ASCII protocol, USART2 (PA2/PA3), 115200 baud
- Motor type register `0x14 = 3` (JGB37-520R90), gear ratio 90:1
- Motor channel: CH1=trái, CH2=phải (xác nhận bằng thực nghiệm)
- BusLinker protocol LEN=7: LEN tính từ chính nó đến CHECKSUM (LEN+CMD+DATA+CHK)
- Kích thước xe: L=304mm, W=268mm, H=84mm, wheelbase=210mm, track=217mm, r_wheel=100mm

### Lỗi đã gặp và fix (tránh lặp lại)
1. **CubeMX Clock**: Nucleo-F446RE dùng HSI (không có HSE crystal) → ấn OK khi Clock Wizard hỏi về HSE
2. **I2C GPIO Pull**: phải đổi `GPIO_NOPULL` → `GPIO_PULLUP` trong `i2c.c` sau mỗi lần CubeMX generate lại
3. **USART1 pins**: PA9/PA10 hoạt động; PB6/PB7 không ra signal trên board Nucleo-64 này
4. **Motor channel**: CH1=trái, CH2=phải — xác nhận bằng thực nghiệm, không phải theo thứ tự vật lý
5. **BusLinker LEN=7**: LEN tính cả chính nó — dùng LEN=6 servo im lặng hoàn toàn, không có NAK
6. **Debug/ folder**: phải có trong `.gitignore` (STM32CubeIDE build artifacts)
7. **TTL Bus Servo Debugging Board hỏng**: thay bằng đấu trực tiếp STM32↔servo qua điện trở nối tiếp (xem Giai đoạn 2) — không cần sửa firmware, chỉ đổi dây

### Vấn đề đang gặp

**🔴 NGHIÊM TRỌNG — Mạch Motor Driver Hiwonder ĐÃ CHÁY (2026-07-05):**

Sau nhiều lần motor không phản hồi qua ROS (servo vẫn OK, chỉ motor im lặng — xem lịch sử debug bên dưới), user phát hiện chip trên mạch Motor Driver bị nóng, tháo tản nhiệt ra thì **chip cháy ngay lập tức**. Đây là hỏng phần cứng vĩnh viễn, KHÔNG phải lỗi phần mềm/node kẹt như nghi ngờ trước đó.

**Lịch sử debug dẫn tới phát hiện này** (để tránh hiểu nhầm lại là lỗi phần mềm):
- 2026-07-04: motor không phản hồi khi test `lane_follow_node` lần đầu → nghi driver hỏng do quá nhiệt (đèn báo tắt) → rút nguồn để nguội → sau đó test lại thấy chạy được qua serial thô + restart `serial_driver_node` sạch → tưởng đã khỏi (chỉ là node ROS bị kẹt state, không phải hỏng thật)
- 2026-07-05: build+nạp firmware watchdog xong, verify OK (dừng trong 85ms). Nhưng khi test lại `lane_follow_node` thật, motor lại không phản hồi (servo vẫn OK) — lần này restart `serial_driver_node` sạch KHÔNG khắc phục được (khác hẳn mẫu hình hôm trước) → đúng lúc này phát hiện chip cháy khi tháo tản nhiệt.

**Kết luận**: mạch Motor Driver Hiwonder cần **thay mới** — không sửa được bằng phần mềm. Việc quá nhiệt lặp lại nhiều lần (dù có tản nhiệt) gợi ý có thể motor bị quá tải liên tục (kẹt cơ khí, tải nặng hơn thiết kế, hoặc driver đã suy yếu từ lần quá nhiệt đầu 2026-07-04) — cân nhắc kiểm tra kỹ tải cơ khí trước khi lắp driver mới.

**✅ Đã hoàn thành trước khi phát hiện hỏng (vẫn còn giá trị, không cần làm lại khi có driver mới):**
- Firmware watchdog `$VEL` timeout: build/nạp + verify xong (dừng trong 85ms khi mất kết nối)
- Lane detection: sửa `roi_top_ratio` (0.35) + `canny_low/high` (20/60) khớp vị trí camera + ánh sáng hiện tại, `/lane_center_error` ổn định 26.8Hz — xem Giai đoạn 6

**Việc tiếp theo**: build/nạp firmware, verify watchdog hoạt động đúng, rồi mới tiếp tục test `lane_follow_node` thật. Khi test lại, đứng xa/lệch sang bên thay vì đứng sát ống kính camera (đã phát hiện: đứng sát che khung hình gây nhận diện làn chập chờn).

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
