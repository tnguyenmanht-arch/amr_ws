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

**Việc tiếp theo:** Test Camera IMX-219 (`/camera/image_raw`). Cân nhắc bù vi sai tốc độ trái/phải khi vào cua (hiện `CALC_Ackermann` trong firmware dùng tốc độ 2 bánh sau bằng nhau — ghi rõ trong `ackermann.h` là mô hình đơn giản hóa).

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

### ⏳ Giai đoạn 6 — Lane Detection & Parking: CHƯA BẮT ĐẦU
- [ ] Lane detection cơ bản (Canny + Hough)
- [ ] Lane keeping controller (PID)
- [ ] Parking state machine

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
_(không có)_

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
