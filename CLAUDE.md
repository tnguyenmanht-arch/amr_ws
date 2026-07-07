# CLAUDE.md — AMR Ackermann Robot | HUST Automotive Engineering

> Dự án môn học — Chương trình Kỹ sư chuyên sâu Ô tô số (Automotive Digital Engineering)
> Đại học Bách Khoa Hà Nội (HUST) · Việt Nam

---

## 1. Tổng quan dự án

Robot AMR 4 bánh dẫn động Ackermann có khả năng tự định vị, lập bản đồ và điều hướng thông minh trong môi trường trong nhà. Mục tiêu cuối: bám làn, phân làn và đỗ xe đơn giản.

**Kiến trúc hệ thống:**

```
[Jetson Orin Nano Super 8GB] ←→ UART/USB ←→ [STM32F411CEU6 "Black Pill"]
        (ROS2 Master)                              (Low-level Slave)
        ↑            ↑                              ↑                    ↑
  IMX-219 Camera  RPLidar A1M8        [BTS7960 x2 (1 board/motor)]   USART1 (single-wire)
  (lane detect)   (SLAM/Nav)          PWM (TIM3) + Encoder (TIM2/TIM4) ↓  qua điện trở nối tiếp ↓
                                     JGB37-520 Motors              HTS-20H Servo (ID=9)
                                      (drive wheels)                (steering)
```
> Servo lái nối **thẳng** vào STM32 (không qua TTL Bus Servo Debugging Board — board này đã hỏng, xem mục 8).
> Motor driver **Hiwonder 4-Ch (I2C) đã cháy 2026-07-05, thay bằng BTS7960 x2 (PWM+DIR)** — encoder đấu thẳng vào STM32 qua TIM hardware Encoder Mode, không còn I2C/bit-bang. Xem mục 8.
> **STM32 slave đã đổi từ F446RE Nucleo-64 sang F411CEU6 "Black Pill" (2026-07-07)** — 2 board F446RE liên tiếp hỏng (nghi do stress điện áp/rò AC tích lũy qua nhiều giờ test), chuyển hẳn sang F411 rời + ST-Link ngoài. Firmware nằm ở `amr_stm32f411/` (project `firmware/` cũ giữ lại làm tham khảo lịch sử/board F446, không còn build/nạp). Xem mục 8 "Migration F446 → F411".

---

## 2. Hardware — Thông số kỹ thuật

### Master: Jetson Orin Nano Super Dev Kit 8GB
- OS: Ubuntu 22.04 LTS (JetPack 6.x)
- ROS2 Distribution: Humble Hawksbill
- GPU: Ampere, 1024 CUDA cores (dùng cho vision inference nếu cần)
- Giao tiếp với slave: UART hoặc USB-Serial
- Thư viện NVIDIA có thể dùng: Isaac ROS, DeepStream, TensorRT (nếu phần cứng đủ điều kiện)

### Slave: STM32F411CEU6 "Black Pill" (rời, không phải Nucleo)
- Firmware: STM32CubeIDE, project tại `amr_stm32f411/` — **project mới tạo từ đầu** (không phải generate lại từ F446), xem bài học NVIC ở mục 8
- Nạp/debug: **ST-Link V2 rời** (không tích hợp trên board như Nucleo) — cắm SWCLK/SWDIO/GND + nguồn (3.3V từ ST-Link hoặc 5V ngoài) vào header SWD của Black Pill
- **Không có cổng UART ảo tích hợp** (khác Nucleo) — khi cần test `$VEL`/`$ODO` qua máy tính (thay vì Jetson), phải dùng thêm **module USB-to-TTL CH340 rời** nối PA2/PA3
- Nhiệm vụ: nhận cmd_vel → điều khiển motor (PWM+DIR qua BTS7960), đọc encoder (TIM hardware Encoder Mode), điều khiển servo qua single-wire trực tiếp
- Giao thức với master: custom UART protocol qua USART2
- Clock: **HSI nội bộ** (không cần thạch anh ngoài), PLL lên **100MHz** (PLLM=8, PLLN=100, PLLP=DIV2, PLLQ=4)

**Pin assignments đã xác nhận:**
| Peripheral | Pins | Kết nối |
|---|---|---|
| TIM3 CH1-CH4 | PA6, PA7, PB0, PB1 | PWM: RPWM/LPWM trái (PA6/PA7), RPWM/LPWM phải (PB0/PB1) → BTS7960 x2, 20kHz (ARR=4999 ở 100MHz timer clock — **khác 4499 của F446**, tính lại theo clock mới) |
| TIM2 (Encoder Mode TI12) | PA0, PA1 | Encoder trái (A/B), 32-bit counter — đấu thẳng vào STM32, không qua BTS7960 |
| TIM4 (Encoder Mode TI12) | PB6, PB7 | Encoder phải (A/B), 16-bit counter (cộng dồn tràn số trong `motor_driver.c`) — **đổi từ TIM8/PC6-PC7 của F446** vì F411 không có TIM8 |
| USART2 | PA2 (TX), PA3 (RX) | Jetson Orin Nano, 115200 baud — lúc test bàn dùng CH340 rời (TX↔PA3, RX↔PA2, GND chung) |
| USART1 | PA9 (TX), PA10 (RX) | PA9 qua điện trở ~1kΩ + PA10 nối thẳng → dây SIG của HTS-20H (single-wire half-duplex, không qua debug board) |

> I2C1 **không dùng tới** trên project F411 (project mới tạo từ đầu, không bật I2C ngay từ lúc cấu hình CubeMX, khác F446 phải tắt thủ công).

### Cơ cấu chấp hành
| Thiết bị | Model | Giao tiếp | Ghi chú |
|---|---|---|---|
| Drive Motor (×2) | JGB37-520 DC w/ Encoder | PWM (TIM3) + Encoder (TIM2/TIM8) | 12V, dòng stall ~2.3A, gear ratio 90:1 |
| Steering Servo | HTS-20H Serial Bus Servo | Serial Bus (TTL, single-wire) | Góc lái Ackermann; STM32 nối **thẳng** qua điện trở nối tiếp (xem mục 8). **`SERVO_ID=9`** (servo hiện tại, không phải mặc định 1 — servo cũ khả năng hỏng, đổi sang con khác lúc migration F411) |
| Motor Driver (×2, 1/motor) | BTS7960 43A module | PWM (RPWM/LPWM) + DIR (R_EN/L_EN) | Thay Hiwonder đã cháy — margin dòng rộng (~10A liên tục an toàn vs 2.3A stall). R_EN/L_EN **PHẢI nối 5V** (không phải 3.3V — xem mục 8) |
| ~~Motor Driver Hiwonder~~ | ~~4-Ch Encoder Motor Driver~~ | ~~I2C~~ | **ĐÃ CHÁY 2026-07-05, không dùng nữa** — xem mục 8 |
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
├── firmware/               ← STM32 firmware CŨ (F446RE, đã hỏng — giữ tham khảo lịch sử, KHÔNG build/nạp nữa)
│   ├── Core/Src/
│   ├── Core/Inc/
│   ├── Drivers/
│   └── amr_stm32.ioc
├── amr_stm32f411/          ← STM32 firmware HIỆN TẠI (F411 Black Pill, build trên Windows)
│   ├── Core/Src/           ← Application code
│   ├── Core/Inc/
│   ├── Drivers/
│   └── amr_stm32f411.ioc
├── docs/
│   └── wiring-f411.html    ← Sơ đồ đấu dây đầy đủ F411 (pin table, star ground, phân phối nguồn)
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

### STM32 build/flash (Windows, F411 hiện tại)

```bash
# Build headless (workspace đã import sẵn project amr_stm32f411)
"C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/stm32cubeidec.exe" --launcher.suppressErrors -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data "c:/Users/admin/STM32CubeIDE/workspace_2.1.1" \
  -cleanBuild amr_stm32f411

# Nạp qua ST-Link rời (không phải ST-Link tích hợp Nucleo)
"C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506/tools/bin/STM32_Programmer_CLI.exe" \
  -c port=SWD -w "c:/Users/admin/Documents/amr_ws/amr_stm32f411/Debug/amr_stm32f411.elf" -v -rst

# Kiểm tra ST-Link + cổng UART (CH340) đang nhận diện
STM32_Programmer_CLI.exe -l

# Đọc thanh ghi qua SWD để chẩn đoán "không phản hồi" (xem mục 8, bug NVIC)
STM32_Programmer_CLI.exe -c port=SWD -r32 0x4000440C 1   # USART2->CR1
STM32_Programmer_CLI.exe -c port=SWD -r32 0x40023830 1   # RCC->AHB1ENR
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

> ⚠️ **Toàn bộ phần I2C bit-bang ở trên đã LỖI THỜI kể từ 2026-07-05** (Hiwonder driver cháy, thay bằng BTS7960 — xem "Migration sang BTS7960" bên dưới). Giữ lại để tham khảo lịch sử/bài học bit-bang I2C nói chung, không còn áp dụng cho `motor_driver.c` hiện tại.

**🔧 Migration sang BTS7960 (PWM+DIR) — thay thế hoàn toàn I2C (2026-07-06):**

Sau khi Hiwonder driver cháy (xem "Vấn đề đang gặp"), viết lại toàn bộ `motor_driver.c`/`.h` sang kiến trúc PWM+Encoder-hardware, **API công khai giữ nguyên** (`DRV_Motor_Init/SetSpeed/GetEncoder/ResetEncoder`) nên `main.c`/`ackermann.c`/`jetson_comm.c` không cần sửa dòng nào:
- **PWM**: TIM3 4 kênh (CH1-CH4 = PA6/PA7/PB0/PB1), Clock Source=Internal, Slave Mode=Disable, Prescaler=0, **Counter Period=4499** (→ PWM 20kHz với timer clock 90MHz — lưu ý CubeMX mặc định để Period=65535, PHẢI tự đổi tay, dễ quên)
- **Encoder**: TIM2 (trái, 32-bit, EncoderMode=**TI12** không phải TI1) + TIM8 (phải, 16-bit, cộng dồn tràn số bằng delta 16-bit trong `DRV_Motor_GetEncoder`) — đấu thẳng 2 dây A/B của JGB37-520 vào STM32, KHÔNG qua BTS7960, loại bỏ hoàn toàn I2C bit-bang
- Build headless: `stm32cubeidec.exe -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data <workspace> -cleanBuild amr_stm32`; flash: `STM32_Programmer_CLI.exe -c port=SWD -w <elf> -v -rst`

**Bài học phần cứng khi đấu BTS7960 (rất tốn thời gian debug, ghi lại tránh lặp):**
1. **R_EN/L_EN của BTS7960 phải nối 5V, KHÔNG phải 3.3V** — dù datasheet ghi chấp nhận 3.3-5V, nhiều board clone dùng opto-coupler có điện trở giới hạn dòng tính sẵn cho 5V; cấp 3.3V dòng qua LED opto không đủ để board nhận chắc chắn là mức HIGH. Nối chung với VCC (5V) của board là an toàn nhất.
2. **Dây header/dupont KHÔNG đáng tin cho đường 12V/PWM công suất** — gặp liên tiếp 2 lần tiếp xúc lỏng (đường 12V B+/B- sụt từ 11V xuống <1V; đường RPWM/PA6 tiếp xúc chập chờn khiến đo ra "có áp rồi tụt về 0"). Nên hàn trực tiếp hoặc dùng terminal vít cho đường công suất, header rời chỉ nên dùng cho tín hiệu logic dòng thấp.
3. **⭐ Watchdog `$VEL` 300ms (đã có từ trước) làm sai lệch MỌI script test "giữ lệnh"**: nếu script chỉ gửi `$VEL` một lần rồi chờ đo/quan sát nhiều giây, motor chỉ chạy thật 300ms đầu rồi tự dừng do watchdog — toàn bộ phép đo sau đó (multimeter, LED, encoder) đều rơi vào giai đoạn đã dừng, dễ kết luận nhầm "phần cứng hỏng". **Bắt buộc gửi lại `$VEL` liên tục (chu kỳ <300ms, ví dụ mỗi 100ms) trong suốt thời gian test/đo.**
4. **Encoder VCC nên dùng 3.3V (không phải 5V)** dù JGB37-520 chấp nhận 3.3-5V — vì tín hiệu ra A/B sẽ dao động theo đúng mức VCC cấp, cấp 5V có thể vượt ngưỡng chịu đựng của 1 số chân STM32F4 không 5V-tolerant. Đây là lựa chọn chủ động, không phải thiếu sót.
5. **2 motor lắp đối xứng trên khung xe → cùng 1 lệnh tốc độ sẽ làm 2 trục quay ngược chiều nhau** (encoder trái dương, phải âm cùng lúc) — không phải lỗi, cần đảo dấu 1 bên trong `DRV_Motor_SetSpeed()` (đã làm cho bánh phải: `set_channel_speed((int8_t)(-right), ...)`) để cả 2 bánh cùng đẩy xe đi thẳng.
6. **Cách kiểm tra encoder còn sống độc lập với motor/driver**: cấp đúng VCC/GND, dùng tay xoay trục, đo điện áp DC tại chân A/B — phải thấy nhảy giữa 0V và VCC. Cách kiểm tra motor còn sống độc lập với driver: tháo dây động lực, chạm thẳng vào pin AA/9V (không cần điện trở, không nối qua STM32/BTS7960) trong 1-2s.
7. Board Hiwonder cũ dùng chung 1 board cho 2 motor (cross-talk nhiệt góp phần gây cháy); BTS7960 x2 (1 board/motor) giảm rủi ro này, nhưng margin dòng (dòng chịu tải >> dòng stall thực tế) mới là yếu tố quyết định, không phải việc tách board.

**🔴 2 board STM32F446RE Nucleo-64 liên tiếp hỏng (2026-07-07) → chuyển hẳn sang F411CEU6 "Black Pill":**

Trong lúc test BTS7960 mới lắp, board Nucleo đầu tiên **nóng bất thường** dù chỉ cấp USB (không nối motor/12V) — dấu hiệu dòng bất thường chạy qua chip, không phải nhiệt CPU bình thường. Board thứ 2 (sau khi mượn/tháo phần ST-Link từ board 1) cũng lặp lại đúng hiện tượng. Không có dấu hiệu cháy/khét nhìn thấy được — hỏng kiểu suy yếu bán dẫn nội bộ, không phải đoản mạch lộ liễu.

**Nghi ngờ nguyên nhân gốc (không khẳng định tuyệt đối, nhiều yếu tố cộng dồn qua nhiều giờ test)**:
- **Dòng rò AC từ sạc laptop 2 chân (không tiếp đất)** — xác nhận thực nghiệm bằng cách chạm tay vào board thấy "tê tê", rút sạc chạy pin thì hết tê. Suốt buổi test, tay người dùng chạm nhiều điểm mass khác nhau (que đo, dây pin 12V, dây STM32) cùng lúc — nếu các mass đó không thông nhau hoàn toàn (đã xảy ra nhiều lần do dây lỏng), dòng rò AC có đường chạy qua GPIO/GND của STM32 liên tục nhiều giờ.
- GND của STM32 từng đấu **xuyên qua chân GND của board BTS7960** (nơi dòng motor lớn chạy qua) thay vì có nhánh riêng — đúng kiểu "ground bounce" kinh điển, dòng lớn tạo sụt áp trên đường GND chung khiến STM32 "nhìn nhầm" mức 0V.

**Bài học phòng ngừa cho lần sau (đã áp dụng ngay khi lắp F411, xem `docs/wiring-f411.html`)**:
- **Rút sạc laptop, chạy pin** khi thao tác/đo đạc board hở mạch trong thời gian dài
- **Nối GND kiểu star qua 1 thanh terminal block riêng** — mỗi thiết bị (STM32, servo, 2× BTS7960, 2× encoder) có dây GND riêng về thanh terminal, rồi CHỈ 1 dây từ thanh đó ra cực (–) pin. GND của STM32 KHÔNG BAO GIỜ đi xuyên qua chân GND của board công suất nào khác.
- Test/nạp/reset theo từng bước nhỏ, không ráp hết hệ thống rồi cấp điện 1 lần

**Bài học quan trọng khi debug — đừng vội kết luận "chip hỏng"**: trong lúc debug lỗi USART2 RX không nhận được `$VEL` (xem bên dưới), từng nghi oan chip F411 mới cũng hỏng (dựa vào `RCC AHB1ENR`/`USART2 CR1` đọc ra 0x00000000 qua SWD, không chạy qua nổi `SystemClock_Config()`). Hóa ra đó là do **soft reset qua ST-Link không đủ để gỡ trạng thái kẹt** — chỉ power-cycle thật sự (rút/cắm lại ST-Link) mới khôi phục được. Kết luận "hỏng" chỉ nên đưa ra sau khi đã thử power-cycle hoàn toàn, không chỉ soft reset.

**🔧 Migration firmware F446 → F411 (2026-07-07):**

Tạo project CubeMX **hoàn toàn mới** (`amr_stm32f411/`, không generate lại từ F446) vì khác họ chip con nhưng cùng dòng F4/HAL — copy nguyên `ackermann.c/h`, `jetson_comm.c/h`, `servo_buslinker.c/h` (không đổi gì), chỉ sửa `motor_driver.c/h` (TIM8→TIM4) và merge logic ứng dụng vào `main.c` mới generate.

**Khác biệt so với F446 cần lưu ý:**
1. **F411 không có TIM8** — encoder phải chuyển từ TIM8(PC6/PC7) sang **TIM4(PB6/PB7)**, vẫn 16-bit nên logic cộng dồn tràn số trong `motor_driver.c` giữ nguyên
2. **PWM ARR đổi từ 4499 → 4999** vì F411 chạy 100MHz (F446 chạy 90MHz cho APB1 timer) — cùng cho ra 20kHz nhưng số ARR khác, phải tính lại theo clock thực tế của chip mới, không copy nguyên số cũ
3. **Không cần đảo dấu bánh phải nữa** — F446 cũ cần `set_channel_speed((int8_t)(-right), ...)` vì quy ước dây M+/M-/encoder A-B lúc đó; F411 đấu lại dây từ đầu nên quy ước khác, đo thực nghiệm cho thấy **không đảo dấu mới đúng** (`enc_l`/`enc_r` cùng dấu khi tiến). Luôn đo lại thực nghiệm sau khi đấu dây lại từ đầu, không giả định giữ nguyên logic đảo dấu cũ.
4. **Project CubeMX mới không tự sinh `tim.c`/`usart.c` riêng** như F446 (tùy theo tùy chọn "Generate peripheral initialization as pair of .c/.h" trong Project Manager) — handle `htim2/htim3/htim4/huart1/huart2` khai báo trực tiếp trong `main.c`, phải tự thêm `extern` cho các handle này vào `main.h` (mục "USER CODE BEGIN ET") để `motor_driver.c`, `jetson_comm.c`, `servo_buslinker.c` include `"main.h"` dùng được (thay vì `"tim.h"`/`"usart.h"` như F446).

**⭐ Bug nghiêm trọng nhất — thiếu cấu hình NVIC cho USART2 (nguyên nhân thật của nghi ngờ "chip hỏng"):**

Sau khi port xong, `$ODO` gửi từ STM32 lên máy tính hoạt động hoàn hảo, nhưng `$VEL` gửi xuống **không bao giờ được xử lý** (`steer`/encoder không đổi dù gửi lệnh liên tục hàng trăm lần) — triệu chứng giống hệt lỗi dây/nguồn nên đã đi vòng qua rất nhiều bước debug sai hướng (nghi dây CH340 lỏng, nghi nguồn nhiễu, nghi chip hỏng) trước khi tìm ra gốc rễ thật.

- **Nguyên nhân**: project CubeMX mới cho F411 **không tự động bật ngắt (NVIC) cho USART2** — khác F446 cũ (project đó có sẵn `NVIC.USART2_IRQn=true...` trong `.ioc` từ trước). Thiếu 2 việc: (1) `HAL_NVIC_EnableIRQ(USART2_IRQn)` trong `HAL_UART_MspInit`, (2) hàm `USART2_IRQHandler()` gọi `HAL_UART_IRQHandler(&huart2)` trong `stm32f4xx_it.c`.
- Vì thiếu ngắt, `HAL_UART_Receive_IT()` gọi trong `APP_Comm_Init()` không bao giờ trigger `HAL_UART_RxCpltCallback` — byte nhận được nằm im trong thanh ghi phần cứng, CPU không bao giờ đọc ra. TX vẫn hoạt động bình thường vì code gửi (`uart2_tx_raw`) dùng polling thanh ghi trực tiếp, không phụ thuộc ngắt.
- **Cách phát hiện**: đọc trực tiếp thanh ghi `USART2->CR1` (địa chỉ `0x4000440C`) qua SWD (`STM32_Programmer_CLI.exe -c port=SWD -r32 0x4000440C 1`) — nếu ra `0x00000000` thì USART2 chưa hề được cấu hình/chạy qua, đây là cách chẩn đoán khách quan không cần đoán mò dây/nguồn.
- **Fix**: thêm 2 dòng NVIC vào `HAL_UART_MspInit` (USER CODE section của `stm32f4xx_hal_msp.c`) + thêm `USART2_IRQHandler` vào `stm32f4xx_it.c`. Build/nạp lại → `$VEL` xử lý ngay lập tức.
- **Bài học khi tạo project CubeMX mới từ đầu (không phải generate lại/copy)**: luôn kiểm tra tab **NVIC Settings** của từng peripheral dùng ngắt (đặc biệt USART/UART dùng `HAL_xxx_Receive_IT`) — CubeMX **không tự động bật** ngắt chỉ vì đã cấu hình chân/mode, đây là bước riêng dễ bỏ sót khi không dùng lại project cũ.

Kết luận: khi gặp "không phản hồi" mà lỗi rất giống hỏng phần cứng, luôn kiểm tra thanh ghi NVIC/peripheral qua SWD **trước khi** kết luận hỏng chip — rẻ và nhanh hơn nhiều so với nghi oan và mua board mới.

**Calibration servo mới (2026-07-07)**: servo cũ (ID=1) nghi hỏng trong lúc test (đo VIN chỉ ra 2V dù đã cấp đúng 12V) → đổi sang servo khác đã có sẵn, **ID=9** — cập nhật `SERVO_ID` trong `servo_buslinker.h`. Thêm hằng số `ACK_STEER_TRIM_DEG=1.5f` trong `ackermann.h`/`.c` (cộng vào `steer_deg` trước khi clamp) để bù lệch cơ khí lúc lắp servo mới — xác định bằng cách quan sát trực tiếp bánh lái, không phải tính toán lý thuyết.

**Đã verify đầy đủ trên F411 (2026-07-07)**: giao tiếp `$VEL`/`$ODO` qua CH340↔USART2, 2 motor chạy đồng bộ đúng chiều (không cần đảo dấu), servo ID=9 phản hồi đúng góc kèm trim, test tổ hợp tiến/lùi/lái đồng thời — encoder trái/phải khớp nhau ở cả 2 chiều.

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
- **STM32 slave: F411CEU6 "Black Pill" rời + ST-Link ngoài** (không phải Nucleo tích hợp) — thay F446RE sau khi 2 board liên tiếp hỏng 2026-07-07. Firmware tại `amr_stm32f411/`.
- Motor driver: **BTS7960 x2 (1 board/motor)**, điều khiển PWM (RPWM/LPWM qua TIM3, 20kHz, ARR=4999 ở 100MHz) + DIR (R_EN/L_EN nối cứng 5V) — thay Hiwonder I2C đã cháy 2026-07-05
- Encoder: đấu thẳng vào STM32 qua TIM Encoder Mode (TIM2 trái/32-bit, **TIM4 phải/16-bit** — đổi từ TIM8 vì F411 không có), VCC encoder dùng 3.3V (không phải 5V — an toàn cho GPIO STM32), không còn qua I2C
- Servo lái: HTS-20H (**ID=9**), USART1 PA9(TX)/PA10(RX), 115200 baud — nối **trực tiếp** qua điện trở nối tiếp ~1kΩ (không qua debug board, đã hỏng), trim `+1.5°` bù lệch cơ khí
- Jetson ↔ STM32: custom UART ASCII protocol, USART2 (PA2/PA3), 115200 baud
- GND nối kiểu **star qua thanh terminal block riêng** (không daisy-chain qua chân board công suất) — bắt buộc từ sau sự cố hỏng 2 board F446, xem `docs/wiring-f411.html`
- Gear ratio motor: 90:1, dòng stall thực tế ~2.3A (quan trọng khi chọn driver thay thế)
- Chiều motor: **KHÔNG đảo dấu** trong `DRV_Motor_SetSpeed()` trên F411 (khác F446 — quy ước dây khác do đấu lại từ đầu)
- Kích thước xe: L=304mm, W=268mm, H=84mm, wheelbase=210mm, track=217mm, r_wheel=100mm

> BusLinker protocol LEN=7, MOTOR_TYPE register `0x14=3`, motor channel CH1/CH2 — các mục này thuộc kiến trúc I2C/Hiwonder cũ, không còn áp dụng nhưng giữ lại trong lịch sử "I2C Bit-bang" (Giai đoạn 2) để tham khảo nếu quay lại driver kiểu register-based sau này.

### Lỗi đã gặp và fix (tránh lặp lại)
1. **CubeMX Clock**: Nucleo-F446RE dùng HSI (không có HSE crystal) → ấn OK khi Clock Wizard hỏi về HSE
2. **I2C GPIO Pull**: phải đổi `GPIO_NOPULL` → `GPIO_PULLUP` trong `i2c.c` sau mỗi lần CubeMX generate lại
3. **USART1 pins**: PA9/PA10 hoạt động; PB6/PB7 không ra signal trên board Nucleo-64 này
4. **Motor channel**: CH1=trái, CH2=phải — xác nhận bằng thực nghiệm, không phải theo thứ tự vật lý
5. **BusLinker LEN=7**: LEN tính cả chính nó — dùng LEN=6 servo im lặng hoàn toàn, không có NAK
6. **Debug/ folder**: phải có trong `.gitignore` (STM32CubeIDE build artifacts)
7. **TTL Bus Servo Debugging Board hỏng**: thay bằng đấu trực tiếp STM32↔servo qua điện trở nối tiếp (xem Giai đoạn 2) — không cần sửa firmware, chỉ đổi dây
8. **Tạo project CubeMX mới từ đầu KHÔNG tự bật NVIC cho ngắt UART** (khác project generate lại/copy) — phải tự thêm `HAL_NVIC_EnableIRQ()` + `USART2_IRQHandler()`, nếu không `HAL_UART_Receive_IT()` không bao giờ trigger callback dù dây/nguồn đều đúng (xem "Migration firmware F446 → F411")
9. **Đọc thanh ghi qua SWD (`STM32_Programmer_CLI -r32`) để chẩn đoán "không phản hồi"** trước khi kết luận hỏng chip — `RCC->AHB1ENR`/`USARTx->CR1` = 0 nghĩa là code chưa chạy qua init đó, thường là bug cấu hình chứ không phải phần cứng chết

### Vấn đề đang gặp

Không có vấn đề nghiêm trọng nào đang mở — xem "✅ Đã giải quyết gần đây" bên dưới cho lịch sử.

**Việc tiếp theo**: sang Jetson tiếp tục test end-to-end (`serial_driver_node` ↔ firmware F411 mới qua USART2 PA2/PA3 thật, không phải CH340 test bàn). Sau đó hạ bánh xuống đất, test di chuyển thẳng ngắn + servo lái thực tế, rồi mới tiếp tục test `lane_follow_node` thật (đứng xa/lệch sang bên khi test, không đứng sát ống kính camera — bài học cũ vẫn còn giá trị). Cân nhắc verify lại calibration `ticks_per_rev`/`steering_trim_angular_z` phía Jetson vì đã đổi cả STM32 lẫn servo — số cũ (đo trên F446 + servo ID=1) chưa chắc còn đúng.

### ✅ Đã giải quyết gần đây

**2 board STM32F446RE Nucleo hỏng liên tiếp (2026-07-07) → chuyển sang F411CEU6 "Black Pill", verify OK cùng ngày:**

Board nóng bất thường dù chỉ cấp USB (không nối motor/12V) — nghi do dòng rò AC từ sạc laptop 2 chân + GND từng đấu xuyên qua chân board BTS7960 (ground bounce), tích lũy qua nhiều giờ test. Đã chuyển hẳn sang F411 + ST-Link rời, áp dụng GND kiểu star qua thanh terminal riêng. Trong lúc port firmware, phát hiện + sửa bug **thiếu cấu hình NVIC cho USART2** (nguyên nhân thật khiến `$VEL` không xử lý được, từng nghi oan là chip hỏng lần 3) — chi tiết đầy đủ xem "Migration firmware F446 → F411" trong Giai đoạn 2 ở trên.

**Đã verify đầy đủ trên F411 (2026-07-07)**: giao tiếp `$VEL`/`$ODO`, 2 motor đồng bộ đúng chiều (không cần đảo dấu, khác F446), servo mới (ID=9, trim +1.5°) phản hồi đúng góc, test tổ hợp tiến/lùi/lái cùng lúc — encoder trái/phải khớp nhau.

**Mạch Motor Driver Hiwonder đã cháy (2026-07-05) → thay bằng BTS7960 x2, verify OK (2026-07-06):**

Sau nhiều lần motor không phản hồi qua ROS (servo vẫn OK, chỉ motor im lặng), phát hiện chip trên mạch Hiwonder bị nóng, tháo tản nhiệt ra thì cháy ngay lập tức — hỏng phần cứng vĩnh viễn (dòng liên tục driver chỉ 2A/2.5A peak trong khi motor stall ~2.3A, gần như không có margin, lại không có bảo vệ nhiệt/quá dòng).

**Đã thay bằng BTS7960 x2 (1 board/motor)** — margin dòng rộng hơn nhiều (~10A liên tục an toàn), có bảo vệ quá dòng/quá nhiệt ở cấp chip. Đồng thời viết lại kiến trúc encoder: đấu thẳng vào STM32 qua TIM hardware Encoder Mode, loại bỏ hoàn toàn I2C bit-bang từng phải vật lộn nhiều với Hiwonder.

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
