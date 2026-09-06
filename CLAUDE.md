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
  IMX-219 Camera  RPLidar A1M8        [DRV8871 x2 (1 module/motor)]   USART1
  (lane detect)   (SLAM/Nav)          PWM (TIM3) + Encoder (TIM2/TIM4) ↓  qua board debug BusLinker-V2.5 ↓
                                     JGB37-520 Motors              HTS-20H Servo (ID=1)
                                      (drive wheels)                (steering)
```
> **STM32 slave HIỆN TẠI: F411CEU6 "Black Pill" (board mới, khác board đã hỏng lần 3)** — không phải F103 nữa. Sau khi F103 verify xong (2026-07-21), quá trình debug "giật cục" BTS7960 tiếp diễn trên 1 board F411 mới mua khác; 2026-08-19 quyết định **bỏ kế hoạch MiniROS Controller** (tạm dừng, xem mục 8), dùng hẳn F411 + DRV8871 làm hướng chính. F103 (`amr_stm32f103/`) vẫn giữ nguyên, đã verify ổn định, là phương án dự phòng nếu cần quay lại.
> **Motor driver: DRV8871 x2 (thay BTS7960 2026-08-19)** — chỉ 2 chân logic IN1/IN2 (không R_EN/L_EN), VM/GND/OUT1/OUT2 qua terminal vít. Đơn giản hơn BTS7960 hẳn, không cần rail 5V riêng cho driver. Xem mục 8.
> **Servo quay lại dùng board debug TTL "BusLinker-V2.5" (mua mới, 2026-08-19)** — thay lối đấu điện trở tạm trước đây. Board chỉ có 1 đường nguồn vào (Vin 5-14V qua terminal) — chân "5V" trên header là OUTPUT tự sinh, KHÔNG cấp nguồn ngoài vào đó. `SERVO_ID` đổi từ 9 → **1** (đổi servo). Xem mục 8.
> **🔧 Kế hoạch thay thế bằng Hiwonder "MiniROS Controller" TẠM DỪNG (2026-08-19)** — chưa xác nhận tình trạng đơn hàng, quyết định tiếp tục dùng F411 rời + DRV8871 thay vì chờ/chuyển sang MiniROS. Xem mục 8 "MiniROS Controller — kế hoạch thay thế" (đã đánh dấu tạm dừng).

---

## 2. Hardware — Thông số kỹ thuật

### Master: Jetson Orin Nano Super Dev Kit 8GB
- OS: Ubuntu 22.04 LTS (JetPack 6.x)
- ROS2 Distribution: Humble Hawksbill
- GPU: Ampere, 1024 CUDA cores (dùng cho vision inference nếu cần)
- Giao tiếp với slave: UART hoặc USB-Serial
- Thư viện NVIDIA có thể dùng: Isaac ROS, DeepStream, TensorRT (nếu phần cứng đủ điều kiện)

### Slave: STM32F103C8T6 "Blue Pill" (rời, không phải Nucleo)
- Firmware: STM32CubeIDE, project tại `amr_stm32f103/` — **project mới tạo từ đầu** (không phải generate lại từ F411), xem mục 8
- Nạp/debug: **ST-Link V2 rời** — cắm SWCLK/SWDIO/GND + nguồn vào header SWD của Blue Pill
- **Không có cổng UART ảo tích hợp** — khi cần test `$VEL`/`$ODO` qua máy tính (thay vì Jetson), dùng thêm **module USB-to-TTL CH340 rời** nối PA2/PA3
- Nhiệm vụ: nhận cmd_vel → điều khiển motor (PWM+DIR qua BTS7960), đọc encoder (TIM hardware Encoder Mode), điều khiển servo qua single-wire trực tiếp
- Giao thức với master: custom UART protocol qua USART2
- Clock: **HSE 8MHz ngoài** (thạch anh trên board), PLL ×9 → **72MHz** (PREDIV_DIV1 — không chia đôi, khác giả định 16MHz ban đầu lúc gen CubeMX, đã sửa)

**Pin assignments đã xác nhận:**
| Peripheral | Pins | Kết nối |
|---|---|---|
| TIM3 CH1-CH4 | PA6, PA7, PB0, PB1 | PWM: RPWM/LPWM trái (PA6/PA7), RPWM/LPWM phải (PB0/PB1) → BTS7960 x2, 20kHz (ARR=3599 ở 72MHz timer clock — **khác 4999 của F411**, tính lại theo clock mới) |
| TIM2 (Encoder Mode TI12) | PA0, PA1 | Encoder trái (A/B), 16-bit counter (cộng dồn tràn số trong `motor_driver.c`) — **F103 không có timer 32-bit nào**, khác F411 (TIM2 32-bit) |
| TIM4 (Encoder Mode TI12) | PB6, PB7 | Encoder phải (A/B), 16-bit counter (cộng dồn tràn số) |
| USART2 | PA2 (TX), PA3 (RX) | Jetson Orin Nano, 115200 baud — lúc test bàn dùng CH340 rời (TX↔PA3, RX↔PA2, GND chung) |
| USART1 | PA9 (TX), PA10 (RX) | PA9 qua điện trở ~1kΩ + PA10 nối thẳng → dây SIG của HTS-20H (single-wire half-duplex, không qua debug board) |

> SYS Debug = **Serial Wire** (không phải "No Debug") — bắt buộc, nếu để "No Debug" CubeMX sinh `__HAL_AFIO_REMAP_SWJ_DISABLE()` sẽ khóa SWD vĩnh viễn sau lần nạp đầu. Linker cần thêm cờ `-u _printf_float -u _scanf_float` ("Use float with printf/scanf" trong project settings) vì `jetson_comm.c` dùng `%.1f`/`strtof`. Xem mục 8.

### Cơ cấu chấp hành
| Thiết bị | Model | Giao tiếp | Ghi chú |
|---|---|---|---|
| Drive Motor (×2) | JGB37-520 DC w/ Encoder | PWM (TIM3) + Encoder (TIM2/TIM4) | 12V, dòng stall ~2.3A, gear ratio 90:1 |
| Steering Servo | HTS-20H Serial Bus Servo | Serial Bus (TTL, single-wire) | Góc lái Ackermann; qua board debug BusLinker-V2.5 (xem mục 8, 2026-08-19). **`SERVO_ID=1`** (đổi từ 9 → 1, đổi servo khác 2026-08-19) |
| Motor Driver (×2, 1/motor) | DRV8871 module | PWM (IN1/IN2) | Thay BTS7960 (2026-08-19) — chỉ 2 chân logic, không R_EN/L_EN, VM/GND/OUT1/OUT2 qua terminal vít. Xem mục 8 |
| TTL Bus Servo Board | Hiwonder TTL Bus Servo Debugging Board (BusLinker-V2.5) | UART 115200 (header) + Vin 5-14V (terminal) | Board mới mua thay board cũ đã hỏng — chân "5V" trên header là OUTPUT, KHÔNG cấp nguồn ngoài vào đó (xem mục 8) |
| ~~Motor Driver BTS7960~~ | ~~BTS7960 43A module x2~~ | ~~PWM (RPWM/LPWM) + DIR~~ | **Thay bằng DRV8871 2026-08-19** — giữ lại tham khảo lịch sử "giật cục" ở mục 8 |
| ~~Motor Driver Hiwonder~~ | ~~4-Ch Encoder Motor Driver~~ | ~~I2C~~ | **ĐÃ CHÁY 2026-07-05, không dùng nữa** — xem mục 8 |

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
├── amr_stm32f411/          ← STM32 firmware HIỆN TẠI (F411 Black Pill — board mới, khác board hỏng lần 3; DRV8871 từ 2026-08-19)
│   ├── Core/Src/
│   ├── Core/Inc/
│   ├── Drivers/
│   └── amr_stm32f411.ioc
├── amr_stm32f103/          ← STM32 firmware DỰ PHÒNG (F103 Blue Pill, đã verify ổn định 2026-07-21, không phải board đang dùng)
│   ├── Core/Src/           ← Application code
│   ├── Core/Inc/
│   ├── Drivers/
│   └── amr_stm32f103.ioc
├── reference/              ← Tài liệu Hiwonder MiniROS/JetAcker (đã .gitignore, KHÔNG push git — license personal use only)
├── docs/
│   └── wiring-f411.html    ← Sơ đồ đấu dây đầy đủ F411 + DRV8871 + board debug servo (pin table, star ground, phân phối nguồn)
│       Artifact: https://claude.ai/code/artifact/6cf962e2-7683-49f2-8d3c-3ae49d44317f
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

### STM32 build/flash (Windows, F411 hiện tại — đổi từ 2026-08-19)

```bash
# Build headless (workspace đã import sẵn project amr_stm32f411)
"C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/stm32cubeidec.exe" --launcher.suppressErrors -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data "c:/Users/admin/STM32CubeIDE/workspace_2.1.1" \
  -cleanBuild amr_stm32f411

# Nạp qua ST-Link rời
"C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506/tools/bin/STM32_Programmer_CLI.exe" \
  -c port=SWD -w "c:/Users/admin/Documents/amr_ws/amr_stm32f411/Debug/amr_stm32f411.elf" -v -rst

# (F103 dự phòng, lệnh tương tự: đổi amr_stm32f411 -> amr_stm32f103)

# Kiểm tra ST-Link + cổng UART (CH340) đang nhận diện
STM32_Programmer_CLI.exe -l

# Đọc thanh ghi qua SWD để chẩn đoán "không phản hồi" — LƯU Ý bắt buộc dùng mode=HOTPLUG
# nếu board đang chạy thật (mode mặc định "Normal" tự halt core, đọc ra toàn 0 giả (false alarm))
STM32_Programmer_CLI.exe -c port=SWD mode=HOTPLUG -r32 0x4000440C 1   # USART2->CR1
STM32_Programmer_CLI.exe -c port=SWD mode=HOTPLUG -r32 0x4002101C 1  # RCC->APB1ENR (F1, khác địa chỉ AHB1ENR của F4)
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

> Board F411 nói trên (2026-07-07) sau đó hỏng lần 3 khi nối Jetson (xem "Đã giải quyết gần đây"), dự án chuyển sang F103 làm chính (2026-07-21). Trong lúc chờ MiniROS Controller, mua **1 board F411 Black Pill khác** để tiếp tục dùng BTS7960 — board này gặp "giật cục" kéo dài (đã điều tra, xem memory phiên trước), rồi 2026-08-19 quyết định bỏ kế hoạch MiniROS, đổi driver BTS7960→DRV8871 trên chính board F411 này. Nội dung dưới đây là về board F411 thứ 2 này, KHÔNG phải board đã verify ở trên.

**🔧 Chuyển hẳn sang F411 (board mới) + DRV8871, bỏ kế hoạch MiniROS (2026-08-19):**

Quyết định: dùng F411 rời + DRV8871 x2 làm hướng chính, **tạm dừng** kế hoạch MiniROS Controller (xem mục "MiniROS Controller — kế hoạch thay thế", đã đánh dấu tạm dừng).

- **Driver DRV8871 thay BTS7960**: chỉ 2 chân logic **IN1/IN2** (không R_EN/L_EN) — nối đúng vị trí RPWM/LPWM cũ (`PA6/PA7` trái, `PB0/PB1` phải). VM/GND/OUT1/OUT2 qua terminal vít (loại module đang dùng: 1 terminal 4 vị trí, VM×2/GND×2 cùng net, chỉ cần dùng 1 vị trí mỗi net). **Firmware KHÔNG cần sửa gì** — `motor_driver.c` vốn điều khiển kiểu "PWM 1 chân, chân kia = 0", DRV8871 dùng đúng interface này.
- **Servo quay lại dùng board debug TTL BusLinker-V2.5** (mua mới) thay lối đấu điện trở tạm. Xác nhận qua user manual chính hãng (`reference/TTL Bus Servo Debugging Board/.../Servo Debug Board User Manual.pdf`) + đo thực tế: board chỉ có **1 đường nguồn vào duy nhất** — terminal `Vin (5~14V)` + `GND`. Header 4 chân `5V/TX/RX/GND` là cổng nối MCU — chân **5V là OUTPUT** tự sinh từ IC ổn áp trên board (đo ra ~5V dù không cấp gì vào đó, xác nhận đúng giả thuyết) — **KHÔNG được cấp nguồn ngoài vào chân này** (2 nguồn đấu đối đầu có thể hỏng IC ổn áp). Đấu: `PA9→RX board`, `PA10→TX board` (chéo), GND header dùng chung 1 dây về star với GND terminal Vin. Không cắm miniUSB debug cùng lúc với header STM32 (2 master tranh nhau TX/RX).
- Sơ đồ đầy đủ đã cập nhật trong `docs/wiring-f411.html` (cũng publish dạng artifact riêng).

**Bug: `SERVO_ID` sai (9 thay vì 1) khiến servo hoàn toàn im lặng dù comm đúng 100%:**
- Sau khi đấu xong, `$VEL`→STM32 tính đúng `steer_deg` (verify qua `$ODO`) nhưng servo không nhúc nhích — đi qua nhiều bước chẩn đoán sai hướng trước khi user nhớ ra **đã đổi sang servo khác có ID=1**, trong khi `servo_buslinker.h` vẫn hardcode `SERVO_ID=9` (từ lần đổi servo cũ 2026-07-07). Servo bus chỉ phản hồi đúng ID được gán trong khung lệnh — sai ID thì im lặng hoàn toàn, không có lỗi/NAK báo ra.
- **Fix**: `SERVO_ID` 9 → 1 trong `servo_buslinker.h`, build + nạp lại → servo phản hồi đúng ngay. **Bài học**: khi servo "không phản hồi" dù dây/nguồn/comm đều đúng, luôn hỏi lại "servo ID hiện tại có đúng với hardcode trong firmware không" trước khi nghi ngờ phần cứng khác.

**Chiều bánh phải bị đảo — xác nhận thực nghiệm bằng quan sát trực tiếp (không phải đo encoder):**
- Với dây/wiring DRV8871 mới, bánh phải quay ngược chiều so với bánh trái khi cùng lệnh dấu (quan sát trực tiếp bánh xe, không suy luận từ encoder). Fix: `DRV_Motor_SetSpeed()` trong `motor_driver.c` đảo dấu bên phải — `set_channel_speed((int8_t)(-right), TIM_CHANNEL_3, TIM_CHANNEL_4)`. **Quy ước dấu không cố định qua các lần đấu dây lại** — luôn đo/quan sát lại thực nghiệm sau mỗi lần đấu mới, đúng bài học đã ghi nhiều lần trong file này.

**⚠️ Điều tra "giật cục" motor ở duty cao — 2 nguyên nhân khác nhau, 1 nguyên nhân CHƯA giải quyết dứt điểm:**

1. **False alarm do script test (không phải hardware/firmware)**: script Python test ban đầu (đơn luồng: gửi → `sleep(0.1)` → đọc) khiến vòng lặp thỉnh thoảng trễ >300ms → watchdog `$VEL` firmware tự cắt cả 2 motor → trông giống "giật cục" dừng 1-2s lặp lại. Sửa bằng **thread riêng chỉ lo gửi `$VEL` mỗi ~50ms**, tách hoàn toàn khỏi việc đọc `$ODO` (đo được khoảng cách gửi thực tế ổn định ~50-54ms, an toàn so với ngưỡng 300ms) — xác nhận đây thuần là lỗi script, không phải phần cứng.
2. **Giật cục thật, do dây M+/M- (OUT1/OUT2 → motor) quá nhỏ**: sau khi sửa script (timing đã đều), vẫn còn giật cục (dừng ~1-2s, lặp lại) ở duty cao (70%). Lay thử dây M+/M- lúc đang chạy → hành vi thay đổi ngay → xác nhận tiếp xúc lỏng, đúng cơ chế đã gặp nhiều lần trong dự án (dây nhỏ mang dòng công suất lớn ~2.3A, khác dây tín hiệu dòng logic nhỏ). Đổi sang dây to hơn → cải thiện rõ rệt, 1 lần test chạy liên tục 190+ giây không có lần dừng nào.
3. **⚠️ CHƯA GIẢI QUYẾT DỨT ĐIỂM**: sau khi đổi dây, giật cục **vẫn tái xuất hiện** ở 1 lần quan sát sau đó — nghi còn sót lại 1 đoạn dây nhỏ/mối nối chưa hoàn thiện hết trên đường M+/M-. **Việc tiếp theo**: hoàn thiện toàn bộ đường M+/M- (không còn đoạn nào nhỏ hơn 18-20AWG, siết chặt terminal, không còn đoạn dupont/mối hàn yếu), rồi test lại bằng script có log đầy đủ (script `test_motor_forever.py` cũ bị mất log khi kill do Python buffer output — cần thêm `flush=True` khi viết lại).
4. **Đã cân nhắc và loại bỏ giả thuyết điện áp encoder (3.3V)**: không khớp triệu chứng (bánh xe **thực sự dừng quay** quan sát bằng mắt, không phải encoder đọc sai trong khi motor vẫn quay) — encoder 3.3V là quyết định chủ động bảo vệ GPIO F411 (xem bài học #4 mục BTS7960 phía trên), **không thử đổi sang 5V** vì rủi ro cháy chân GPIO chưa xác nhận 5V-tolerant, không đáng đánh đổi cho giả thuyết ít khả năng đúng.
5. **Bài học khái niệm quan trọng, dùng lại được cho các lần đấu dây sau này**: track PCB (dù mảnh) là khối đồng liền, điện trở cố định → chỉ gây sụt áp đều, không giật cục. Dây rời + đầu nối (dupont/terminal chưa chặt) là tiếp xúc cơ khí, điện trở thay đổi theo rung động/nhiệt → đây mới là nguồn gốc giật chập chờn. Board Hiwonder 4-Ch cũ (track PCB mảnh) chạy ổn định (không giật) nhưng vẫn cháy sau cùng — 2 kiểu lỗi khác nhau (nhiệt tích lũy do thiếu margin dòng, KHÔNG phải giật cục) dù cùng gốc "dòng lớn qua đường dẫn mảnh".

**File đã sửa (2026-08-19, session này)**: `amr_stm32f411/Core/Inc/servo_buslinker.h` (SERVO_ID 9→1), `amr_stm32f411/Core/Src/motor_driver.c` (đảo dấu bánh phải), `docs/wiring-f411.html` (sơ đồ DRV8871 + board debug servo).

**🔬 BNO055 (GY-BNO055) — test độc lập trước khi tích hợp (2026-08-19):**

Test cảm biến IMU rời (module clone GY-BNO055, Shopee, 8 chân VIN/GND/SCL-Rx/SDA-Tx/ADD/INT/BOOT/REST) qua I2C1 trên chính `amr_stm32f411/` — project đang chạy thật, KHÔNG đụng PWM/Encoder/USART1/USART2 hiện có. Chỉ để xác nhận module sống trước khi quyết định tích hợp, CHƯA route vào `ackermann.c`/`jetson_comm.c`.

- **Wiring breadboard (chưa hàn cố định)**: VIN→3.3V, GND→GND, SCL\Rx→**PB8**, SDA\Tx→**PB9** (xác nhận rảnh — đối chiếu `amr_stm32f411.ioc` trước khi thêm, chỉ dùng PA0/PA1/PA2/PA3/PA6/PA7/PB0/PB1/PA9/PA10 + TIM3 clock nội), ADD/INT/BOOT/REST thả nổi.
- **File tạo mới (không đổi qua các lần thử)**: `Core/Inc/bno055_test.h`, `Core/Src/bno055_test.c` (`BNO055_Test_Init` quét địa chỉ 0x28→0x29 bằng `HAL_I2C_IsDeviceReady`, đọc `CHIP_ID` nếu ACK, chuyển NDOF mode nếu đúng `0xA0`; `BNO055_Test_ReadHeading` đọc `EUL_HEADING_LSB/MSB` chia 16.0 ra độ). Gọi 1 lần trong `USER CODE BEGIN 2` (sau `APP_Comm_Init`), gửi kết quả qua `APP_Comm_DebugPrint()` có sẵn (raw register TX, không đụng HAL UART lock) dạng `$IMU,<found>,<addr hex>,<chip_id hex>\n`, đọc được qua CH340 lẫn với `$ODO`. Đọc heading (nếu found=1) đặt SAU logic động cơ/servo trong `while(1)`, throttle 150ms bằng `HAL_GetTick()` (không `HAL_Delay` lớn, không ảnh hưởng watchdog `$VEL` 300ms).

**Lần thử 1 — viết tay `MX_I2C1_Init()`/MSP, không dùng CubeMX generate:**
- Không tìm được lệnh headless code-gen (`.ioc` → C) đã xác nhận an toàn trong bộ STM32CubeIDE 2.1.1 (khác lệnh build ở mục 6, lệnh đó chỉ compile). Viết tay `MX_I2C1_Init()` + `HAL_I2C_MspInit/MspDeInit` bám sát pattern CubeMX (Standard 100kHz, `GPIO_MODE_AF_OD` + `GPIO_PULLUP`, `GPIO_SPEED_FREQ_LOW`, không NVIC).
- 2 bug build đã gặp + fix: (1) `stm32f4xx_hal_conf.h` comment sẵn `/* #define HAL_I2C_MODULE_ENABLED */` — phải bỏ comment. (2) `Drivers/STM32F4xx_HAL_Driver/{Inc,Src}` thiếu hẳn `stm32f4xx_hal_i2c[.c/.h]` + `_ex` (CubeMX chỉ copy file cho peripheral đã cấu hình từ đầu, I2C chưa từng thêm nên thiếu file, không phải lỗi cấu hình) — copy 4 file này từ 1 project STM32F4 chuẩn khác trong `reference/` (xác nhận đúng source gốc STMicroelectronics/MCD Application Team, an toàn dùng dù `reference/` nói chung ghi "personal use only" cho code/tài liệu riêng của Hiwonder).
- Build sạch, nạp, test: **`found=0`** (`$IMU,0,0x00,0x00`) — không ACK ở cả 0x28 lẫn 0x29.
- ⚠️ **Nguyên nhân thật (phát hiện sau)**: lúc test này **chưa cấp nguồn VIN 3.3V** cho module (user quên, xác nhận lại sau) — `found=0` là hệ quả tất yếu của việc module chưa có điện, **không phải bằng chứng cho việc module ở UART mode, và cũng không có bằng chứng quy được cho code viết tay sai** (thiếu điện thì bất kỳ cách viết code I2C nào, kể cả CubeMX generate chuẩn, cũng cho kết quả y hệt). Kết luận "nghi UART mode" ở bản ghi trước đó của mục này là **kết luận vội, đã bị thay thế** — giữ lại bài học quy trình bên dưới, không phải kết luận về module.

**Lần thử 2 — làm lại đúng quy trình an toàn hơn (theo yêu cầu user, để loại trừ khả năng code viết tay sai):**
- Revert sạch toàn bộ code I2C1 viết tay (`main.h`, `main.c`, `stm32f4xx_hal_msp.c`, `stm32f4xx_hal_conf.h`) về đúng bản gốc trước khi có I2C1 (xác nhận bằng `git diff` = rỗng) — giữ nguyên `bno055_test.h/.c` (logic không phụ thuộc cách I2C1 được init).
- Mở `amr_stm32f411.ioc` bằng **STM32CubeIDE GUI thật** (Device Configuration Tool): Connectivity → I2C1 → Mode I2C; xác nhận Pinout tự gán đúng **PB8=SCL/PB9=SDA** (F411 có 2 vị trí khả dụng cho I2C1, PB6/PB7 kia đang bận TIM4 encoder — phải kiểm tra kỹ tránh xung đột); Parameter Settings: Standard Mode, 100000Hz; NVIC: không tick — rồi Generate Code.
- **Diff sau generate xác nhận an toàn**: `stm32f4xx_hal_conf.h` đổi đúng 1 dòng; `main.c` +38 dòng thuần thêm (`hi2c1`, `MX_I2C1_Init()`); `stm32f4xx_hal_msp.c` +68 dòng thuần thêm (`HAL_I2C_MspInit/MspDeInit`) — **không dòng nào trong TIM2/TIM3/TIM4/USART1/USART2 bị sửa/xóa**.
- **So sánh với bản viết tay**: `MX_I2C1_Init()` CubeMX sinh ra **giống hệt 100%** bản viết tay trước đó (cùng `ClockSpeed=100000`, `DutyCycle`, mọi field) — phần logic Init không hề sai. Chỉ khác ở GPIO MSP: CubeMX dùng `GPIO_NOPULL` (bản viết tay dùng `GPIO_PULLUP`) và `GPIO_SPEED_FREQ_VERY_HIGH` (bản viết tay dùng `GPIO_SPEED_FREQ_LOW`) — đây là default thật của CubeMX cho I2C, giữ nguyên không tự sửa lại.
- Nối lại lời gọi `BNO055_Test_Init()`/`BNO055_Test_ReadHeading()` vào đúng vị trí cũ trong `main.c` (dùng `hi2c1` mới). Build sạch 0 lỗi 0 warning.
- **Xác nhận đã cấp VIN→3.3V thật trước khi nạp lần này** (khác lần thử 1).
- **✅ Kết quả: `found=1`** — `$IMU,1,0x29,0xA0` (t=0.809s sau reset). Địa chỉ **0x29** (không phải 0x28 mặc định — chân ADD của module này thả nổi nhưng resolve về mức khiến địa chỉ là 0x29, không phải lỗi). **CHIP_ID = 0xA0, đúng datasheet Bosch.**
- **Xác nhận heading sống**: đọc `$IMUH` liên tục ~150ms/lần trong lúc xoay module bằng tay — giá trị dao động rõ theo thời gian (ví dụ 1 đoạn log: 359.4→0.0→0.7→359.8→7.2→18.8→356.2→8.8→0.6, không đứng yên 1 chỗ) — xác nhận module NDOF mode đang hoạt động thật, không phải giá trị rác/đứng yên.
- **Cách đọc log trên Windows**: lệnh `screen`/`minicom` ở mục 6 chỉ áp dụng cho Jetson (Linux) — trên Windows dùng Python/`pyserial` mở `COM3` (CH340) đọc trực tiếp. Lưu ý: dòng `$IMU` chỉ gửi **1 lần lúc boot** (không lặp lại như `$ODO`) — nếu mở serial monitor SAU khi chip đã boot xong sẽ lỡ mất dòng này; phải mở listener TRƯỚC rồi mới trigger reset (`STM32_Programmer_CLI -c port=SWD -rst`) **trong cùng 1 script** (tách làm 2 lệnh Bash riêng bị lỡ mất dòng đầu do độ trễ giữa 2 lệnh — gộp reset vào cùng script Python bằng `subprocess`/thread mới bắt chính xác).
- **⚠️ Bài học quy trình quan trọng nhất của cả 2 lần thử**: trước khi kết luận bất kỳ điều gì về "code sai" hay "phần cứng/module lỗi mode" dựa trên 1 lần test thất bại, **luôn tự hỏi lại checklist nguồn/dây cơ bản trước tiên** (đặc biệt nguồn cấp — lỗi rẻ nhất, dễ bỏ sót nhất, nhưng gây triệu chứng giống hệt lỗi phức tạp hơn) — đúng tinh thần các bài học "R_EN/L_EN tiếp xúc lỏng", "SERVO_ID sai" đã ghi trong file này. Việc làm lại bằng CubeMX GUI thật vẫn có giá trị (xác nhận logic Init đúng, loại trừ hẳn nghi ngờ code), nhưng nguyên nhân gốc của lần fail đầu tiên hóa ra đơn giản hơn nhiều.
- **Đã xác nhận không ảnh hưởng hệ thống chính suốt cả 2 lần thử**: build sạch, nạp thành công, `$ODO` (motor/encoder/servo) vẫn chạy đều — I2C1 (PB8/PB9) hoàn toàn độc lập với TIM2/TIM3/TIM4/USART1/USART2 đang dùng thật.
- **Quyết định (2026-08-19)**: **dừng ở mức test độc lập**, KHÔNG route vào `ackermann.c`/`jetson_comm.c` lúc này — ưu tiên các việc dang dở khác (verify PCB motor sau gia công, chạy SLAM thật, bắt đầu Nav2) trước khi mở rộng sang tích hợp IMU. Đã xác nhận an toàn để commit lên board đang chạy thật: khi module không cắm, `found=0` → không lệnh I2C nào chạy trong `while(1)`, `$ODO`/`$VEL`/watchdog giữ nguyên hành vi, chỉ cộng thêm ~200ms 1 lần lúc boot (trước khi Jetson kịp gửi `$VEL` đầu tiên).
- **Việc tiếp theo (CHƯA làm, ưu tiên thấp hơn PCB/SLAM/Nav2)**: route heading vào `jetson_comm.c` (thêm vào giao thức `$ODO` hoặc tách topic riêng `$IMU`), rồi cấu hình `robot_localization` (EKF) bên ROS2 để fuse heading với `/odom` — hiện `CALC_Ackermann` mới chỉ dùng encoder differential-drive, chưa dùng IMU/servo cho hướng đi (xem gap kiến trúc đã ghi ở Giai đoạn 3).

**🔧 Thiết kế PCB power (M+/M-) để thay dây rời — review xong, sẵn sàng gia công (2026-08-20):**

Hướng giải quyết dứt điểm "giật cục" (mục trên): thay toàn bộ dây rời/đầu nối M+/M- (OUT1/OUT2 của DRV8871 → motor) bằng track PCB đồng liền, đúng bài học đã rút ra ("track PCB là khối đồng liền, điện trở cố định → không giật cục; dây rời + đầu nối lỏng mới là nguồn gốc giật chập chờn"). Đã thiết kế xong 1 board PCB 2 lớp (KiCad, `gerber/Stm32-*.gbr`) gộp: STM32F411 BlackPill (U1) + 2× DRV8871 (U2/U3) + terminal 3.5mm ra motor (J2/J11) + header encoder/động lực ra tận motor (J3/J4) + header servo UART1 đi board debug BusLinker qua dây jumper (J1) + header UART2 đi Jetson (J5).

**Đối chiếu pin assignment (ưu tiên kiểm tra, dựa trên schematic ảnh user gửi) — khớp 100% với chuẩn đã xác nhận ở trên, không phát hiện lỗi:**
- Motor trái: PA6→IN1, PA7→IN2 (U2 DRV8871) — đúng
- Motor phải: PB0→IN1, PB1→IN2 (U3 DRV8871) — đúng
- Encoder trái PA0/PA1, encoder phải PB6/PB7 — đúng, ra qua J3/J4
- Servo UART1: PA9→RX board, PA10→TX board qua J1 (dây jumper, U4 chỉ là outline vị trí đặt module TTL Board, không có pad điện) — đấu chéo TX/RX đúng chuẩn
- J1 pin 5V bỏ trống — đúng chủ đích (chân 5V header board debug là OUTPUT, không cấp nguồn ngoài)
- UART2 Jetson: PA2/PA3 qua J5 — đúng (đã xác nhận, không phải encoder)
- Domain công suất (J2/J11 terminal 3.5mm nối thẳng OUT1/OUT2) tách biệt hoàn toàn khỏi domain tín hiệu logic (J3/J4 encoder+5V) — thiết kế tốt

**Đo đạc định lượng từ Gerber (parse trực tiếp track width/net qua script Python, RS-274X):**
- Track `/M1+`, `/M1-`, `/M2+`, `/M2-` (B.Cu): **1.5mm**, đồng 1oz (0.035mm, xác nhận qua `Stm32-job.gbrjob` MaterialStackup) — không có via chuyển lớp trên đường công suất
- Tính theo công thức IPC-2221 (external layer, k=0.048): 1.5mm/1oz chịu **~3.2A ở ΔT10°C, ~4.35A ở ΔT20°C, ~5.2A ở ΔT30°C** — so với dòng stall thực tế JGB37-520 ~2.3A (trường hợp xấu nhất) → margin ~40-125% tùy ngưỡng nhiệt chấp nhận. **Kết luận: 1.5mm đủ dùng, không cần tăng độ rộng.**
- Lỗ khoan terminal 3.5mm (J2/J11) = 0.75mm (tool T3 trong `Stm32-PTH.drl`) — kích thước tiêu chuẩn cho terminal pitch 3.5mm, không phải điểm nghẽn
- GND phủ toàn bộ B.Cu (pour, ~1323mm track length) — tốt cho return path, giảm nguy cơ ground bounce (bài học từ sự cố hỏng 2 board F446)

**⚠️ Giới hạn của PCB này — không giải quyết được toàn bộ vấn đề giật cục:** PCB chỉ thay thế đoạn dây từ DRV8871 đến terminal J2/J11. Đoạn dây rời từ terminal J2/J11 ra tận motor thật (ngoài PCB, gắn trên khung xe) vẫn là dây rời — vẫn cần đảm bảo ≥18-20AWG, siết chặt terminal, không còn đoạn dupont/mối hàn yếu như đã ghi ở mục "giật cục" phía trên. Việc tiếp theo sau khi gia công + lắp board: test lại bằng script có `flush=True` để không mất log, xác nhận giật cục đã hết dứt điểm.

**⚠️ Nguồn 5V qua pin header (pin 18/40 trên PCB) — an toàn NHƯNG có điều kiện bắt buộc (2026-08-20):**

PCB thiết kế cấp 5V ngoài (từ hệ thống, không qua USB-C) vào 2 chân "5V" của STM32F411 BlackPill (pin 18 và pin 40 trên header, đúng vị trí U1 trong schematic). Đã xác nhận qua tài liệu chính thức board (stm32-base.org, WeAct schematic): đây là cách cấp nguồn **hợp lệ và chuẩn** — 2 chân 5V này đi vào regulator on-board **AP7343** (input 3.52-5.25V) tự chuyển xuống 3.3V nuôi MCU, không cần cắm USB-C để chạy.

**⚠️ CẢNH BÁO BẮT BUỘC — không có bảo vệ phần cứng nào cả:** 2 chân 5V trên header **nối thẳng, không qua diode bảo vệ**, vào chính đường +5V của cổng USB-C trên board. Nguyên văn cảnh báo chính hãng: *"The +5V pins on this board are directly connected to the +5V pin of the USB connector. There is no protection in place. Do not power this board through USB and an external power supply at the same time."*

- **Quy tắc vận hành bắt buộc**: KHÔNG BAO GIỜ cắm USB-C (nạp code/debug/xem log) đồng thời với việc đang cấp 5V ngoài qua pin 18/40 — 2 nguồn 5V khác nhau đấu đối đầu trực tiếp trên cùng 1 rail, nguy cơ hỏng cổng USB máy tính hoặc hỏng board. Đây gần như đúng kịch bản đã gây "F411 hỏng lần 3 khi nối Jetson" (2 đường nguồn độc lập cùng lúc, nghi ground loop) — cùng nguyên tắc "chỉ 1 đường nguồn tại 1 thời điểm" đã áp dụng cho F103 và kế hoạch MiniROS.
- Khi cần nạp code/debug qua USB-C: **rút nguồn 5V ngoài trước**, chỉ dùng USB-C làm nguồn duy nhất lúc đó.
- Khi chạy thật (không debug): **không cắm USB-C**, chỉ dùng nguồn 5V ngoài qua header.
- Nếu bắt buộc cần cả 2 cùng lúc (vừa chạy vừa xem log serial): dùng cáp USB-C đã cắt dây VBUS (chỉ giữ D+/D-/GND).

**🔧 Cập nhật Gerber lần 2 — thêm header BNO055 vào PCB, sẵn sàng gia công (2026-08-23):**

Sau khi test độc lập BNO055 thành công (mục trên), thêm 1 header 8 chân (footprint gốc của module GY-BNO055, drill 1.0mm, tool T5 trong `Stm32-PTH.drl`) vào PCB đang thiết kế — chỉ route đúng 4 chân đã quyết định dùng: **VIN→3.3V, GND, SCL\Rx→PB8, SDA\Tx→PB9** (RESET/INT/ADD/BOOT để trống, không route — quyết định chủ động: phạm vi đồ án không cần watchdog tự-reset qua GPIO, nếu I2C treo thì rút nguồn cắm lại tương đương reset cứng).

**Đối chiếu Gerber cũ vs mới (parse lại qua script Python, so track/net):**
- Track `/M1+`, `/M1-`, `/M2+`, `/M2-` (động lực motor): **không đổi**, vẫn 1.5mm, cùng tọa độ/chiều dài như lần review trước (7.6/19.1/30.4/41.7mm) — xác nhận lần cập nhật này chỉ thêm BNO055, không đụng phần công suất đã đạt yêu cầu.
- `/PB8`, `/PB9` (I2C1 mới): mỗi net nối đúng 2 điểm (MCU ↔ header BNO055 tại x=133.26, y=-100.87/-103.41), track 0.3mm — hợp lý cho tín hiệu I2C 100kHz, không cần rộng hơn.
- `/3V3`: dùng chung 1 rail nối tới nhiều điểm (header BNO055, header J3/J4 encoder, terminal VM/GND của DRV8871) — bình thường, đúng thiết kế rail chia sẻ sẵn có từ trước; tổng dòng tải 3.3V (BNO055 ~12.3mA + encoder vài mA) còn rất xa mức 300mA của regulator AP7343, không có rủi ro quá tải.

**Kết luận: PCB đã sẵn sàng gia công** — đấu chân đúng theo quyết định đã chốt (4 chân BNO055, không RESET/INT/ADD/BOOT), track động lực giữ nguyên margin đã tính (IPC-2221, ~3.2-5.2A cho track 1.5mm/1oz so với stall 2.3A), không phát hiện thêm vấn đề nào ở lần review này.

**🔴🔴 BUG WATCHDOG TRÀN SỐ — nguyên nhân THẬT của "giật cục" suốt nhiều tháng (tìm ra 2026-09-05):**

Trong lúc làm PID closed-loop, đo được `$VEL` "mất 80-100%": target chỉ hợp lệ 4-9% số chu kỳ PID. Đã loại trừ tuần tự (mỗi cái đều test thật, không suy đoán): script Python (threading/blocking/tốc độ gửi), đấu chéo TX/RX, dây jumper mới, GND, BNO055 block CPU, MCU tự reset, **đổi hẳn chip CH340 → CP2102 chính hãng**, và **thử cả board STM32F103 khác** — tất cả đều cho triệu chứng Y HỆT.

Bước quyết định: thêm bộ đếm **byte thô** (`$RXST`: byte vào ISR / lỗi ORE / byte bỏ do buffer đầy / dòng hoàn chỉnh / dòng parse OK). Kết quả `$RXST,357,18,0,21,21` — **STM32 nhận đủ 21 dòng/giây và parse thành công 100%**. Tức `$VEL` KHÔNG hề mất! Thủ phạm nằm ở chỗ khác:

```c
uint32_t now = HAL_GetTick();          // chụp ở ĐẦU vòng lặp
...
APP_Comm_SendOdom(...);                // blocking ~2-3ms
APP_Comm_Parse();                      // -> on_cmd_vel: last_vel_rx_ms = HAL_GetTick()  (MỚI HƠN 'now')
if ((now - last_vel_rx_ms) > 300)      // 'now' CŨ trừ mốc MỚI -> TRÀN SỐ unsigned ~4.29 tỷ
    DRV_Motor_SetSpeed(0, 0);          // -> watchdog trip OAN, giết lệnh vừa nhận
```

`now` và `last_vel_rx_ms` đều `uint32_t`. Chỉ cần `last_vel_rx_ms` lớn hơn `now` 1ms (luôn xảy ra khi `SendOdom` blocking xen giữa) là phép trừ tràn xuống ~4.29 tỷ > 300 → watchdog cắt động cơ **ngay sau mỗi lệnh `$VEL` hợp lệ**. Fix: đọc lại `HAL_GetTick()` vào biến `now_wd` ngay tại chỗ so sánh (sau `APP_Comm_Parse()`), đảm bảo `now_wd >= last_vel_rx_ms`.

**Kết quả sau fix**: `$TCNT` từ 4-9% → **100%** chu kỳ có target hợp lệ, ngay lập tức.

**Bài học quan trọng nhất (áp dụng cho mọi code timing sau này):**
1. **KHÔNG dùng lại 1 mốc thời gian chụp từ đầu vòng lặp để so với mốc được cập nhật ở giữa vòng lặp** — nhất là khi giữa 2 điểm đó có hàm blocking. Luôn đọc lại `HAL_GetTick()` ngay tại chỗ so sánh.
2. Phép trừ `uint32_t` tràn xuống **âm thầm, không cảnh báo, không crash** — chỉ biểu hiện thành hành vi kỳ lạ ở tầng ứng dụng.
3. **Đo ở mức thấp nhất có thể trước khi đổ lỗi phần cứng**: 2 ngày nghi dây/chip/board đều sai; 1 bộ đếm byte thô giải quyết trong 10 phút. Khi triệu chứng giống hệt nhau qua **2 board khác nhau + 2 chip USB-UART khác nhau**, thứ chung duy nhất là PHẦN MỀM — đó là dấu hiệu rõ ràng phải quay vào soi code.
4. ⚠️ **`amr_stm32f103/` có cùng bug này** (copy cùng pattern `main.c`) — chưa sửa vì không còn dùng F103. Nếu quay lại F103 phải sửa trước.

**🔧 PID tốc độ closed-loop (2026-09-05, commit `1d4f0c9`):**

DRV8871 chỉ là H-bridge thuần (khác board Hiwonder 4-Ch cũ có MCU tự PID nội bộ) → tự viết PID trong firmware, không cần thêm phần cứng (encoder + PWM đã đủ).

- **`motor_pid.c/h` (mới)**: bộ PI tách riêng, có anti-windup (kẹp output + rút integral khi bão hoà). **Bỏ khâu D** — tick encoder rời rạc, đạo hàm chỉ khuếch đại nhiễu 1-2 tick thành dao động PWM.
- **`DRV_Motor_UpdatePID()`** chạy mỗi 10ms (dùng chung nhịp với `$ODO` 100Hz sẵn có, không cần timer mới). `DRV_Motor_SetSpeed()` giờ chỉ LƯU target; interface (-100..100) giữ nguyên nên `ackermann.c`/`jetson_comm.c` không phải sửa.
- **⚠️ AN TOÀN — khi `target=0` phải ép `duty=0` TRỰC TIẾP, KHÔNG chạy `PID_Update()`**: nếu encoder đọc sai (nhiễu/đứt dây/lỗi dấu), PID sẽ "tưởng" còn sai số cần sửa và **đè lên lệnh dừng lẫn watchdog** → xe không dừng được bằng phần mềm, phải cắt nguồn (**đã xảy ra thật** với lỗi dấu encoder trái). Nguyên tắc chung: lệnh dừng phải nằm NGOÀI vòng feedback, không bao giờ phụ thuộc cảm biến.
- **`LEFT_ENCODER_SIGN = -1.0f`**: encoder trái đếm NGƯỢC chiều với PWM dương (khác chuyện đảo dấu output bánh phải — 2 việc độc lập). Phát hiện qua log: `out_l` dính cứng +100 trong khi `delta_l` luôn âm ổn định.
- **`MAX_TICKS_PER_INTERVAL = 69.0f`** — ĐO THỰC NGHIỆM (chạy hết ga: trái 68.7, phải 70.0 tick/10ms; lấy bánh chậm hơn để cả 2 đều bám được setpoint). Để tạm 400 trước đó khiến PID **bão hoà 100% duty vĩnh viễn** → thoái hoá thành open-loop full ga (target 25% nhưng chạy 100% tốc độ).
- **`Kp=1.5`, `Ki=8.0`** — verify 30s @ target 50%: bám sai số ~2% dưới mục tiêu, dao động bánh trái **±0.3%**, bánh phải **±1.7%**. KHÔNG tăng Ki để triệt nốt 2% (rủi ro mang overshoot quay lại, không đáng).
- **Tốc độ thực tế**: full ga ≈ **0.55 m/s** (khớp `ACK_MAX_SPEED_MS=0.5` đang đặt). Khuyến nghị chạy indoor ở **0.25-0.3 m/s (50-60%)** để PID còn "khoảng dự trữ ga" bù dốc/tải — chạy sát 100% thì PID bão hoà, mất khả năng điều tiết.
- **Test tổ hợp motor + servo đồng thời (8 góc lái, 32s)**: KHÔNG sụt tốc tại bất kỳ thời điểm servo đánh lái nào → USART1 (servo) và PID/encoder/USART2 không xung đột.
- **Chiều lái xác nhận thực nghiệm**: `angular_z=+0.5` → `steer=+16.5°` → bánh chỉ sang **TRÁI**, khớp chuẩn ROS REP-103. Comment cũ trong `ackermann.h` ghi "dương = phải" là SAI, đã sửa.

**Việc còn treo về servo (chưa làm, cần sàn rộng vài mét):**
- Servo là **bus servo có vòng kín nội bộ** → **KHÔNG cần và KHÔNG nên** thêm PID bên STM32 (2 vòng kín lồng nhau sẽ đánh nhau; và ta cũng không có cảm biến đo góc bánh lái độc lập).
- Vấn đề thật là **hiệu chuẩn**: `ACK_STEER_TRIM_DEG=1.5°` và `K_ANGULAR_TO_DEG=30.0` đều đang là số ước lượng bằng mắt. Cách chuẩn: **test vòng tròn** (chạy tốc độ + góc lái cố định, đo đường kính vòng tròn bằng thước) rồi suy góc lái thật theo mô hình xe đạp `δ = atan(L/R)` với `L=0.21m`.
- **Trim quan trọng hơn hệ số góc**: Nav2/lane-following tự bù được sai số hệ số, nhưng KHÔNG bù được lệch tâm (sai số hằng số kéo xe về 1 phía).
- ⚠️ Hiện có **2 chỗ trim cùng lúc**: `ACK_STEER_TRIM_DEG=1.5°` (firmware) và `steering_trim_angular_z=-0.06` (ROS, `hardware.launch.py`) — cộng dồn/triệt tiêu lẫn nhau rất khó lần. **Nên bỏ 1 chỗ (giữ firmware) TRƯỚC khi hiệu chuẩn lại.**

---

## ✅ VI SAI BÁNH SAU — ĐÃ SỬA XONG TRÊN FIRMWARE (2026-09-06, phiên Windows)

> **Trạng thái: đã code + verify tĩnh xong.** Còn lại duy nhất **test vòng tròn dưới sàn** (phép đo phân xử) — phải làm bên phiên Jetson vì xe cần chạy không dây. Chi tiết chẩn đoán gốc giữ nguyên bên dưới để tham khảo.

### Kết quả sau khi sửa (test tĩnh, bánh nhấc khỏi đất, `linear_x=0.2`)

| Trạng thái | Trái (tick/s) | Phải (tick/s) | Tỉ số đo | Tỉ số lý thuyết | Lệch |
|---|---|---|---|---|---|
| Đi thẳng | 2660 | 2658 | 0.999 | 1.000 | **0.1%** |
| Cua trái (ω=+0.5) | 1916 | 3327 | **1.736** | 1.745 | **0.5%** |
| Đi thẳng lại | 2659 | 2655 | 0.999 | 1.000 | 0.1% |
| Cua phải (ω=−0.5) | 3324 | 1913 | **0.576** | 0.573 | **0.5%** |

- Vi sai **đúng chiều** (cua trái → bánh phải nhanh hơn) và **đúng độ lớn** (lệch 0.5% so với công thức).
- Góc servo firmware báo (+29.2° / −26.2°) khớp chính xác mô phỏng; chênh 1.5° giữa 2 chiều đúng bằng trim → góc vật lý thật ±27.7° đối xứng.
- **PID không hề bị ảnh hưởng**: đi thẳng bám 2660 tick/s vs target 2760 (sai số 3.6%, đúng tầm sai số dư vốn có), 3 lần về thẳng đều cho tỉ số 0.999/0.999/1.004 — không dao động, không bão hoà.
- Phạm vi sửa: **chỉ `ackermann.c`/`.h`**. `motor_pid.c`, `motor_driver.c`, `main.c` (watchdog, bypass an toàn target=0) **không đụng một dòng nào**.

### ⚠️ 1 SỬA ĐỔI so với bản bàn giao từ Jetson — trim KHÔNG được đưa vào công thức vi sai

Bản bàn giao ghi *"θ phải là góc lái đã tính ở bước 2 (đã cộng trim, đã clamp)"* — **chỗ này sai**, đã sửa khi cài đặt.

`ACK_STEER_TRIM_DEG=1.5°` bù **lệch tâm cơ khí**: khi firmware xuất `steer_deg = 1.5°` thì bánh **đang thẳng về mặt vật lý**. Nên `steer_deg` là *lệnh servo*, không phải góc bánh thật (góc thật ≈ `steer_deg − trim`).

Nếu tính vi sai từ góc đã cộng trim thì lúc **đi thẳng** (ω=0 → servo=1.5°) sẽ sinh vi sai giả `D·tan(1.5°)/2H = 1.35%` → ở 0.25 m/s tạo **góc quay ma ~0.9°/s** trong `/odom` → đi thẳng 1 phút lệch **~54°**, đủ phá SLAM. Tức là sửa xong lỗi này lại đẻ ra lỗi khác cùng loại.

Trình tự đúng đã cài đặt:
```
θ_thật  = atan(H·ω / v)                     // góc vật lý cần có
θ_servo = clamp(θ_thật + TRIM, ±30°)        // giá trị gửi servo
θ_đạt   = θ_servo − TRIM                    // góc vật lý THỰC SỰ đạt (sau clamp)
V_L = V·(1 − D·tan(θ_đạt)/2H) ; V_R = V·(1 + D·tan(θ_đạt)/2H)
```
Kết quả đo xác nhận: đi thẳng cho hệ số vi sai **đúng bằng 0** (tỉ số 0.999).

### Đã đối chiếu trực tiếp tài liệu gốc (không tin trích dẫn gián tiếp)

Đọc `reference/2 Motion Control Course/1. Kinematics Analysis.pdf` trang 6 — công thức khớp nguyên văn, và đã **tự dẫn lại độc lập** để kiểm chứng (`V_L = ω·R_L = (V/R)(R−D/2) = V(1−D/2R)`, thay `1/R = tanθ/H`). Công thức góc lái `θ = atan(H·ω/v)` cũng xác nhận đúng (trang 8, hàm `set_velocity`).

**Phát hiện thêm — code thật của Hiwonder dùng dạng khác, và ta KHÔNG nên bắt chước:**
```python
vr = linear_speed + angular_speed * track_width/2   # dạng ω, không có tan()
vl = linear_speed - angular_speed * track_width/2
```
Hai dạng **tương đương tuyệt đối về đại số** (thay `ω = V·tanθ/H` vào là ra). Nhưng ta **phải dùng dạng `tanθ`**: firmware ta **clamp** góc ở ±30°, còn Hiwonder **từ chối hẳn lệnh** nếu |θ|>37° (đặt rps=0). Nếu clamp góc mà tính vi sai từ `ω` chưa clamp → vi sai lớn hơn góc lái thực tế cho phép → lại trượt lốp, chỉ đổi chiều sai.

**Thông số khung gầm Hiwonder KHÔNG nhất quán giữa chính 2 tài liệu của họ** (Kinematics: `wheelbase=0.213, track=0.222`; Motion Control trang 17: `0.216/0.195`; trang 19 còn dùng ký hiệu `D` cho wheelbase) → **dùng số đo thực của xe ta** (H=0.21, D=0.217), không lấy số của họ.

### ✅ Test vòng tròn ĐÃ LÀM XONG (Jetson, 2026-09-06) — vi sai chạy đúng

**Vi sai có tác dụng lớn, xác nhận giả thuyết gốc:**

| | Bán kính cua | Góc lái hiệu dụng | Hệ số understeer |
|---|---|---|---|
| Trước (khoá vi sai) | **1.45 m** | 8.2° | 3.99 |
| Sau (có vi sai) | **0.55 m** | 20.9° | 1.82 |

Bán kính thu **2.6 lần**, hệ số understeer giảm hơn một nửa.

> ⚠️ **Bẫy đơn vị đã suýt dẫn tới kết luận sai — ghi lại để tránh lặp:** số "1.45m" ngày 2026-09-05 là **BÁN KÍNH**, còn "1.10m" ngày 2026-09-06 là **ĐƯỜNG KÍNH**. Lúc đầu so nhầm hai đại lượng khác nhau → kết luận sai rằng "vi sai chỉ cải thiện 8%, không phải nguyên nhân chính". **Luôn hỏi lại đơn vị trước khi so hai phép đo của hai buổi khác nhau.**

### 🔬 HIỆU CHUẨN TỈ SỐ TRUYỀN LÁI — phát hiện mới, đã verify 3 điểm (2026-09-06)

Sau khi có vi sai, vẫn còn understeer hệ số 1.82. Truy tiếp bằng 2 test vòng tròn ở **2 góc lái khác nhau**, ra một mô hình tuyến tính khớp rất chặt:

```
góc_bánh_THẬT = 0.597 × (steer_deg + 4.8)
```

| servo `steer_deg` | Đường kính đo | Bán kính | Góc bánh thật | Hệ số suy ra |
|---|---|---|---|---|
| −4.8° | (xe đi thẳng) | ∞ | 0° | — (điểm gốc) |
| +22.2° | 1.46 m | 0.730 m | 16.05° | **0.594** |
| +30.0° | 1.10 m | 0.550 m | 20.90° | **0.601** |

Hai phép đo độc lập ở 2 góc khác nhau cho hệ số lệch nhau **chỉ 1%** → mô hình tuyến tính, đáng tin.

**Ý nghĩa:** bánh xe chỉ quay được **~60%** góc mà servo quay. Đây là **tỉ số truyền tay đòn lái** (chiều dài cánh tay servo so với cánh tay lái), cộng thêm phần trượt lốp bánh trước do khung này lái **song song** chứ không phải Ackermann thật (tài liệu Hiwonder trang 2: *"the two wheels are in a parallel state"*). Không tách riêng được 2 thành phần, cũng không cần — cứ hiệu chuẩn gộp là đủ.

⚠️ Đây là hệ số **hiệu dụng** đo trên sàn cứng với lốp hiện tại, đã gộp cả trượt lốp. Đổi mặt sàn (thảm) hoặc thay lốp thì phải đo lại.

**Và `ACK_STEER_TRIM_DEG = 1.5` là SAI.** Đo thực tế: xe đi thẳng khi `steer_deg = −4.8°`, không phải +1.5°. Trước đây trim ROS `−0.206` che được sai số này, nhưng sau khi đổi sang `θ = atan(H·ω/v)` thì **không che được nữa** — trim tính bằng rad/s cho ra góc **phụ thuộc tốc độ**, trong khi lệch tâm cơ khí là **góc không đổi**. Đo xác nhận (bánh nhấc, lệnh đi thẳng, trim ROS −0.206):

| v (m/s) | `steer_deg` báo về | Lệch tốc độ 2 bánh sau |
|---|---|---|
| 0.15 | −14.9° | 27% |
| 0.20 | −10.9° | 21% |
| 0.30 | −6.9° | 14% |

(lẽ ra phải là −4.8° và **0%** ở mọi tốc độ). `steering_trim_angular_z` đã đặt về **0** bên ROS, chờ firmware sửa.

### 🔴 VIỆC CẦN LÀM Ở PHIÊN WINDOWS TIẾP THEO

Sửa `ackermann.c` / `ackermann.h` — **2 hằng số + áp dụng theo cả 2 chiều**:

```c
#define ACK_STEER_TRIM_DEG   -4.8f   /* was +1.5f — đo thực tế, xem bảng trên */
#define ACK_STEER_GAIN        0.597f /* tỉ số truyền tay đòn lái, verify 3 điểm */
```

```c
/* 1. Góc lái VẬT LÝ mong muốn (giữ nguyên) */
theta_phys = atan(H*w/v) * RAD_TO_DEG;

/* 2. Đổi sang lệnh servo: chia GAIN rồi mới cộng trim */
servo_deg = theta_phys / ACK_STEER_GAIN + ACK_STEER_TRIM_DEG;
clamp(servo_deg, ±ACK_MAX_STEER_DEG);

/* 3. Góc vật lý THỰC SỰ đạt (sau clamp) — dùng cho vi sai */
theta_ach = (servo_deg - ACK_STEER_TRIM_DEG) * ACK_STEER_GAIN;
```

Kiểm tra nhanh: `theta_phys=0` → `servo=−4.8` (đúng vị trí xe đi thẳng) → `theta_ach=0` → vi sai `k=0` ✓

**Lợi ích:** với `theta_ach` là góc bánh THẬT, `/odom` tự đúng theo. Đối chiếu với số đo:

| servo | `/odom` sẽ báo | Thực tế đo | Lệch |
|---|---|---|---|
| +22.2° | 15.8 °/s | 15.7 °/s | **0.5%** |
| +30.0° | 20.7 °/s | 20.8 °/s | **0.6%** |

(hiện tại, khi chưa có GAIN, `/odom` báo thừa **1.54 lần** — 32°/s so với thực 20.8°/s)

**Lưu ý phụ:** clamp đang áp lên **lệnh servo**. Với `GAIN=0.597`, servo ±30° chỉ cho góc bánh thật ±20.9° → **bán kính cua nhỏ nhất 0.55m**. Nếu Nav2 cần cua gắt hơn thì phải nới `ACK_MAX_STEER_DEG`, nhưng **phải thử tay trước** xem tay đòn có bị kẹt cơ khí không.

**Sau khi nạp firmware mới, phía Jetson cần:** chạy lại test vòng tròn xác nhận `/odom` khớp thực tế, kiểm tra xe đi thẳng đúng khi `angular.z=0`, rồi mới chạy SLAM. `steering_trim_angular_z` giữ **0** vĩnh viễn (trim gộp về 1 chỗ trong firmware — đúng quyết định đã ghi từ trước).

---

## 📋 CHẨN ĐOÁN GỐC (2026-09-05, đo trên Jetson — giữ lại để tham khảo)

### Tóm tắt 1 câu

`CALC_Ackermann()` đang cho **2 bánh sau chạy cùng tốc độ khi cua**, PID lại ép chặt setpoint đó → xe **understeer nặng (cua rộng gấp ~2 lần hình học)** và `/odom` **mù hoàn toàn về hướng** → **SLAM sẽ hỏng nếu chạy bây giờ**.

### Triệu chứng đo được (Jetson, xe thật, CP2102 `/dev/ttyUSB0`)

| Test | Lệnh | `steer_deg` firmware báo | Quỹ đạo THẬT đo được | Góc lái suy ngược |
|---|---|---|---|---|
| Đi thẳng | `angular.z=0` | −4.8° | thẳng (lệch 6-7mm/1.09m) | ~0° |
| Cua nhỏ | `angular.z=0.3` | +4.2° | chỉ xoay **5°** sau 1.06m | ~1.0° |
| Hết cỡ | `angular.z=1.0` | +25.2° | vòng tròn **đường kính 1.4-1.5m** | ~16.2° |

Ở test hết cỡ: hình học Ackermann thuần tuý cho `R = L/tan(30°) = 0.364m` (đường kính 0.73m). Thực tế đo **1.45m — rộng gấp 2 lần**.

### Nguyên nhân (đã xác định, không phải suy đoán)

Khi cua bán kính `R`, 2 bánh sau **buộc phải** quay khác tốc độ vì chúng đi trên 2 cung tròn bán kính khác nhau (`R ± D/2`):

| Góc lái | R (m) | Chênh tốc độ 2 bánh sau BẮT BUỘC |
|---|---|---|
| 5° | 2.400 | 9% |
| 10° | 1.191 | 20% |
| 16.2° | 0.723 | **35%** ← xe thực tế dừng ở đây |
| 30° | 0.364 | **85%** ← firmware ra lệnh, bất khả thi |

Firmware hiện đặt `*speed_l = *speed_r = spd`, **và PID (thêm 2026-09-05) còn ép chặt** mỗi bánh bám đúng setpoint. Chênh 85% là không thể → lốp phải **trượt ngang** → sinh moment chống lại việc quay đầu xe → xe tự giãn ra bán kính lớn hơn cho tới khi mức chênh cần thiết đủ nhỏ để lốp trượt được (dừng ở 35%).

**⚠️ PID làm vấn đề TỆ ĐI.** Trước khi có PID, chạy hở, bánh bị cản sẽ tự chậm lại → tạo một phần vi sai "miễn phí". PID chủ động triệt tiêu đúng cái vi sai tự nhiên đó. Đây là **hệ quả phụ ngoài ý muốn của chính thay đổi PID hôm nay** — không có nghĩa PID sai, chỉ là nó phơi bày một thiếu sót vốn có.

**Giải thích được cả tính phi tuyến**: lực cản do trượt lốp bị giới hạn bởi ma sát nên gần như *không đổi*, còn lực lái sinh ra thì *tỉ lệ với góc lái*. Nên góc nhỏ → lực cản lấn át → understeer nặng (9×); góc lớn → lực lái thắng → understeer nhẹ hơn (2×). Khớp đúng cả 3 điểm đo.

> ❌ **Giả thuyết đã bị bác bỏ, đừng đi lại đường này**: ban đầu nghi "rơ cơ khí 7.6° (backlash)" và fit được mô hình khớp cả 3 điểm rất đẹp. Nhưng mô hình đó cần **2 tham số tự do bịa thêm** để khớp 3 điểm — khớp đẹp không có nghĩa là đúng. Cơ chế khoá vi sai giải thích mọi thứ chỉ bằng **1 cơ chế vật lý có thật**, và đã kiểm chứng bằng test tĩnh: bánh trước **đi mượt theo từng bước lệnh, về đúng vị trí cũ, không có rơ đáng kể**.

### ~~CẦN SỬA~~ ĐÃ SỬA — `amr_stm32f411/Core/Src/ackermann.c`

Chỗ cũ (trước 2026-09-06):
```c
/* Mô hình đơn giản hóa: 2 bánh sau cùng tốc độ, chưa bù vi sai khi cua */
int8_t spd = (int8_t)speed_f;
*speed_l = spd;
*speed_r = spd;
```

Thay bằng công thức Ackermann chuẩn (nguồn: `reference/2 Motion Control Course/1. Kinematics Analysis.pdf`, Hiwonder — khung gầm của họ wheelbase 0.213 / track 0.222 / bánh Ø0.101, gần như trùng xe ta 0.21/0.217/0.10 nên áp dụng được):

```
V_L = V · (1 − D·tan(θ) / 2H)
V_R = V · (1 + D·tan(θ) / 2H)
```

với `D` = track width = **0.217m**, `H` = wheelbase = **0.21m**, `θ` = góc lái (rad).

Lưu ý khi cài đặt:
- `θ` phải là góc lái đã tính ở bước 2 của `CALC_Ackermann` (đã cộng trim, đã clamp), **đổi sang radian** trước khi `tanf()`.
- Sau khi nhân hệ số, `speed_l`/`speed_r` vẫn phải **clamp về −100..100** (bánh ngoài có thể vượt 100 khi `V` đã gần max — khi đó nên **hạ tỉ lệ CẢ HAI bánh** giữ đúng tỉ số, thay vì clamp cụt bánh ngoài làm sai vi sai).
- **Dấu**: quy ước đã xác nhận thực nghiệm là `steer_deg` **dương = rẽ TRÁI**. Khi rẽ trái, bánh **phải** là bánh ngoài → phải quay **nhanh hơn**. Kiểm tra lại dấu bằng thực nghiệm sau khi nạp, đừng tin suy luận trên giấy (bài học lặp lại nhiều lần trong dự án này).
- Việc đảo dấu bánh phải trong `DRV_Motor_SetSpeed()` là chuyện **độc lập**, giữ nguyên, không đụng tới.

### Lỗi firmware THỨ HAI phát hiện cùng lúc — `K_ANGULAR_TO_DEG` bỏ quên vận tốc

`ackermann.c` hiện tính:
```c
float angle = angular_z * K_ANGULAR_TO_DEG + ACK_STEER_TRIM_DEG;   /* K = 30.0f */
```

Công thức đúng (cùng tài liệu Hiwonder, hàm `set_velocity`):
```python
theta = atan(wheelbase * angular_speed / linear_speed)
```

**Thiếu hẳn `linear_speed`.** Về vật lý: cùng một `angular_z`, xe chạy chậm phải đánh lái **nhiều hơn** mới đạt được tốc độ quay đó. Công thức hiện tại cho ra cùng một góc bất kể tốc độ, nên chỉ đúng tại **một tốc độ duy nhất**:

| v (m/s) | Góc đúng (ω=0.3) | Firmware ta tính | Tỉ lệ |
|---|---|---|---|
| 0.15 | 22.8° | 9.0° | 2.53 |
| 0.20 | 17.5° | 9.0° | 1.94 |
| 0.30 | 11.9° | 9.0° | 1.32 |
| **0.40** | **9.0°** | **9.0°** | **1.00** ✓ |
| 0.50 | 7.2° | 9.0° | 0.80 |

→ `K=30` chỉ đúng tại **v ≈ 0.40 m/s**. Ở tốc độ indoor khuyến nghị 0.25-0.3 m/s, xe cua chậm hơn Nav2 yêu cầu **1.3-1.6 lần**. Nav2 sẽ liên tục "đòi" nhiều hơn mức nhận được.

Cần xử lý `linear_x ≈ 0` (chia 0) — Hiwonder dùng ngưỡng `abs(linear_speed) >= 1e-8`, và họ **từ chối lệnh** nếu `|steering_angle| > 37°`. Ta clamp ở 30° (`ACK_MAX_STEER_DEG`), giữ nguyên.

### Sau khi nạp firmware mới — thứ tự verify

1. ✅ **Test tĩnh** (bánh nhấc khỏi đất) — **ĐÃ LÀM XONG 2026-09-06, ĐẠT**: xem bảng kết quả ở đầu mục. Vi sai đúng chiều, lệch 0.5% so với lý thuyết, dấu không cần đảo.
2. ⏳ **Test vòng tròn** (bánh chạm đất) — **CHƯA LÀM**, để phiên Jetson (xe cần chạy không dây, cáp CP2102 từ Windows vướng).
3. ⏳ **Hiệu chuẩn lại `steering_trim_angular_z`** — chưa làm, phải sau bước 2.
4. Chỉ sau khi 2-3 xong mới tính tới SLAM.

### Phía ROS đã chuẩn bị sẵn, KHÔNG cần sửa gì thêm

`serial_driver_node.cpp` đã để công thức `dtheta = (dr − dl) / track_width` với `track_width=0.217` (tham số mới, thay cho việc dùng nhầm `wheel_base=0.21` trước đây). Công thức này **hiện cho ra ~0 vì firmware chưa xuất vi sai** — nhưng sẽ **tự nhiên đúng ngay** khi firmware mới lên, không cần mô hình xe đạp, không cần hằng số hiệu chuẩn nào.

> Chiều 2026-09-05 đã thử vá bằng **mô hình xe đạp** lấy `steer_deg` từ `$ODO` (`dtheta = d·tan(δ)/L` + tham số `steering_offset_deg=4.9`). **ĐÃ HOÀN NGUYÊN** — nó bịa ra góc quay gấp ~2 lần thực tế (test vòng tròn: `/odom` báo xoay 360°, thực tế xe mới đi được ~190° của một vòng tròn to gấp đôi). Lý do thất bại: `steer_deg` là **góc LỆNH**, mà do understeer thì góc lệnh không phản ánh quỹ đạo THẬT. Vá ở ROS là vá sai chỗ — gốc rễ nằm ở firmware.

### Vẫn nên làm sau đó: BNO055 cho heading

Kể cả sau khi có vi sai, `(dr−dl)` vẫn suy ra hướng **gián tiếp** qua encoder và vẫn nhiễm sai số trượt lốp. **BNO055 đã test thành công 2026-08-19** (địa chỉ 0x29, heading đọc tốt) nhưng chưa dùng vào việc gì — nó **đo trực tiếp** hướng xe. Route heading vào `$ODO` rồi fuse bằng `robot_localization` EKF là giải pháp bền vững nhất. Ưu tiên sau khi vi sai chạy đúng.

---

### 🔧 Giai đoạn 3 — ROS2 Hardware Nodes: ĐANG TRIỂN KHAI
- [x] `serial_driver_node` (`amr_hardware`) đã có sẵn khung ROS2 tốt: sub `/cmd_vel`, pub `/odom` + TF `odom→base_link`, công thức odometry differential-drive, tham số khớp xe thật
- [x] **Hiệu chuẩn lại toàn bộ odometry trên phần cứng hiện tại (2026-09-05, F411#4+DRV8871+PID)** — xem mục chi tiết bên dưới. Giá trị chốt: `wheel_radius=0.049`, `encoder_ppr=44.0` × `gear_ratio=90` → `ticks_per_rev=3960`, `track_width=0.217`, `left_encoder_sign=-1.0`, `steering_trim_angular_z=-0.206`
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

**✅ HIỆU CHUẨN LẠI TOÀN BỘ ODOMETRY (2026-09-05, trên Jetson với F411#4 + DRV8871 + PID, CP2102 `/dev/ttyUSB0`)** — thay thế mọi số liệu hiệu chuẩn phía trên (đo trên phần cứng cũ, đã lỗi thời).

**Bug 1 — `/odom` đứng yên dù xe chạy thật:** firmware `$ODO` gửi tick **THÔ** đúng như timer đếm, KHÔNG bù dấu (firmware chỉ bù nội bộ cho vòng PID qua `LEFT_ENCODER_SIGN` trong `motor_driver.c`). Với wiring DRV8871 hiện tại, lệnh tiến làm `enc_l` chạy **ÂM** (0→−4543) còn `enc_r` chạy **DƯƠNG** (0→+4518), độ lớn khớp nhau → `d=(dl+dr)/2` **triệt tiêu về ~0**. Fix: thêm 2 tham số ROS `left_encoder_sign=-1.0` / `right_encoder_sign=+1.0` (không hardcode, vì quy ước dấu đổi mỗi lần đấu lại dây — bài học lặp lại nhiều lần). Commit `42fbedb`.

**Bug 2 — hai lỗi sai bù trừ nhau trong calibration cũ:** `wheel_radius=0.10` thực ra là **ĐƯỜNG KÍNH** bị ghi nhầm thành bán kính (bánh thật Ø100mm → bán kính 0.05m). `encoder_ppr=87.61` khi đó phải **gấp đôi** giá trị thật để bù lại, nên `/odom` vẫn ra đúng quãng đường — chỉ **tích** `2πr/ticks_per_rev` là được hiệu chuẩn thật, từng hằng số riêng đều sai. Tham số hoá lại tách bạch:
- `encoder_ppr=44.0` × `gear_ratio=90` → **`ticks_per_rev=3960`**: SỐ NGUYÊN CHÍNH XÁC suy từ phần cứng (11 PPR datasheet × **4 cạnh quadrature** do TIM chạy Encoder Mode TI12 × gear 90:1). Không phải số dò.
- **`wheel_radius=0.049`**: bán kính **LĂN hiệu dụng** (nhỏ hơn danh nghĩa 50mm do lốp nén dưới tải). Đây mới là đại lượng cần hiệu chuẩn thực nghiệm.

**Cách đo (dựa trên tick THÔ, không phụ thuộc tham số nào đang sai):** chạy thẳng 8s @0.2m/s, đọc delta tick 2 bánh, đo quãng đường bằng thước. Kết quả: delta tick trung bình **21864** (lệch trái/phải chỉ **0.58%**), quãng đường **~1.70m** → `ticks_per_rev` suy ra nằm trong **3970..4113**, **bao trọn giá trị lý thuyết 3960** → xác nhận encoder hoàn toàn bình thường.

**Kiểm chứng ĐỘC LẬP (quan trọng — tránh suy luận vòng tròn):** hiệu chuẩn ở quãng 8s, rồi kiểm chứng ở quãng **KHÁC** (5s, không dùng để hiệu chuẩn) → `/odom` báo **1.079m** vs đo thước **1.08m**, khớp **<0.1%**.

> ⚠️ **Bài học phương pháp:** trong phiên này đã một lần kết luận sai rằng "calibration cũ vẫn tốt" bằng cách so `MAX_TICKS_PER_INTERVAL=69 × ticks_per_rev` ra 0.55 m/s "khớp với 0.55 m/s đo được". Nhưng con số 0.55 trong CLAUDE.md **chính là được tính ra từ đúng 2 hằng số đó** — tức lấy một số xác nhận chính nó. **Kiểm chứng chỉ có giá trị khi dùng dữ liệu ĐỘC LẬP với dữ liệu đã dùng để hiệu chuẩn.**

**Hiệu chuẩn lại trim lái: `steering_trim_angular_z` −0.06 → −0.206.** Giá trị cũ từ 2026-07-03 đã lỗi thời (sau đó đổi servo sang ID=1 + đấu lại cơ khí). Cách đo: chạy thẳng 1.08m, đo lệch ngang 21cm → coi quỹ đạo là cung tròn (`s=Rθ`, `y=R(1−cos θ)`) → θ=22.57°, R=2.741m → góc lái thật `atan(L/R)=+4.38°` trong khi lệnh chỉ −0.30° → **lệch tâm cơ khí ~4.7-4.9°**. Giải ngược qua công thức firmware: `angular_trimmed=(−4.68−1.5)/30=−0.206`. **Kết quả: lệch ngang từ 21cm giảm còn 6-7mm trên 1.09m (0.6%).**

⚠️ Trim này đo trong điều kiện xe **đang understeer do khoá vi sai** — sẽ phải đo lại sau khi firmware có vi sai.

**Lưu ý khi đo lệch ngang — ĐO Ở TÂM TRỤC SAU**, không phải bánh trước: odometry bám trục sau, và `δ=atan(L/R)` cũng lấy R của trục sau. Đo ở trục trước lẫn cả chuyển động lái vào, chênh `L·sin θ` (tới 8cm khi θ=22°). Cũng phải chú ý **mốc tham chiếu**: mặt ngoài bánh trước cách mặt bên khung xe **5cm** trên xe này — đo 2 mốc khác nhau rồi so trực tiếp sẽ ra kết luận sai.

**Việc tiếp theo:** xem mục **"VIỆC CẦN LÀM Ở PHIÊN WINDOWS TIẾP THEO"** ở Giai đoạn 2 — phải sửa vi sai bánh sau trong firmware trước, mọi hiệu chuẩn còn lại (trim, `K_ANGULAR_TO_DEG`) đều phải làm LẠI sau đó. **Chưa chạy SLAM trước khi xong việc này.**

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
- [ ] ⏳ **CHƯA CHẠY** — test vòng tròn đã làm xong (2026-09-06): vi sai chạy đúng, bán kính cua 1.45m → 0.55m. Nhưng `/odom` vẫn báo hướng **thừa 1.54 lần** vì firmware chưa biết tỉ số truyền tay đòn lái (`góc bánh thật = 0.597 × (steer_deg + 4.8)`, đã verify 3 điểm). **Phải nạp firmware có `ACK_STEER_GAIN` + trim đúng trước** — khi đó `/odom` khớp thực tế trong 0.6%. Xem "HIỆU CHUẨN TỈ SỐ TRUYỀN LÁI" ở Giai đoạn 2.
- [ ] Chạy thực tế: `ros2 launch amr_slam slam.launch.py` và kiểm tra `/map`
- [ ] Lưu bản đồ `.yaml` + `.pgm` (`ros2 run nav2_map_server map_saver_cli`)

### ⏳ Giai đoạn 5 — Nav2: CHƯA BẮT ĐẦU
- [ ] AMCL localization trên bản đồ có sẵn
- [ ] Điều hướng tự động A → B

### Quyết định kỹ thuật đã chốt
- **STM32 slave HIỆN TẠI (2026-08-19): F411CEU6 "Black Pill" rời (board mới) + ST-Link ngoài** — không phải F103 nữa. F103 (`amr_stm32f103/`) vẫn giữ nguyên, đã verify ổn định 2026-07-21, giữ làm phương án dự phòng.
- **⏸️ Kế hoạch Hiwonder "MiniROS Controller" TẠM DỪNG (2026-08-19)** — xem "MiniROS Controller — kế hoạch thay thế" bên dưới (đã đánh dấu tạm dừng). Quyết định tiếp tục dùng F411 rời + DRV8871 thay vì chuyển sang MiniROS.
- Motor driver: **DRV8871 x2 (1 module/motor), thay BTS7960 2026-08-19** — chỉ 2 chân logic IN1/IN2 (PA6/PA7 trái, PB0/PB1 phải), không R_EN/L_EN, VM/GND/OUT1/OUT2 qua terminal vít, dây ≥18-20AWG. Firmware không đổi (cùng interface "PWM 1 chân, chân kia=0"). Xem mục "Chuyển hẳn sang F411 (board mới) + DRV8871" phía trên.
- Encoder: đấu thẳng vào STM32 qua TIM Encoder Mode (TIM2 trái 32-bit + TIM4 phải 16-bit trên F411), VCC encoder dùng 3.3V (không phải 5V — an toàn cho GPIO STM32, đã cân nhắc và loại bỏ giả thuyết đổi 5V khi debug giật cục 2026-08-19)
- Servo lái: HTS-20H (**`SERVO_ID=1`**, đổi từ 9 → 1 ngày 2026-08-19), USART1 PA9(TX)/PA10(RX), 115200 baud — qua board debug BusLinker-V2.5 (chỉ 1 đường nguồn Vin 5-14V qua terminal, chân 5V header là OUTPUT không phải input), trim `+1.5°` bù lệch cơ khí (**cần verify lại trim sau khi đổi servo ID=1**, chưa làm)
- Chiều motor: bánh phải **đảo dấu** trong `DRV_Motor_SetSpeed()` cho wiring DRV8871 hiện tại (xác nhận thực nghiệm 2026-08-19 bằng quan sát trực tiếp) — quy ước dấu KHÔNG cố định qua các lần đấu dây lại, luôn đo/quan sát lại sau mỗi lần đấu mới
- **✅ Giật cục motor: ĐÃ GIẢI QUYẾT DỨT ĐIỂM (2026-09-05)** — nguyên nhân thật là **bug tràn số unsigned trong watchdog `$VEL`** ở `main.c`, không phải dây lỏng/driver/CH340 như nghi suốt nhiều tháng. Xem mục "🔴 Bug watchdog tràn số" ở Giai đoạn 2. Các nghi vấn dây M+/M- trước đây có thể đã góp phần nhưng KHÔNG phải nguyên nhân chính.
- **Điều khiển tốc độ: closed-loop PID (PI) từ 2026-09-05** — `motor_pid.c/h` + `DRV_Motor_UpdatePID()` chạy mỗi 10ms. `MAX_TICKS_PER_INTERVAL=69` (đo thực nghiệm), `Kp=1.5`, `Ki=8.0` (đã verify 30s, sai số bám ~2%). Bỏ khâu D (tick encoder rời rạc → đạo hàm chỉ khuếch đại nhiễu).
- **Vi sai 2 bánh sau + góc lái theo vận tốc (2026-09-06)** — `ackermann.c` dùng `V_L,R = V·(1 ∓ D·tanθ/2H)` (D=0.217, H=0.21) và `θ = atan(H·ω/v)` thay hằng số `K_ANGULAR_TO_DEG=30` cũ (chỉ đúng tại v≈0.4 m/s). **Góc tính vi sai phải TRỪ trim** (trim là bù lệch tâm cơ khí, không phải góc bánh thật) — nếu không, đi thẳng sinh vi sai giả 1.35% → `/odom` quay ma 0.9°/s. `K_ANGULAR_TO_DEG` chỉ còn dùng cho nhánh `v≈0` (chỉnh trước góc lái khi test bàn). Verify tĩnh đạt, **test vòng tròn dưới sàn chưa làm**.
- Jetson ↔ STM32: custom UART ASCII protocol, USART2 (PA2/PA3), 115200 baud
- GND nối kiểu **star qua thanh terminal block riêng** (không daisy-chain qua chân board công suất) — bắt buộc từ sau sự cố hỏng 2 board F446, xem `docs/wiring-f411.html`
- Gear ratio motor: 90:1, dòng stall thực tế ~2.3A (quan trọng khi chọn driver thay thế)
- Kích thước xe: L=304mm, W=268mm, H=84mm, wheelbase=210mm, track=217mm, **bánh xe ĐƯỜNG KÍNH 100mm → BÁN KÍNH 50mm** (sửa 2026-09-05: dòng cũ ghi "r_wheel=100mm" là SAI — đó là đường kính. Sai này đã lan vào `hardware.launch.py` dưới dạng `wheel_radius=0.10` suốt nhiều tháng; dấu hiệu lẽ ra phải nhận ra sớm: cả xe chỉ cao 84mm, bánh bán kính 100mm là vô lý)

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
10. **CubeMX SYS Debug để "No Debug" sẽ khóa SWD vĩnh viễn** sau lần nạp đầu (sinh `__HAL_AFIO_REMAP_SWJ_DISABLE()`) — luôn để **"Serial Wire"** khi tạo project mới
11. **`STM32_Programmer_CLI -r32` mặc định connect mode "Normal" tự halt core khi attach** → đọc thanh ghi ra toàn 0 dù firmware đang chạy thật (giả "chip treo"). Phải thêm `mode=HOTPLUG` để đọc đúng giá trị real-time không làm gián đoạn core
12. **R_EN/L_EN (hoặc bất kỳ chân enable nào qua opto-coupler) tiếp xúc lỏng có thể đo tay ra ĐỦ điện áp nhưng vẫn lỗi chức năng** — vôn kế gần như không rút dòng nên không phát hiện được sụt áp thật khi mạch cần dòng qua LED opto. Test bằng cách ấn/lay dây trong lúc mạch đang hoạt động thật, không chỉ đo tĩnh

### ⏸️ MiniROS Controller — kế hoạch thay thế (2026-07-21, TẠM DỪNG 2026-08-19)

> **Cập nhật 2026-08-19: TẠM DỪNG kế hoạch này.** Quyết định tiếp tục dùng F411 rời + DRV8871 x2 (xem mục "Chuyển hẳn sang F411 (board mới) + DRV8871, bỏ kế hoạch MiniROS" ở Giai đoạn 2) thay vì chuyển sang MiniROS Controller. Chưa xác nhận tình trạng đơn hàng/hàng đã về hay chưa. Giữ lại toàn bộ nội dung bên dưới để tham khảo nếu quay lại hướng này sau.

Đã đặt mua board Hiwonder **"MiniROS Controller"** (STM32F407VETx, không phải chỉ driver rời) — ETA ~14 ngày (~2026-08-04). Đọc kỹ schematic + protocol SDK + tài liệu kinematics thật trong `reference/` (xem chi tiết dưới) trước khi hàng về, để có kế hoạch rõ ràng ngay khi bắt tay vào.

**Phần cứng board (xác nhận qua schematic `SCH_Ros Robot Controller Mini V2.0.pdf`):**
- 4× driver IC **SA8870** tích hợp sẵn (H-bridge, có sense resistor 0.25R) — không cần BTS7960 rời, loại bỏ vấn đề R_EN/L_EN
- **USB-C tích hợp chip CH9102F** (USB-to-serial) — không cần module CH340 rời
- Bus servo qua USART6 + IC đệm **74HC125** (half-duplex thật qua TX_EN/RX_EN) — thay mẹo điện trở đang dùng cho HTS-20H
- IMU **QMI8658** (6-trục) qua I2C2 — hiện robot chưa có IMU
- 4× encoder hardware timer (TIM2/3/4/5), 4× PWM channel (TIM1/9/10/11)

**Quyết định kiến trúc firmware (đã cân nhắc kỹ, chốt 2026-07-21):** giữ nguyên firmware bare-metal tự viết (copy gần nguyên `ackermann.c/h`, `jetson_comm.c/h`, `servo_buslinker.c/h`, chỉ viết lại `motor_driver.c` cho SA8870), **KHÔNG** chuyển sang full-stack FreeRTOS của Hiwonder (protocol binary `AA 55 FUNC LEN DATA CRC8`, motor PID closed-loop theo rps, Ackermann kinematics tính bên Jetson qua `ackermann.py`). Lý do: (1) source code JetAcker bản Ackermann thật bị khóa license, phải xin `support@hiwonder.com` kèm mã đơn hàng; (2) firmware generic MiniROS chỉ có differential/mecanum, không có Ackermann sẵn; (3) đổi FreeRTOS + protocol mới + PID tune cùng lúc = quá nhiều rủi ro cho deadline đồ án; (4) mất khả năng debug bằng mắt qua ASCII terminal.

**⚠️ An toàn khi nối Jetson (rút kinh nghiệm từ vụ F411 hỏng lần 3)**: board có 1 cổng USB-C tích hợp vừa cấp nguồn vừa data. Quy tắc bắt buộc — **chỉ 1 đường nguồn tại 1 thời điểm**: hoặc (a) board tự cấp nguồn riêng + cáp USB-C cắt chân VBUS khi nối Jetson, hoặc (b) lấy nguồn thẳng từ USB-C Jetson và KHÔNG cắm thêm nguồn ngoài nào khác cùng lúc. Test độc lập trên bàn trước (ST-Link + nguồn riêng) trước khi thử nối Jetson, đúng quy trình đã áp dụng cho F103.

**Việc tiếp theo khi hàng về:**
1. Soi kỹ mạch "电源输入接口电路" (power input) trong schematic — xem có bảo vệ cách ly VBUS sẵn không
2. Xác nhận mapping chân thật SA8870 IN1/IN2 ↔ TIM1/9/10/11 kênh nào cho từng motor (M1-M4 header)
3. Đổi toolchain `.ioc` từ MDK-ARM sang STM32CubeIDE (project gốc Hiwonder set sẵn Keil), generate lại
4. Port 3 file cũ + viết `motor_driver.c` mới cho SA8870, test tăng dần như đã làm với F103

**Tài liệu reference đã đọc** (trong `reference/`, đã .gitignore — license personal use only, không redistribute): schematic `SCH_Ros Robot Controller Mini V2.0.pdf`, protocol SDK `ros_robot_controller_sdk.py`, kinematics Ackermann chuẩn `1. Kinematics Analysis.pdf` (công thức bicycle model + bù vi sai bánh sau theo góc lái — tốt hơn `ackermann.c` hiện tại, cân nhắc áp dụng sau này dù không dùng full-stack Hiwonder).

### ✅ Đã giải quyết gần đây

**Port firmware sang STM32F103C8T6 "Blue Pill" hoàn tất + verify đầy đủ (2026-07-21):**

Sau khi F411 hỏng lần 3 khi nối Jetson (nghi ground loop, xem lịch sử bên dưới), chuyển sang F103C8T6 có sẵn. Tạo project CubeMX mới, port `motor_driver.c/h` (cộng dồn tràn số cho CẢ 2 encoder vì F103 không có timer 32-bit), copy nguyên `ackermann.c/h`, `jetson_comm.c/h`, `servo_buslinker.c/h`. Bật NVIC USART2 đúng ngay từ đầu (áp dụng bài học từ F411). Phát hiện + sửa 2 lỗi cấu hình lúc kiểm tra project CubeMX (trước khi build lần đầu): SYS Debug để "No Debug" (sẽ khóa SWD vĩnh viễn sau lần nạp đầu — phải đổi "Serial Wire"), Clock giả định HSE 16MHz trong khi board thật là 8MHz (đã sửa PREDIV). Build sạch 0 lỗi 0 warning cả Debug/Release, thêm cờ linker `-u _printf_float -u _scanf_float` còn thiếu (jetson_comm.c dùng `%.1f`).

**Đã verify đầy đủ trên phần cứng thật**: `$VEL`/`$ODO` round-trip, servo bám đúng góc + trim, watchdog dừng motor khi mất kết nối, motor+encoder chạy đúng ở duty đủ cao (30% không đủ thắng stiction hộp số, 80% chạy tốt), test tổ hợp motor+servo chạy đồng thời 10s không glitch (servo bám 100%, motor mượt sau khi sửa dây).

**Debug đáng chú ý**: ban đầu nghi "chip treo" vì đọc thanh ghi SWD ra toàn 0 dù firmware đang chạy thật — hóa ra do lệnh `STM32_Programmer_CLI -r32` mặc định dùng connect mode "Normal" tự halt core khi attach; phải thêm `mode=HOTPLUG` mới đọc đúng giá trị thực (không phải bug firmware, chỉ là công cụ chẩn đoán tự gây nhiễu — bài học mới, khác vụ "chip treo cần power-cycle" của F411 trước đây).

**Nguyên nhân "giật cục" motor khi chạy liên tục — đã xác định**: không phải nhiệt (sờ chip không nóng bất thường), không phải BMS pin (pin 80A/8000mAh dư dòng rất nhiều so với tải). Test tay: giữ chặt đầu nối 12V (B+/B-) → cải thiện, không hết hẳn (vẫn còn 1 lần đứng yên ~3.5s trong 20s). Ấn giữ dây **R_EN/L_EN** → motor quay nhanh hơn rõ rệt → xác nhận tiếp xúc lỏng ẩn tại R_EN/L_EN (đo tay ra đủ 5V vì vôn kế không rút dòng, nhưng mạch opto-coupler thật cần dòng qua LED thì bị sụt áp do lỏng dây — khớp đúng bài học cũ đã ghi). Hàn cứng B+/B-/R_EN/L_EN → thời gian đứng yên giảm từ 3.5-3.9s xuống còn 0.39s (cải thiện ~10 lần).

**Đã push git** (commit `3e98901`, `5626521..3e98901 main -> main`) — `amr_stm32f103/` đã lên GitHub, giữ nguyên `amr_stm32f411/` không đổi để tham khảo lịch sử.

**2 board STM32F446RE Nucleo hỏng liên tiếp (2026-07-07) → chuyển sang F411CEU6 "Black Pill", verify OK cùng ngày:**

Board nóng bất thường dù chỉ cấp USB (không nối motor/12V) — nghi do dòng rò AC từ sạc laptop 2 chân + GND từng đấu xuyên qua chân board BTS7960 (ground bounce), tích lũy qua nhiều giờ test. Đã chuyển hẳn sang F411 + ST-Link rời, áp dụng GND kiểu star qua thanh terminal riêng. Trong lúc port firmware, phát hiện + sửa bug **thiếu cấu hình NVIC cho USART2** (nguyên nhân thật khiến `$VEL` không xử lý được, từng nghi oan là chip hỏng lần 3) — chi tiết đầy đủ xem "Migration firmware F446 → F411" trong Giai đoạn 2 ở trên.

**Đã verify đầy đủ trên F411 (2026-07-07)**: giao tiếp `$VEL`/`$ODO`, 2 motor đồng bộ đúng chiều (không cần đảo dấu, khác F446), servo mới (ID=9, trim +1.5°) phản hồi đúng góc, test tổ hợp tiến/lùi/lái cùng lúc — encoder trái/phải khớp nhau.

**🔴 F411 hỏng lần 3 khi nối Jetson (2026-07-07, cuối phiên trước) → đã chuyển sang F103, không lặp lại vấn đề:**

F411 verify hoàn toàn OK trên Windows nhưng khi cắm sang Jetson thì hỏng — không nhận diện được gì qua SWD (power-cycle cũng không cứu được, khác lần trước). Setup lúc hỏng: Type-C (cấp nguồn) + CH340 (data, cổng USB khác) cắm Jetson **cùng lúc** — 2 đường USB độc lập. Giả thuyết nghi ngờ nhất (⚠️ chưa từng kiểm chứng thực nghiệm lặp lại): ground loop giữa 2 đường GND riêng biệt. Bài học phòng ngừa đã áp dụng cho F103 và sẽ áp dụng tiếp cho MiniROS Controller: chỉ 1 đường kết nối/nguồn duy nhất tới Jetson tại 1 thời điểm.

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
