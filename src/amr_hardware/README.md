# amr_hardware

Driver C++ giao tiếp UART/USB giữa Jetson Orin Nano và STM32F446RE.

## Kiến trúc phần cứng

```
Jetson Orin Nano
     │ UART/USB
     ▼
STM32F446RE Nucleo-64
     ├─── I2C ──────────► Hiwonder 4-Ch Encoder Motor Driver ──► JGB37-520 (×2)
     └─── USART (TTL) ──► Hiwonder TTL Bus Servo Debugging Board ──► HTS-20H Servo
```

## Nodes

### `serial_driver_node`

| Direction | Format | Mô tả |
|-----------|--------|-------|
| TX (→ STM32) | `V<speed_mm_s> S<steer_deg>\n` | Gửi tốc độ và góc lái |
| RX (← STM32) | `E<left_ticks> <right_ticks>\n` | Nhận encoder ticks |

**Subscribe:** `/cmd_vel` (Twist), `/steering_angle` (Float32)  
**Publish:** `/odom` (Odometry), TF `odom → base_link`

## Parameters

| Parameter | Default | Ghi chú |
|-----------|---------|---------|
| `serial_port` | `/dev/ttyUSB0` | Kiểm tra `ls /dev/ttyUSB*` |
| `baud_rate` | `115200` | Phải khớp STM32 firmware |
| `wheel_radius` | `0.10` m | Đo thực tế |
| `wheel_base` | `0.21` m | Đo thực tế |
| `encoder_ppr` | `11` | PPR trên trục motor JGB37-520 |
| `gear_ratio` | `90.0` | Tỉ số truyền JGB37-520 |

## Giao thức STM32

File này dùng **custom UART ASCII protocol**. Nếu muốn chuyển sang micro-ROS:
1. Comment out node này
2. Cài `micro_ros_agent` trên Jetson
3. Flash micro-ROS firmware lên STM32

## Chạy

```bash
ros2 launch amr_hardware hardware.launch.py serial_port:=/dev/ttyUSB0
```
