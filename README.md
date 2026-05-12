# AMR Ackermann Robot — HUST Automotive Engineering

Robot AMR 4 bánh dẫn động Ackermann, tự định vị, lập bản đồ và điều hướng thông minh trong môi trường trong nhà.

**Dự án môn học** — Chương trình Kỹ sư chuyên sâu Ô tô số  
Đại học Bách Khoa Hà Nội (HUST) · ROS2 Humble · Ubuntu 22.04 · Jetson Orin Nano

---

## Kiến trúc hệ thống

```
[Jetson Orin Nano 8GB]  ←UART/USB→  [STM32F446RE Nucleo-64]
   (ROS2 Master)                        (Low-level Slave)
        │                                      │
   ┌────┴────┐                         ┌───────┴──────┐
   │IMX-219  │                         │JGB37-520 (×2)│
   │Camera   │                         │Motors+Encoder│
   └─────────┘                         └──────────────┘
   ┌─────────┐                         ┌──────────────┐
   │RPLidar  │                         │HTS-20H Serial│
   │A1M8     │                         │Bus Servo     │
   └─────────┘                         └──────────────┘
```

---

## Cấu trúc workspace

```
amr_project/               ← tương đương ~/amr_ws/ trên Jetson
└── src/
    ├── amr_bringup/       # launch files tổng hợp
    ├── amr_description/   # URDF/Xacro mô tả robot
    ├── amr_hardware/      # C++ driver serial ↔ STM32
    ├── amr_control/       # C++ Ackermann kinematics
    ├── amr_slam/          # SLAM Toolbox config + launch
    ├── amr_navigation/    # Nav2 config + launch
    └── amr_perception/    # Python: lane detection + lidar filter
```

---

## Quick Start

### 1. Clone và build

```bash
# Trên Jetson Orin Nano
mkdir -p ~/amr_ws/src
cd ~/amr_ws
git clone <repo_url> src/
colcon build --symlink-install
source install/setup.bash
```

### 2. Cài dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-sllidar-ros2 \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  python3-opencv

# Kiểm tra port STM32
ls /dev/ttyUSB* /dev/ttyACM*
```

### 3. Bringup

```bash
# Cơ bản (hardware + lidar + ackermann control + rviz)
ros2 launch amr_bringup amr_bringup.launch.py

# Vẽ bản đồ (SLAM mode)
ros2 launch amr_bringup amr_bringup.launch.py use_slam:=true

# Điều hướng tự động (cần map sẵn)
ros2 launch amr_bringup amr_bringup.launch.py use_nav:=true

# Teleop manual
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Topics chính

| Topic | Type | Mô tả |
|-------|------|-------|
| `/scan` | LaserScan | LiDAR raw |
| `/scan_filtered` | LaserScan | LiDAR sau lọc |
| `/camera/image_raw` | Image | Camera IMX-219 |
| `/cmd_vel` | Twist | Lệnh điều khiển |
| `/odom` | Odometry | Odometry từ encoder |
| `/map` | OccupancyGrid | Bản đồ SLAM |
| `/steering_angle` | Float32 | Góc lái thực (độ) |
| `/lane_center_error` | Float32 | Sai số tâm làn |

---

## Packages

| Package | Ngôn ngữ | Mô tả |
|---------|----------|-------|
| [amr_bringup](src/amr_bringup/README.md) | CMake | Launch files tổng |
| [amr_description](src/amr_description/README.md) | CMake/Xacro | URDF robot |
| [amr_hardware](src/amr_hardware/README.md) | C++ | Driver STM32 UART |
| [amr_control](src/amr_control/README.md) | C++ | Ackermann kinematics |
| [amr_slam](src/amr_slam/README.md) | CMake | SLAM Toolbox config |
| [amr_navigation](src/amr_navigation/README.md) | CMake | Nav2 config |
| [amr_perception](src/amr_perception/README.md) | Python | Lane detection + LiDAR filter |

---

## Checklist trước khi chạy lần đầu

- [ ] Kiểm tra port STM32: `ls /dev/ttyUSB*` → sửa `serial_port` trong `hardware.launch.py`
- [ ] Đo và cập nhật kích thước xe trong `amr.urdf.xacro` (wheelbase, track_width, wheel_radius)
- [ ] Đo và cập nhật `wheelbase_m`, `track_width_m` trong `control.launch.py`
- [ ] Xác nhận `gear_ratio` của JGB37-520 (mặc định: 90:1)
- [ ] Kiểm tra góc lái tối đa của HTS-20H servo → sửa `max_steering_deg`
- [ ] Build và source workspace: `colcon build && source install/setup.bash`

---

## Tài liệu tham khảo

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)
- [micro-ROS for STM32](https://micro.ros.org/)
