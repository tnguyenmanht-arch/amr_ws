# amr_bringup

Package tổng hợp — chứa launch files khởi động toàn bộ hệ thống AMR.

## Launch files

| File | Mô tả |
|------|-------|
| `amr_bringup.launch.py` | Launch toàn hệ thống (hardware + lidar + control + tùy chọn slam/nav) |
| `rviz.launch.py` | Chỉ khởi động RViz2 với config có sẵn |

## Cách dùng

```bash
# Bringup cơ bản (hardware + lidar + control + rviz)
ros2 launch amr_bringup amr_bringup.launch.py

# Bringup với SLAM
ros2 launch amr_bringup amr_bringup.launch.py use_slam:=true

# Bringup với Navigation (cần map sẵn)
ros2 launch amr_bringup amr_bringup.launch.py use_nav:=true

# Tắt RViz (headless mode trên Jetson)
ros2 launch amr_bringup amr_bringup.launch.py use_rviz:=false
```

## Config

- `config/amr_rviz.rviz` — layout RViz2 mặc định với LaserScan, Map, TF, Odometry
