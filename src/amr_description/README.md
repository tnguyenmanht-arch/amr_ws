# amr_description

Mô tả robot AMR Ackermann bằng URDF/Xacro — dùng cho RViz visualization, SLAM và Nav2.

## Cấu trúc

```
urdf/
  amr.urdf.xacro     # mô tả robot đầy đủ
meshes/              # mesh 3D (thêm vào nếu có)
launch/
  description.launch.py
```

## Frames TF

```
base_footprint → base_link → rear_left_wheel
                           → rear_right_wheel
                           → front_left_wheel
                           → front_right_wheel
                           → laser_frame
                           → camera_frame → camera_optical_frame
```

## Điều chỉnh cần thiết

Mở `urdf/amr.urdf.xacro` và điều chỉnh:

| Property | Giá trị mặc định | Ghi chú |
|----------|-----------------|---------|
| `base_length` | 0.30 m | Đo khung xe thực tế |
| `base_width` | 0.20 m | Đo khung xe thực tế |
| `wheel_radius` | 0.04 m | Đo bánh xe thực tế |
| `wheelbase` | 0.22 m | Khoảng cách trục trước-sau |
| `track_width` | 0.18 m | Khoảng cách 2 bánh |
| LiDAR origin | `xyz="0.10 0 0.06"` | Vị trí gắn RPLidar |
| Camera origin | `xyz="0.12 0 0.05"` | Vị trí gắn IMX-219 |

## Chạy standalone

```bash
ros2 launch amr_description description.launch.py
```
