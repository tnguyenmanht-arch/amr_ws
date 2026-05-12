# amr_slam

Config và launch cho SLAM Toolbox — vẽ bản đồ môi trường bằng RPLidar A1M8.

## Yêu cầu

```bash
sudo apt install ros-humble-slam-toolbox
```

## Cách dùng

```bash
# Vẽ bản đồ mới (async SLAM)
ros2 launch amr_slam slam.launch.py

# Lưu bản đồ khi xong
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Bản đồ được lưu thành: my_map.yaml + my_map.pgm
```

## Config chính (`config/slam_toolbox_params.yaml`)

| Parameter | Giá trị | Ghi chú |
|-----------|---------|---------|
| `resolution` | `0.05` m | Kích thước ô bản đồ |
| `min_laser_range` | `0.15` m | Bỏ qua noise gần RPLidar |
| `max_laser_range` | `10.0` m | Giới hạn tin cậy của A1M8 |
| `map_update_interval` | `5.0` s | Cập nhật bản đồ mỗi 5 giây |

## Lưu ý

- Chạy SLAM trước khi chạy Nav2 để có bản đồ.
- Với localization trên bản đồ có sẵn: đổi `mode: localization` trong YAML.
- TF chain cần đầy đủ: `odom → base_link → laser_frame` từ `amr_hardware` và `amr_description`.
