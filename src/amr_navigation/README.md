# amr_navigation

Config Nav2 cho robot AMR Ackermann — điều hướng tự động từ điểm A đến điểm B.

## Yêu cầu

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Cách dùng

```bash
# Navigation với bản đồ có sẵn
ros2 launch amr_navigation navigation.launch.py map:=/home/user/maps/my_map.yaml

# Navigation không có bản đồ (dùng kết hợp với SLAM online)
ros2 launch amr_navigation navigation.launch.py
```

## Stack

```
/goal_pose → bt_navigator → planner_server (NavFn) → controller_server (RPP) → /cmd_vel
                          ↑
               amcl (localization từ /scan + /map)
```

## Điều chỉnh quan trọng

Mở `config/nav2_params.yaml` và điều chỉnh:

| Section | Parameter | Ghi chú |
|---------|-----------|---------|
| `local/global_costmap` | `robot_radius` | Bán kính xe thực tế |
| `controller_server` | `desired_linear_vel` | Tốc độ tối đa mong muốn |
| `controller_server` | `use_rotate_to_heading: false` | Ackermann không quay tại chỗ |
| `velocity_smoother` | `max_velocity` | Giới hạn vận tốc |

## Lưu ý Ackermann

Nav2 mặc định dùng differential drive. Để phù hợp Ackermann:
- Dùng `RegulatedPurePursuitController` thay vì DWB
- Set `use_rotate_to_heading: false`
- Set `allow_reversing: false` (tùy yêu cầu)
