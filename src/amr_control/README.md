# amr_control

Ackermann kinematics controller — chuyển đổi `cmd_vel` (Twist từ Nav2/teleop) sang góc servo và tốc độ từng bánh.

## Node: `ackermann_controller_node`

**Subscribe:** `/cmd_vel` (geometry_msgs/Twist)  
**Publish:**
- `/steering_angle` (Float32) — góc lái tính bằng độ → gửi sang HTS-20H qua STM32
- `/left_wheel_speed` (Float32) — tốc độ bánh trái (m/s)
- `/right_wheel_speed` (Float32) — tốc độ bánh phải (m/s)

## Công thức Ackermann

```
R       = linear_x / angular_z        (bán kính quay)
δ       = atan(L / R)                 (góc lái trung bình)
v_left  = angular_z × (R - W/2)
v_right = angular_z × (R + W/2)
```

Trong đó: `L` = wheelbase, `W` = track width

## Parameters

| Parameter | Default | Ghi chú |
|-----------|---------|---------|
| `wheelbase_m` | `0.22` | Khoảng cách trục trước-sau (m) |
| `track_width_m` | `0.18` | Khoảng cách 2 bánh sau (m) |
| `max_steering_deg` | `30.0` | Giới hạn vật lý servo HTS-20H |

## Lưu ý

- Node này **không** điều khiển vận tốc PID — chỉ tính kinematics.
- PID tốc độ từng bánh được thực hiện trong firmware STM32.
- `max_steering_deg` cần kiểm tra lại theo spec vật lý của servo và góc cơ khí thực tế.
