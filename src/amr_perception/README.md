# amr_perception

Perception package (Python) — xử lý camera và LiDAR cho robot AMR.

## Yêu cầu

```bash
sudo apt install python3-opencv ros-humble-cv-bridge
pip3 install numpy
```

## Nodes

### `lane_detection_node`

Phát hiện làn đường từ camera IMX-219 bằng Canny edge + Hough transform.

| Topic | Type | Ghi chú |
|-------|------|---------|
| Subscribe: `/camera/image_raw` | Image | Raw từ camera |
| Publish: `/lane_center_error` | Float32 | +: lệch phải, -: lệch trái |
| Publish: `/lane_debug_image` | Image | Ảnh debug với overlay |

**Parameters cần tune theo thực tế:**
- `canny_low`, `canny_high` — ngưỡng Canny (điều chỉnh theo ánh sáng)
- `roi_top_ratio` — tỉ lệ vùng quan tâm từ trên xuống (0.55 = 55% trở xuống)

### `lidar_filter_node`

Lọc điểm nhiễu từ RPLidar A1M8.

| Topic | Type | Ghi chú |
|-------|------|---------|
| Subscribe: `/scan` | LaserScan | Raw từ LiDAR |
| Publish: `/scan_filtered` | LaserScan | Đã lọc noise |

## Chạy

```bash
# Cả 2 nodes
ros2 launch amr_perception perception.launch.py

# Chỉ LiDAR filter
ros2 run amr_perception lidar_filter_node

# Xem debug lane detection
ros2 run rqt_image_view rqt_image_view /lane_debug_image
```

## Nâng cấp lên DNN

Khi muốn thay Hough bằng neural network:
1. Train model lane segmentation (ONNX/TensorRT)
2. Thay hàm `_detect_lane()` trong `lane_detection_node.py`
3. Dùng `tensorrt` hoặc `onnxruntime` trên Jetson GPU
