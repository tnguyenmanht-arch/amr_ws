#!/bin/bash
# Bat toan bo he SLAM de ve ban do. Tu nhan dien cong, tu reset LiDAR.
#   Dung:  ./scripts/start_mapping.sh
#   Dung lai: Ctrl-C
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ">>> Nhan dien cong..."
eval "$(python3 scripts/find_ports.py)" || true
if [ -z "$STM32_PORT" ] || [ -z "$LIDAR_PORT" ]; then
    echo "!!! Khong nhan dien duoc cong. Kiem tra ca 2 day USB da cam chua." >&2
    exit 1
fi
echo "    STM32 = $STM32_PORT   |   LiDAR = $LIDAR_PORT"

# LiDAR de bi ket o trang thai dang phun du lieu neu lan truoc bi kill giua
# chung -> lan ket noi sau doc phai rac va bao timeout. STOP+RESET cho sach.
echo ">>> Reset LiDAR + kiem chung no quet duoc..."
# KHONG chi reset roi di tiep: neu LiDAR ket thi Nav2/SLAM van khoi dong, chay
# duoc mot doan roi moi chet mo ho. Kiem chung ngay tai day, hong thi dung han.
python3 scripts/lidar_reset.py "$LIDAR_PORT" || exit 1

echo ">>> Bat SLAM. Ctrl-C de dung."
exec ros2 launch amr_slam slam.launch.py \
     stm32_port:="$STM32_PORT" lidar_port:="$LIDAR_PORT"
