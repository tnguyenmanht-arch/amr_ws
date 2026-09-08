#!/bin/bash
# Bat toan bo he SLAM de ve ban do. Tu nhan dien cong, tu reset LiDAR.
#   Dung:  ./scripts/start_mapping.sh
#   Dung lai: Ctrl-C
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ">>> Nhan dien cong..."
eval "$(python3 scripts/find_ports.py)"
echo "    STM32 = $STM32_PORT   |   LiDAR = $LIDAR_PORT"

# LiDAR de bi ket o trang thai dang phun du lieu neu lan truoc bi kill giua
# chung -> lan ket noi sau doc phai rac va bao timeout. STOP+RESET cho sach.
echo ">>> Reset LiDAR..."
python3 - "$LIDAR_PORT" <<'PY'
import serial, sys, time
s = serial.Serial(sys.argv[1], 115200, timeout=0.3); s.dtr = False
s.write(bytes([0xA5, 0x25])); time.sleep(0.1)
s.write(bytes([0xA5, 0x40])); time.sleep(2.5)
s.reset_input_buffer(); s.close()
PY

echo ">>> Bat SLAM. Ctrl-C de dung."
exec ros2 launch amr_slam slam.launch.py \
     stm32_port:="$STM32_PORT" lidar_port:="$LIDAR_PORT"
