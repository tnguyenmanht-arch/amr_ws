#!/bin/bash
# Bat Nav2 tren ban do da luu. Tu nhan dien cong, tu reset LiDAR.
#   Dung:  ./scripts/start_nav.sh [duong_dan_map.yaml]
#   Dung lai: Ctrl-C
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

MAP="${1:-$PWD/maps/map_20260909_0000.yaml}"
[ -f "$MAP" ] || { echo "!!! Khong thay ban do: $MAP" >&2; exit 1; }

echo ">>> Nhan dien cong..."
eval "$(python3 scripts/find_ports.py)" || true
if [ -z "$STM32_PORT" ] || [ -z "$LIDAR_PORT" ]; then
    echo "!!! Khong nhan dien duoc cong. Con tien trinh cu giu cong?" >&2
    echo "    Thu: pkill -9 -f 'slam_toolbox|sllidar_node|serial_driver_node'" >&2
    exit 1
fi
echo "    STM32 = $STM32_PORT   |   LiDAR = $LIDAR_PORT"

echo ">>> Reset LiDAR..."
python3 - "$LIDAR_PORT" <<'PY'
import serial, sys, time
s = serial.Serial(sys.argv[1], 115200, timeout=0.3); s.dtr = False
s.write(bytes([0xA5, 0x25])); time.sleep(0.1)
s.write(bytes([0xA5, 0x40])); time.sleep(2.5)
s.reset_input_buffer(); s.close()
PY

echo ">>> Bat Nav2 voi ban do: $MAP"
echo "    LUU Y: xe CHUA chay cho toi khi ban dat initial pose + goal trong RViz."
exec ros2 launch amr_navigation navigation.launch.py \
     map:="$MAP" stm32_port:="$STM32_PORT" lidar_port:="$LIDAR_PORT"
