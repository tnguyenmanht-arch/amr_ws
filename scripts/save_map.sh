#!/bin/bash
# Luu ban do dang co trong bo nho ra file. CHAY TRUOC KHI TAT SLAM.
# Ban do chi nam trong RAM cua slam_toolbox — tat la mat trang.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p maps
NAME="${1:-map_$(date +%Y%m%d_%H%M)}"
ros2 run nav2_map_server map_saver_cli -f "maps/$NAME" --ros-args -p save_map_timeout:=10000.0
echo ">>> Da luu: maps/$NAME.pgm + maps/$NAME.yaml"
ls -la "maps/$NAME".*
