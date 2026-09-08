#!/bin/bash
# Lai xe bang ban phim. Chay trong CUA SO RIENG, sau khi start_mapping.sh da chay.
#   speed 0.15 m/s : toc do da kiem chung /odom chinh xac
#   turn  0.20     : goc banh ~15.6 do, chua het lai (con du cho phim 'e')
# ⚠️ KHONG chay lenh mac dinh cua teleop: speed 0.5 = TOC DO TOI DA cua xe.
source /opt/ros/humble/setup.bash
echo "  u i o     i=tien thang   u/o=tien+cua trai/phai"
echo "  j k l     k=DUNG NGAY    j/l KHONG chay (xe Ackermann khong xoay tai cho)"
echo "  m , .     ,=lui thang    ./m=lui+cua trai/phai"
echo "  e/c = cua gat/thoai hon  |  tha phim = xe dung sau 300ms"
echo
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args -p speed:=0.15 -p turn:=0.20
