#!/bin/bash
# Bat RViz de xem ban do + dat vi tri ban dau + chon diem dich bang chuot.
# Chay o CUA SO RIENG, sau khi start_nav.sh da chay.
#
# Dung cau hinh nav2_default_view.rviz co san cua Nav2: da co sang Map, LaserScan,
# ParticleCloud (dam hat AMCL), Path (duong planner ve), ca 2 costmap, va 2 cong cu
# "2D Pose Estimate" / "2D Goal Pose" tren thanh cong cu.
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../install/setup.bash" 2>/dev/null
RV=/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
cat <<'TXT'
--------------------------------------------------------------------
 CACH DUNG
 1. Nhin ban do (den = tuong) va cac cham do = vong quet LiDAR truc tiep.
    Luc dau cham do se KHONG nam tren tuong, vi AMCL dang tin nham rang
    xe o goc ban do.
 2. Bam "2D Pose Estimate" tren thanh cong cu, roi bam-va-keo tren ban do
    tai vi tri THAT cua xe, keo theo huong mui xe dang quay.
 3. Nhin lai cham do: khi chung NAM DE LEN tuong den thi vi tri da dung.
    Chua dung thi lam lai buoc 2, khong sao ca.
 4. Chi khi cham do da khop moi bam "2D Goal Pose" de chon dich.
    !!! Bam Goal la XE SE CHAY THAT. Dam bao duong di trong truoc da.
--------------------------------------------------------------------
TXT
exec rviz2 -d "$RV"
