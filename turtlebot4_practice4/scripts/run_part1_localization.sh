#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 1: Khởi chạy Simulation, Load Bản Đồ & Localization (AMCL + Nav2)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/clean_ros.sh" > /dev/null 2>&1 || true

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

MAP_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/maps/map.yaml"

echo "======================================================================"
echo "   PHẦN 1: KHỞI CHẠY SIMULATION & LOCALIZATION (AMCL + NAV2)          "
echo "======================================================================"
echo "Bản đồ nạp: $MAP_FILE"
echo "Thế giới Gazebo: warehouse (kèm các mô hình người, ghế, bàn)"
echo "Robot: TurtleBot 4 Standard (Tích hợp OAK-D RGB Camera + RPLiDAR)"
echo "----------------------------------------------------------------------"
echo "Các bước hệ thống sẽ tự động thực hiện:"
echo " 1. Khởi chạy Gazebo Simulation và thêm các mô hình vật thể (người, ghế, bàn)."
echo " 2. Khởi chạy Map Server nạp bản đồ 2D (map.yaml) và AMCL Localization."
echo " 3. Tự động Undock robot ra vị trí thông thoáng (x = 0.35m, y = 0.0m, yaw = 3.14)."
echo " 4. Khởi chạy toàn bộ Navigation2 Stack (Costmaps, Planner, Controller)."
echo " 5. Khởi chạy giao diện trực quan RViz2 hiển thị:"
echo "    - Bản đồ Occupancy Grid (Map topic: /map)"
echo "    - Mô hình robot TurtleBot 4 (RobotModel & TF)"
echo "    - Tia quét laser LiDAR (LaserScan topic: /scan trùng khớp tường)"
echo "    - Đám hạt định vị AMCL (ParticleCloud topic: /particle_cloud)"
echo "    - Camera RGB & YOLO (Image topics: /oakd/rgb/preview/image_raw, /yolo/detections/image)"
echo "======================================================================"

ros2 launch turtlebot4_practice4 sim_navigation.launch.py \
    map:="$MAP_FILE" \
    world:=warehouse \
    rviz:=true \
    spawn_objects:=true \
    enable_yolo:=false \
    auto_navigate:=false
