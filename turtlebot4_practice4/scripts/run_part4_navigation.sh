#!/usr/bin/env bash
# ==============================================================================
# Script chạy tích hợp toàn bộ: Localization + Nav2 Auto 03 Goals + YOLOv8
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
echo "   CHẠY TÍCH HỢP: LOCALIZATION + NAV2 (03 GOALS) + YOLOv8 DETECTION   "
echo "======================================================================"
echo "Bản đồ sử dụng: $MAP_FILE"
echo "1. Khởi chạy Gazebo simulation (warehouse) + thêm mô hình người, ghế, bàn."
echo "2. Khởi chạy Map Server + AMCL Localization + Nav2 Stack + RViz2."
echo "3. Tự động Undock robot và thiết lập Initial Pose (0.35, 0.0, 3.14)."
echo "4. Khởi chạy Node YOLOv8 nhận diện vật thể từ Camera OAK-D."
echo "5. Tự động điều khiển robot đi qua tuần tự Goal 1 -> Goal 2 -> Goal 3."
echo "======================================================================"

ros2 launch turtlebot4_practice4 sim_navigation.launch.py \
    map:="$MAP_FILE" \
    world:=warehouse \
    rviz:=true \
    spawn_objects:=true \
    enable_yolo:=true \
    auto_navigate:=true
 