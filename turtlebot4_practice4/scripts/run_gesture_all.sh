#!/usr/bin/env bash
# ==============================================================================
# Script chạy tích hợp toàn bộ Bài tập Điều khiển TurtleBot 4 bằng Cử chỉ tay AI
# (Phần 1: Gazebo Sim + Load Map & AMCL + Auto Undock -> Phần 2: MediaPipe -> Phần 3: Behavior)
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
echo "   TÍCH HỢP: ĐIỀU KHIỂN HÀNH VI TURTLEBOT 4 BẰNG CỬ CHỈ TAY AI       "
echo "======================================================================"
echo " Bản đồ nạp trên RViz2: $MAP_FILE"
echo " 1. Khởi chạy Gazebo Simulation (world: warehouse)"
echo " 2. Nạp bản đồ đã lưu (map.yaml) + AMCL Localization + Mở RViz2 (sau 6s)"
echo " 3. Tự động Undock và di chuyển robot ra khu vực rộng thoáng (sau 16s)"
echo " 4. Khởi chạy robot_behavior_node (sau 18s)"
echo " 5. Khởi chạy gesture_recognition_node + Cửa sổ Webcam AI (sau 20s)"
echo "----------------------------------------------------------------------"
echo " Hướng dẫn điều khiển bằng cử chỉ trước Webcam:"
echo "  - Vẫy tay (Wave)               : Chọn hành vi xoay trái-phải chào lại"
echo "  - Giơ 1 ngón tay (Pointing_Up) : Chọn hành vi tiến-lùi 4 lần"
echo "  - Giơ 2 ngón tay (Victory)     : Chọn hành vi di chuyển quỹ đạo tròn"
echo "  - Giơ ngón cái (Thumbs_Up)     : XÁC NHẬN và thực hiện hành vi đã chọn"
echo "  - Xòe bàn tay (Open_Palm)      : Robot DỪNG NGAY LẬP TỨC"
echo "  - Nắm tay (Closed_Fist)        : HỦY hành vi hiện tại, về trạng thái chờ"
echo "======================================================================"

ros2 launch turtlebot4_practice4 sim_gesture_control.launch.py \
    map:="$MAP_FILE" \
    world:=warehouse \
    rviz:=true \
    "$@"
