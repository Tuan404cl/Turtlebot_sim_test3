#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 1 (Bài tập Cử chỉ tay):
# Khởi chạy TurtleBot 4 trong Gazebo, kiểm tra /cmd_vel và mở Webcam bằng OpenCV
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

echo "======================================================================"
echo "   PHẦN 1: KHỞI CHẠY SIMULATION, KIỂM TRA /cmd_vel & WEBCAM OPENCV    "
echo "======================================================================"
echo " - Kiểm tra nhận lệnh vận tốc của robot qua topic /cmd_vel"
echo " - Đọc và hiển thị hình ảnh từ Webcam laptop bằng OpenCV"
echo " - Nhấn phím 'q' trên cửa sổ Webcam để kết thúc kiểm tra."
echo "======================================================================"

ros2 run turtlebot4_practice4 webcam_cmdvel_check

