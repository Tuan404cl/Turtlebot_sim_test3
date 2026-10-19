#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 4: Nhận diện vật thể từ Camera OAK-D với YOLOv8
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

echo "======================================================================"
echo "   PHẦN 4: NHẬN DIỆN VẬT THỂ TỪ CAMERA OAK-D BẰNG MÔ HÌNH YOLOv8     "
echo "======================================================================"
echo " - Image Subscribe : /oakd/rgb/preview/image_raw"
echo " - Image Publish   : /yolo/detections/image (Xem trên RViz2 & OpenCV)"
echo " - Model Weights   : /home/tuan/turtelbot4_ws/yolov8n.pt"
echo "======================================================================"

ros2 run turtlebot4_practice4 yolo_detector --ros-args -p use_sim_time:=true

