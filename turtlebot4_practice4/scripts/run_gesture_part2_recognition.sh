#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 2 (Bài tập Cử chỉ tay):
# Chạy ROS 2 Node gesture_recognition_node (OpenCV + MediaPipe Gesture Recognizer)
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

echo "======================================================================"
echo "   PHẦN 2: NHẬN DIỆN CỬ CHỈ TAY BẰNG AI (gesture_recognition_node)    "
echo "======================================================================"
echo " - Đọc hình ảnh từ Webcam bằng OpenCV"
echo " - Phát hiện & phân loại 06 cử chỉ tay bằng MediaPipe Gesture Recognizer:"
echo "     1. Wave        (Vẫy tay / ILoveYou) -> Xoay trái-phải chào lại"
echo "     2. Pointing_Up (Giơ 1 ngón tay)     -> Chọn tiến-lùi 4 lần"
echo "     3. Victory     (Giơ 2 ngón tay)     -> Chọn đi theo quỹ đạo tròn"
echo "     4. Thumbs_Up   (Giơ ngón cái)       -> Xác nhận thực hiện hành vi"
echo "     5. Open_Palm   (Xòe bàn tay)        -> Dừng robot ngay lập tức"
echo "     6. Closed_Fist (Nắm tay)            -> Hủy hành vi, về trạng thái chờ"
echo " - Publish kết quả lên topic: /detected_gesture (std_msgs/msg/String)"
echo "======================================================================"

ros2 run turtlebot4_practice4 gesture_recognition_node "$@"

