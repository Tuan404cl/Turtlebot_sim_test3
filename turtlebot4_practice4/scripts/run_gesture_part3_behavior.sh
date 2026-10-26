#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 3 (Bài tập Cử chỉ tay):
# Chạy ROS 2 Node robot_behavior_node (Subscribe /detected_gesture -> /cmd_vel)
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

echo "======================================================================"
echo "   PHẦN 3: ĐIỀU KHIỂN HÀNH VI TURTLEBOT 4 (robot_behavior_node)       "
echo "======================================================================"
echo " - Subscribe: /detected_gesture (std_msgs/msg/String)"
echo " - Publish  : /cmd_vel (geometry_msgs/msg/Twist)"
echo " - Quy tắc  : Chọn hành vi (Wave / Pointing_Up / Victory)"
echo "              -> Xác nhận bằng Thumbs_Up để thực thi!"
echo "              -> Open_Palm: Dừng ngay lập tức | Closed_Fist: Hủy hành vi"
echo "======================================================================"

ros2 run turtlebot4_practice4 robot_behavior_node "$@"

