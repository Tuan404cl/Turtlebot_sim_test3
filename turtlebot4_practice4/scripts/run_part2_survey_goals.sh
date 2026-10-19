#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 2: Khảo sát và lấy tọa độ mục tiêu (x, y, yaw) trên bản đồ
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

ros2 run turtlebot4_practice4 goal_survey --ros-args -p use_sim_time:=true

