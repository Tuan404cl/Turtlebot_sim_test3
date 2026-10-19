#!/usr/bin/env bash
# ==============================================================================
# Script thực thi Phần 3: Lập trình Navigation tự động qua 03 Goals bằng Nav2
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"

echo "======================================================================"
echo "   PHẦN 3: ĐIỀU HƯỚNG TỰ ĐỘNG QUA 03 GOALS (NAV2 SIMPLE COMMANDER)    "
echo "======================================================================"
echo "Lộ trình tự động:"
echo "  - Goal 1: Hành lang trung tâm (x = -1.50 m, y = 0.00 m, yaw = 3.14 rad)"
echo "  - Goal 2: Hành lang phía Bắc  (x =  0.50 m, y = 1.20 m, yaw = 1.57 rad)"
echo "  - Goal 3: Khu vực nhân viên   (x =  0.35 m, y = 0.00 m, yaw = -0.99 rad)"
echo "======================================================================"

ros2 run turtlebot4_practice4 auto_nav_goals --ros-args -p use_sim_time:=true

