#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 2: Node khảo sát và lưu tọa độ mục tiêu (x, y, yaw) trên bản đồ (hệ tọa độ map)
- Lắng nghe đồng thời:
  1. Topic /clicked_point (geometry_msgs/msg/PointStamped) từ công cụ "Publish Point" trên RViz2
  2. Topic /amcl_pose (geometry_msgs/msg/PoseWithCovarianceStamped) từ bộ định vị AMCL
- Tự động lưu các tọa độ vừa click vào file config/surveyed_goals.json để Node điều hướng
  ở Phần 3 (auto_nav_goals.py) đọc và chạy trực tiếp.
"""

import json
import math
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped

SURVEY_FILE = '/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/surveyed_goals.json'

DEFAULT_GOALS = [
    {
        'name': 'Goal 1: Khu vực hành lang trung tâm (Central Aisle)',
        'x': -1.50,
        'y': 0.00,
        'yaw': 3.14159,
    },
    {
        'name': 'Goal 2: Khu vực hành lang phía Bắc (North Aisle)',
        'x': 0.50,
        'y': 1.20,
        'yaw': 1.5708,
    },
    {
        'name': 'Goal 3: Khu vực nhân viên kho & Trạm sạc (South-East View)',
        'x': 0.35,
        'y': 0.00,
        'yaw': -0.99,
    },
]


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Chuyển đổi Quaternion (x, y, z, w) sang góc Euler Yaw (radian) quanh trục Z."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalSurveyNode(Node):
    def __init__(self):
        super().__init__('goal_survey_node')
        self.clicked_goals = []
        self.last_amcl_pose = None

        # Nếu chưa có file surveyed_goals.json thì khởi tạo sẵn với 3 tọa độ chuẩn
        if not os.path.isfile(SURVEY_FILE):
            self._save_goals_to_file(DEFAULT_GOALS)

        self.sub_clicked = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10,
        )
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10,
        )

        self.print_banner()

    def _save_goals_to_file(self, goals_list):
        os.makedirs(os.path.dirname(SURVEY_FILE), exist_ok=True)
        with open(SURVEY_FILE, 'w', encoding='utf-8') as f:
            json.dump({'goals': goals_list}, f, indent=2, ensure_ascii=False)

    def print_banner(self):
        print('\n' + '=' * 80)
        print('   PHẦN 2: KHẢO SÁT VÀ LƯU TỌA ĐỘ MỤC TIÊU (x, y, yaw) TRÊN BẢN ĐỒ MAP')
        print('=' * 80)
        print(f'File lưu tọa độ tự động: {SURVEY_FILE}')
        print('Bảng 03 tọa độ mục tiêu (Goals) mặc định đã được lưu sẵn:')
        print('-' * 80)
        print(f"{'Điểm mục tiêu':<15} | {'Khu vực trên bản đồ':<32} | {'x (m)':>7} | {'y (m)':>7} | {'yaw (rad / deg)':>16}")
        print('-' * 80)
        print(f"{'Initial Pose':<15} | {'Vị trí sau khi rời trạm sạc':<32} | {0.35:>7.2f} | {0.00:>7.2f} | {'3.14 (180.0°)':>16}")
        print(f"{'Goal 1':<15} | {'Hành lang trung tâm (Central)':<32} | {-1.50:>7.2f} | {0.00:>7.2f} | {'3.14 (180.0°)':>16}")
        print(f"{'Goal 2':<15} | {'Hành lang phía Bắc (North)':<32} | {0.50:>7.2f} | {1.20:>7.2f} | {'1.57 ( 90.0°)':>16}")
        print(f"{'Goal 3':<15} | {'Khu vực nhân viên / Trạm sạc':<32} | {0.35:>7.2f} | {0.00:>7.2f} | {'-0.99 (-56.7°)':>16}")
        print('-' * 80)
        print('Cách lấy và lưu tọa độ mới trực tiếp trên RViz2:')
        print('  1. Chọn nút "Publish Point" trên thanh công cụ phía trên của RViz2.')
        print('  2. Click chuột vào 3 vị trí trên bản đồ (mỗi lần click chỉ lưu 1 điểm nên nút Publish Point sẽ tự nhả ra, hãy bấm lại "Publish Point" cho điểm tiếp theo).')
        print('  3. Khi đủ 03 điểm, hệ thống sẽ báo [ĐÃ LƯU THÀNH CÔNG 03 GOALS] vào file JSON!')
        print('=' * 80 + '\n')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw)

        if self.last_amcl_pose is None:
            self.last_amcl_pose = (x, y, yaw)
            print(
                f'[AMCL POSE HIỆN TẠI] frame="{msg.header.frame_id}" | '
                f'x = {x:.3f} m, y = {y:.3f} m, yaw = {yaw:.3f} rad ({yaw_deg:.1f}°)'
            )
            return

        lx, ly, lyaw = self.last_amcl_pose
        if math.hypot(x - lx, y - ly) > 0.10 or abs(yaw - lyaw) > 0.15:
            self.last_amcl_pose = (x, y, yaw)
            print(
                f'[AMCL POSE CẬP NHẬT] frame="{msg.header.frame_id}" | '
                f'x = {x:.3f} m, y = {y:.3f} m, yaw = {yaw:.3f} rad ({yaw_deg:.1f}°)'
            )

    def clicked_point_callback(self, msg: PointStamped):
        x = round(float(msg.point.x), 2)
        y = round(float(msg.point.y), 2)

        # Tính góc yaw gợi ý hướng từ điểm trước đó (hoặc vị trí robot hiện tại) tới điểm vừa click
        if self.clicked_goals:
            prev = self.clicked_goals[-1]
            yaw = round(math.atan2(y - prev['y'], x - prev['x']), 2)
        elif self.last_amcl_pose is not None:
            rx, ry, _ = self.last_amcl_pose
            yaw = round(math.atan2(y - ry, x - rx), 2)
        else:
            yaw = 0.0

        yaw_deg = math.degrees(yaw)
        idx = len(self.clicked_goals) + 1
        goal_entry = {
            'name': f'Goal {idx}: Điểm khảo sát trên RViz2 ({x:.2f}, {y:.2f})',
            'x': x,
            'y': y,
            'yaw': yaw,
        }
        self.clicked_goals.append(goal_entry)

        print(f'\n>>> [ĐÃ GHI NHẬN GOAL {idx} TỪ /clicked_point] <<<')
        print(f'  Hệ tọa độ (Frame) : {msg.header.frame_id}')
        print(f'  Tọa độ (x, y, yaw): x = {x:.2f} m, y = {y:.2f} m, yaw = {yaw:.2f} rad ({yaw_deg:.1f}°)')

        if len(self.clicked_goals) >= 3:
            self._save_goals_to_file(self.clicked_goals)
            print('  ====================================================================')
            print(f'  [ĐÃ LƯU THÀNH CÔNG {len(self.clicked_goals)} GOALS VÀO FILE]:')
            print(f'  -> {SURVEY_FILE}')
            print('  Bạn đã có thể nhấn Ctrl+C hoặc chuyển sang chạy Phần 3 (run_part3_auto_nav.sh)!')
            print('  ====================================================================')
        else:
            remaining = 3 - len(self.clicked_goals)
            print(f'  -> Cần click thêm {remaining} điểm nữa trên RViz2 (bấm "Publish Point" -> click bản đồ) để đủ bộ 03 Goals.')


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    node = GoalSurveyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
