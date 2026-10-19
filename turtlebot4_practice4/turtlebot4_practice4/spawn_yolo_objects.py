#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Node hỗ trợ Phần 1 & Phần 4:
Thêm các mô hình vật thể (người, ghế, bàn) từ bộ nhớ đệm Ignition Fuel
vào môi trường Gazebo 'warehouse' tại các vị trí thông thoáng quanh 3 điểm Goal
để camera OAK-D của TurtleBot 4 quan sát và nhận diện bằng YOLO.
"""

import os
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node


class SpawnYoloObjects(Node):
    def __init__(self):
        super().__init__('spawn_yolo_objects')
        self.declare_parameter('world', 'warehouse')
        self.world = self.get_parameter('world').value

        fuel_root = os.path.expanduser('~/.ignition/fuel/fuel.gazebosim.org')
        self.objects_to_spawn = [
            {
                'name': 'person_standing_goal1',
                'sdf': os.path.join(
                    fuel_root,
                    'plateau/models/casual female/1/model.sdf',
                ),
                'x': -3.2,
                'y': 0.25,
                'z': 0.0,
                'yaw': 0.0,
                'desc': 'Người đứng gần Goal 1 (Hành lang trung tâm)',
            },
            {
                'name': 'chair_goal1',
                'sdf': os.path.join(
                    fuel_root,
                    'openrobotics/models/chair/3/model.sdf',
                ),
                'x': -3.0,
                'y': -0.55,
                'z': 0.0,
                'yaw': 0.4,
                'desc': 'Ghế gần Goal 1 (Hành lang trung tâm)',
            },
            {
                'name': 'chair_goal2',
                'sdf': os.path.join(
                    fuel_root,
                    'openrobotics/models/foldable_chair/3/model.sdf',
                ),
                'x': 0.35,
                'y': 2.85,
                'z': 0.0,
                'yaw': -1.57,
                'desc': 'Ghế gấp gần Goal 2 (Hành lang phía Bắc)',
            },
            {
                'name': 'table_goal2',
                'sdf': os.path.join(
                    fuel_root,
                    'openrobotics/models/table/4/model.sdf',
                ),
                'x': 1.25,
                'y': 3.05,
                'z': 0.0,
                'yaw': -1.57,
                'desc': 'Bàn gần Goal 2 (Hành lang phía Bắc)',
            },
        ]

    def spawn_all(self):
        self.get_logger().info(
            f'Đang thêm các mô hình vật thể (người, ghế, bàn) vào thế giới Gazebo "{self.world}"...'
        )
        for obj in self.objects_to_spawn:
            if not os.path.isfile(obj['sdf']):
                self.get_logger().warn(f"Không tìm thấy file SDF: {obj['sdf']}, bỏ qua.")
                continue

            cmd = [
                'ros2', 'run', 'ros_ign_gazebo', 'create',
                '-world', str(self.world),
                '-file', obj['sdf'],
                '-name', obj['name'],
                '-x', str(obj['x']),
                '-y', str(obj['y']),
                '-z', str(obj['z']),
                '-Y', str(obj['yaw']),
            ]
            try:
                subprocess.run(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=8.0,
                    check=False,
                )
                self.get_logger().info(
                    f"  [+] Đã thêm '{obj['name']}' ({obj['desc']}) tại "
                    f"(x={obj['x']}, y={obj['y']}, yaw={obj['yaw']})"
                )
            except Exception as exc:
                self.get_logger().warn(f"  [!] Lỗi khi thêm {obj['name']}: {exc}")
            time.sleep(0.3)

        self.get_logger().info(
            'Hoàn tất chuẩn bị mô hình vật thể cho bài toán nhận diện YOLO!'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpawnYoloObjects()
    try:
        node.spawn_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

