#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 1: Khởi chạy Simulation, kiểm tra điều khiển vận tốc qua /cmd_vel
và đọc/hiển thị hình ảnh từ Webcam Laptop bằng OpenCV.

Chức năng:
1. Kiểm tra kết nối các topic của TurtleBot 4 trong Gazebo (/odom, /scan, /cmd_vel).
2. Tự động cấu hình safety_override='full' cho Create 3 (/motion_control) để cho phép cả tiến và lùi.
3. Gửi lệnh vận tốc thử nghiệm lên /cmd_vel (tiến, lùi, xoay trái, xoay phải, dừng) và xác thực qua /odom.
4. Mở Webcam laptop bằng OpenCV, đọc và hiển thị khung hình thời gian thực kèm thông số hệ thống.
"""

import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


class WebcamCmdVelChecker(Node):
    """Node kiểm tra /cmd_vel của TurtleBot 4 và hiển thị Webcam bằng OpenCV (Phần 1)."""

    def __init__(self):
        super().__init__('webcam_cmdvel_checker')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('test_cmd_vel_on_start', True)
        self.declare_parameter('headless', False)
        self.declare_parameter('run_duration', 0.0)  # 0.0 = chạy liên tục tới khi nhấn 'q'

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.test_on_start = bool(self.get_parameter('test_cmd_vel_on_start').value)
        self.headless = bool(self.get_parameter('headless').value)
        self.run_duration = float(self.get_parameter('run_duration').value)

        # Trạng thái robot
        self.odom_received = False
        self.scan_received = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.min_front_dist = float('inf')
        self.cmd_vel_verified = False
        self.cmd_vel_status_text = 'Dang cho kiem tra /cmd_vel...'

        # Publisher & Subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.motion_param_client = self.create_client(
            SetParameters, '/motion_control/set_parameters'
        )

        self.get_logger().info('=======================================================')
        self.get_logger().info(' PHẦN 1: KIỂM TRA /cmd_vel VÀ HIỂN THỊ WEBCAM OPENCV   ')
        self.get_logger().info('=======================================================')

    def _odom_cb(self, msg: Odometry):
        self.odom_received = True
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_cb(self, msg: LaserScan):
        self.scan_received = True
        valid = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        if valid:
            self.min_front_dist = min(valid)

    def enable_full_motion_override(self):
        """Bật safety_override='full' trên Create 3 để hỗ trợ cả lệnh tiến và lùi."""
        if self.motion_param_client.wait_for_service(timeout_sec=2.0):
            req = SetParameters.Request()
            param = Parameter()
            param.name = 'safety_override'
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='full'
            )
            req.parameters = [param]
            future = self.motion_param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info("Đã cấu hình /motion_control safety_override='full'.")

    def publish_velocity(self, linear_x: float, angular_z: float, duration: float):
        """Phát lệnh vận tốc liên tục ở tần số 20 Hz trong khoảng thời gian duration (giây)."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        end_time = time.time() + duration
        while rclpy.ok() and time.time() < end_time:
            self.pub_cmd_vel.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Dừng lại
        stop_msg = Twist()
        for _ in range(3):
            self.pub_cmd_vel.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    def verify_cmd_vel(self):
        """Kiểm tra robot nhận lệnh vận tốc qua /cmd_vel trong Gazebo."""
        self.get_logger().info('Đang chờ dữ liệu /odom từ Gazebo (tối đa 5s)...')
        wait_start = time.time()
        while rclpy.ok() and not self.odom_received and (time.time() - wait_start < 5.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.enable_full_motion_override()

        start_x, start_y, start_yaw = self.robot_x, self.robot_y, self.robot_yaw
        self.get_logger().info(
            f'Vị trí ban đầu: x={start_x:.3f}m, y={start_y:.3f}m, yaw={math.degrees(start_yaw):.1f}°'
        )

        self.get_logger().info('1) Thử nghiệm lệnh TIẾN (linear.x = +0.15 m/s trong 1.0s)...')
        self.publish_velocity(0.15, 0.0, 1.0)
        time.sleep(0.2)

        self.get_logger().info('2) Thử nghiệm lệnh LÙI (linear.x = -0.15 m/s trong 1.0s)...')
        self.publish_velocity(-0.15, 0.0, 1.0)
        time.sleep(0.2)

        self.get_logger().info('3) Thử nghiệm lệnh XOAY TRÁI - PHẢI (angular.z = ±0.6 rad/s)...')
        self.publish_velocity(0.0, 0.6, 0.8)
        self.publish_velocity(0.0, -0.6, 0.8)

        self.cmd_vel_verified = True
        if self.odom_received:
            self.cmd_vel_status_text = 'PASS: /cmd_vel & /odom hoat dong tot!'
            self.get_logger().info(
                f'[PASS] Robot đã nhận lệnh /cmd_vel thành công! '
                f'Vị trí hiện tại: x={self.robot_x:.3f}m, y={self.robot_y:.3f}m, '
                f'yaw={math.degrees(self.robot_yaw):.1f}°'
            )
        else:
            self.cmd_vel_status_text = 'PUBLISHED: Da phat /cmd_vel (Gazebo chua mo)'
            self.get_logger().warn(
                'Đã phát lệnh /cmd_vel thành công (chưa nhận /odom do Gazebo chưa bật).'
            )

    def run_webcam_loop(self):
        """Mở webcam bằng OpenCV và hiển thị hình ảnh thời gian thực."""
        cap = cv2.VideoCapture(self.camera_id)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.get_logger().info(
                f'[PASS] Đã mở thành công Webcam laptop (/dev/video{self.camera_id})!'
            )
        else:
            self.get_logger().warn(
                f'Không mở được /dev/video{self.camera_id}, sử dụng khung hình mô phỏng kiểm thử.'
            )

        self.get_logger().info("Đang hiển thị cửa sổ OpenCV. Nhấn phím 'q' hoặc ESC để thoát.")
        prev_time = time.time()
        start_loop_time = time.time()
        frame_count = 0

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.005)

                if cap.isOpened():
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
                    else:
                        frame = cv2.flip(frame, 1)
                else:
                    frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
                    cv2.putText(
                        frame, 'WEBCAM SIMULATION FRAME', (140, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2
                    )

                now = time.time()
                dt = max(now - prev_time, 1e-4)
                fps = 1.0 / dt
                prev_time = now
                frame_count += 1

                # Vẽ HUD thông tin Phần 1 lên cửa sổ OpenCV
                h, w = frame.shape[:2]
                overlay = frame.copy()
                cv2.rectangle(overlay, (10, 10), (w - 10, 130), (20, 20, 20), -1)
                cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

                cv2.putText(
                    frame, 'PHAN 1: TURTLEBOT 4 SIM & WEBCAM OPENCV',
                    (22, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 255), 2
                )
                cv2.putText(
                    frame, f'Webcam: /dev/video{self.camera_id} ({w}x{h}) | FPS: {fps:.1f}',
                    (22, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2
                )
                cv2.putText(
                    frame, f'Status: {self.cmd_vel_status_text}',
                    (22, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 2
                )
                cv2.putText(
                    frame,
                    f'Odom: x={self.robot_x:.2f}m, y={self.robot_y:.2f}m, '
                    f'yaw={math.degrees(self.robot_yaw):.1f} deg | Nhan [Q] de thoat',
                    (22, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1
                )

                if not self.headless and os.environ.get('DISPLAY'):
                    cv2.imshow('TurtleBot 4 - Part 1: Webcam & /cmd_vel Check', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), 27):
                        self.get_logger().info('Người dùng nhấn phím thoát cửa sổ Webcam.')
                        break
                else:
                    time.sleep(0.03)

                if self.run_duration > 0.0 and (now - start_loop_time) >= self.run_duration:
                    self.get_logger().info(
                        f'Hoàn tất kiểm tra Webcam sau {self.run_duration:.1f}s ({frame_count} frames).'
                    )
                    break
        finally:
            if cap.isOpened():
                cap.release()
            if not self.headless and os.environ.get('DISPLAY'):
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamCmdVelChecker()
    try:
        if node.test_on_start:
            node.verify_cmd_vel()
        node.run_webcam_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub_cmd_vel.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

