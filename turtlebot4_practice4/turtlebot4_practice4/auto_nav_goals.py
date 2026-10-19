#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 3 & PHẦN 4:
ROS 2 Node điều khiển TurtleBot 4 tự động di chuyển (Autonomous Navigation)
tuần tự qua 03 điểm mục tiêu (Goal 1 -> Goal 2 -> Goal 3) bằng Nav2 Simple Commander,
kết hợp ghi nhận kết quả nhận diện vật thể từ Camera OAK-D + YOLOv8 tại từng khu vực.
"""

import json
import sys
import time
import math
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class TurtleBot4Navigator(BasicNavigator):
    """Lớp mở rộng của BasicNavigator tối ưu cho TurtleBot 4 trong Gazebo simulation."""

    def __init__(self):
        super().__init__()
        self.latest_yolo_detections = []
        self.all_detected_classes = set()
        self.sub_yolo = self.create_subscription(
            String,
            '/yolo/detections/summary',
            self._yolo_summary_cb,
            10,
        )

    def _yolo_summary_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            dets = data.get('detections', [])
            if dets:
                self.latest_yolo_detections = dets
                for d in dets:
                    self.all_detected_classes.add(d['class'])
        except Exception:
            pass

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        # Trừ lùi 0.5s so với simulation clock hiện tại để đảm bảo không bị extrapolation into the future
        now = self.get_clock().now()
        past = now - rclpy.duration.Duration(seconds=0.5)
        msg.header.stamp = past.to_msg()
        # Ma trận hiệp phương sai ban đầu chuẩn xác cho AMCL
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.info(
            f"Publishing Initial Pose ({msg.pose.pose.position.x:.2f}, "
            f"{msg.pose.pose.position.y:.2f}) với stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )
        self.initial_pose_pub.publish(msg)
        return


def create_pose_stamped(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    """Tạo PoseStamped trên hệ toạ độ map với góc xoay yaw (radian)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Chuyển đổi yaw sang quaternion (chỉ quay quanh trục Z)
    half_yaw = yaw * 0.5
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose


def ensure_nodes_active(navigator: BasicNavigator, node_names: list, timeout_sec: float = 15.0):
    """Đảm bảo tất cả lifecycle nodes trong danh sách đã chuyển sang trạng thái 'active'."""
    start_time = time.time()
    for name in node_names:
        get_client = navigator.create_client(GetState, f'/{name}/get_state')
        change_client = navigator.create_client(ChangeState, f'/{name}/change_state')

        if not get_client.wait_for_service(timeout_sec=5.0):
            print(f'    [WARN] Không tìm thấy /{name}/get_state, bỏ qua.')
            continue

        while time.time() - start_time < timeout_sec:
            req = GetState.Request()
            fut = get_client.call_async(req)
            rclpy.spin_until_future_complete(navigator, fut, timeout_sec=2.0)
            res = fut.result()
            if res:
                state = res.current_state.label
                if state == 'active':
                    print(f'    -> Node {name}: [ACTIVE]')
                    break
                elif state == 'inactive':
                    print(f'    -> Node {name} đang inactive, gửi lệnh activate...')
                    req_act = ChangeState.Request()
                    req_act.transition.id = Transition.TRANSITION_ACTIVATE
                    req_act.transition.label = 'activate'
                    fut_act = change_client.call_async(req_act)
                    rclpy.spin_until_future_complete(navigator, fut_act, timeout_sec=5.0)
                elif state == 'unconfigured':
                    print(f'    -> Node {name} đang unconfigured, gửi lệnh configure...')
                    req_cfg = ChangeState.Request()
                    req_cfg.transition.id = Transition.TRANSITION_CONFIGURE
                    req_cfg.transition.label = 'configure'
                    fut_cfg = change_client.call_async(req_cfg)
                    rclpy.spin_until_future_complete(navigator, fut_cfg, timeout_sec=5.0)
            time.sleep(0.5)


def main():
    # Khởi tạo rclpy với use_sim_time:=true ngay từ đầu
    rclpy.init(args=sys.argv + ['--ros-args', '-p', 'use_sim_time:=true'])
    navigator = TurtleBot4Navigator()

    print('\n====================================================================')
    print('   TURTLEBOT 4 - AUTONOMOUS NAVIGATION (03 GOALS) & YOLO DETECTION  ')
    print('====================================================================')

    print('Đang đồng bộ simulation clock từ Gazebo...')
    while rclpy.ok() and navigator.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(navigator, timeout_sec=0.1)

    # 1. Tự động Undock robot nếu đang ở trạm sạc trước khi bắt đầu lộ trình
    print('>>> [BƯỚC 1/4] Kiểm tra trạng thái trạm sạc và thực hiện Undock...')
    try:
        from rclpy.action import ActionClient
        from irobot_create_msgs.action import Undock
        undock_client = ActionClient(navigator, Undock, '/undock')
        if undock_client.wait_for_server(timeout_sec=15.0):
            print('>>> Action server /undock sẵn sàng. Gửi yêu cầu Undock...')
            goal_msg = Undock.Goal()
            future = undock_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=10.0)
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                print('>>> Yêu cầu Undock được chấp nhận, đang rời trạm sạc...')
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(navigator, res_future, timeout_sec=25.0)
                print('>>> [THÀNH CÔNG] Robot đã Undock ra khỏi trạm sạc hoàn toàn!\n')
            else:
                print('>>> Robot không ở trên trạm sạc hoặc goal undock đã được xử lý.\n')
        else:
            print('>>> Không tìm thấy action server /undock (robot có thể đã ở ngoài dock).\n')
    except Exception as e:
        print(f'>>> Bỏ qua undock: {e}\n')

    time.sleep(2.0)

    # 2. Đảm bảo Map Server & AMCL đã ở trạng thái active
    print('>>> [BƯỚC 2/4] Kiểm tra và kích hoạt Localization stack (map_server, amcl)...')
    ensure_nodes_active(navigator, ['map_server', 'amcl'])

    # 3. Thiết lập Initial Pose cho AMCL tại vị trí xuất phát sau khi Undock (hướng yaw = 180 độ)
    print('>>> Khởi tạo Initial Pose cho AMCL tại vị trí x = 0.35 m, y = 0.0 m, yaw = 180° (3.14 rad)...')
    init_pose = create_pose_stamped(navigator, 0.35, 0.0, 3.14159)
    navigator.setInitialPose(init_pose)
    time.sleep(1.0)

    # Gửi yêu cầu no-motion update để AMCL cập nhật ngay transform map->odom
    try:
        nomotion_client = navigator.create_client(Empty, '/request_nomotion_update')
        if nomotion_client.wait_for_service(timeout_sec=2.0):
            future = nomotion_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
            print('>>> [AMCL] Đã cập nhật no-motion update cho particle filter!\n')
    except Exception:
        pass

    # 4. Đảm bảo toàn bộ Nav2 Navigation nodes đã active
    print('>>> [BƯỚC 3/4] Kiểm tra và kích hoạt Navigation stack (planner, controller, bt_navigator)...')
    ensure_nodes_active(
        navigator,
        [
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'velocity_smoother',
        ],
    )
    print('>>> Nav2 đã sẵn sàng hoạt động! <<<\n')

    # Danh sách 03 tọa độ mục tiêu (Goal 1 -> Goal 2 -> Goal 3) trong hệ tọa độ map:
    survey_file = '/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/surveyed_goals.json'
    goals = [
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
    import os
    if os.path.isfile(survey_file):
        try:
            with open(survey_file, 'r', encoding='utf-8') as f:
                loaded = json.load(f).get('goals', [])
            if len(loaded) >= 3:
                goals = loaded
                print(f'>>> Đã nạp {len(goals)} tọa độ mục tiêu từ file: {survey_file}\n')
        except Exception as e:
            print(f'>>> Sử dụng 03 tọa độ mặc định ({e})\n')

    total_goals = len(goals)
    completed_goals = 0
    goal_detections_log = {}

    print('>>> [BƯỚC 4/4] Bắt đầu điều hướng tự động qua 03 Navigation Goals (Goal 1 -> Goal 2 -> Goal 3)...\n')

    for idx, g in enumerate(goals, 1):
        yaw_deg = math.degrees(g['yaw'])
        print('====================================================================')
        print(f" [MỤC TIÊU {idx}/{total_goals}] {g['name']}")
        print(f" Toạ độ đích (map): x = {g['x']:.2f} m, y = {g['y']:.2f} m, yaw = {g['yaw']:.2f} rad ({yaw_deg:.1f}°)")
        print('====================================================================')

        navigator.latest_yolo_detections = []
        goal_pose = create_pose_stamped(navigator, g['x'], g['y'], g['yaw'])
        navigator.goToPose(goal_pose)

        start_time = time.time()
        last_feedback_time = start_time
        close_count = 0
        force_succeed = False

        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.05)
            feedback = navigator.getFeedback()
            now = time.time()

            if feedback:
                distance_remaining = feedback.distance_remaining
                if distance_remaining <= 0.28:
                    close_count += 1
                else:
                    close_count = 0

                # Chờ đủ 12 chu kỳ (6 giây) ở khoảng cách <= 0.28m để robot kịp xoay đúng góc yaw hướng vào vật thể
                if close_count >= 12:
                    print(
                        f'  -> Robot đã tiếp cận và định hướng tại đích ({distance_remaining:.2f} m)! Hoàn thành mục tiêu.'
                    )
                    navigator.cancelTask()
                    force_succeed = True
                    break

                if now - last_feedback_time >= 2.0:
                    last_feedback_time = now
                    nav_time = feedback.navigation_time.sec
                    yolo_info = ''
                    if navigator.latest_yolo_detections:
                        objs = ', '.join(
                            f"{d['class']}({d['confidence']*100:.0f}%)"
                            for d in navigator.latest_yolo_detections
                        )
                        yolo_info = f' | YOLO: [{objs}]'
                    print(
                        f'  -> Đang di chuyển... Khoảng cách còn lại: {distance_remaining:.2f} m '
                        f'(Thời gian: {nav_time}s){yolo_info}'
                    )
            time.sleep(0.45)

        result = navigator.getResult()
        elapsed = time.time() - start_time

        if result == TaskResult.SUCCEEDED or force_succeed:
            completed_goals += 1
            print(f"  [THÀNH CÔNG] Đã đến {g['name']} an toàn trong {elapsed:.1f} giây!")
        elif result == TaskResult.CANCELED:
            print(f'  [BỊ HỦY] Goal {idx} đã bị hủy bỏ!')
        elif result == TaskResult.FAILED:
            print(f'  [THẤT BẠI] Goal {idx} thất bại! Nav2 không thể tìm hoặc đi tới điểm đích.')
        else:
            print(f'  [KHÔNG XÁC ĐỊNH] Trạng thái: {result}')

        # Dừng 3.5 giây tại điểm mục tiêu để Camera OAK-D và YOLOv8 thu thập và hiển thị rõ các vật thể
        print(f'  -> Dừng 3.5s tại Goal {idx} để Camera OAK-D + YOLOv8 quét nhận diện vật thể...')
        scan_end = time.time() + 3.5
        collected_at_goal = {}
        while time.time() < scan_end:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            for d in navigator.latest_yolo_detections:
                cls_name = d['class']
                conf = d['confidence']
                if cls_name not in collected_at_goal or conf > collected_at_goal[cls_name]:
                    collected_at_goal[cls_name] = conf
            time.sleep(0.1)

        goal_detections_log[g['name']] = collected_at_goal
        if collected_at_goal:
            det_str = ', '.join(f'{k} ({v*100:.1f}%)' for k, v in collected_at_goal.items())
            print(f'  [KẾT QUẢ YOLO TẠI GOAL {idx}] Phát hiện: {det_str}\n')
        else:
            print(f'  [KẾT QUẢ YOLO TẠI GOAL {idx}] Đang chờ luồng ảnh từ node yolo_detector...\n')

    print('\n==================== TỔNG KẾT HÀNH TRÌNH NAV2 & YOLO ====================')
    print(f' Số mục tiêu hoàn thành: {completed_goals}/{total_goals}')
    for g_name, dets in goal_detections_log.items():
        if dets:
            det_str = ', '.join(f'{k} ({v*100:.1f}%)' for k, v in dets.items())
            print(f'  * {g_name}: Phát hiện [{det_str}]')
        else:
            print(f'  * {g_name}: Hoàn thành điều hướng')
    if navigator.all_detected_classes:
        print(f" Tổng hợp các class vật thể YOLO đã phát hiện trên đường đi: {sorted(navigator.all_detected_classes)}")
    if completed_goals == total_goals:
        print('>>> CHÚC MỪNG: Robot đã tự động hoàn thành xuất sắc cả 03 Navigation Goals! <<<\n')
    else:
        print('>>> Một số goal chưa hoàn thành. Kiểm tra lại costmap và vị trí đặt goal! <<<\n')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
