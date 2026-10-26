#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 2: ROS 2 Node nhận diện cử chỉ tay bằng AI (MediaPipe Gesture Recognizer + OpenCV)
Node: gesture_recognition_node

Chức năng:
1. Đọc hình ảnh thời gian thực từ Webcam laptop bằng OpenCV.
2. Sử dụng MediaPipe Gesture Recognizer kết hợp phân tích động học bàn tay (21 Landmarks)
   để phát hiện và phân loại chính xác 06 cử chỉ tay:
   - Wave        : Vẫy tay (Phát hiện dao động ngang của bàn tay xòe hoặc cử chỉ tĩnh ILoveYou)
   - Pointing_Up : Giơ 1 ngón tay (Chọn hành vi tiến-lùi 4 lần)
   - Victory     : Giơ 2 ngón tay chữ V (Chọn hành vi di chuyển quỹ đạo tròn)
   - Thumbs_Up   : Giơ ngón cái (Xác nhận và thực hiện hành vi đã chọn)
   - Open_Palm   : Xòe bàn tay giữ tĩnh (Robot dừng ngay lập tức)
   - Closed_Fist : Nắm tay (Hủy hành vi hiện tại và trở về trạng thái chờ)
3. Áp dụng bộ lọc chống nhiễu (Temporal Debounce & Confidence Threshold) tránh kích hoạt sai.
4. Hiển thị khung xương bàn tay, tên cử chỉ, độ tin cậy (%) và trạng thái Robot trên cửa sổ OpenCV.
5. Publish kết quả nhận diện lên topic /detected_gesture (std_msgs/msg/String).
"""

import collections
import json
import math
import os
import sys
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision


# Các kết nối khớp xương bàn tay (21 Landmarks chuẩn của MediaPipe Hands)
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # Ngón cái (Thumb)
    (0, 5), (5, 6), (6, 7), (7, 8),        # Ngón trỏ (Index)
    (5, 9), (9, 10), (10, 11), (11, 12),   # Ngón giữa (Middle)
    (9, 13), (13, 14), (14, 15), (15, 16), # Ngón áp út (Ring)
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)  # Ngón út & Lòng bàn tay (Pinky & Palm)
]

# Mô tả tiếng Việt hiển thị trên giao diện OpenCV cho 06 cử chỉ
GESTURE_DESCRIPTIONS = {
    'Wave': 'Vay tay -> Chon: Xoay trai-phai chao lai',
    'Pointing_Up': 'Gio 1 ngon -> Chon: Tien-lui 4 lan',
    'Victory': 'Gio 2 ngon -> Chon: Quy dao tron',
    'Thumbs_Up': 'Gio ngon cai -> XAC NHAN thuc hien',
    'Open_Palm': 'Xoe ban tay -> DUNG KHAN CAP ngay lap tuc',
    'Closed_Fist': 'Nam tay -> HUY hanh vi, ve trang thai cho',
    'None': 'Khong co cu chi (Dua tay vao khung hinh)',
}

# Màu sắc BGR cho từng cử chỉ trên HUD OpenCV
GESTURE_COLORS = {
    'Wave': (255, 200, 0),         # Xanh lam sáng
    'Pointing_Up': (255, 165, 60), # Xanh dương nhạt
    'Victory': (255, 100, 255),    # Tím hồng
    'Thumbs_Up': (0, 255, 0),      # Xanh lá (Xác nhận)
    'Open_Palm': (0, 0, 255),      # Đỏ (Dừng khẩn cấp)
    'Closed_Fist': (0, 165, 255),  # Cam (Hủy hành vi)
    'None': (160, 160, 160),       # Xám
}


def resolve_model_path() -> str:
    """Tìm đường dẫn file model gesture_recognizer.task hoặc tự động tải nếu chưa có."""
    candidates = []
    try:
        pkg_share = get_package_share_directory('turtlebot4_practice4')
        candidates.append(os.path.join(pkg_share, 'config', 'gesture_recognizer.task'))
    except Exception:
        pass

    candidates.extend([
        '/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/gesture_recognizer.task',
        os.path.join(os.path.dirname(__file__), '..', 'config', 'gesture_recognizer.task'),
        '/tmp/gesture_recognizer.task',
    ])

    for path in candidates:
        if os.path.isfile(path) and os.path.getsize(path) > 100_000:
            return os.path.abspath(path)

    # Tải tự động nếu chưa tồn tại
    target = '/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/gesture_recognizer.task'
    os.makedirs(os.path.dirname(target), exist_ok=True)
    url = (
        'https://storage.googleapis.com/mediapipe-models/'
        'gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'
    )
    urllib.request.urlretrieve(url, target)
    return target


class GestureRecognitionNode(Node):
    """
    ROS 2 Node: gesture_recognition_node
    Đọc hình ảnh từ webcam bằng OpenCV, nhận diện tối thiểu 05 (thực tế 06) cử chỉ tay
    bằng MediaPipe Gesture Recognizer và publish lên topic /detected_gesture (std_msgs/msg/String).
    """

    def __init__(self):
        super().__init__('gesture_recognition_node')

        # Khai báo các tham số cấu hình ROS 2
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('min_confidence', 0.58)
        self.declare_parameter('debounce_frames', 4)
        self.declare_parameter('wave_window_sec', 0.85)
        self.declare_parameter('wave_min_span_x', 0.055)
        self.declare_parameter('wave_min_path_x', 0.115)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('headless', False)
        self.declare_parameter('model_path', '')

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.debounce_frames = int(self.get_parameter('debounce_frames').value)
        self.wave_window_sec = float(self.get_parameter('wave_window_sec').value)
        self.wave_min_span_x = float(self.get_parameter('wave_min_span_x').value)
        self.wave_min_path_x = float(self.get_parameter('wave_min_path_x').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.headless = bool(self.get_parameter('headless').value)
        custom_model_path = str(self.get_parameter('model_path').value).strip()

        # Publisher chính theo yêu cầu đề bài: /detected_gesture (std_msgs/msg/String)
        self.pub_gesture = self.create_publisher(String, '/detected_gesture', 10)
        # Publisher phụ cung cấp thêm metadata (độ tin cậy, raw gesture)
        self.pub_gesture_info = self.create_publisher(String, '/detected_gesture_info', 10)
        # Publisher hình ảnh nhận diện cử chỉ tay lên RViz2
        self.pub_gesture_img = self.create_publisher(Image, '/gesture/detections/image', 5)
        self.pub_rviz_img = self.create_publisher(Image, '/yolo/detections/image', 5)

        # Subscribe trạng thái của robot_behavior_node để hiển thị trực quan lên cửa sổ Webcam
        self.robot_status_text = 'IDLE: Dang cho chon hanh vi (Wave / Pointing_Up / Victory)'
        self.sub_robot_status = self.create_subscription(
            String,
            '/robot_behavior_status',
            self._robot_status_cb,
            10
        )

        # Khởi tạo MediaPipe Gesture Recognizer
        model_path = custom_model_path if custom_model_path else resolve_model_path()
        self.get_logger().info(f'Sử dụng MediaPipe model: {model_path}')

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.55,
            min_hand_presence_confidence=0.55,
            min_tracking_confidence=0.55,
        )
        self.recognizer = mp_vision.GestureRecognizer.create_from_options(options)

        # Bộ đệm theo dõi chuyển động ngang của bàn tay để phát hiện cử chỉ "Vẫy tay (Wave)"
        self.hand_x_history = collections.deque(maxlen=45)  # lưu (timestamp, palm_x)
        self.last_wave_detected_time = 0.0
        self.wave_hold_duration = 0.55  # giữ trạng thái Wave mượt qua các điểm đổi chiều lắc tay

        # Bộ đệm chống nhiễu (Temporal Debounce) qua nhiều frame liên tiếp
        self.gesture_history = collections.deque(maxlen=max(self.debounce_frames, 3))
        self.stable_gesture = 'None'
        self.stable_confidence = 0.0
        self.last_logged_gesture = None
        self.last_publish_time = 0.0

        # Mở Webcam bằng OpenCV
        self.cap = cv2.VideoCapture(self.camera_id)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.get_logger().info(
                f'Đã kết nối Webcam (/dev/video{self.camera_id}) ở độ phân giải '
                f'{self.frame_width}x{self.frame_height}.'
            )
        else:
            self.get_logger().warn(
                f'Không thể mở /dev/video{self.camera_id}! Chạy ở chế độ khung hình giả lập.'
            )

        self.prev_frame_time = time.time()
        self.fps = 0.0

        # Timer xử lý khung hình ở tần số ~30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self._process_frame_cb)

        self.get_logger().info('====================================================================')
        self.get_logger().info(' NODE NHẬN DIỆN CỬ CHỈ TAY AI (gesture_recognition_node) SẴN SÀNG! ')
        self.get_logger().info(' Topic publish: /detected_gesture (std_msgs/msg/String)             ')
        self.get_logger().info(' Danh sách 06 cử chỉ hỗ trợ:                                        ')
        self.get_logger().info('  1. Wave        (Vẫy tay / ILoveYou) -> Xoay trái-phải chào lại     ')
        self.get_logger().info('  2. Pointing_Up (Giơ 1 ngón trỏ)     -> Tiến-lùi 4 lần              ')
        self.get_logger().info('  3. Victory     (Giơ 2 ngón chữ V)   -> Di chuyển quỹ đạo tròn      ')
        self.get_logger().info('  4. Thumbs_Up   (Giơ ngón cái)       -> Xác nhận thực hiện hành vi  ')
        self.get_logger().info('  5. Open_Palm   (Xòe bàn tay tĩnh)   -> Dừng robot ngay lập tức     ')
        self.get_logger().info('  6. Closed_Fist (Nắm tay)            -> Hủy hành vi, về chờ (IDLE)  ')
        self.get_logger().info('====================================================================')

    def _robot_status_cb(self, msg: String):
        self.robot_status_text = msg.data

    @staticmethod
    def _count_extended_fingers(landmarks) -> tuple:
        """
        Kiểm tra trạng thái duỗi của 4 ngón dài (trỏ, giữa, áp út, út) dựa vào tọa độ chuẩn hóa.
        Trả về (index_up, middle_up, ring_up, pinky_up, count_4_fingers).
        """
        # Trong hệ tọa độ ảnh, trục y hướng xuống dưới nên đầu ngón (tip) cao hơn khớp PIP nghĩa là y_tip < y_pip
        wrist = landmarks[0]
        index_up = landmarks[8].y < landmarks[6].y - 0.02
        middle_up = landmarks[12].y < landmarks[10].y - 0.02
        ring_up = landmarks[16].y < landmarks[14].y - 0.02
        pinky_up = landmarks[20].y < landmarks[18].y - 0.02

        # Đảm bảo bàn tay đang hướng lên (khớp MCP cao hơn hoặc ngang cổ tay)
        upright = landmarks[9].y < wrist.y + 0.05
        if not upright:
            return False, False, False, False, 0

        count = int(index_up) + int(middle_up) + int(ring_up) + int(pinky_up)
        return index_up, middle_up, ring_up, pinky_up, count

    def _detect_dynamic_wave(self, landmarks, now: float) -> tuple:
        """
        Phát hiện cử chỉ Vẫy tay (Wave) dựa trên dao động ngang của bàn tay mở.
        Trả về (is_waving: bool, wave_confidence: float).
        """
        # Tâm lòng bàn tay tính trung bình giữa cổ tay (0) và khớp gốc ngón giữa (9)
        palm_x = 0.5 * (landmarks[0].x + landmarks[9].x)
        self.hand_x_history.append((now, palm_x))

        # Loại bỏ các mẫu cũ ngoài cửa sổ wave_window_sec
        while self.hand_x_history and (now - self.hand_x_history[0][0]) > self.wave_window_sec:
            self.hand_x_history.popleft()

        if len(self.hand_x_history) < 6:
            return False, 0.0

        xs = [pt[1] for pt in self.hand_x_history]
        span_x = max(xs) - min(xs)
        path_x = sum(abs(xs[i] - xs[i - 1]) for i in range(1, len(xs)))

        # Bàn tay vẫy khi biên độ ngang và tổng quãng đường di chuyển ngang vượt ngưỡng
        if span_x >= self.wave_min_span_x and path_x >= self.wave_min_path_x:
            self.last_wave_detected_time = now
            conf = min(0.99, 0.82 + (span_x - self.wave_min_span_x) * 1.5)
            return True, conf

        # Duy trì trạng thái Wave ngắn hạn khi tay đang ở điểm đổi chiều vẫy
        if (now - self.last_wave_detected_time) <= self.wave_hold_duration and span_x >= 0.035:
            return True, 0.86

        return False, 0.0

    def _classify_gesture(self, mp_result, now: float) -> tuple:
        """
        Kết hợp kết quả từ MediaPipe Gesture Recognizer và phân tích động học 21 Landmarks
        để phân loại thành 1 trong 06 cử chỉ chuẩn:
        ('Wave', 'Pointing_Up', 'Victory', 'Thumbs_Up', 'Open_Palm', 'Closed_Fist') hoặc ('None', 0.0).
        """
        if not mp_result.hand_landmarks:
            self.hand_x_history.clear()
            return 'None', 0.0, 'None'

        landmarks = mp_result.hand_landmarks[0]
        raw_category = 'None'
        raw_score = 0.0

        if mp_result.gestures and mp_result.gestures[0]:
            top_gesture = mp_result.gestures[0][0]
            raw_category = top_gesture.category_name or 'None'
            raw_score = float(top_gesture.score)

        index_up, middle_up, ring_up, pinky_up, open_fingers = self._count_extended_fingers(landmarks)
        is_waving, wave_conf = self._detect_dynamic_wave(landmarks, now)

        # 1. Cử chỉ Vẫy tay (Wave):
        #    - Khi bàn tay mở (>= 3 ngón duỗi hoặc MediaPipe nhận Open_Palm) VÀ có chuyển động vẫy ngang
        #    - Hoặc khi người dùng giơ cử chỉ tĩnh ILoveYou của MediaPipe (phương án dự phòng tĩnh)
        if (open_fingers >= 3 or raw_category == 'Open_Palm') and is_waving:
            return 'Wave', max(raw_score, wave_conf), f'Wave(Dynamic:{raw_category})'

        if raw_category == 'ILoveYou' and raw_score >= self.min_confidence:
            return 'Wave', raw_score, 'ILoveYou->Wave'

        # 2. Ánh xạ trực tiếp các cử chỉ chuẩn của MediaPipe Gesture Recognizer
        mapping = {
            'Pointing_Up': 'Pointing_Up',
            'Victory': 'Victory',
            'Thumb_Up': 'Thumbs_Up',
            'Open_Palm': 'Open_Palm',
            'Closed_Fist': 'Closed_Fist',
        }

        if raw_category in mapping and raw_score >= self.min_confidence:
            mapped = mapping[raw_category]
            # Nếu là Open_Palm vừa mới bắt đầu di chuyển ngang mạnh, chờ 1 nhịp để phân biệt với Wave
            return mapped, raw_score, raw_category

        # 3. Hỗ trợ hình học bổ trợ khi MediaPipe trả về 'None' do góc nghiêng nhẹ của ngón tay
        if raw_category == 'None':
            if index_up and not middle_up and not ring_up and not pinky_up:
                return 'Pointing_Up', 0.75, 'Landmark_Pointing_Up'
            if index_up and middle_up and not ring_up and not pinky_up:
                return 'Victory', 0.75, 'Landmark_Victory'

        return 'None', raw_score, raw_category

    def _update_debounce_filter(self, detected_gesture: str, confidence: float) -> tuple:
        """
        Bộ lọc chống nhiễu qua nhiều khung hình liên tiếp:
        - Yêu cầu cử chỉ xuất hiện ổn định trong cửa sổ debounce_frames mới đổi trạng thái.
        - Riêng Open_Palm (Dừng khẩn cấp) chỉ cần 2/4 frame để đảm bảo phản ứng dừng tức thì.
        """
        self.gesture_history.append((detected_gesture, confidence))
        counts = collections.Counter(g for g, _ in self.gesture_history)
        most_common_gesture, count = counts.most_common(1)[0]

        # Tính độ tin cậy trung bình của cử chỉ chiếm đa số
        confs = [c for g, c in self.gesture_history if g == most_common_gesture]
        avg_conf = sum(confs) / len(confs) if confs else 0.0

        required_count = max(2, self.debounce_frames - 1)
        if most_common_gesture == 'Open_Palm':
            # Ưu tiên lệnh dừng khẩn cấp phản hồi nhanh
            required_count = min(2, self.debounce_frames)

        if count >= required_count:
            self.stable_gesture = most_common_gesture
            self.stable_confidence = avg_conf

        stability_ratio = min(1.0, count / float(max(1, required_count)))
        return self.stable_gesture, self.stable_confidence, stability_ratio

    def _draw_landmarks_and_hud(
        self,
        frame: np.ndarray,
        mp_result,
        gesture: str,
        confidence: float,
        stability: float,
    ):
        """Vẽ khung xương bàn tay (21 Landmarks), Bounding Box, tên cử chỉ và độ tin cậy lên cửa sổ OpenCV."""
        h, w = frame.shape[:2]
        color = GESTURE_COLORS.get(gesture, (200, 200, 200))

        # 1. Vẽ 21 Hand Landmarks và Bounding Box nếu phát hiện bàn tay
        if mp_result.hand_landmarks:
            landmarks = mp_result.hand_landmarks[0]
            pts = []
            for lm in landmarks:
                px = int(np.clip(lm.x * w, 0, w - 1))
                py = int(np.clip(lm.y * h, 0, h - 1))
                pts.append((px, py))

            # Vẽ các đoạn nối xương bàn tay
            for start_idx, end_idx in HAND_CONNECTIONS:
                cv2.line(frame, pts[start_idx], pts[end_idx], (0, 230, 180), 2, cv2.LINE_AA)

            # Vẽ các khớp ngón tay
            for idx, (px, py) in enumerate(pts):
                radius = 6 if idx in (0, 4, 8, 12, 16, 20) else 4
                pt_color = (0, 0, 255) if idx in (4, 8, 12, 16, 20) else (255, 255, 255)
                cv2.circle(frame, (px, py), radius, pt_color, -1, cv2.LINE_AA)
                cv2.circle(frame, (px, py), radius + 1, (20, 20, 20), 1, cv2.LINE_AA)

            # Tính Bounding Box quanh bàn tay
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x1, y1 = max(0, min(xs) - 22), max(0, min(ys) - 22)
            x2, y2 = min(w - 1, max(xs) + 22), min(h - 1, max(ys) + 22)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label = f'{gesture} ({confidence * 100:.1f}%)'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            label_y = max(y1 - 8, th + 10)
            cv2.rectangle(
                frame,
                (x1, label_y - th - 8),
                (x1 + tw + 12, label_y + 4),
                color,
                -1,
            )
            cv2.putText(
                frame,
                label,
                (x1 + 6, label_y - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (15, 15, 15),
                2,
                cv2.LINE_AA,
            )

        # 2. Vẽ Panel thông tin phía trên (Header HUD)
        overlay = frame.copy()
        cv2.rectangle(overlay, (8, 8), (w - 8, 142), (18, 18, 24), -1)
        cv2.addWeighted(overlay, 0.72, frame, 0.28, 0, frame)

        cv2.putText(
            frame,
            f'MEDIAPIPE GESTURE RECOGNIZER | FPS: {self.fps:.1f}',
            (18, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        conf_pct = confidence * 100.0 if gesture != 'None' else 0.0
        cv2.putText(
            frame,
            f'Gesture: {gesture}   |   Confidence: {conf_pct:.1f}%',
            (18, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            color,
            2,
            cv2.LINE_AA,
        )

        desc = GESTURE_DESCRIPTIONS.get(gesture, '')
        cv2.putText(
            frame,
            f'Action : {desc}',
            (18, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )

        # Hiển thị trạng thái thực thi hiện tại của Robot
        cv2.putText(
            frame,
            f'Robot  : {self.robot_status_text[:68]}',
            (18, 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (120, 255, 140),
            1,
            cv2.LINE_AA,
        )

        # Thanh tiến trình ổn định chống nhiễu (Debounce Bar)
        bar_x, bar_y, bar_w, bar_h = 18, 124, 240, 10
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (80, 80, 80), 1)
        fill_w = int(bar_w * (stability if gesture != 'None' else 0.0))
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), color, -1)
        cv2.putText(
            frame,
            'Anti-Noise Lock',
            (bar_x + bar_w + 10, bar_y + 9),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )

        # 3. Bảng chú giải nhanh 06 cử chỉ ở góc dưới màn hình
        footer = frame.copy()
        cv2.rectangle(footer, (8, h - 78), (w - 8, h - 8), (18, 18, 24), -1)
        cv2.addWeighted(footer, 0.68, frame, 0.32, 0, frame)
        cv2.putText(
            frame,
            '[Chon Hanh Vi] Wave (Vay tay): Chao | Pointing_Up (1 ngon): Tien-Lui 4x | Victory (2 ngon): Di vong tron',
            (15, h - 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.37,
            (215, 240, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            '[Dieu Khien]   Thumbs_Up (Ngon cai): XAC NHAN CHAY | Open_Palm (Xoe tay): DUNG NGAY | Closed_Fist (Nam tay): HUY',
            (15, h - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.37,
            (180, 255, 190),
            1,
            cv2.LINE_AA,
        )

    def _publish_annotated_image(self, bgr_frame: np.ndarray):
        """Chuyển đổi ảnh OpenCV BGR sang sensor_msgs/msg/Image để hiển thị đồng thời trên RViz2."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'webcam_optical_frame'
        h, w = bgr_frame.shape[:2]
        msg.height = int(h)
        msg.width = int(w)
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(w * 3)
        msg.data = np.ascontiguousarray(bgr_frame).tobytes()
        self.pub_gesture_img.publish(msg)
        self.pub_rviz_img.publish(msg)

    def _process_frame_cb(self):
        now = time.time()
        dt = max(now - self.prev_frame_time, 1e-4)
        self.fps = 0.85 * self.fps + 0.15 * (1.0 / dt)
        self.prev_frame_time = now

        if self.cap.isOpened():
            ret, bgr_frame = self.cap.read()
            if not ret or bgr_frame is None:
                bgr_frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
            else:
                # Lật gương ngang để người dùng thuận tiện vẫy tay và quan sát trực diện
                bgr_frame = cv2.flip(bgr_frame, 1)
        else:
            bgr_frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)

        # Chuyển đổi BGR -> RGB cho MediaPipe GestureRecognizer
        rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        mp_result = self.recognizer.recognize(mp_image)

        # Phân loại cử chỉ và lọc chống nhiễu
        detected_gesture, raw_conf, raw_name = self._classify_gesture(mp_result, now)
        stable_gesture, stable_conf, stability = self._update_debounce_filter(
            detected_gesture, raw_conf
        )

        # Publish lên topic /detected_gesture (std_msgs/msg/String)
        publish_interval = 1.0 / max(1.0, self.publish_rate_hz)
        gesture_changed = stable_gesture != self.last_logged_gesture
        if gesture_changed or (now - self.last_publish_time) >= publish_interval:
            msg = String()
            msg.data = stable_gesture
            self.pub_gesture.publish(msg)

            info_msg = String()
            info_msg.data = json.dumps({
                'gesture': stable_gesture,
                'raw_gesture': raw_name,
                'confidence': round(float(stable_conf), 4),
                'timestamp': round(now, 3),
            })
            self.pub_gesture_info.publish(info_msg)
            self.last_publish_time = now

            if gesture_changed:
                self.last_logged_gesture = stable_gesture
                if stable_gesture != 'None':
                    self.get_logger().info(
                        f'[PHÁT HIỆN CỬ CHỈ] {stable_gesture:<12} | '
                        f'Độ tin cậy: {stable_conf * 100:5.1f}% | '
                        f'{GESTURE_DESCRIPTIONS.get(stable_gesture, "")}'
                    )
                else:
                    self.get_logger().info('[TRẠNG THÁI TAY] Không có cử chỉ (None)')

        # Hiển thị trên cửa sổ OpenCV và publish lên RViz2
        self._draw_landmarks_and_hud(
            bgr_frame, mp_result, stable_gesture, stable_conf, stability
        )
        self._publish_annotated_image(bgr_frame)

        if not self.headless and os.environ.get('DISPLAY'):
            cv2.imshow('TurtleBot 4 - AI Hand Gesture Recognition (MediaPipe)', bgr_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self.get_logger().info('Đóng cửa sổ nhận diện cử chỉ tay theo yêu cầu người dùng.')
                rclpy.shutdown()

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap.isOpened():
                self.cap.release()
            if hasattr(self, 'recognizer') and self.recognizer is not None:
                self.recognizer.close()
            if not self.headless and os.environ.get('DISPLAY'):
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

