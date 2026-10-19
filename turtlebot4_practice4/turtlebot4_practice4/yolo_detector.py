#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 4: ROS 2 Node nhận diện vật thể từ Camera RGB của TurtleBot 4 bằng mô hình YOLOv8
Chức năng:
1. Subscribe dữ liệu hình ảnh từ camera RGB của TurtleBot 4 (/oakd/rgb/preview/image_raw).
2. Chuyển đổi message ảnh của ROS 2 (sensor_msgs/msg/Image) sang định dạng OpenCV bằng cv_bridge.
3. Đưa hình ảnh qua mô hình YOLOv8 (yolov8n.pt) để nhận diện các vật thể trong môi trường Gazebo.
4. Hiển thị kết quả nhận diện:
   - Vẽ Bounding Box, tên class và độ tự tin (Confidence) lên cửa sổ OpenCV (cv2.imshow).
   - Publish ngược lại lên topic /yolo/detections/image (sensor_msgs/msg/Image) để xem trên RViz2.
   - Publish tóm tắt JSON/text lên /yolo/detections/summary để đồng bộ với Node điều hướng.
   - In thông tin vật thể phát hiện được kèm tọa độ robot hiện tại ra màn hình Terminal.
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge.core import CvBridge


class CompatCvBridge(CvBridge):
    """
    Lớp mở rộng của cv_bridge.CvBridge đảm bảo tương thích hoàn toàn
    với cả NumPy 1.x / 2.x và OpenCV 4.x / 5.x trong ROS 2 Humble.
    """

    _DIRECT_ENCODINGS = {
        'rgb8': ('uint8', 3),
        'bgr8': ('uint8', 3),
        'rgba8': ('uint8', 4),
        'bgra8': ('uint8', 4),
        'mono8': ('uint8', 1),
        '8UC1': ('uint8', 1),
        '8UC3': ('uint8', 3),
        '8UC4': ('uint8', 4),
        'mono16': ('uint16', 1),
        '16UC1': ('uint16', 1),
        '32FC1': ('float32', 1),
    }

    def __init__(self):
        super().__init__()
        for depth_idx, t in enumerate(['8U', '8S', '16U', '16S', '32S', '32F', '64F']):
            for c in [1, 2, 3, 4]:
                cv4_code = depth_idx + ((c - 1) << 3)
                self.cvtype_to_name[cv4_code] = f'{t}C{c}'

    def encoding_to_dtype_with_channels(self, encoding: str):
        if encoding in self._DIRECT_ENCODINGS:
            return self._DIRECT_ENCODINGS[encoding]
        return super().encoding_to_dtype_with_channels(encoding)

    def encoding_to_cvtype2(self, encoding: str) -> int:
        if encoding in ('rgb8', 'bgr8', '8UC3'):
            return 16  # CV_8UC3
        if encoding in ('rgba8', 'bgra8', '8UC4'):
            return 24  # CV_8UC4
        if encoding in ('mono8', '8UC1'):
            return 0   # CV_8UC1
        return super().encoding_to_cvtype2(encoding)

    def imgmsg_to_cv2(self, img_msg: Image, desired_encoding: str = 'passthrough') -> np.ndarray:
        dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = np.dtype(dtype)
        img_buf = (
            np.asarray(img_msg.data, dtype=dtype)
            if isinstance(img_msg.data, list)
            else img_msg.data
        )

        if n_channels == 1:
            im = np.ndarray(
                shape=(img_msg.height, int(img_msg.step // dtype.itemsize)),
                dtype=dtype,
                buffer=img_buf,
            )
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width])
        else:
            im = np.ndarray(
                shape=(
                    img_msg.height,
                    int(img_msg.step // dtype.itemsize // n_channels),
                    n_channels,
                ),
                dtype=dtype,
                buffer=img_buf,
            )
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width, :])

        if desired_encoding in ('passthrough', img_msg.encoding):
            return im
        if img_msg.encoding == 'rgb8' and desired_encoding == 'bgr8':
            return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        if img_msg.encoding == 'bgr8' and desired_encoding == 'rgb8':
            return cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        if img_msg.encoding == 'rgba8' and desired_encoding == 'bgr8':
            return cv2.cvtColor(im, cv2.COLOR_RGBA2BGR)
        if img_msg.encoding == 'bgra8' and desired_encoding == 'bgr8':
            return cv2.cvtColor(im, cv2.COLOR_BGRA2BGR)
        return im


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# Bảng màu BGR nổi bật cho các nhóm đối tượng
CLASS_COLORS = {
    'person': (0, 255, 0),         # Xanh lá cây sáng
    'chair': (255, 165, 0),        # Cam / Xanh dương
    'couch': (255, 140, 0),
    'dining table': (0, 215, 255), # Vàng
    'bench': (255, 0, 255),        # Tím
    'fire hydrant': (0, 0, 255),   # Đỏ
    'car': (255, 255, 0),          # Cyan
    'truck': (255, 200, 0),
    'suitcase': (180, 105, 255),
}


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        default_weights = '/home/tuan/turtelbot4_ws/yolov8n.pt'
        if not os.path.isfile(default_weights):
            default_weights = 'yolov8n.pt'

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('output_image_topic', '/yolo/detections/image')
        self.declare_parameter('model_path', default_weights)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('show_cv_window', True)
        self.declare_parameter('log_interval_sec', 1.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.show_cv_window = bool(self.get_parameter('show_cv_window').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        # Kiểm tra môi trường GUI cho cv2.imshow
        self.has_display = bool(os.environ.get('DISPLAY')) and self.show_cv_window
        self.window_name = 'TurtleBot 4 OAK-D Camera - YOLOv8 Object Detection'

        # Khởi tạo cv_bridge
        self.bridge = CompatCvBridge()

        # Tải mô hình YOLOv8
        self.get_logger().info(f'Đang nạp mô hình YOLO từ: {self.model_path} ...')
        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        self.get_logger().info(
            f'Đã nạp thành công mô hình YOLO ({len(self.model.names)} classes)! '
            f'Ngưỡng độ tự tin (conf_threshold): {self.conf_threshold}'
        )

        # Lưu vị trí hiện tại của robot từ /amcl_pose
        self.robot_pose = None  # (x, y, yaw)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10,
        )

        # QoS Best Effort cho subscriber để nhận được từ cả publisher Reliable và Best Effort
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # QoS Reliable cho publisher để cả RViz2 Reliable và Best Effort đều xem được
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_image = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sub_qos,
        )
        self.pub_image = self.create_publisher(
            Image,
            self.output_image_topic,
            pub_qos,
        )
        self.pub_summary = self.create_publisher(
            String,
            '/yolo/detections/summary',
            10,
        )

        self.frame_count = 0
        self.last_log_time = 0.0
        self.last_fps_time = time.time()
        self.fps = 0.0

        self.get_logger().info(
            f'Đang lắng nghe ảnh RGB từ "{self.image_topic}" '
            f'và xuất ảnh nhận diện lên "{self.output_image_topic}"...'
        )

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.robot_pose = (x, y, yaw)

    def image_callback(self, msg: Image):
        # 1. Chuyển đổi ROS 2 Image -> OpenCV BGR bằng cv_bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Lỗi chuyển đổi ảnh qua cv_bridge: {exc}')
            return

        now = time.time()
        self.frame_count += 1
        dt = now - self.last_fps_time
        if dt >= 0.5:
            self.fps = self.frame_count / dt
            self.frame_count = 0
            self.last_fps_time = now

        # 2. Đưa ảnh qua mô hình YOLOv8
        results = self.model.predict(
            source=cv_image,
            conf=self.conf_threshold,
            verbose=False,
        )

        annotated = cv_image.copy()
        detections = []

        if results and len(results) > 0:
            res = results[0]
            boxes = res.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    class_name = self.model.names.get(cls_id, str(cls_id))
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

                    detections.append({
                        'class': class_name,
                        'confidence': round(conf, 3),
                        'bbox': [x1, y1, x2, y2],
                    })

                    # Vẽ Bounding Box & nhãn tên class + độ tự tin
                    color = CLASS_COLORS.get(class_name, (0, 255, 128))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

                    label = f'{class_name}: {conf * 100:.1f}%'
                    (tw, th), _ = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                    )
                    y_top = max(y1, th + 6)
                    cv2.rectangle(
                        annotated,
                        (x1, y_top - th - 6),
                        (x1 + tw + 6, y_top),
                        color,
                        -1,
                    )
                    cv2.putText(
                        annotated,
                        label,
                        (x1 + 3, y_top - 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 0),
                        1,
                        cv2.LINE_AA,
                    )

        # Vẽ thanh trạng thái HUD phía trên ảnh
        h, w = annotated.shape[:2]
        cv2.rectangle(annotated, (0, 0), (w, 26), (25, 25, 25), -1)
        pose_str = (
            f'Pose: ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f})'
            if self.robot_pose
            else 'Pose: waiting'
        )
        hud_text = f'YOLOv8 | Objects: {len(detections)} | FPS: {self.fps:.1f} | {pose_str}'
        cv2.putText(
            annotated,
            hud_text,
            (6, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # 3. Publish ngược lại lên Image topic (/yolo/detections/image) để xem trên RViz2
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_image.publish(out_msg)
        except Exception as exc:
            self.get_logger().error(f'Lỗi xuất ảnh nhận diện: {exc}')

        # Publish tóm tắt JSON cho node điều hướng
        summary_payload = {
            'timestamp': now,
            'count': len(detections),
            'detections': detections,
            'robot_pose': list(self.robot_pose) if self.robot_pose else None,
        }
        summary_msg = String()
        summary_msg.data = json.dumps(summary_payload)
        self.pub_summary.publish(summary_msg)

        # 4. Hiển thị cửa sổ OpenCV trực quan
        if self.has_display:
            try:
                # Phóng to nhẹ nếu ảnh preview của OAK-D nhỏ (ví dụ 250x250 -> 500x500) để dễ quan sát
                display_img = (
                    cv2.resize(annotated, (500, 500), interpolation=cv2.INTER_LINEAR)
                    if w < 400
                    else annotated
                )
                cv2.imshow(self.window_name, display_img)
                cv2.waitKey(1)
            except Exception:
                self.has_display = False

        # 5. In thông tin vật thể phát hiện được ra màn hình Terminal
        if detections and (now - self.last_log_time >= self.log_interval_sec):
            self.last_log_time = now
            items_str = ', '.join(
                f"{d['class']} ({d['confidence'] * 100:.1f}%, bbox={d['bbox']})"
                for d in detections
            )
            if self.robot_pose:
                rx, ry, ryaw = self.robot_pose
                self.get_logger().info(
                    f'[YOLO DETECTED] Tại (x={rx:.2f}m, y={ry:.2f}m, yaw={ryaw:.2f}rad) '
                    f'-> Phát hiện {len(detections)} vật thể: {items_str}'
                )
            else:
                self.get_logger().info(
                    f'[YOLO DETECTED] Phát hiện {len(detections)} vật thể: {items_str}'
                )


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
