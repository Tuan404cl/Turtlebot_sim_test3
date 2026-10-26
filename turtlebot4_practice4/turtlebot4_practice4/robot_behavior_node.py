#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 3: ROS 2 Node điều khiển hành vi TurtleBot 4 từ cử chỉ tay AI
Node: robot_behavior_node

Chức năng:
1. Subscribe kết quả nhận diện cử chỉ tay từ topic /detected_gesture (std_msgs/msg/String).
2. Điều khiển hành vi của TurtleBot 4 qua topic /cmd_vel (geometry_msgs/msg/Twist) theo bảng:
   - Vẫy tay (Wave)               : Chọn hành vi xoay trái-phải để chào lại
   - Giơ 1 ngón tay (Pointing_Up) : Chọn hành vi tiến-lùi 4 lần
   - Giơ 2 ngón tay (Victory)     : Chọn hành vi di chuyển theo quỹ đạo tròn
   - Giơ ngón cái (Thumbs_Up)     : Xác nhận và thực hiện hành vi đã chọn
   - Xòe bàn tay (Open_Palm)      : Robot dừng ngay lập tức
   - Nắm tay (Closed_Fist)        : Hủy hành vi hiện tại và trở về trạng thái chờ (IDLE)
3. Đảm bảo đầy đủ các yêu cầu kỹ thuật:
   - Robot chỉ thực hiện hành vi sau khi nhận được cử chỉ xác nhận (Thumbs_Up).
   - Cơ chế chống nhiễu (Debounce + Edge Trigger + Cooldown) tránh kích hoạt lệnh liên tục.
   - Tự động dừng ngay khi nhận tín hiệu dừng (Open_Palm) hoặc khi mất tín hiệu điều khiển (Watchdog Timeout).
   - Giám sát khoảng cách an toàn qua LiDAR (/scan) để đảm bảo di chuyển không va chạm.
   - Tự động cấu hình safety_override='full' trên Create 3 (/motion_control) để hỗ trợ lùi (linear.x < 0).
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


# Chuẩn hóa tên cử chỉ để tương thích với nhiều cách viết khác nhau
GESTURE_ALIASES = {
    'wave': 'Wave',
    'vay_tay': 'Wave',
    'iloveyou': 'Wave',
    'pointing_up': 'Pointing_Up',
    'pointing up': 'Pointing_Up',
    'one_finger': 'Pointing_Up',
    'victory': 'Victory',
    'two_fingers': 'Victory',
    'thumbs_up': 'Thumbs_Up',
    'thumb_up': 'Thumbs_Up',
    'thumbs up': 'Thumbs_Up',
    'open_palm': 'Open_Palm',
    'open palm': 'Open_Palm',
    'stop': 'Open_Palm',
    'closed_fist': 'Closed_Fist',
    'closed fist': 'Closed_Fist',
    'fist': 'Closed_Fist',
    'none': 'None',
    '': 'None',
}

BEHAVIOR_NAMES_VI = {
    'Wave': 'Xoay trai-phai chao lai (Wave Greeting)',
    'Pointing_Up': 'Tien-lui 4 lan (Forward-Backward 4x)',
    'Victory': 'Di chuyen theo quy dao tron (Circular Trajectory)',
}


class RobotBehaviorNode(Node):
    """
    ROS 2 Node: robot_behavior_node
    Máy trạng thái hữu hạn (FSM) điều khiển hành vi TurtleBot 4 qua /cmd_vel dựa trên /detected_gesture.
    """

    # Các trạng thái của Finite State Machine (FSM)
    STATE_IDLE = 'IDLE'           # Trạng thái chờ chọn hành vi
    STATE_SELECTED = 'SELECTED'   # Đã chọn hành vi, đang chờ Thumbs_Up xác nhận
    STATE_EXECUTING = 'EXECUTING' # Đang thực hiện hành vi đã được xác nhận
    STATE_STOPPED = 'STOPPED'     # Đã dừng khẩn cấp (do Open_Palm hoặc mất tín hiệu)

    def __init__(self):
        super().__init__('robot_behavior_node')

        # Tham số cấu hình chuyển động và an toàn
        self.declare_parameter('linear_speed', 0.20)           # Vận tốc tiến/lùi (m/s)
        self.declare_parameter('forward_back_duration', 1.8)   # Thời gian mỗi pha tiến hoặc lùi (s)
        self.declare_parameter('forward_back_Pause', 0.35)     # Thời gian dừng đệm giữa tiến và lùi (s)
        self.declare_parameter('forward_back_reps', 4)         # Số lần tiến-lùi (mặc định: 4 lần)
        self.declare_parameter('wave_angular_speed', 0.85)     # Vận tốc góc khi vẫy chào (rad/s)
        self.declare_parameter('wave_half_swing_sec', 0.80)    # Nửa biên độ lắc chào (s)
        self.declare_parameter('circle_linear_speed', 0.18)    # Vận tốc dài quỹ đạo tròn (m/s)
        self.declare_parameter('circle_angular_speed', 0.45)   # Vận tốc góc quỹ đạo tròn (rad/s) -> R = 0.40m
        self.declare_parameter('signal_timeout_sec', 1.8)      # Thời gian tối đa mất tín hiệu trước khi tự dừng (s)
        self.declare_parameter('command_cooldown_sec', 1.2)    # Cooldown chống kích hoạt lặp lệnh (s)
        self.declare_parameter('min_confirm_frames', 2)        # Số bản tin liên tiếp để xác nhận lệnh chọn/chạy
        self.declare_parameter('obstacle_stop_dist', 0.25)     # Khoảng cách an toàn tối thiểu trên LiDAR (m)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.fb_duration = float(self.get_parameter('forward_back_duration').value)
        self.fb_pause = float(self.get_parameter('forward_back_Pause').value)
        self.fb_reps = int(self.get_parameter('forward_back_reps').value)
        self.wave_w = float(self.get_parameter('wave_angular_speed').value)
        self.wave_half_t = float(self.get_parameter('wave_half_swing_sec').value)
        self.circle_v = float(self.get_parameter('circle_linear_speed').value)
        self.circle_w = float(self.get_parameter('circle_angular_speed').value)
        self.signal_timeout = float(self.get_parameter('signal_timeout_sec').value)
        self.command_cooldown = float(self.get_parameter('command_cooldown_sec').value)
        self.min_confirm_frames = int(self.get_parameter('min_confirm_frames').value)
        self.obstacle_stop_dist = float(self.get_parameter('obstacle_stop_dist').value)

        # Trạng thái FSM
        self.state = self.STATE_IDLE
        self.selected_behavior = None   # 'Wave' | 'Pointing_Up' | 'Victory' | None
        self.active_behavior = None     # Hành vi đang thực thi trong trạng thái EXECUTING
        self.exec_start_time = 0.0
        self.last_progress_log_time = 0.0

        # Biến chống nhiễu & theo dõi tín hiệu
        self.last_msg_time = None
        self.signal_lost_warned = False
        self.candidate_gesture = 'None'
        self.candidate_count = 0
        self.last_triggered_gesture = 'None'
        self.last_trigger_time = 0.0

        # Giám sát vật cản LiDAR (/scan) và Odometry (/odom)
        self.front_clear_dist = float('inf')
        self.rear_clear_dist = float('inf')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Publishers & Subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/robot_behavior_status', 10)

        self.sub_gesture = self.create_subscription(
            String,
            '/detected_gesture',
            self._gesture_callback,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        # Client cấu hình safety_override='full' trên Create 3 để hỗ trợ đi lùi (linear.x < 0)
        self.motion_param_client = self.create_client(
            SetParameters, '/motion_control/set_parameters'
        )
        self.safety_override_configured = False
        self.override_timer = self.create_timer(2.0, self._ensure_safety_override)

        # Vòng lặp điều khiển 20 Hz (50 ms)
        self.control_timer = self.create_timer(0.05, self._control_loop_cb)

        self._publish_status('IDLE: San sang! Hay gio cu chi (Wave / Pointing_Up / Victory) de chon hanh vi')
        self.get_logger().info('====================================================================')
        self.get_logger().info(' NODE ĐIỀU KHIỂN HÀNH VI TURTLEBOT 4 (robot_behavior_node) SẴN SÀNG ')
        self.get_logger().info(' Subscribe : /detected_gesture (std_msgs/msg/String)                ')
        self.get_logger().info(' Publish   : /cmd_vel (geometry_msgs/msg/Twist)                     ')
        self.get_logger().info(' Quy trình : [Chọn hành vi] -> [Giơ Thumbs_Up xác nhận] -> [Thực thi]')
        self.get_logger().info(' An toàn   : Open_Palm (Dừng ngay) | Closed_Fist (Hủy & Về chờ)     ')
        self.get_logger().info('====================================================================')

    def _ensure_safety_override(self):
        """Đảm bảo Create 3 cho phép lùi (safety_override='full') khi thực hiện tiến-lùi 4 lần."""
        if self.safety_override_configured:
            return
        if self.motion_param_client.service_is_ready():
            req = SetParameters.Request()
            param = Parameter()
            param.name = 'safety_override'
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='full'
            )
            req.parameters = [param]
            self.motion_param_client.call_async(req)
            self.safety_override_configured = True
            self.get_logger().info(
                "Đã kích hoạt safety_override='full' trên /motion_control (hỗ trợ tiến & lùi)."
            )

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_callback(self, msg: LaserScan):
        """Tính khoảng cách trống phía trước và phía sau robot để chống va chạm."""
        n = len(msg.ranges)
        if n == 0:
            return
        front_dists = []
        rear_dists = []
        angle = msg.angle_min
        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r) and r > msg.range_min:
                # Chuyển góc tia quét về [-pi, pi]
                norm_ang = math.atan2(math.sin(angle), math.cos(angle))
                if abs(norm_ang) <= math.radians(30):
                    front_dists.append(r)
                elif abs(norm_ang) >= math.radians(150):
                    rear_dists.append(r)
            angle += msg.angle_increment

        self.front_clear_dist = min(front_dists) if front_dists else float('inf')
        self.rear_clear_dist = min(rear_dists) if rear_dists else float('inf')

    def _publish_stop(self):
        """Phát lệnh vận tốc 0.0 lên /cmd_vel để dừng robot ngay lập tức."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(stop_msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    @staticmethod
    def _normalize_gesture(raw_str: str) -> str:
        clean = raw_str.strip()
        if clean in ('Wave', 'Pointing_Up', 'Victory', 'Thumbs_Up', 'Open_Palm', 'Closed_Fist', 'None'):
            return clean
        return GESTURE_ALIASES.get(clean.lower(), 'None')

    def _gesture_callback(self, msg: String):
        """Xử lý bản tin cử chỉ nhận từ /detected_gesture kèm cơ chế chống nhiễu."""
        now = time.time()
        self.last_msg_time = now
        self.signal_lost_warned = False

        gesture = self._normalize_gesture(msg.data)

        # 1. Ưu tiên cao nhất: Cử chỉ DỪNG KHẨN CẤP (Open_Palm) - kích hoạt tức thì!
        if gesture == 'Open_Palm':
            self.candidate_gesture = 'Open_Palm'
            self.candidate_count += 1
            if self.state != self.STATE_STOPPED or self.last_triggered_gesture != 'Open_Palm':
                self._handle_emergency_stop()
                self.last_triggered_gesture = 'Open_Palm'
                self.last_trigger_time = now
            return

        # 2. Ưu tiên số 2: Cử chỉ HỦY HÀNH VI (Closed_Fist)
        if gesture == 'Closed_Fist':
            if self.candidate_gesture == 'Closed_Fist':
                self.candidate_count += 1
            else:
                self.candidate_gesture = 'Closed_Fist'
                self.candidate_count = 1

            # Nếu robot đang chạy (EXECUTING), cho phép hủy ngay ở frame đầu tiên;
            # nếu đang chờ, yêu cầu min_confirm_frames
            needed = 1 if self.state == self.STATE_EXECUTING else self.min_confirm_frames
            if self.candidate_count >= needed and (
                self.last_triggered_gesture != 'Closed_Fist'
                or (now - self.last_trigger_time) > self.command_cooldown
            ):
                self._handle_cancel_behavior()
                self.last_triggered_gesture = 'Closed_Fist'
                self.last_trigger_time = now
            return

        # 3. Khi không có cử chỉ ('None'), reset trạng thái edge-trigger để sẵn sàng nhận lệnh mới
        if gesture == 'None':
            self.candidate_gesture = 'None'
            self.candidate_count = 0
            if (now - self.last_trigger_time) > 0.4:
                self.last_triggered_gesture = 'None'
            return

        # 4. Đếm số bản tin liên tiếp cho các cử chỉ chọn hành vi (Wave, Pointing_Up, Victory) và xác nhận (Thumbs_Up)
        if gesture == self.candidate_gesture:
            self.candidate_count += 1
        else:
            self.candidate_gesture = gesture
            self.candidate_count = 1

        if self.candidate_count < self.min_confirm_frames:
            return

        # Kiểm tra chống lặp lệnh liên tục (Edge-Trigger + Cooldown)
        if (
            gesture == self.last_triggered_gesture
            and (now - self.last_trigger_time) < self.command_cooldown
        ):
            return

        # Thực hiện chuyển trạng thái FSM theo cử chỉ hợp lệ
        if gesture in ('Wave', 'Pointing_Up', 'Victory'):
            self._handle_select_behavior(gesture, now)
        elif gesture == 'Thumbs_Up':
            self._handle_confirm_behavior(now)

    def _handle_emergency_stop(self):
        """Xử lý cử chỉ Xòe bàn tay (Open_Palm): Dừng robot ngay lập tức."""
        self._publish_stop()
        prev_state = self.state
        prev_behavior = self.active_behavior or self.selected_behavior
        self.state = self.STATE_STOPPED
        self.active_behavior = None
        status = (
            f'STOPPED (Open_Palm): Robot da DUNG NGAY LAP TUC! '
            f'(Truoc do: {prev_state} / {prev_behavior or "None"})'
        )
        self._publish_status(status)
        self.get_logger().warn(f'[DỪNG KHẨN CẤP - Open_Palm] {status}')

    def _handle_cancel_behavior(self):
        """Xử lý cử chỉ Nắm tay (Closed_Fist): Hủy hành vi hiện tại và trở về trạng thái chờ (IDLE)."""
        self._publish_stop()
        canceled = self.active_behavior or self.selected_behavior or 'None'
        self.state = self.STATE_IDLE
        self.selected_behavior = None
        self.active_behavior = None
        status = f'IDLE (Closed_Fist): Da HUY hanh vi [{canceled}], tro ve trang thai cho.'
        self._publish_status(status)
        self.get_logger().info(f'[HỦY HÀNH VI - Closed_Fist] {status}')

    def _handle_select_behavior(self, gesture: str, now: float):
        """Xử lý cử chỉ chọn hành vi (Wave, Pointing_Up, Victory) - Chờ Thumbs_Up xác nhận."""
        if self.state == self.STATE_EXECUTING:
            # Khi đang thực thi một hành vi, không tự ý đổi hành vi nếu chưa dừng hoặc hủy
            return

        self._publish_stop()
        changed = (self.selected_behavior != gesture) or (self.state != self.STATE_SELECTED)
        self.selected_behavior = gesture
        self.state = self.STATE_SELECTED
        self.last_triggered_gesture = gesture
        self.last_trigger_time = now

        if changed:
            desc = BEHAVIOR_NAMES_VI.get(gesture, gesture)
            status = f'SELECTED [{gesture}]: {desc} -> Gio Thumbs_Up de XAC NHAN!'
            self._publish_status(status)
            self.get_logger().info(
                f'[ĐÃ CHỌN HÀNH VI] {gesture} ({desc}) | '
                f'Robot đang chờ cử chỉ Giơ ngón cái (Thumbs_Up) để xác nhận thực thi.'
            )

    def _handle_confirm_behavior(self, now: float):
        """Xử lý cử chỉ Giơ ngón cái (Thumbs_Up): Xác nhận và thực hiện hành vi đã chọn."""
        self.last_triggered_gesture = 'Thumbs_Up'
        self.last_trigger_time = now

        if self.state == self.STATE_EXECUTING:
            return

        if self.selected_behavior is None:
            status = 'IDLE: Chua chon hanh vi! Hay gio Wave / Pointing_Up / Victory truoc khi Thumbs_Up.'
            self._publish_status(status)
            self.get_logger().warn(
                '[CHƯA CHỌN HÀNH VI] Nhận được Thumbs_Up nhưng chưa có hành vi nào được chọn!'
            )
            return

        # Bắt đầu thực hiện hành vi đã chọn
        self.active_behavior = self.selected_behavior
        self.state = self.STATE_EXECUTING
        self.exec_start_time = now
        self.last_progress_log_time = 0.0

        desc = BEHAVIOR_NAMES_VI.get(self.active_behavior, self.active_behavior)
        status = f'EXECUTING [{self.active_behavior}]: Dang thuc hien {desc}...'
        self._publish_status(status)
        self.get_logger().info(
            f'[XÁC NHẬN THÀNH CÔNG - Thumbs_Up] Bắt đầu thực hiện hành vi: '
            f'{self.active_behavior} ({desc})'
        )

    def _compute_wave_velocity(self, elapsed: float) -> tuple:
        """
        Hành vi 1 - Vẫy tay (Wave): Robot xoay trái - phải để chào lại.
        Chu trình 4 pha đối xứng giúp robot kết thúc đúng góc hướng ban đầu:
          - Pha 1 (0 -> T)   : Xoay trái  (+w)
          - Pha 2 (T -> 3T)  : Xoay phải  (-w)
          - Pha 3 (3T -> 5T) : Xoay trái  (+w)
          - Pha 4 (5T -> 6T) : Xoay phải về tâm (-w)
        """
        t = self.wave_half_t
        total_duration = 6.0 * t
        if elapsed >= total_duration:
            return 0.0, 0.0, True, 'Hoan thanh xoay trai-phai chao lai!'

        if elapsed < t:
            return 0.0, +self.wave_w, False, 'Wave (1/4): Xoay sang TRAI chao'
        elif elapsed < 3.0 * t:
            return 0.0, -self.wave_w, False, 'Wave (2/4): Xoay sang PHAI chao'
        elif elapsed < 5.0 * t:
            return 0.0, +self.wave_w, False, 'Wave (3/4): Xoay sang TRAI chao'
        else:
            return 0.0, -self.wave_w, False, 'Wave (4/4): Quay ve huong ban dau'

    def _compute_forward_backward_velocity(self, elapsed: float) -> tuple:
        """
        Hành vi 2 - Giơ 1 ngón tay (Pointing_Up): Tiến - lùi 4 lần.
        Mỗi chu kỳ gồm:
          - Tiến (fb_duration giây) với v = +linear_speed
          - Nghỉ (fb_pause giây)    với v = 0.0
          - Lùi  (fb_duration giây) với v = -linear_speed
          - Nghỉ (fb_pause giây)    với v = 0.0
        """
        cycle_time = 2.0 * (self.fb_duration + self.fb_pause)
        total_time = self.fb_reps * cycle_time

        if elapsed >= total_time:
            return 0.0, 0.0, True, f'Hoan thanh tien-lui du {self.fb_reps} lan!'

        rep_idx = int(elapsed // cycle_time) + 1
        t_in_cycle = elapsed % cycle_time

        if t_in_cycle < self.fb_duration:
            return (
                +self.linear_speed,
                0.0,
                False,
                f'Tien-Lui lan {rep_idx}/{self.fb_reps}: DANG TIEN (+{self.linear_speed:.2f} m/s)',
            )
        elif t_in_cycle < self.fb_duration + self.fb_pause:
            return (
                0.0,
                0.0,
                False,
                f'Tien-Lui lan {rep_idx}/{self.fb_reps}: Dung chuyen chieu',
            )
        elif t_in_cycle < 2.0 * self.fb_duration + self.fb_pause:
            return (
                -self.linear_speed,
                0.0,
                False,
                f'Tien-Lui lan {rep_idx}/{self.fb_reps}: DANG LUI (-{self.linear_speed:.2f} m/s)',
            )
        else:
            return (
                0.0,
                0.0,
                False,
                f'Tien-Lui lan {rep_idx}/{self.fb_reps}: Hoan tat chu ky {rep_idx}',
            )

    def _compute_circle_velocity(self, elapsed: float) -> tuple:
        """
        Hành vi 3 - Giơ 2 ngón tay (Victory): Di chuyển theo quỹ đạo tròn.
        Robot chạy đồng thời linear.x = circle_v và angular.z = circle_w
        với bán kính R = v / w trong 1 vòng tròn khép kín (T = 2*pi / |w|).
        """
        circle_duration = (2.0 * math.pi) / max(abs(self.circle_w), 0.05)
        if elapsed >= circle_duration:
            return 0.0, 0.0, True, 'Hoan thanh quy dao tron 360 do!'

        progress_pct = min(100.0, (elapsed / circle_duration) * 100.0)
        radius = self.circle_v / max(abs(self.circle_w), 0.05)
        return (
            self.circle_v,
            self.circle_w,
            False,
            f'Quy dao tron (R={radius:.2f}m): {progress_pct:.0f}% ({elapsed:.1f}s/{circle_duration:.1f}s)',
        )

    def _control_loop_cb(self):
        """Vòng lặp điều khiển chính (20 Hz): Kiểm tra an toàn, Watchdog và phát lệnh /cmd_vel."""
        now = time.time()

        # 1. Watchdog kiểm tra mất tín hiệu điều khiển khi robot đang di chuyển
        if self.state == self.STATE_EXECUTING and self.last_msg_time is not None:
            if (now - self.last_msg_time) > self.signal_timeout:
                self._publish_stop()
                self.state = self.STATE_STOPPED
                self.active_behavior = None
                if not self.signal_lost_warned:
                    self.signal_lost_warned = True
                    status = (
                        f'STOPPED (Signal Timeout): Mat tin hieu /detected_gesture qua '
                        f'{self.signal_timeout:.1f}s! Robot da dung an toan.'
                    )
                    self._publish_status(status)
                    self.get_logger().error(f'[WATCHDOG DỪNG AN TOÀN] {status}')
                return

        # 2. Nếu không ở trạng thái EXECUTING, đảm bảo robot đứng yên
        if self.state != self.STATE_EXECUTING or self.active_behavior is None:
            return

        # 3. Tính toán vận tốc mục tiêu theo hành vi đang thực thi
        elapsed = now - self.exec_start_time
        if self.active_behavior == 'Wave':
            lin_x, ang_z, finished, detail = self._compute_wave_velocity(elapsed)
        elif self.active_behavior == 'Pointing_Up':
            lin_x, ang_z, finished, detail = self._compute_forward_backward_velocity(elapsed)
        elif self.active_behavior == 'Victory':
            lin_x, ang_z, finished, detail = self._compute_circle_velocity(elapsed)
        else:
            lin_x, ang_z, finished, detail = 0.0, 0.0, True, 'Unknown behavior'

        # 4. Kiểm tra chống va chạm bằng cảm biến LiDAR (/scan)
        if lin_x > 0.0 and self.front_clear_dist < self.obstacle_stop_dist:
            self._publish_stop()
            self.state = self.STATE_STOPPED
            self.active_behavior = None
            status = (
                f'STOPPED (Obstacle): Phat hien vat can phia truoc '
                f'({self.front_clear_dist:.2f}m < {self.obstacle_stop_dist:.2f}m)! Robot dung an toan.'
            )
            self._publish_status(status)
            self.get_logger().warn(f'[BẢO VỆ CHỐNG VA CHẠM] {status}')
            return

        if lin_x < 0.0 and self.rear_clear_dist < self.obstacle_stop_dist:
            self._publish_stop()
            self.state = self.STATE_STOPPED
            self.active_behavior = None
            status = (
                f'STOPPED (Obstacle): Phat hien vat can phia sau '
                f'({self.rear_clear_dist:.2f}m < {self.obstacle_stop_dist:.2f}m)! Robot dung an toan.'
            )
            self._publish_status(status)
            self.get_logger().warn(f'[BẢO VỆ CHỐNG VA CHẠM] {status}')
            return

        # 5. Nếu hành vi đã hoàn thành đủ chu trình -> Dừng robot và về trạng thái chờ (IDLE)
        if finished:
            self._publish_stop()
            done_behavior = self.active_behavior
            self.state = self.STATE_IDLE
            self.selected_behavior = None
            self.active_behavior = None
            status = f'IDLE: Da hoan thanh [{done_behavior}] ({detail}). San sang nhan cu chi moi!'
            self._publish_status(status)
            self.get_logger().info(f'[HOÀN THÀNH HÀNH VI] {status}')
            return

        # 6. Phát lệnh vận tốc lên /cmd_vel
        cmd = Twist()
        cmd.linear.x = float(lin_x)
        cmd.angular.z = float(ang_z)
        self.pub_cmd_vel.publish(cmd)

        # Cập nhật trạng thái hiển thị định kỳ mỗi 0.4 giây
        if (now - self.last_progress_log_time) >= 0.4:
            self.last_progress_log_time = now
            status = f'EXECUTING [{self.active_behavior}]: {detail}'
            self._publish_status(status)
            self.get_logger().info(
                f'[ĐANG CHẠY] {detail} | v={lin_x:+.2f} m/s, w={ang_z:+.2f} rad/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

