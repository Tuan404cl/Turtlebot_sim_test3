# Bài Tập Thực Hành – Điều Khiển Robot Tự Hành TurtleBot 4
## Định Vị Bản Đồ (AMCL Localization), Điều Hướng Tự Động (Nav2) & Nhận Diện Vật Thể Với Camera OAK-D + YOLOv8

Dự án thực hành hoàn chỉnh cho robot **TurtleBot 4 Standard** trong môi trường mô phỏng **Gazebo Fortress** trên nền tảng **ROS 2 Humble (Ubuntu 22.04 LTS)**, kết hợp định vị **AMCL**, điều hướng tự động qua 03 tọa độ mục tiêu bằng **Nav2 (`nav2_simple_commander`)** và nhận diện vật thể thời gian thực từ **Camera RGB OAK-D** bằng mô hình **YOLOv8 (`yolov8n.pt`)**.

---

## 📌 Thông Tin Nộp Bài
* **Họ và tên sinh viên:** Nguyễn Quang Tuân
* **Mã số sinh viên (MSSV):** 23020767
* **Giảng viên hướng dẫn:** PGS.TS. Hoàng Văn Xiêm, CN. Nguyễn Quốc Bảo
* **Video Demo (Google Drive - Chế độ công khai):** [Xem Video Demo trên Google Drive](https://drive.google.com/drive/folders/1vl4O0C9A4C3Ed_HwHmJAeVCzwEQZulCx?usp=drive_link)
* **GitHub Repository (Public):** `https://github.com/Tuan404cl/Turtlebot_sim_test2.git`
* **Báo cáo kỹ thuật chi tiết (Markdown):** [turtlebot4_practice4/docs/bao_cao_thuc_hanh_04.md](turtlebot4_practice4/docs/bao_cao_thuc_hanh_04.md)
* **Báo cáo kỹ thuật (HTML):** [turtlebot4_practice4/docs/bao_cao_thuc_hanh_04.html](turtlebot4_practice4/docs/bao_cao_thuc_hanh_04.html)
* **Báo cáo kỹ thuật (PDF):** [turtlebot4_practice4/docs/bao_cao_thuc_hanh_04.pdf](turtlebot4_practice4/docs/bao_cao_thuc_hanh_04.pdf)

---

## 📂 Cấu Trúc Gói Chương Trình Trong `src/`

```
turtlebot4_ws/src/
├── turtlebot4_practice4/
│   ├── package.xml                       # Khai báo phụ thuộc package ROS 2
│   ├── setup.py                          # Cấu hình cài đặt package & 07 console_scripts
│   ├── turtlebot4_practice4/
│   │   ├── spawn_yolo_objects.py         # (Phần 1) Tự động thêm mô hình người, ghế, bàn vào Gazebo
│   │   ├── auto_undock.py                # (Phần 1) Tự động điều khiển robot rời trạm sạc (Undock)
│   │   ├── tf_relay.py                   # (Phần 1) Đồng bộ thời gian TF & mock hardware services
│   │   ├── goal_survey.py                # (Phần 2) Khảo sát /clicked_point & /amcl_pose lưu JSON
│   │   ├── auto_nav_goals.py             # (Phần 3) Điều hướng tự động 03 Goals bằng Nav2 Code
│   │   ├── yolo_detector.py              # (Phần 4) Nhận diện vật thể Camera OAK-D + YOLOv8
│   │   └── check_topics.py               # Kiểm tra tự động /scan, /odom, /cmd_vel
│   ├── launch/
│   │   ├── sim_robot.launch.py           # Khởi chạy Gazebo Simulation (TurtleBot 4 Standard)
│   │   ├── sim_localization.launch.py    # Khởi chạy AMCL Localization & Map Server
│   │   └── sim_navigation.launch.py      # Khởi chạy tích hợp: Gazebo + Objects + AMCL + Nav2 + YOLO
│   ├── config/
│   │   ├── surveyed_goals.json           # Tọa độ (x, y, yaw) của 03 Goals khảo sát ở Phần 2
│   │   ├── localization_params.yaml      # Cấu hình tham số AMCL và Map Server
│   │   ├── nav2_params.yaml              # Cấu hình Costmaps, DWB Controller, NavFn Planner
│   │   ├── fastdds_udp_only.xml          # Cấu hình FastDDS UDP-only chống lỗi SHM deadlock
│   │   └── turtlebot4_nav2.rviz          # Giao diện RViz2 cho Localization, Nav2 & Camera YOLO
│   ├── maps/
│   │   └── map.yaml & map.pgm            # Bản đồ 2D đã xây dựng từ tuần trước
│   ├── scripts/
│   │   ├── run_part1_localization.sh     # Chạy Phần 1: Simulation + Load Map + AMCL + Nav2
│   │   ├── run_part2_survey_goals.sh     # Chạy Phần 2: Khảo sát tọa độ 03 Goals trên RViz2
│   │   ├── run_part3_auto_nav.sh         # Chạy Phần 3: Lập trình tự hành qua 03 Goals
│   │   ├── run_part4_yolo.sh             # Chạy Phần 4: Nhận diện vật thể Camera với YOLOv8
│   │   └── run_part4_navigation.sh       # Chạy tích hợp toàn bộ Phần 1 -> Phần 4 trong 1 lệnh
│   └── docs/
│       └── bao_cao_thuc_hanh_04.(md|html|pdf) & report.tex
└── irobot_create_control/                # Package điều khiển động học vi sai cho đế Create 3
```

---

## 🚀 Hướng Dẫn Thực Thi Nhanh

```bash
# 1. Build workspace
cd /home/tuan/turtelbot4_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. Chạy tích hợp tự động toàn bộ Phần 1 -> Phần 4:
./src/turtlebot4_practice4/scripts/run_part4_navigation.sh
```
