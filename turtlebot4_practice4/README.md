# Bài Tập Thực Hành – Điều Khiển Robot Tự Hành TurtleBot 4
## Định Vị Bản Đồ (AMCL Localization), Điều Hướng Tự Động (Nav2) & Nhận Diện Vật Thể Với Camera OAK-D + YOLOv8

Dự án thực hành hoàn chỉnh cho robot **TurtleBot 4 Standard** trong môi trường mô phỏng **Gazebo Fortress** trên nền tảng **ROS 2 Humble (Ubuntu 22.04 LTS)**, kết hợp định vị **AMCL**, điều hướng tự động qua 03 tọa độ mục tiêu bằng **Nav2 (`nav2_simple_commander`)** và nhận diện vật thể thời gian thực từ **Camera RGB OAK-D** bằng mô hình **YOLOv8 (`yolov8n.pt`)**.

---

## 📌 Thông Tin Nộp Bài
* **Họ và tên sinh viên:** Nguyễn Quang Tuân
* **Mã số sinh viên (MSSV):** 23020767
* **Giảng viên hướng dẫn:** PGS.TS. Hoàng Văn Xiêm, CN. Nguyễn Quốc Bảo
* **Video Demo (Google Drive - Chế độ công khai):** [Xem Video Demo trên Google Drive](https://drive.google.com/drive/folders/1vl4O0C9A4C3Ed_HwHmJAeVCzwEQZulCx?usp=drive_link)
* **GitHub Repository (Public):** `https://github.com/Tuan404cl/Turtlebot_sim_test4`
* **File báo cáo kỹ thuật Bài tập 7 (Phân vùng Bản đồ & Điều hướng Giọng nói - Markdown chuẩn):** [docs/bao_cao_bai_tap_07.md](docs/bao_cao_bai_tap_07.md)
* **File báo cáo kỹ thuật (Bài tập Phân vùng Bản đồ & Điều hướng Giọng nói - PDF 2 trang):** [docs/bao_cao_voice_navigation.pdf](docs/bao_cao_voice_navigation.pdf)
* **File báo cáo kỹ thuật (Bài tập Phân vùng Bản đồ & Điều hướng Giọng nói - Markdown):** [docs/bao_cao_voice_navigation.md](docs/bao_cao_voice_navigation.md)
* **File báo cáo kỹ thuật Bài tập 6 (Điều khiển cử chỉ tay - Markdown):** [docs/bao_cao_bai_tap_06.md](docs/bao_cao_bai_tap_06.md)
* **File báo cáo kỹ thuật Markdown:** [docs/bao_cao_thuc_hanh_04.md](docs/bao_cao_thuc_hanh_04.md)
* **File báo cáo kỹ thuật HTML:** [docs/bao_cao_thuc_hanh_04.html](docs/bao_cao_thuc_hanh_04.html)
* **File báo cáo PDF hoàn chỉnh:** [docs/bao_cao_thuc_hanh_04.pdf](docs/bao_cao_thuc_hanh_04.pdf)
* **Mã nguồn LaTeX báo cáo:** [docs/report.tex](docs/report.tex)

---

## 📂 Cấu Trúc Gói Chương Trình Trong `src/`

```
turtlebot4_ws/src/
├── turtlebot4_practice4/
│   ├── package.xml                       # Khai báo phụ thuộc package ROS 2
│   ├── setup.py                          # Cấu hình cài đặt package & 07 console_scripts
│   ├── setup.cfg                         # Đường dẫn thực thi script
│   ├── turtlebot4_practice4/
│   │   ├── __init__.py
│   │   ├── spawn_yolo_objects.py         # (Phần 1) Tự động thêm mô hình người, ghế, bàn vào Gazebo
│   │   ├── auto_undock.py                # (Phần 1) Tự động điều khiển robot rời trạm sạc (Undock)
│   │   ├── tf_relay.py                   # (Phần 1) Đồng bộ thời gian TF & mock hardware services
│   │   ├── goal_survey.py                # (Phần 2) Khảo sát /clicked_point & /amcl_pose lưu JSON
│   │   ├── auto_nav_goals.py             # (Phần 3) Điều hướng tự động 03 Goals bằng Nav2 Code
│   │   ├── yolo_detector.py              # (Phần 4) Nhận diện vật thể Camera OAK-D + YOLOv8
│   │   └── check_topics.py               # Kiểm tra tự động /scan, /odom, /cmd_vel
│   ├── launch/
│   │   ├── sim_robot.launch.py           # Khởi chạy Gazebo Simulation (TurtleBot 4 Standard)
│   │   ├── sim_slam.launch.py            # Khởi chạy SLAM Toolbox & RViz2
│   │   ├── sim_localization.launch.py    # Khởi chạy AMCL Localization & Map Server
│   │   ├── sim_navigation.launch.py      # Khởi chạy tích hợp: Gazebo + Objects + AMCL + Nav2 + YOLO
│   │   └── view_rviz.launch.py           # Khởi chạy giao diện trực quan RViz2
│   ├── config/
│   │   ├── surveyed_goals.json           # Tọa độ (x, y, yaw) của 03 Goals khảo sát ở Phần 2
│   │   ├── localization_params.yaml      # Cấu hình tham số AMCL và Map Server
│   │   ├── nav2_params.yaml              # Cấu hình Costmaps, DWB Controller, NavFn Planner
│   │   ├── fastdds_udp_only.xml          # Cấu hình FastDDS UDP-only chống lỗi SHM deadlock
│   │   ├── slam_toolbox_params.yaml      # Cấu hình tham số SLAM Toolbox
│   │   ├── turtlebot4_mapping.rviz       # Giao diện RViz2 cho lập bản đồ
│   │   └── turtlebot4_nav2.rviz          # Giao diện RViz2 cho Localization, Nav2 & Camera YOLO
│   ├── maps/
│   │   ├── map.yaml                      # Metadata bản đồ đã xây dựng từ tuần trước
│   │   ├── map.pgm                       # Ảnh lưới chiếm dụng (Occupancy Grid) 2D
│   │   ├── warehouse_map.yaml & .pgm     # Bản đồ dự phòng warehouse
│   │   └── maze_map.yaml & .pgm          # Bản đồ dự phòng maze
│   ├── scripts/
│   │   ├── run_part1_localization.sh     # Chạy Phần 1: Simulation + Load Map + AMCL + Nav2
│   │   ├── run_part2_survey_goals.sh     # Chạy Phần 2: Khảo sát tọa độ 03 Goals trên RViz2
│   │   ├── run_part3_auto_nav.sh         # Chạy Phần 3: Lập trình tự hành qua 03 Goals
│   │   ├── run_part4_yolo.sh             # Chạy Phần 4: Nhận diện vật thể Camera với YOLOv8
│   │   ├── run_part4_navigation.sh       # Chạy tích hợp toàn bộ Phần 1 -> Phần 4 trong 1 lệnh
│   │   └── clean_ros.sh                  # Dọn dẹp các tiến trình ROS 2 / Gazebo chạy ngầm
│   └── docs/
│       ├── bao_cao_thuc_hanh_04.md       # Báo cáo kỹ thuật chi tiết định dạng Markdown
│       ├── bao_cao_thuc_hanh_04.html     # Định dạng HTML hiển thị chuẩn
│       ├── bao_cao_thuc_hanh_04.pdf      # File PDF hoàn chỉnh để nộp bài
│       └── report.tex                    # Mã nguồn LaTeX của báo cáo
└── irobot_create_control/                # Package điều khiển động học vi sai cho đế Create 3
```

---

## 🚀 Hướng Dẫn Biên Dịch & Thiết Lập Môi Trường

```bash
cd /home/tuan/turtelbot4_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 📖 Hướng Dẫn Thực Hiện Chi Tiết 04 Phần Của Bài Tập

### ⚡ Cách Nhanh Nhất: Chạy Tích Hợp Toàn Bộ (Phần 1 + Phần 3 + Phần 4)
Chỉ cần mở 01 Terminal duy nhất và chạy:
```bash
./src/turtlebot4_practice4/scripts/run_part4_navigation.sh
```
Script này sẽ tự động:
1. Mở Gazebo `warehouse` với robot **TurtleBot 4 Standard** (có Camera OAK-D) và thêm 04 mô hình vật thể (người đứng, ghế tựa, ghế gấp, bàn).
2. Nạp bản đồ `maps/map.yaml` từ tuần trước, khởi chạy **AMCL Localization** + **Nav2** + **RViz2**.
3. Tự động **Undock** robot khỏi trạm sạc và thiết lập **Initial Pose** khớp 100% tia LiDAR `/scan` với tường bản đồ.
4. Khởi chạy Node **YOLOv8 (`yolo_detector`)** hiển thị cửa sổ OpenCV, publish ảnh nhận diện lên RViz2 (`/yolo/detections/image`) và in kết quả ra Terminal.
5. Tự động điều khiển robot đi tuần tự qua **Goal 1 $\rightarrow$ Goal 2 $\rightarrow$ Goal 3** bằng Node **`auto_nav_goals`**.

---

### 🔹 Chạy Riêng Từng Phần (4 Phần)

#### Phần 1 – Khởi chạy Simulation & Load bản đồ (Localization)
```bash
./src/turtlebot4_practice4/scripts/run_part1_localization.sh
```
* Bản đồ `map.yaml` và `map.pgm` hiển thị rõ nét trên RViz2.
* Tia quét LiDAR `/scan` trùng khớp hoàn toàn với các bức tường trên bản đồ.

#### Phần 2 – Khảo sát và lấy tọa độ mục tiêu trên bản đồ
Mở Terminal mới và chạy:
```bash
./src/turtlebot4_practice4/scripts/run_part2_survey_goals.sh
```
* Sử dụng công cụ **Publish Point** trên thanh công cụ RViz2 để click vào 03 điểm mục tiêu trên bản đồ (node sẽ tự động lắng nghe `/clicked_point` và `/amcl_pose`, quy đổi ra $(x, y, \text{yaw})$ và lưu vào `config/surveyed_goals.json`).

#### Phần 3 – Lập trình Navigation tự động với Nav2 (Bằng Code)
Mở Terminal mới và chạy:
```bash
./src/turtlebot4_practice4/scripts/run_part3_auto_nav.sh
```
* Node `auto_nav_goals.py` sử dụng `nav2_simple_commander` (`BasicNavigator`) tự động điều khiển robot đi qua **Goal 1 $\rightarrow$ Goal 2 $\rightarrow$ Goal 3** mà không cần điều khiển thủ công hay dùng chuột trên RViz2.

#### Phần 4 – Nhận diện vật thể từ Camera với YOLO
Mở Terminal mới và chạy:
```bash
./src/turtlebot4_practice4/scripts/run_part4_yolo.sh
```
* Node `yolo_detector.py` subscribe `/oakd/rgb/preview/image_raw`, chuyển đổi qua `cv_bridge`, nhận diện bằng `yolov8n.pt`, hiển thị Bounding Box + Class + Confidence lên cửa sổ OpenCV, publish lên `/yolo/detections/image` (xem trên RViz2) và in log ra Terminal.

---

## 🖐️ Điều Khiển Hành Vi TurtleBot 4 Bằng Cử Chỉ Tay AI (MediaPipe Gesture Recognizer + OpenCV)

### 1. Chạy Tích Hợp Toàn Bộ Hệ Thống Điều Khiển Cử Chỉ Tay
```bash
./src/turtlebot4_practice4/scripts/run_gesture_all.sh
```
Lệnh trên tự động khởi chạy Gazebo Simulation (`warehouse`), tự động Undock và đưa robot ra khu vực trung tâm rộng rãi không vật cản, sau đó khởi chạy `robot_behavior_node` và `gesture_recognition_node`.

### 2. Chạy Riêng Từng Phần
* **Phần 1 – Khởi chạy Simulation, kiểm tra `/cmd_vel` và mở Webcam OpenCV:**
  ```bash
  # Terminal 1: Khởi chạy Gazebo Simulation
  ./src/turtlebot4_practice4/scripts/run_part1_sim.sh

  # Terminal 2: Kiểm tra /cmd_vel và hiển thị Webcam bằng OpenCV
  ./src/turtlebot4_practice4/scripts/run_gesture_part1_sim_webcam.sh
  ```
* **Phần 2 – Nhận diện cử chỉ tay bằng AI (`gesture_recognition_node`):**
  ```bash
  ./src/turtlebot4_practice4/scripts/run_gesture_part2_recognition.sh
  ```
* **Phần 3 – Điều khiển hành vi TurtleBot 4 (`robot_behavior_node`):**
  ```bash
  ./src/turtlebot4_practice4/scripts/run_gesture_part3_behavior.sh
  ```

### 3. Bảng 06 Cử Chỉ Tay Và Hành Vi Tương Ứng Của Robot
| Cử chỉ | Nhãn `/detected_gesture` | Hành vi của TurtleBot 4 |
| :--- | :--- | :--- |
| **Vẫy tay (Wave)** | `Wave` | Chọn hành vi: **Robot xoay trái–phải để chào lại** |
| **Giơ 1 ngón tay (Pointing Up)** | `Pointing_Up` | Chọn hành vi: **Tiến–lùi 4 lần** |
| **Giơ 2 ngón tay (Victory)** | `Victory` | Chọn hành vi: **Di chuyển theo quỹ đạo tròn** |
| **Giơ ngón cái (Thumbs Up)** | `Thumbs_Up` | **Xác nhận và thực hiện** hành vi đã chọn |
| **Xòe bàn tay (Open Palm)** | `Open_Palm` | **Robot dừng ngay lập tức** |
| **Nắm tay (Closed Fist)** | `Closed_Fist` | **Hủy hành vi hiện tại** và trở về trạng thái chờ |

---

## 🎙️ Phân Vùng Bản Đồ Nhà Kho & Điều Hướng Bằng Giọng Nói Trực Tiếp Qua Microphone Với Nav2

### 1. Chạy Tích Hợp Toàn Bộ Hệ Thống Điều Hướng Giọng Nói
```bash
./src/turtlebot4_practice4/scripts/run_voice_navigation_all.sh
```
* **Thu âm trực tiếp từ Microphone**: Nhấn phím `[SPACE]` hoặc `[R]` trên cửa sổ *Warehouse Live Microphone Voice Navigation Studio* để nói câu lệnh trực tiếp vào Microphone (4 giây), hoặc nhấn phím `[M]` để bật chế độ **Auto-Listen VAD** (tự động lắng nghe liên tục từ Microphone).
* **05 Khu vực chức năng nhà kho (`config/semantic_regions.json` & `.yaml`)**:
  * **`A` — Khu nhập hàng** (`x = 1.80 m, y = 13.00 m, yaw = 0.00 rad`)
  * **`B` — Khu phân loại** (`x = 2.80 m, y = 5.30 m, yaw = 1.57 rad`)
  * **`C` — Khu xuất hàng** (`x = 3.20 m, y = 1.20 m, yaw = 0.00 rad`)
  * **`D` — Khu kệ hàng** (`x = 2.00 m, y = -8.50 m, yaw = -1.57 rad`)
  * **`E` — Trạm sạc** (`x = -1.60 m, y = 0.00 m, yaw = 3.14 rad`)

---

## 📤 Hướng Dẫn Đẩy Toàn Bộ Dự Án Lên GitHub (`Turtlebot_sim_test2.git`)

```bash
cd /home/tuan/turtelbot4_ws/src
git init
git remote remove origin 2>/dev/null || true
git remote add origin https://github.com/Tuan404cl/Turtlebot_sim_test2.git
git add .
git commit -m "Hoan thanh Bai tap TurtleBot 4: AMCL Localization, Auto Nav2 (03 Goals) va YOLOv8 Object Detection"
git branch -M main
git push -u origin main
```
