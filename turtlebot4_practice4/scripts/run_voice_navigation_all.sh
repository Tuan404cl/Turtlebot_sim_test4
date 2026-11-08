#!/usr/bin/env bash
# ==============================================================================
# Script chạy tích hợp toàn bộ Hệ thống Phân vùng Bản đồ Nhà kho (Warehouse)
# & Điều hướng Bằng Giọng nói Trực tiếp từ Microphone trên ROS 2 Humble & Nav2
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/clean_ros.sh" > /dev/null 2>&1 || true

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/fastdds_udp_only.xml"
MAP_FILE="/home/tuan/turtelbot4_ws/src/turtlebot4_practice4/maps/map.yaml"

# Đảm bảo bản đồ phân vùng Nhà kho và dữ liệu 05 khu vực đã được cập nhật mới nhất
python3 /home/tuan/turtelbot4_ws/src/turtlebot4_practice4/turtlebot4_practice4/map_segmentation.py --verify

echo "========================================================================"
echo "   PHÂN VÙNG BẢN ĐỒ NHÀ KHO & ĐIỀU HƯỚNG GIỌNG NÓI TRỰC TIẾP QUA MIC   "
echo "========================================================================"
echo " * Bản đồ gốc       : $MAP_FILE"
echo " * Bản đồ phân vùng : /home/tuan/turtelbot4_ws/src/turtlebot4_practice4/maps/map_segmented.png"
echo " * Dữ liệu khu vực  : /home/tuan/turtelbot4_ws/src/turtlebot4_practice4/config/semantic_regions.json"
echo "------------------------------------------------------------------------"
echo " 05 Khu vực chức năng trong Nhà kho (Warehouse Semantic Zones):"
echo "  - [A] Khu nhập hàng  : x =  1.80 m, y = 13.00 m, yaw =  0.00 rad"
echo "  - [B] Khu phân loại  : x =  2.80 m, y =  5.30 m, yaw =  1.57 rad"
echo "  - [C] Khu xuất hàng  : x =  3.20 m, y =  1.20 m, yaw =  0.00 rad"
echo "  - [D] Khu kệ hàng    : x =  2.00 m, y = -8.50 m, yaw = -1.57 rad"
echo "  - [E] Trạm sạc       : x = -1.60 m, y =  0.00 m, yaw =  3.14 rad"
echo "------------------------------------------------------------------------"
echo " Hướng dẫn điều khiển bằng giọng nói trực tiếp từ Microphone:"
echo "  - Nhấn phím [SPACE] hoặc [R] trên cửa sổ Voice Studio để thu âm 4 giây từ Mic"
echo "  - Nhấn phím [M] để Bật/Tắt chế độ Tự động lắng nghe liên tục (Auto-Listen VAD)"
echo "  - Ví dụ câu lệnh nói vào Microphone:"
echo "      + \"Đi đến khu vực A\" / \"Đi đến khu nhập hàng\""
echo "      + \"Đi đến khu phân loại\" / \"Đi đến khu xuất hàng\""
echo "      + \"Di chuyển từ khu vực B đến khu vực C\""
echo "      + \"Đi đến khu kệ hàng\" / \"Quay về trạm sạc\""
echo "      + \"Đi đến khu vực Z\" (Kiểm thử từ chối khu vực không tồn tại)"
echo "========================================================================"

ros2 launch turtlebot4_practice4 sim_voice_navigation.launch.py \
    map:="$MAP_FILE" \
    world:=warehouse \
    rviz:=true \
    "$@"
