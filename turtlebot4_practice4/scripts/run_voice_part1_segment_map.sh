#!/usr/bin/env bash
# ==============================================================================
# Phần 1 & 2: Phân vùng bản đồ 2D Occupancy Grid (.pgm) & Gắn nhãn ngữ nghĩa
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

python3 /home/tuan/turtelbot4_ws/src/turtlebot4_practice4/turtlebot4_practice4/map_segmentation.py --verify --show "$@"

