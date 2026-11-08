#!/usr/bin/env bash
# ==============================================================================
# Phần 3: Kiểm thử Nhận dạng Giọng nói (Speech-to-Text) & Phân tích Ngữ nghĩa NLP
# ==============================================================================
set -e

source /opt/ros/humble/setup.bash
if [ -f "/home/tuan/turtelbot4_ws/install/setup.bash" ]; then
    source /home/tuan/turtelbot4_ws/install/setup.bash
fi

python3 /home/tuan/turtelbot4_ws/src/turtlebot4_practice4/turtlebot4_practice4/voice_command_node.py --self-test
python3 /home/tuan/turtelbot4_ws/src/turtlebot4_practice4/turtlebot4_practice4/voice_navigation_node.py --self-test

