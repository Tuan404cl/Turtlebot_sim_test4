#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 4: ĐIỀU HƯỚNG NHÀ KHO BẰNG GIỌNG NÓI VỚI NAV2 (WAREHOUSE VOICE NAVIGATION NODE)
------------------------------------------------------------------------------------
1. Đọc cơ sở dữ liệu ngữ nghĩa 05 khu vực nhà kho (config/semantic_regions.json):
   - Khu vực A: Khu nhập hàng (Inbound / Receiving Bay)
   - Khu vực B: Khu phân loại (Sorting & Inspection Zone)
   - Khu vực C: Khu xuất hàng (Outbound / Packing & Shipping Zone)
   - Khu vực D: Khu kệ hàng (Storage Racks Aisles)
   - Khu vực E: Trạm sạc (Charging Dock & Control Station)
2. Lắng nghe câu lệnh văn bản từ giọng nói trực tiếp qua Microphone trên topic /voice_command.
3. Phân tích ngôn ngữ tự nhiên (Vietnamese / English Warehouse NLP Command Parser):
   - Câu lệnh đơn đích: "Đi đến khu vực A", "Đi đến khu nhập hàng", "Đi đến khu kệ hàng"...
   - Câu lệnh đa chặng: "Di chuyển từ khu vực A đến khu vực B" -> Kiểm tra vị trí hiện tại
     của robot qua /amcl_pose; nếu robot chưa ở A thì điều hướng qua A trước rồi mới đến B.
   - Xử lý ngoại lệ: Phát hiện câu lệnh chứa khu vực không tồn tại trong nhà kho (ví dụ:
     "Đi đến khu vực Z", "Đi đến phòng học") -> Cảnh báo rõ ràng và từ chối điều hướng.
4. Hiển thị trực quan các khu vực trên RViz2 (/semantic_regions_markers) và cửa sổ OpenCV.
"""

import argparse
import json
import math
import os
import re
import sys
import time
import unicodedata
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


def remove_accents(text: str) -> str:
    """Chuyển chuỗi tiếng Việt có dấu thành không dấu chữ thường để so khớp bền vững."""
    text = text.replace('đ', 'd').replace('Đ', 'D')
    nfkd = unicodedata.normalize('NFKD', text)
    return ''.join([c for c in nfkd if not unicodedata.combining(c)]).lower().strip()


def normalize_phrase(text: str) -> str:
    """Chuẩn hóa câu lệnh: chữ thường, loại bỏ dấu câu thừa ở đầu/cuối."""
    cleaned = re.sub(r'[.,!?;:"\'()\[\]{}]+', ' ', text.lower())
    cleaned = re.sub(r'\s+', ' ', cleaned).strip()
    return cleaned


def load_semantic_regions(json_path: Optional[str] = None) -> Dict:
    """Đọc dữ liệu các vùng ngữ nghĩa từ file JSON."""
    if not json_path:
        ws_pkg = '/home/tuan/turtelbot4_ws/src/turtlebot4_practice4'
        if not os.path.isdir(ws_pkg):
            ws_pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        json_path = os.path.join(ws_pkg, 'config', 'semantic_regions.json')

    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def match_region_name(phrase: str, regions: List[Dict]) -> Optional[Dict]:
    """
    Đối chiếu cụm từ chỉ khu vực (ví dụ: 'phòng A', 'khu vực A', 'hành lang C', 'phòng B', 'khu kệ hàng')
    với danh sách các khu vực trong cơ sở dữ liệu ngữ nghĩa.
    """
    p_norm = normalize_phrase(phrase)
    p_ascii = remove_accents(p_norm)
    if not p_ascii:
        return None

    # Loại bỏ các từ đệm ở đầu cụm từ nếu có
    p_stripped = re.sub(
        r'^(?:den|toi|vao|qua|sang|ve|tai|o|roi den|roi qua|sau do den|sau do qua|ghe qua)\s+',
        '',
        p_ascii,
    ).strip()

    # 1. So khớp chính xác theo alias hoặc ID hoặc tên
    for reg in regions:
        reg_id_lower = reg['id'].lower()
        candidates = [
            reg_id_lower,
            f'khu vuc {reg_id_lower}',
            f'phong {reg_id_lower}',
            f'vung {reg_id_lower}',
            f'diem {reg_id_lower}',
            f'zone {reg_id_lower}',
            f'area {reg_id_lower}',
            remove_accents(reg['name']),
            remove_accents(reg['full_label']),
        ] + [remove_accents(a) for a in reg.get('aliases', [])]

        if p_ascii in candidates or p_stripped in candidates:
            return reg

    # 2. So khớp biểu thức "khu vực <ID>", "phòng <ID>", "hành lang <ID>" nằm trong cụm từ
    m_zone = re.search(r'\b(?:khu vuc|phong|vung|diem|hanh lang|zone|area|region)\s+([a-z0-9]+)\b', p_ascii)
    if m_zone:
        zone_token = m_zone.group(1).upper()
        for reg in regions:
            if reg['id'].upper() == zone_token:
                return reg
        return None

    # 3. So khớp theo tên ngữ nghĩa dài (ưu tiên cụm từ dài trước)
    for reg in regions:
        for alias in sorted(reg.get('aliases', []), key=len, reverse=True):
            alias_ascii = remove_accents(alias)
            if len(alias_ascii) >= 3 and re.search(r'\b' + re.escape(alias_ascii) + r'\b', p_ascii):
                return reg

    return None


def parse_voice_command(command_text: str, regions_db: Dict) -> Dict:
    """
    Phân tích câu lệnh giọng nói tiếng Việt / tiếng Anh và trích xuất chuỗi các khu vực cần đến.
    Hỗ trợ:
      1. Câu lệnh chuỗi nhiều điểm liên hoàn phức tạp:
         - "qua A qua B qua C qua D rồi qua E"
         - "Đi qua A, B, C, D rồi đến E"
         - "Đi từ A qua B, C, D rồi về E"
         - "Đi qua khu nhập hàng, qua phân loại, qua xuất hàng, qua kệ hàng rồi về trạm sạc"
         - "Đi từ phòng A qua hành lang C rồi đến phòng B"
      2. Câu lệnh đa điểm 2 chặng: "Di chuyển từ khu vực A đến khu vực B".
      3. Câu lệnh đơn đích: "Đi đến khu vực A", "Đi đến khu nhập hàng", "Quay về trạm sạc".
      4. Phát hiện khu vực không tồn tại: "Đi đến khu vực Z", "Đi qua A qua B qua Z rồi đến E".
    """
    regions = regions_db.get('regions', [])
    valid_summary = ', '.join([f"{r['id']} ({r['name']})" for r in regions])

    raw_text = command_text.strip()
    # Chuẩn hóa văn bản nhưng giữ lại ranh giới dấu phẩy để tách danh sách liệt kê A, B, C...
    cleaned = re.sub(r'[.!?;:\"\'()\[\]{}]+', ' ', raw_text.lower())
    cleaned = re.sub(r'\s*,\s*', ' , ', cleaned)
    cleaned = re.sub(r'\s+', ' ', cleaned).strip()
    ascii_text = remove_accents(cleaned)

    if not ascii_text:
        return {
            'valid': False,
            'command_type': 'EMPTY',
            'raw_text': raw_text,
            'source_region': None,
            'target_region': None,
            'target_regions': [],
            'unknown_region': '',
            'message': 'Câu lệnh trống. Vui lòng nói rõ khu vực cần đến.',
        }

    # Bỏ các động từ bắt đầu câu lệnh
    asc = re.sub(
        r'^(?:hay\s+|robot\s+)?(?:di chuyen|di|chuyen|chay|tien|quay|move|navigate|go)\s*',
        '',
        ascii_text,
    ).strip()

    # Chuẩn hóa các liên từ nối tự nhiên phong phú
    asc = re.sub(r'\b(?:sau do qua|roi qua|ghe qua|tiep theo qua|tiep tuc qua)\b', ' qua ', asc)
    asc = re.sub(
        r'\b(?:sau do den|roi den|sau do toi|roi toi|sau do ve|roi ve|cuoi cung den|va den|va qua|tiep theo den|tiep tuc den|tiep tuc toi)\b',
        ' den ',
        asc,
    )
    asc = re.sub(r'\b(?:roi|sau do|tiep theo|tiep tuc|cuoi cung)\b', ' ', asc)
    asc = re.sub(r'\s+', ' ', asc).strip()

    # Tách chuỗi theo các mốc từ khóa chuyển tiếp: tu (start), qua (via), den / toi / ve (dest)
    tokens = re.split(r'\b(tu|qua|den|toi|ve)\b', asc)
    tokens = [t.strip() for t in tokens if t.strip()]

    stages = []
    i = 0
    while i < len(tokens):
        item = tokens[i]
        if item in ('tu',):
            i += 1
            if i < len(tokens):
                stages.append(('START', tokens[i]))
        elif item in ('qua',):
            i += 1
            if i < len(tokens):
                stages.append(('VIA', tokens[i]))
        elif item in ('den', 'toi', 've'):
            i += 1
            if i < len(tokens):
                stages.append(('DEST', tokens[i]))
        else:
            stages.append(('DEST', item))
        i += 1

    if not stages:
        return {
            'valid': False,
            'command_type': 'INVALID_REGION',
            'raw_text': raw_text,
            'source_region': None,
            'target_region': None,
            'target_regions': [],
            'unknown_region': raw_text,
            'message': f"Khu vực '{raw_text.upper()}' không tồn tại trên bản đồ! Các khu vực hợp lệ: {valid_summary}.",
        }

    # Mở rộng các giai đoạn có liệt kê danh sách qua dấu phẩy hoặc từ 'và'
    expanded_stages = []
    for st_type, phrase in stages:
        sub_items = re.split(r'\s*,\s*|\bva\b', phrase)
        sub_items = [s.strip() for s in sub_items if s.strip()]
        for s in sub_items:
            expanded_stages.append((st_type, s))

    resolved_targets = []
    has_start = False
    start_region = None

    for st_type, phrase in expanded_stages:
        reg = match_region_name(phrase, regions)
        if reg is None:
            return {
                'valid': False,
                'command_type': 'INVALID_REGION',
                'raw_text': raw_text,
                'source_region': None,
                'target_region': None,
                'target_regions': [],
                'unknown_region': phrase,
                'message': (
                    f"Khu vực '{phrase.upper()}' không tồn tại trên bản đồ! "
                    f"Các khu vực hợp lệ: {valid_summary}."
                ),
            }
        if st_type == 'START' and not start_region:
            has_start = True
            start_region = reg
        else:
            resolved_targets.append(reg)

    if not resolved_targets and start_region:
        resolved_targets.append(start_region)
        has_start = False
        start_region = None

    if len(resolved_targets) > 1 or (has_start and len(resolved_targets) >= 1):
        cmd_type = 'MULTI_WAYPOINT'
    else:
        cmd_type = 'SINGLE_GOAL'

    names_str = ' -> '.join([f"[{r['id']}] {r['name']}" for r in resolved_targets])
    if has_start and start_region:
        msg = f"Nhận lệnh hành trình từ [{start_region['id']}] {start_region['name']} qua {names_str}."
    else:
        msg = f"Nhận lệnh hành trình đến: {names_str}."

    return {
        'valid': True,
        'command_type': cmd_type,
        'raw_text': raw_text,
        'has_start': has_start,
        'source_region': start_region,
        'target_region': resolved_targets[-1] if resolved_targets else None,
        'target_regions': resolved_targets,
        'unknown_region': '',
        'message': msg,
    }


def plan_waypoints_for_command(
    parsed_cmd: Dict,
    robot_xy: Tuple[float, float],
    arrival_radius_m: float = 1.25,
) -> Tuple[List[Dict], str]:
    """
    Xây dựng danh sách các điểm đích (Waypoints) cần đi qua:
    - Nếu câu lệnh là SINGLE_GOAL: trả về [target_reg].
    - Nếu câu lệnh là MULTI_WAYPOINT:
      + Nếu có điểm xuất phát (source_region):
        * Nếu robot đang ở ngoài điểm xuất phát (khoảng cách > arrival_radius_m),
          đưa source_region vào đầu chuỗi waypoints [start, via1, via2, ..., dest].
        * Nếu robot đã ở sẵn tại điểm xuất phát, bỏ qua điểm xuất phát và đi thẳng [via1, via2, ..., dest].
      + Nếu không có điểm xuất phát riêng: đi tuần tự qua danh sách [target_regions].
    """
    if not parsed_cmd.get('valid'):
        return [], parsed_cmd.get('message', 'Lệnh không hợp lệ')

    target_regions = parsed_cmd.get('target_regions', [])
    if not target_regions:
        tgt = parsed_cmd.get('target_region')
        target_regions = [tgt] if tgt else []

    has_start = parsed_cmd.get('has_start', False)
    source_reg = parsed_cmd.get('source_region')

    route = []
    rx, ry = robot_xy

    if has_start and source_reg:
        sx, sy = source_reg['world_goal']['x'], source_reg['world_goal']['y']
        dist_to_src = math.hypot(rx - sx, ry - sy)
        if dist_to_src > arrival_radius_m:
            route.append(source_reg)

    for r in target_regions:
        if not route or route[-1]['id'] != r['id']:
            route.append(r)

    if len(route) == 1:
        reg = route[0]
        gx, gy = reg['world_goal']['x'], reg['world_goal']['y']
        plan_msg = (
            f"Lộ trình 1 chặng: Đi trực tiếp đến Khu vực {reg['id']} "
            f"({reg['name']}) tại ({gx:.2f}, {gy:.2f}) m."
        )
    else:
        steps_txt = []
        for idx, r in enumerate(route, 1):
            gx, gy = r['world_goal']['x'], r['world_goal']['y']
            steps_txt.append(f"Chặng {idx}: [{r['id']}] {r['name']} ({gx:.2f}, {gy:.2f})m")
        plan_msg = f"Lộ trình đa điểm ({len(route)} chặng): " + " -> ".join(steps_txt)

    return route, plan_msg


# =============================================================================
# ROS 2 VOICE NAVIGATION NODE (KẾT NỐI NAV2 & RVIZ2 MARKERS)
# =============================================================================
def run_ros2_node(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from nav_msgs.msg import Path
    from std_msgs.msg import ColorRGBA, String
    from std_srvs.srv import Empty
    from visualization_msgs.msg import Marker, MarkerArray

    from turtlebot4_practice4.map_segmentation import get_default_paths, world_to_pixel

    class VoiceNavigationServer(BasicNavigator):
        """Node ROS 2 tích hợp NLP Voice Parser + Nav2 NavigateToPose + RViz2 Semantic Markers & Path."""

        def __init__(self, regions_db: Dict, segmented_map_path: str, enable_gui: bool = True):
            super().__init__()
            self.regions_db = regions_db
            self.regions = regions_db.get('regions', [])
            self.map_meta = regions_db.get('map_metadata', {})
            self.enable_gui = enable_gui and bool(os.environ.get('DISPLAY'))

            # Vị trí hiện tại của robot trong hệ quy chiếu map (mặc định tại điểm sau Undock)
            self.robot_x = 0.35
            self.robot_y = 0.00
            self.robot_yaw = 3.14159
            self.pending_commands: List[str] = []
            self.last_command_text = 'Đang chờ câu lệnh giọng nói...'
            self.last_status_text = 'Sẵn sàng nhận lệnh trên /voice_command'
            self.active_waypoints: List[Dict] = []
            self.current_target: Optional[Dict] = None

            self.base_map_img = cv2.imread(segmented_map_path) if os.path.isfile(segmented_map_path) else None

            # Subscribers & Publishers
            self.sub_voice = self.create_subscription(
                String, '/voice_command', self._on_voice_command, 10
            )
            self.sub_amcl = self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10
            )
            self.pub_status = self.create_publisher(String, '/voice_nav/status', 10)
            self.pub_markers = self.create_publisher(MarkerArray, '/semantic_regions_markers', 10)
            self.pub_planned_path = self.create_publisher(Path, '/voice_nav/planned_path', 10)
            self.marker_timer = self.create_timer(2.0, self.publish_semantic_markers)

        def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.robot_x = float(p.x)
            self.robot_y = float(p.y)
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        def _on_voice_command(self, msg: String):
            cmd_str = msg.data.strip()
            if cmd_str:
                print(f'\n[VOICE_CMD RECEIVED] Nhận câu lệnh từ /voice_command: "{cmd_str}"')
                self.pending_commands.append(cmd_str)

        def publish_status(self, status_dict: Dict):
            msg = String()
            msg.data = json.dumps(status_dict, ensure_ascii=False)
            self.pub_status.publish(msg)
            self.last_status_text = status_dict.get('message', '')

        def publish_semantic_markers(self):
            """Publish nhãn 3D, đĩa tròn các khu vực, và lộ trình dự kiến (Planned Route Path & Waypoint Order) lên RViz2."""
            ma = MarkerArray()
            now_msg = self.get_clock().now().to_msg()
            for idx, r in enumerate(self.regions):
                gx = float(r['world_goal']['x'])
                gy = float(r['world_goal']['y'])
                rgb = r.get('color_rgb', [40, 120, 180])

                # 1. Đĩa tròn đánh dấu vùng đích trên mặt sàn
                cyl = Marker()
                cyl.header.frame_id = 'map'
                cyl.header.stamp = now_msg
                cyl.ns = 'semantic_zone_disks'
                cyl.id = idx * 2
                cyl.type = Marker.CYLINDER
                cyl.action = Marker.ADD
                cyl.pose.position.x = gx
                cyl.pose.position.y = gy
                cyl.pose.position.z = 0.03
                cyl.pose.orientation.w = 1.0
                cyl.scale.x = 1.1
                cyl.scale.y = 1.1
                cyl.scale.z = 0.05
                cyl.color = ColorRGBA(
                    r=rgb[0] / 255.0, g=rgb[1] / 255.0, b=rgb[2] / 255.0, a=0.55
                )
                ma.markers.append(cyl)

                # 2. Nhãn chữ 3D hiển thị tên khu vực trên RViz2
                txt = Marker()
                txt.header.frame_id = 'map'
                txt.header.stamp = now_msg
                txt.ns = 'semantic_zone_labels'
                txt.id = idx * 2 + 1
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = gx
                txt.pose.position.y = gy
                txt.pose.position.z = 0.65
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.35
                txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                txt.text = f"[{r['id']}] {r['name_ascii']}\n({gx:.2f}, {gy:.2f})"
                ma.markers.append(txt)

            # 3. Trực quan hóa lộ trình đa điểm (Planned Route Line & Waypoint Order Badges) lên RViz2
            if self.active_waypoints:
                # Đường nối lộ trình (Line Strip)
                route_line = Marker()
                route_line.header.frame_id = 'map'
                route_line.header.stamp = now_msg
                route_line.ns = 'planned_route_path'
                route_line.id = 100
                route_line.type = Marker.LINE_STRIP
                route_line.action = Marker.ADD
                route_line.scale.x = 0.12  # Đường dày 12cm
                route_line.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.90)  # Vàng neon

                p_start = Marker().pose.position
                p_start.x = self.robot_x
                p_start.y = self.robot_y
                p_start.z = 0.10
                route_line.points.append(p_start)

                for idx, wp in enumerate(self.active_waypoints, 1):
                    pw = Marker().pose.position
                    pw.x = float(wp['world_goal']['x'])
                    pw.y = float(wp['world_goal']['y'])
                    pw.z = 0.10
                    route_line.points.append(pw)

                    # Đánh dấu số thứ tự chặng [1], [2], [3] trên RViz2
                    badge = Marker()
                    badge.header.frame_id = 'map'
                    badge.header.stamp = now_msg
                    badge.ns = 'route_waypoint_badges'
                    badge.id = 200 + idx
                    badge.type = Marker.TEXT_VIEW_FACING
                    badge.action = Marker.ADD
                    badge.pose.position.x = pw.x
                    badge.pose.position.y = pw.y
                    badge.pose.position.z = 1.15
                    badge.pose.orientation.w = 1.0
                    badge.scale.z = 0.50
                    badge.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
                    badge.text = f">> STEP {idx} <<"
                    ma.markers.append(badge)

                ma.markers.append(route_line)

            self.pub_markers.publish(ma)
            self.publish_planned_nav_path()

        def publish_planned_nav_path(self):
            """Publish lộ trình dự kiến dạng nav_msgs/msg/Path lên topic /voice_nav/planned_path."""
            if not self.active_waypoints:
                return
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()

            # Điểm đầu là vị trí hiện tại của Robot
            start_p = PoseStamped()
            start_p.header = path_msg.header
            start_p.pose.position.x = float(self.robot_x)
            start_p.pose.position.y = float(self.robot_y)
            start_p.pose.position.z = 0.05
            start_p.pose.orientation.w = 1.0
            path_msg.poses.append(start_p)

            for wp in self.active_waypoints:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(wp['world_goal']['x'])
                ps.pose.position.y = float(wp['world_goal']['y'])
                ps.pose.position.z = 0.05
                half_yaw = float(wp['world_goal']['yaw']) * 0.5
                ps.pose.orientation.z = math.sin(half_yaw)
                ps.pose.orientation.w = math.cos(half_yaw)
                path_msg.poses.append(ps)

            self.pub_planned_path.publish(path_msg)

        def update_live_gui(self, distance_remaining: Optional[float] = None):
            """Cập nhật cửa sổ OpenCV hiển thị bản đồ phân vùng, vị trí Robot và lộ trình điều hướng nhiều chặng."""
            if not self.enable_gui or self.base_map_img is None:
                return
            try:
                canvas = self.base_map_img.copy()
                h, w = canvas.shape[:2]
                res = float(self.map_meta.get('resolution', 0.05))
                origin = self.map_meta.get('origin', [-15.1, -25.1, 0.0])

                ru, rv = world_to_pixel(self.robot_x, self.robot_y, origin, res, h)

                # Vẽ đường lộ trình và đánh số thứ tự chặng (Waypoint 1, 2, 3...)
                if self.active_waypoints:
                    pts = [(ru, rv)]
                    for wp in self.active_waypoints:
                        pts.append((wp['pixel_goal']['u'], wp['pixel_goal']['v']))
                    for i in range(len(pts) - 1):
                        cv2.arrowedLine(
                            canvas, pts[i], pts[i + 1], (0, 215, 255), 3, cv2.LINE_AA, tipLength=0.08
                        )
                    # Vẽ huy hiệu số thứ tự Waypoint trên từng điểm đích
                    for idx, wp in enumerate(self.active_waypoints, 1):
                        wu, wv = wp['pixel_goal']['u'], wp['pixel_goal']['v']
                        cv2.circle(canvas, (wu, wv), 14, (0, 0, 255), -1)
                        cv2.circle(canvas, (wu, wv), 14, (255, 255, 255), 2)
                        cv2.putText(
                            canvas, str(idx), (wu - 5, wv + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2, cv2.LINE_AA
                        )

                # Vẽ vị trí hiện tại của Robot TurtleBot 4
                cv2.circle(canvas, (ru, rv), 10, (0, 255, 255), -1)
                cv2.circle(canvas, (ru, rv), 10, (0, 0, 0), 2)
                end_u = int(round(ru + 18 * math.cos(self.robot_yaw)))
                end_v = int(round(rv - 18 * math.sin(self.robot_yaw)))
                cv2.arrowedLine(canvas, (ru, rv), (end_u, end_v), (0, 0, 0), 2, tipLength=0.35)

                # Vẽ thanh trạng thái ở đáy bản đồ
                cv2.rectangle(canvas, (10, h - 88), (w - 10, h - 10), (25, 30, 40), -1)
                cv2.rectangle(canvas, (10, h - 88), (w - 10, h - 10), (0, 180, 255), 2)
                cmd_ascii = remove_accents(self.last_command_text)
                status_ascii = remove_accents(self.last_status_text)
                dist_txt = f" | Dist: {distance_remaining:.2f}m" if distance_remaining is not None else ""
                cv2.putText(
                    canvas,
                    f"Voice Cmd: {cmd_ascii[:58]}",
                    (20, h - 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (120, 255, 160),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    canvas,
                    f"Status: {status_ascii[:62]}{dist_txt}",
                    (20, h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 230, 120),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    canvas,
                    f"Robot Pose: x={self.robot_x:.2f}m, y={self.robot_y:.2f}m, yaw={math.degrees(self.robot_yaw):.0f} deg",
                    (20, h - 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (210, 210, 210),
                    1,
                    cv2.LINE_AA,
                )

                # Cắt vùng hiển thị bao phủ toàn bộ nhà kho (từ Khu nhập hàng phía Bắc đến Khu kệ hàng phía Nam)
                crop = canvas[145:950, 40:w - 20]
                header = canvas[0:144, 0:w]
                footer = canvas[h - 92:h, 0:w]
                crop_resized = cv2.resize(crop, (w, 540), interpolation=cv2.INTER_AREA)
                display_img = np.vstack([header, crop_resized, footer])
                cv2.imshow('TurtleBot 4 - Warehouse Voice Navigation Map', display_img)
                cv2.waitKey(1)
            except Exception:
                pass

        def _setInitialPose(self):
            msg = PoseWithCovarianceStamped()
            msg.pose.pose = self.initial_pose.pose
            msg.header.frame_id = self.initial_pose.header.frame_id
            now = self.get_clock().now()
            if now.nanoseconds > 500_000_000:
                past = now - rclpy.duration.Duration(seconds=0.5)
                msg.header.stamp = past.to_msg()
            else:
                msg.header.stamp = now.to_msg()
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.068
            self.initial_pose_pub.publish(msg)

    def create_pose_stamped(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        half_yaw = yaw * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def ensure_nodes_active(navigator: BasicNavigator, node_names: list, timeout_sec: float = 15.0):
        start_time = time.time()
        for name in node_names:
            get_client = navigator.create_client(GetState, f'/{name}/get_state')
            change_client = navigator.create_client(ChangeState, f'/{name}/change_state')
            if not get_client.wait_for_service(timeout_sec=4.0):
                continue
            while time.time() - start_time < timeout_sec:
                req = GetState.Request()
                fut = get_client.call_async(req)
                rclpy.spin_until_future_complete(navigator, fut, timeout_sec=2.0)
                res = fut.result()
                if res:
                    state = res.current_state.label
                    if state == 'active':
                        break
                    elif state == 'inactive':
                        req_act = ChangeState.Request()
                        req_act.transition.id = Transition.TRANSITION_ACTIVATE
                        req_act.transition.label = 'activate'
                        fut_act = change_client.call_async(req_act)
                        rclpy.spin_until_future_complete(navigator, fut_act, timeout_sec=5.0)
                    elif state == 'unconfigured':
                        req_cfg = ChangeState.Request()
                        req_cfg.transition.id = Transition.TRANSITION_CONFIGURE
                        req_cfg.transition.label = 'configure'
                        fut_cfg = change_client.call_async(req_cfg)
                        rclpy.spin_until_future_complete(navigator, fut_cfg, timeout_sec=5.0)
                time.sleep(0.4)

    paths = get_default_paths()
    regions_db = load_semantic_regions(args.regions_json or paths['regions_json'])
    rclpy.init(args=sys.argv + ['--ros-args', '-p', 'use_sim_time:=true'])
    server = VoiceNavigationServer(
        regions_db=regions_db,
        segmented_map_path=paths['segmented_png'],
        enable_gui=not args.no_gui,
    )

    print('\n========================================================================')
    print('   TURTLEBOT 4 - HỆ THỐNG ĐIỀU HƯỚNG BẰNG GIỌNG NÓI (VOICE NAV2)       ')
    print('========================================================================')
    for r in server.regions:
        wx, wy, wyaw = r['world_goal']['x'], r['world_goal']['y'], r['world_goal']['yaw']
        print(f" * Khu vực {r['id']} ({r['name']:<17}) -> Goal: x={wx:>5.2f}m, y={wy:>5.2f}m, yaw={wyaw:>5.2f} rad")
    print('========================================================================')

    if not args.skip_nav2_init:
        print('Đang đồng bộ simulation clock từ Gazebo...')
        wait_start = time.time()
        while rclpy.ok() and server.get_clock().now().nanoseconds == 0 and time.time() - wait_start < 15.0:
            rclpy.spin_once(server, timeout_sec=0.1)

        # Kiểm tra và kích hoạt Localization & Nav2
        print('>>> Kiểm tra trạng thái Map Server, AMCL và Nav2 stack...')
        ensure_nodes_active(
            server,
            [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ],
        )

        # Thiết lập Initial Pose nếu AMCL mới khởi động
        init_pose = create_pose_stamped(server, 0.35, 0.0, 3.14159)
        server.setInitialPose(init_pose)
        try:
            nomotion_client = server.create_client(Empty, '/request_nomotion_update')
            if nomotion_client.wait_for_service(timeout_sec=1.5):
                fut = nomotion_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(server, fut, timeout_sec=1.5)
        except Exception:
            pass

    server.publish_semantic_markers()
    print('>>> Node voice_navigation_node đã sẵn sàng lắng nghe topic /voice_command!\n')

    # Nếu truyền câu lệnh trực tiếp qua tham số dòng lệnh --command thì nạp luôn
    if args.command:
        server.pending_commands.append(args.command)

    try:
        while rclpy.ok():
            rclpy.spin_once(server, timeout_sec=0.1)
            server.update_live_gui()

            if not server.pending_commands:
                continue

            cmd_text = server.pending_commands.pop(0)
            server.last_command_text = cmd_text
            parsed = parse_voice_command(cmd_text, regions_db)

            print('------------------------------------------------------------------------')
            print(f' [CÂU LỆNH GIỌNG NÓI] : "{cmd_text}"')
            print(f" [PHÂN TÍCH NGỮ NGHĨA]: Loại={parsed['command_type']} | Hợp lệ={parsed['valid']}")
            print(f" [THÔNG BÁO]          : {parsed['message']}")

            if not parsed['valid']:
                print(f" [CẢNH BÁO AN TOÀN]   : Từ chối điều hướng vì khu vực không tồn tại trên bản đồ!")
                print('------------------------------------------------------------------------\n')
                server.publish_status({
                    'state': 'ERROR_INVALID_REGION',
                    'command': cmd_text,
                    'unknown_region': parsed.get('unknown_region', ''),
                    'message': parsed['message'],
                })
                server.update_live_gui()
                if args.once and not server.pending_commands:
                    break
                continue

            waypoints, plan_msg = plan_waypoints_for_command(
                parsed, (server.robot_x, server.robot_y)
            )
            print(f' [KẾ HOẠCH LỘ TRÌNH]  : {plan_msg}')
            print('------------------------------------------------------------------------')

            server.active_waypoints = list(waypoints)
            server.publish_status({
                'state': 'NAVIGATING',
                'command': cmd_text,
                'waypoints': [w['id'] for w in waypoints],
                'message': plan_msg,
            })

            all_succeeded = True
            for step_idx, wp in enumerate(waypoints, 1):
                gx = wp['world_goal']['x']
                gy = wp['world_goal']['y']
                gyaw = wp['world_goal']['yaw']
                server.current_target = wp

                step_title = (
                    f"[CHẶNG {step_idx}/{len(waypoints)}] Đi tới Khu vực {wp['id']} "
                    f"({wp['name']}) tại (x={gx:.2f}m, y={gy:.2f}m, yaw={math.degrees(gyaw):.1f}°)"
                )
                print(f' >>> {step_title}')
                server.last_status_text = step_title
                server.update_live_gui()

                goal_pose = create_pose_stamped(server, gx, gy, gyaw)
                server.goToPose(goal_pose)

                start_t = time.time()
                last_log_t = start_t
                close_count = 0
                force_succeed = False

                while not server.isTaskComplete():
                    rclpy.spin_once(server, timeout_sec=0.05)
                    fb = server.getFeedback()
                    now_t = time.time()
                    dist_rem = fb.distance_remaining if fb else None
                    server.update_live_gui(distance_remaining=dist_rem)

                    if fb:
                        if dist_rem <= 0.28:
                            close_count += 1
                        else:
                            close_count = 0

                        if close_count >= 10:
                            print(f"  -> Robot đã tiếp cận đích Khu vực {wp['id']} ({dist_rem:.2f}m)!")
                            server.cancelTask()
                            force_succeed = True
                            break

                        if now_t - last_log_t >= 2.0:
                            last_log_t = now_t
                            print(
                                f"  -> Đang di chuyển tới [{wp['id']}] {wp['name']}... "
                                f"Còn lại: {dist_rem:.2f} m ({fb.navigation_time.sec}s)"
                            )
                    time.sleep(0.35)

                result = server.getResult()
                elapsed = time.time() - start_t
                if result == TaskResult.SUCCEEDED or force_succeed:
                    server.robot_x, server.robot_y, server.robot_yaw = gx, gy, gyaw
                    print(
                        f"  [HOÀN THÀNH CHẶNG {step_idx}/{len(waypoints)}] "
                        f"Robot đã đến Khu vực {wp['id']} ({wp['name']}) sau {elapsed:.1f}s!\n"
                    )
                    if server.active_waypoints:
                        server.active_waypoints.pop(0)
                    time.sleep(1.5)
                else:
                    all_succeeded = False
                    print(f"  [CẢNH BÁO] Chặng {step_idx} đến Khu vực {wp['id']} không hoàn tất (status={result}).\n")
                    break

            if all_succeeded:
                final_wp = waypoints[-1]
                done_msg = f"Đã hoàn thành lệnh '{cmd_text}' -> Robot đang ở Khu vực {final_wp['id']} ({final_wp['name']})."
                print(f' ===> [THÀNH CÔNG] {done_msg}\n')
                server.publish_status({
                    'state': 'SUCCEEDED',
                    'command': cmd_text,
                    'current_region': final_wp['id'],
                    'message': done_msg,
                })
            server.active_waypoints = []
            server.update_live_gui()

            if args.once and not server.pending_commands:
                break

    except KeyboardInterrupt:
        print('\nĐã dừng node điều hướng giọng nói.')
    finally:
        if server.enable_gui:
            cv2.destroyAllWindows()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_self_test():
    """Kiểm thử tự động bộ phân tích câu lệnh giọng nói nhà kho và lập lộ trình đa chặng / đa điểm."""
    regions_db = load_semantic_regions()
    print('========================================================================')
    print('   KIỂM THỬ BỘ PHÂN TÍCH CÂU LỆNH GIỌNG NÓI NHÀ KHO (WAREHOUSE NLP)     ')
    print('========================================================================')

    test_cases = [
        ('Đi đến khu vực A.', (0.35, 0.0), True, ['A']),
        ('Di chuyển từ khu vực A đến khu vực B.', (0.35, 0.0), True, ['A', 'B']),
        ('Di chuyển từ khu vực B đến khu vực C.', (2.75, 5.25), True, ['C']),
        ('Đi đến khu nhập hàng.', (0.35, 0.0), True, ['A']),
        ('Đi đến khu phân loại.', (0.35, 0.0), True, ['B']),
        ('Di chuyển tới khu xuất hàng.', (0.35, 0.0), True, ['C']),
        ('Đi đến khu kệ hàng.', (0.35, 0.0), True, ['D']),
        ('Quay về trạm sạc.', (3.20, 1.20), True, ['E']),
        # Mở rộng: Câu lệnh nhiều điểm đến
        ('Đi từ phòng A qua hành lang C rồi đến phòng B.', (0.35, 0.0), True, ['A', 'C', 'B']),
        ('Đi từ khu vực A qua khu vực C rồi đến khu vực B.', (1.80, 12.95), True, ['C', 'B']),
        ('Đi từ khu nhập hàng qua khu phân loại rồi đến khu xuất hàng.', (0.35, 0.0), True, ['A', 'B', 'C']),
        ('Đi qua khu vực A rồi đến khu vực B.', (0.35, 0.0), True, ['A', 'B']),
        ('Đi đến khu vực B rồi đến khu vực D.', (0.35, 0.0), True, ['B', 'D']),
        # Chuỗi liên hoàn 5 điểm đến
        ('qua A qua B qua C qua D rồi qua E', (0.35, 0.0), True, ['A', 'B', 'C', 'D', 'E']),
        ('Đi qua A, B, C, D rồi đến E', (0.35, 0.0), True, ['A', 'B', 'C', 'D', 'E']),
        ('Đi từ A qua B, C, D rồi về E', (0.35, 0.0), True, ['A', 'B', 'C', 'D', 'E']),
        ('Đi từ A qua B, C, D rồi về E', (1.80, 13.00), True, ['B', 'C', 'D', 'E']),
        ('Đi qua khu nhập hàng, qua phân loại, qua xuất hàng, qua kệ hàng rồi về trạm sạc', (0.35, 0.0), True, ['A', 'B', 'C', 'D', 'E']),
        ('Đi qua A rồi qua B sau đó qua C tiếp tục qua D và cuối cùng đến E', (0.35, 0.0), True, ['A', 'B', 'C', 'D', 'E']),
        # Bắt lỗi an toàn khi vùng không tồn tại
        ('Đi đến khu vực Z.', (0.35, 0.0), False, []),
        ('Đi từ phòng A qua phòng Z rồi đến phòng B.', (0.35, 0.0), False, []),
        ('qua A qua B qua C qua Z rồi qua E', (0.35, 0.0), False, []),
    ]

    for idx, (cmd, robot_xy, exp_valid, exp_route) in enumerate(test_cases, 1):
        parsed = parse_voice_command(cmd, regions_db)
        waypoints, plan_msg = plan_waypoints_for_command(parsed, robot_xy)
        route_ids = [w['id'] for w in waypoints]
        assert parsed['valid'] == exp_valid, f"Case {idx} failed valid check: {parsed}"
        assert route_ids == exp_route, f"Case {idx} expected route {exp_route}, got {route_ids}"
        status_tag = 'HỢP LỆ' if parsed['valid'] else 'TỪ CHỐI (KHU VỰC KHÔNG TỒN TẠI TRONG NHÀ KHO)'
        print(f" [{idx:02d}] Câu lệnh : \"{cmd}\" (Robot tại {robot_xy})")
        print(f"      Trạng thái: {status_tag} | Lộ trình Waypoints: {route_ids}")
        print(f"      Chi tiết  : {plan_msg}\n")

    print('========================================================================')
    print(f'[SELF-TEST PASSED] Tất cả {len(test_cases)} kịch bản câu lệnh giọng nói nhà kho chính xác 100%!')
    print('========================================================================')


def main():
    parser = argparse.ArgumentParser(description='Node điều hướng bằng giọng nói cho TurtleBot 4.')
    parser.add_argument('--regions-json', default='', help='Đường dẫn file semantic_regions.json')
    parser.add_argument('--command', default='', help='Thực thi ngay một câu lệnh giọng nói')
    parser.add_argument('--once', action='store_true', help='Thoát sau khi thực thi xong câu lệnh')
    parser.add_argument('--no-gui', action='store_true', help='Tắt cửa sổ hiển thị OpenCV')
    parser.add_argument('--skip-nav2-init', action='store_true', help='Bỏ qua bước đợi lifecycle init')
    parser.add_argument('--self-test', action='store_true', help='Chạy kiểm thử tự động bộ NLP Parser')
    args, _ = parser.parse_known_args()

    if args.self_test:
        run_self_test()
        return

    run_ros2_node(args)


if __name__ == '__main__':
    main()

