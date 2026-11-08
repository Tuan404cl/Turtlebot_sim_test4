#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 1 & PHẦN 2: PHÂN VÙNG BẢN ĐỒ NHÀ KHO (WAREHOUSE MAP SEGMENTATION) & GẮN NHÃN NGỮ NGHĨA (SEMANTIC MAPPING)
---------------------------------------------------------------------------------------------------------------
Xử lý bản đồ 2D Occupancy Grid của Nhà kho (maps/map.pgm & maps/map.yaml) bằng OpenCV:
1. Binary Thresholding: Tách vùng trống di chuyển được (Free Space >= 250), vật cản (Occupied == 0) và vùng kệ kín (205).
2. Morphological Filtering: Khử nhiễu tia quét LiDAR và làm liền mạch biên tường/kệ hàng bằng phép đóng/mở hình thái học.
3. Euclidean Distance Transform: Tính khoảng cách an toàn từ mọi điểm trống trong nhà kho tới tường/kệ hàng gần nhất.
4. Marker-Controlled Watershed Segmentation: Phân chia toàn bộ nhà kho thành 05 khu vực vận hành kho vận (Warehouse Zones):
   - Khu vực A: Khu nhập hàng (Inbound / Receiving Bay - Khoang phía Bắc)
   - Khu vực B: Khu phân loại (Sorting & Inspection Zone - Sảnh trung tâm)
   - Khu vực C: Khu xuất hàng (Outbound / Packing & Shipping Zone - Sảnh phía Đông)
   - Khu vực D: Khu kệ hàng (Storage Racks Aisles - Dãy hành lang kệ hàng dọc phía Nam)
   - Khu vực E: Trạm sạc (Charging Dock & Control Station - Khu vực phía Tây quanh Dock)
5. Quy đổi tọa độ Pixel (u, v) <-> Tọa độ thực ROS map (x, y, yaw) và xuất dữ liệu ra:
   - config/semantic_regions.json & config/semantic_regions.yaml
   - maps/map_segmented.png & maps/map_segmented.pgm
   - docs/figures/map_segmentation_pipeline.png
"""

import argparse
import json
import math
import os
from typing import Dict, List, Tuple

import cv2
import numpy as np
import yaml


def get_default_paths() -> Dict[str, str]:
    """Lấy đường dẫn mặc định tới các file trong package turtlebot4_practice4."""
    ws_src_pkg = '/home/tuan/turtelbot4_ws/src/turtlebot4_practice4'
    if not os.path.isdir(ws_src_pkg):
        ws_src_pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    return {
        'pkg_dir': ws_src_pkg,
        'map_yaml': os.path.join(ws_src_pkg, 'maps', 'map.yaml'),
        'map_pgm': os.path.join(ws_src_pkg, 'maps', 'map.pgm'),
        'segmented_png': os.path.join(ws_src_pkg, 'maps', 'map_segmented.png'),
        'segmented_pgm': os.path.join(ws_src_pkg, 'maps', 'map_segmented.pgm'),
        'pipeline_fig': os.path.join(ws_src_pkg, 'docs', 'figures', 'map_segmentation_pipeline.png'),
        'regions_json': os.path.join(ws_src_pkg, 'config', 'semantic_regions.json'),
        'regions_yaml': os.path.join(ws_src_pkg, 'config', 'semantic_regions.yaml'),
    }


def world_to_pixel(x: float, y: float, origin: List[float], resolution: float, height: int) -> Tuple[int, int]:
    """Chuyển đổi tọa độ thực ROS map (x, y) [m] sang tọa độ ảnh pixel (u, v)."""
    ox, oy = float(origin[0]), float(origin[1])
    u = int(round((x - ox) / resolution))
    v = int(round(height - 1 - (y - oy) / resolution))
    return u, v


def pixel_to_world(u: int, v: int, origin: List[float], resolution: float, height: int) -> Tuple[float, float]:
    """Chuyển đổi tọa độ ảnh pixel (u, v) sang tọa độ thực ROS map (x, y) [m]."""
    ox, oy = float(origin[0]), float(origin[1])
    x = ox + (u + 0.5) * resolution
    y = oy + (height - 1 - v + 0.5) * resolution
    return round(float(x), 3), round(float(y), 3)


def define_semantic_seeds() -> List[Dict]:
    """
    Định nghĩa 05 khu vực chức năng chuẩn trên bản đồ Nhà kho (Warehouse) của TurtleBot 4.
    Kết hợp định danh chữ cái (A, B, C, D, E) và tên chức năng kho vận thực tế
    (Khu nhập hàng, Khu phân loại, Khu xuất hàng, Khu kệ hàng, Trạm sạc).
    """
    return [
        {
            'id': 'A',
            'name': 'Khu nhập hàng',
            'name_ascii': 'Khu nhap hang',
            'full_label': 'Khu vực A - Khu nhập hàng',
            'description': 'Khoang tiếp nhận và tập kết hàng hóa nhập kho phía Bắc (phía trên kệ ngang)',
            'aliases': [
                'khu vực a', 'khu vuc a', 'vùng a', 'vung a', 'điểm a', 'diem a', 'a',
                'phòng a', 'phong a', 'khu a',
                'khu nhập hàng', 'khu nhap hang', 'kho nhập hàng', 'kho nhap hang',
                'nhập hàng', 'nhap hang', 'kho phía bắc', 'kho phia bac',
                'inbound', 'inbound bay', 'receiving zone', 'zone a', 'area a', 'region a'
            ],
            'seed_world': {'x': 1.80, 'y': 13.00, 'yaw': 0.00},
            'roi_bounds_world': {'x_min': -10.0, 'x_max': 12.2, 'y_min': 9.6, 'y_max': 16.8},
            'label_offset_px': (-65, -34),
            'color_bgr': [190, 125, 35],   # Xanh dương đậm (Steel Blue)
            'color_rgb': [35, 125, 190],
            'color_hex': '#237DBE',
        },
        {
            'id': 'B',
            'name': 'Khu phân loại',
            'name_ascii': 'Khu phan loai',
            'full_label': 'Khu vực B - Khu phân loại',
            'description': 'Sảnh trung tâm kiểm đếm và phân loại hàng hóa phía dưới dãy kệ ngang',
            'aliases': [
                'khu vực b', 'khu vuc b', 'vùng b', 'vung b', 'điểm b', 'diem b', 'b',
                'phòng b', 'phong b', 'khu b',
                'khu phân loại', 'khu phan loai', 'khu phân loại hàng', 'khu phan loai hang',
                'phân loại', 'phan loai', 'phân loại hàng', 'phan loai hang',
                'khu kiểm hàng', 'khu kiem hang', 'sảnh trung tâm', 'sanh trung tam',
                'sorting', 'sorting zone', 'zone b', 'area b', 'region b'
            ],
            'seed_world': {'x': 2.80, 'y': 5.30, 'yaw': 1.57},
            'roi_bounds_world': {'x_min': -10.0, 'x_max': 12.4, 'y_min': 3.0, 'y_max': 8.2},
            'label_offset_px': (-65, -32),
            'color_bgr': [75, 175, 75],    # Xanh lá (Emerald Green)
            'color_rgb': [75, 175, 75],
            'color_hex': '#4BAF4B',
        },
        {
            'id': 'C',
            'name': 'Khu xuất hàng',
            'name_ascii': 'Khu xuat hang',
            'full_label': 'Khu vực C - Khu xuất hàng',
            'description': 'Khu vực đóng gói và điều phối xuất hàng phía Đông nhà kho',
            'aliases': [
                'khu vực c', 'khu vuc c', 'vùng c', 'vung c', 'điểm c', 'diem c', 'c',
                'phòng c', 'phong c', 'khu c', 'hành lang c', 'hanh lang c',
                'khu xuất hàng', 'khu xuat hang', 'kho xuất hàng', 'kho xuat hang',
                'xuất hàng', 'xuat hang', 'khu đóng gói', 'khu dong goi',
                'đóng gói', 'dong goi', 'khu giao hàng', 'khu giao hang',
                'outbound', 'shipping zone', 'packing zone', 'zone c', 'area c', 'region c'
            ],
            'seed_world': {'x': 3.20, 'y': 1.20, 'yaw': 0.00},
            'roi_bounds_world': {'x_min': 0.3, 'x_max': 12.4, 'y_min': -3.9, 'y_max': 3.0},
            'label_offset_px': (14, -22),
            'color_bgr': [40, 155, 240],   # Cam hổ phách (Warm Amber)
            'color_rgb': [240, 155, 40],
            'color_hex': '#F09B28',
        },
        {
            'id': 'D',
            'name': 'Khu kệ hàng',
            'name_ascii': 'Khu ke hang',
            'full_label': 'Khu vực D - Khu kệ hàng',
            'description': 'Hệ thống hành lang và 3 dãy kệ hàng lưu trữ dọc phía Nam nhà kho',
            'aliases': [
                'khu vực d', 'khu vuc d', 'vùng d', 'vung d', 'điểm d', 'diem d', 'd',
                'phòng d', 'phong d', 'khu d', 'hành lang d', 'hanh lang d',
                'khu kệ hàng', 'khu ke hang', 'kệ hàng', 'ke hang',
                'khu lưu trữ', 'khu luu tru', 'kho lưu trữ', 'kho luu tru',
                'hành lang kệ hàng', 'hanh lang ke hang', 'hành lang', 'hanh lang',
                'storage', 'storage racks', 'aisle', 'zone d', 'area d', 'region d'
            ],
            'seed_world': {'x': 2.00, 'y': -8.50, 'yaw': -1.57},
            'roi_bounds_world': {'x_min': -10.0, 'x_max': 7.2, 'y_min': -22.0, 'y_max': -3.9},
            'label_offset_px': (-55, 22),
            'color_bgr': [175, 165, 45],   # Xanh ngọc lam (Teal Cyan)
            'color_rgb': [45, 165, 175],
            'color_hex': '#2DA5AF',
        },
        {
            'id': 'E',
            'name': 'Trạm sạc',
            'name_ascii': 'Tram sac',
            'full_label': 'Khu vực E - Trạm sạc',
            'description': 'Trạm sạc xuất phát (Docking Station) và khu vực điều phối phía Tây',
            'aliases': [
                'khu vực e', 'khu vuc e', 'vùng e', 'vung e', 'điểm e', 'diem e', 'e',
                'phòng e', 'phong e', 'khu e',
                'trạm sạc', 'tram sac', 'khu trạm sạc', 'khu tram sac',
                'khu điều phối', 'khu dieu phoi', 'trạm bảo trì', 'tram bao tri',
                'vị trí xuất phát', 'vi tri xuat phat', 'dock', 'charging station',
                'zone e', 'area e', 'region e'
            ],
            'seed_world': {'x': -1.60, 'y': 0.00, 'yaw': 3.14},
            'roi_bounds_world': {'x_min': -10.0, 'x_max': 0.3, 'y_min': -3.9, 'y_max': 3.0},
            'label_offset_px': (-158, 22),
            'color_bgr': [155, 85, 190],   # Tím hồng (Amethyst Purple)
            'color_rgb': [190, 85, 155],
            'color_hex': '#BE559B',
        },
    ]


def build_warehouse_interior_mask(
    free_clean: np.ndarray,
    origin: List[float],
    resolution: float,
    height: int,
    width: int,
) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    """
    Xây dựng mặt nạ không gian bên trong nhà kho (loại bỏ viền trắng bên ngoài tường bao),
    bao phủ toàn bộ từ Khoang nhập hàng phía Bắc (y = +16.8m) xuống hết Hành lang kệ hàng phía Nam (y = -22.0m).
    """
    interior_geom = np.zeros_like(free_clean)

    # 1. Khoang phía Bắc (North Inbound Bay: x in [-10.0, 12.2], y in [8.5, 16.85])
    u1_n, v2_n = world_to_pixel(-10.0, 8.5, origin, resolution, height)
    u2_n, v1_n = world_to_pixel(12.2, 16.85, origin, resolution, height)
    interior_geom[max(0, v1_n):min(height, v2_n), max(0, u1_n):min(width, u2_n)] = 255

    # 2. Sảnh trung tâm, Khu xuất hàng & Trạm sạc (Central & Middle Hall: x in [-10.0, 12.5], y in [-4.0, 9.0])
    u1_c, v2_c = world_to_pixel(-10.0, -4.0, origin, resolution, height)
    u2_c, v1_c = world_to_pixel(12.5, 9.0, origin, resolution, height)
    interior_geom[max(0, v1_c):min(height, v2_c), max(0, u1_c):min(width, u2_c)] = 255

    # 3. Khu vực 3 dãy kệ hàng dọc phía Nam (South Storage Rack Aisles: x in [-10.0, 7.2], y in [-22.1, -3.8])
    u1_s, v2_s = world_to_pixel(-10.0, -22.1, origin, resolution, height)
    u2_s, v1_s = world_to_pixel(7.2, -3.8, origin, resolution, height)
    interior_geom[max(0, v1_s):min(height, v2_s), max(0, u1_s):min(width, u2_s)] = 255

    active_free_mask = cv2.bitwise_and(free_clean, interior_geom)
    u_min_roi = max(0, min(u1_n, u1_c, u1_s))
    u_max_roi = min(width - 1, max(u2_n, u2_c, u2_s))
    v_min_roi = max(0, min(v1_n, v1_c, v1_s))
    v_max_roi = min(height - 1, max(v2_n, v2_c, v2_s))

    return active_free_mask, (u_min_roi, v_min_roi, u_max_roi, v_max_roi)


def segment_occupancy_map(
    map_yaml_path: str,
    output_segmented_png: str,
    output_segmented_pgm: str,
    output_pipeline_fig: str,
    output_json: str,
    output_yaml: str,
) -> Dict:
    """
    Thực hiện toàn bộ quy trình xử lý ảnh phân vùng bản đồ Occupancy Grid (.pgm) của Nhà kho
    và lưu bản đồ phân vùng + dữ liệu ngữ nghĩa JSON/YAML.
    """
    with open(map_yaml_path, 'r', encoding='utf-8') as f:
        map_meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    pgm_filename = map_meta.get('image', 'map.pgm')
    pgm_path = pgm_filename if os.path.isabs(pgm_filename) else os.path.join(map_dir, pgm_filename)

    img_gray = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img_gray is None:
        raise FileNotFoundError(f'Không thể đọc ảnh bản đồ PGM tại: {pgm_path}')

    height, width = img_gray.shape
    resolution = float(map_meta['resolution'])
    origin = [float(v) for v in map_meta['origin']]

    # -------------------------------------------------------------------------
    # BƯỚC 1: Binary Thresholding - Tách vùng không gian trống (Free Space)
    # Trong map ROS trinary: 254 = Free, 205 = Unknown (bên trong kệ hàng), 0 = Occupied (tường/vách kệ)
    # -------------------------------------------------------------------------
    _, free_raw = cv2.threshold(img_gray, 250, 255, cv2.THRESH_BINARY)
    obstacle_mask = (img_gray < 100).astype(np.uint8) * 255

    # -------------------------------------------------------------------------
    # BƯỚC 2: Morphological Filtering - Khử nhiễu và làm liền mạch cấu trúc kệ kho
    # -------------------------------------------------------------------------
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    free_clean = cv2.morphologyEx(free_raw, cv2.MORPH_OPEN, kernel_open, iterations=1)
    free_clean = cv2.morphologyEx(free_clean, cv2.MORPH_CLOSE, kernel_close, iterations=1)

    # Bảo toàn tuyệt đối các bức tường và vách kệ hàng
    free_clean[obstacle_mask > 0] = 0
    free_clean[img_gray < 250] = 0

    active_free_mask, (u_min_roi, v_min_roi, u_max_roi, v_max_roi) = build_warehouse_interior_mask(
        free_clean, origin, resolution, height, width
    )

    # -------------------------------------------------------------------------
    # BƯỚC 3: Euclidean Distance Transform - Tính khoảng cách tới tường/kệ hàng thực tế
    # -------------------------------------------------------------------------
    dist_pixels = cv2.distanceTransform(free_clean, cv2.DIST_L2, 5)
    dist_meters = dist_pixels * resolution

    # -------------------------------------------------------------------------
    # BƯỚC 4: Marker-Controlled Watershed Segmentation (05 Khu vực Nhà kho A..E)
    # -------------------------------------------------------------------------
    seeds_cfg = define_semantic_seeds()
    markers = np.zeros((height, width), dtype=np.int32)
    # Gán nhãn nền = 1 cho toàn bộ vùng tường/kệ hàng/ngoài nhà kho
    markers[active_free_mask == 0] = 1

    # Đặt hạt giống (Seeds) cho 5 khu vực A (nhãn 2), B (nhãn 3), C (nhãn 4), D (nhãn 5), E (nhãn 6)
    # Chỉ để vùng đệm tự do (unlabeled = 0) tại các vùng chuyển tiếp giữa các khu vực liền kề
    # để thuật toán Watershed tự động phân chia ranh giới theo trường gradient khoảng cách
    for idx, reg in enumerate(seeds_cfg):
        label_id = idx + 2
        sx, sy = reg['seed_world']['x'], reg['seed_world']['y']
        su, sv = world_to_pixel(sx, sy, origin, resolution, height)

        b = reg['roi_bounds_world']
        bu1, bv2 = world_to_pixel(b['x_min'], b['y_min'], origin, resolution, height)
        bu2, bv1 = world_to_pixel(b['x_max'], b['y_max'], origin, resolution, height)
        core_u1 = max(0, bu1 + (10 if b['x_min'] > -5.0 else 0))
        core_u2 = min(width, bu2 - (10 if b['x_max'] < 5.0 else 0))
        core_v1 = max(0, bv1 + (12 if b['y_max'] < 15.0 else 0))
        core_v2 = min(height, bv2 - (12 if b['y_min'] > -20.0 else 0))

        core_slice = active_free_mask[core_v1:core_v2, core_u1:core_u2] > 0
        markers[core_v1:core_v2, core_u1:core_u2][core_slice] = label_id
        cv2.circle(markers, (su, sv), 8, int(label_id), -1)

    markers[active_free_mask == 0] = 1

    # Tạo ảnh 3 kênh từ trường gradient của Distance Transform để Watershed chia ranh giới tự nhiên tại các lối giao
    dist_norm = cv2.normalize(dist_pixels, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    dist_inv = 255 - dist_norm
    watershed_input = cv2.cvtColor(dist_inv, cv2.COLOR_GRAY2BGR)
    cv2.watershed(watershed_input, markers)

    # -------------------------------------------------------------------------
    # BƯỚC 5: Trích xuất thông tin từng khu vực, kiểm tra điểm đích an toàn & vẽ bản đồ
    # -------------------------------------------------------------------------
    vis_img = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    seg_pgm = img_gray.copy()
    regions_output = []

    for idx, reg in enumerate(seeds_cfg):
        label_id = idx + 2
        region_mask = (markers == label_id) & (active_free_mask > 0)
        area_px = int(np.count_nonzero(region_mask))
        area_m2 = round(area_px * (resolution ** 2), 2)

        ys, xs = np.where(region_mask)
        if len(xs) > 0:
            cu = int(round(float(np.mean(xs))))
            cv_y = int(round(float(np.mean(ys))))
            u_min, u_max = int(np.min(xs)), int(np.max(xs))
            v_min, v_max = int(np.min(ys)), int(np.max(ys))
        else:
            cu, cv_y = world_to_pixel(reg['seed_world']['x'], reg['seed_world']['y'], origin, resolution, height)
            u_min, v_min, u_max, v_max = cu - 20, cv_y - 20, cu + 20, cv_y + 20

        gx, gy, gyaw = reg['seed_world']['x'], reg['seed_world']['y'], reg['seed_world']['yaw']
        gu, gv = world_to_pixel(gx, gy, origin, resolution, height)
        clearance_m = float(dist_meters[gv, gu])

        if clearance_m < 0.45 and len(xs) > 0:
            masked_dist = np.where(region_mask, dist_meters, -1.0)
            best_v, best_u = np.unravel_index(np.argmax(masked_dist), masked_dist.shape)
            gu, gv = int(best_u), int(best_v)
            gx, gy = pixel_to_world(gu, gv, origin, resolution, height)
            clearance_m = float(dist_meters[gv, gu])

        cx_world, cy_world = pixel_to_world(cu, cv_y, origin, resolution, height)
        w_min_x, w_min_y = pixel_to_world(u_min, v_max, origin, resolution, height)
        w_max_x, w_max_y = pixel_to_world(u_max, v_min, origin, resolution, height)

        color_bgr = np.array(reg['color_bgr'], dtype=np.uint8)
        vis_img[region_mask] = (
            0.46 * vis_img[region_mask].astype(np.float32) + 0.54 * color_bgr.astype(np.float32)
        ).astype(np.uint8)

        gray_shade = 125 + idx * 22
        seg_pgm[region_mask] = gray_shade

        reg_uint8 = (region_mask.astype(np.uint8)) * 255
        contours, _ = cv2.findContours(reg_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis_img, contours, -1, [int(c * 0.62) for c in reg['color_bgr']], 2)
        cv2.drawContours(seg_pgm, contours, -1, 40, 1)

        region_info = {
            'id': reg['id'],
            'name': reg['name'],
            'name_ascii': reg['name_ascii'],
            'full_label': reg['full_label'],
            'description': reg['description'],
            'aliases': reg['aliases'],
            'pixel_goal': {'u': int(gu), 'v': int(gv)},
            'pixel_centroid': {'u': int(cu), 'v': int(cv_y)},
            'world_goal': {
                'x': round(float(gx), 2),
                'y': round(float(gy), 2),
                'yaw': round(float(gyaw), 2),
            },
            'world_centroid': {
                'x': round(float(cx_world), 2),
                'y': round(float(cy_world), 2),
            },
            'bounding_box_pixel': {
                'u_min': int(u_min),
                'v_min': int(v_min),
                'u_max': int(u_max),
                'v_max': int(v_max),
            },
            'bounding_box_world': {
                'x_min': round(float(w_min_x), 2),
                'y_min': round(float(w_min_y), 2),
                'x_max': round(float(w_max_x), 2),
                'y_max': round(float(w_max_y), 2),
            },
            'area_pixels': area_px,
            'area_m2': area_m2,
            'clearance_m': round(clearance_m, 2),
            'is_valid_free_space': bool(img_gray[gv, gu] >= 250 and clearance_m >= 0.45),
            'color_bgr': reg['color_bgr'],
            'color_rgb': reg['color_rgb'],
            'color_hex': reg['color_hex'],
        }
        regions_output.append(region_info)

    # Vẽ nhãn, tâm đích, hướng Yaw và chú thích lên ảnh phân vùng
    offset_map = {r['id']: r.get('label_offset_px', (-68, -36)) for r in seeds_cfg}
    for reg_info in regions_output:
        gu, gv = reg_info['pixel_goal']['u'], reg_info['pixel_goal']['v']
        gx, gy, gyaw = (
            reg_info['world_goal']['x'],
            reg_info['world_goal']['y'],
            reg_info['world_goal']['yaw'],
        )
        color = tuple(int(c) for c in reg_info['color_bgr'])

        arrow_len = 22
        end_u = int(round(gu + arrow_len * math.cos(gyaw)))
        end_v = int(round(gv - arrow_len * math.sin(gyaw)))
        cv2.arrowedLine(vis_img, (gu, gv), (end_u, end_v), (0, 0, 220), 2, tipLength=0.35)

        cv2.circle(vis_img, (gu, gv), 7, (255, 255, 255), -1)
        cv2.circle(vis_img, (gu, gv), 5, (0, 0, 220), -1)
        cv2.circle(seg_pgm, (gu, gv), 5, 0, -1)

        off_u, off_v = offset_map.get(reg_info['id'], (-68, -36))
        title_txt = f"[{reg_info['id']}] {reg_info['name_ascii']}"
        coord_txt = f"Goal: ({gx:.2f}, {gy:.2f})m"
        box_u = max(12, min(gu + off_u, width - 165))
        box_v = max(150, min(gv + off_v, height - 35))

        cv2.line(vis_img, (gu, gv), (box_u + 70, box_v), color, 1, cv2.LINE_AA)
        cv2.rectangle(vis_img, (box_u - 4, box_v - 16), (box_u + 152, box_v + 18), (255, 255, 255), -1)
        cv2.rectangle(vis_img, (box_u - 4, box_v - 16), (box_u + 152, box_v + 18), color, 2)
        cv2.putText(
            vis_img, title_txt, (box_u, box_v - 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (20, 20, 20), 1, cv2.LINE_AA
        )
        cv2.putText(
            vis_img, coord_txt, (box_u, box_v + 13),
            cv2.FONT_HERSHEY_SIMPLEX, 0.37, (60, 60, 60), 1, cv2.LINE_AA
        )
        cv2.putText(
            seg_pgm, f"Zone {reg_info['id']}", (gu - 22, gv - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, 0, 1, cv2.LINE_AA
        )

    # Vẽ vị trí Trạm sạc xuất phát (Dock 0.0, 0.0)
    du, dv = world_to_pixel(0.0, 0.0, origin, resolution, height)
    cv2.drawMarker(vis_img, (du, dv), (0, 0, 0), cv2.MARKER_DIAMOND, 12, 2)
    cv2.putText(
        vis_img, "Dock (0,0)", (du + 8, dv + 4),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 1, cv2.LINE_AA
    )

    # Vẽ bảng chú giải (Legend Header) ở góc trên bản đồ
    cv2.rectangle(vis_img, (10, 8), (width - 10, 142), (250, 250, 252), -1)
    cv2.rectangle(vis_img, (10, 8), (width - 10, 142), (40, 60, 100), 2)
    cv2.putText(
        vis_img,
        "WAREHOUSE SEMANTIC MAP SEGMENTATION - TURTLEBOT 4 VOICE NAV2",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 45, 98),
        2,
        cv2.LINE_AA,
    )
    for idx, reg_info in enumerate(regions_output):
        col = idx % 2
        row = idx // 2
        lx = 20 + col * 290
        ly = 54 + row * 24
        c_bgr = tuple(int(c) for c in reg_info['color_bgr'])
        cv2.rectangle(vis_img, (lx, ly - 11), (lx + 16, ly + 3), c_bgr, -1)
        cv2.rectangle(vis_img, (lx, ly - 11), (lx + 16, ly + 3), (30, 30, 30), 1)
        legend_str = (
            f"[{reg_info['id']}] {reg_info['name_ascii']}: "
            f"({reg_info['world_goal']['x']:.2f}, {reg_info['world_goal']['y']:.2f})m"
        )
        cv2.putText(
            vis_img, legend_str, (lx + 22, ly),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (20, 20, 20), 1, cv2.LINE_AA
        )

    cv2.putText(
        vis_img,
        f"Map: warehouse.sdf | Res: {resolution}m/px | Size: {width}x{height}px",
        (310, 102),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.36,
        (80, 80, 80),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis_img,
        "Method: Binary Thresholding + Morphology + Distance Transform + Watershed (5 Warehouse Zones)",
        (20, 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (70, 70, 70),
        1,
        cv2.LINE_AA,
    )

    os.makedirs(os.path.dirname(output_segmented_png), exist_ok=True)
    os.makedirs(os.path.dirname(output_segmented_pgm), exist_ok=True)
    cv2.imwrite(output_segmented_png, vis_img)
    cv2.imwrite(output_segmented_pgm, seg_pgm)

    # -------------------------------------------------------------------------
    # BƯỚC 6: Tạo hình minh họa quy trình 4 bước (Pipeline Figure cho Báo cáo PDF)
    # -------------------------------------------------------------------------
    if output_pipeline_fig:
        os.makedirs(os.path.dirname(output_pipeline_fig), exist_ok=True)
        crop_v1, crop_v2 = max(145, v_min_roi - 15), min(height, v_max_roi + 20)
        crop_u1, crop_u2 = max(0, u_min_roi - 20), min(width, u_max_roi + 20)

        p1 = cv2.cvtColor(img_gray[crop_v1:crop_v2, crop_u1:crop_u2], cv2.COLOR_GRAY2BGR)
        p2 = cv2.cvtColor(active_free_mask[crop_v1:crop_v2, crop_u1:crop_u2], cv2.COLOR_GRAY2BGR)
        dist_crop = dist_pixels[crop_v1:crop_v2, crop_u1:crop_u2]
        dist_crop_norm = cv2.normalize(dist_crop, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        p3 = cv2.applyColorMap(dist_crop_norm, cv2.COLORMAP_JET)
        p3[active_free_mask[crop_v1:crop_v2, crop_u1:crop_u2] == 0] = (30, 30, 30)
        p4 = vis_img[crop_v1:crop_v2, crop_u1:crop_u2].copy()

        target_h, target_w = 430, 310
        panels = []
        titles = [
            "(a) Original Map (.pgm)",
            "(b) Threshold & Morphology",
            "(c) Distance Transform",
            "(d) Watershed Warehouse Zones",
        ]
        for panel_img, title in zip([p1, p2, p3, p4], titles):
            resized = cv2.resize(panel_img, (target_w, target_h), interpolation=cv2.INTER_AREA)
            canvas = np.full((target_h + 44, target_w + 16, 3), 255, dtype=np.uint8)
            canvas[36:36 + target_h, 8:8 + target_w] = resized
            cv2.rectangle(canvas, (8, 36), (8 + target_w, 36 + target_h), (80, 80, 80), 1)
            cv2.putText(
                canvas, title, (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.49, (0, 45, 98), 2, cv2.LINE_AA
            )
            panels.append(canvas)

        pipeline_combined = np.hstack(panels)
        cv2.imwrite(output_pipeline_fig, pipeline_combined)

    # -------------------------------------------------------------------------
    # BƯỚC 7: Lưu dữ liệu các khu vực nhà kho ra JSON và YAML
    # -------------------------------------------------------------------------
    database = {
        'map_metadata': {
            'map_yaml': os.path.basename(map_yaml_path),
            'image': pgm_filename,
            'segmented_image_png': os.path.basename(output_segmented_png),
            'segmented_image_pgm': os.path.basename(output_segmented_pgm),
            'resolution': resolution,
            'origin': origin,
            'width': width,
            'height': height,
            'segmentation_method': 'OpenCV Binary Thresholding + Morphological Filtering + Distance Transform + Marker-Controlled Watershed (5 Warehouse Functional Zones)',
        },
        'regions': regions_output,
    }

    os.makedirs(os.path.dirname(output_json), exist_ok=True)
    with open(output_json, 'w', encoding='utf-8') as f_json:
        json.dump(database, f_json, ensure_ascii=False, indent=2)

    os.makedirs(os.path.dirname(output_yaml), exist_ok=True)
    with open(output_yaml, 'w', encoding='utf-8') as f_yaml:
        yaml.safe_dump(database, f_yaml, allow_unicode=True, sort_keys=False)

    return database


def main():
    parser = argparse.ArgumentParser(
        description='Phân vùng bản đồ Nhà kho 2D Occupancy Grid (.pgm) và gắn nhãn ngữ nghĩa cho TurtleBot 4.'
    )
    paths = get_default_paths()
    parser.add_argument('--map-yaml', default=paths['map_yaml'], help='Đường dẫn file map.yaml')
    parser.add_argument('--output-png', default=paths['segmented_png'], help='Đường dẫn lưu ảnh map_segmented.png')
    parser.add_argument('--output-pgm', default=paths['segmented_pgm'], help='Đường dẫn lưu ảnh map_segmented.pgm')
    parser.add_argument('--output-pipeline', default=paths['pipeline_fig'], help='Đường dẫn lưu ảnh pipeline 4 bước')
    parser.add_argument('--output-json', default=paths['regions_json'], help='Đường dẫn lưu file semantic_regions.json')
    parser.add_argument('--output-yaml', default=paths['regions_yaml'], help='Đường dẫn lưu file semantic_regions.yaml')
    parser.add_argument('--show', action='store_true', help='Hiển thị cửa sổ OpenCV trực quan')
    parser.add_argument('--verify', action='store_true', help='Kiểm tra tự động tính hợp lệ của các vùng')

    args, _ = parser.parse_known_args()

    print('========================================================================')
    print('   PHÂN VÙNG BẢN ĐỒ NHÀ KHO (WAREHOUSE SEGMENTATION) & GẮN NHÃN OPENCV  ')
    print('========================================================================')
    db = segment_occupancy_map(
        map_yaml_path=args.map_yaml,
        output_segmented_png=args.output_png,
        output_segmented_pgm=args.output_pgm,
        output_pipeline_fig=args.output_pipeline,
        output_json=args.output_json,
        output_yaml=args.output_yaml,
    )

    meta = db['map_metadata']
    print(f" * Bản đồ gốc       : {meta['image']} ({meta['width']}x{meta['height']} px, res={meta['resolution']} m/px)")
    print(f" * Origin (ROS map) : {meta['origin']}")
    print(f" * Bản đồ phân vùng : {args.output_png}")
    print(f" * File JSON ngữ nghĩa: {args.output_json}")
    print(f" * File YAML ngữ nghĩa: {args.output_yaml}")
    print('------------------------------------------------------------------------')
    print(f" {'ID':<4} | {'Tên khu vực nhà kho':<20} | {'Pixel (u, v)':<14} | {'Tọa độ ROS (x, y, yaw)':<24} | {'Diện tích':<10} | {'Clearance'}")
    print('------------------------------------------------------------------------')
    for r in db['regions']:
        pu, pv = r['pixel_goal']['u'], r['pixel_goal']['v']
        wx, wy, wyaw = r['world_goal']['x'], r['world_goal']['y'], r['world_goal']['yaw']
        print(
            f" {r['id']:<4} | {r['name']:<20} | ({pu:>4}, {pv:>4})   | "
            f"({wx:>6.2f}, {wy:>6.2f}, {wyaw:>5.2f}) | {r['area_m2']:>6.2f} m2 | {r['clearance_m']:.2f} m"
        )
    print('========================================================================')

    if args.verify:
        assert len(db['regions']) >= 3, 'Phải có tối thiểu 3 khu vực được phân vùng!'
        for r in db['regions']:
            assert r['is_valid_free_space'], f"Điểm đích khu vực {r['id']} không nằm trong vùng trống an toàn!"
            assert r['area_m2'] > 5.0, f"Diện tích khu vực {r['id']} quá nhỏ!"
        print('[VERIFY PASSED] Toàn bộ 05 khu vực nhà kho (A, B, C, D, E) đều hợp lệ và an toàn cho Nav2!')

    if args.show and os.environ.get('DISPLAY'):
        vis = cv2.imread(args.output_png)
        if vis is not None:
            cv2.imshow('Warehouse Semantic Map Segmentation - TurtleBot 4', vis)
            print('Nhấn phím bất kỳ trên cửa sổ ảnh để đóng...')
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
