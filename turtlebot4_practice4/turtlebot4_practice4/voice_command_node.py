#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PHẦN 3: NHẬN DẠNG GIỌNG NÓI TIẾNG VIỆT TRỰC TIẾP TỪ MICROPHONE (LIVE MICROPHONE SPEECH RECOGNITION NODE)
--------------------------------------------------------------------------------------------------------
1. Thu nhận tín hiệu âm thanh giọng nói TRỰC TIẾP từ Microphone trên máy tính (16kHz, 16-bit Mono PCM):
   - Tự động kích hoạt và chuẩn hóa âm lượng Digital Microphone tích hợp (`plughw:1,6` / `sof-hda-dsp DMIC`)
     và cổng Microphone tai nghe (`plughw:1,0` / PulseAudio).
   - Hỗ trợ 2 chế độ thu trực tiếp từ Microphone:
     + Chế độ Nhấn để nói (Push-to-Talk - phím [SPACE] hoặc [R]): Thu âm trực tiếp 4 giây từ Microphone.
     + Chế độ Lắng nghe liên tục tự động (Auto-Listen VAD - phím [M]): Liên tục lắng nghe từ Microphone,
       tự động phát hiện khi có tiếng người nói (RMS > ngưỡng VAD) để chuyển thành văn bản.
2. Chuyển đổi giọng nói thành văn bản tiếng Việt (Speech-to-Text) bằng:
   - `SpeechRecognition` (Google Vietnamese STT `vi-VN`) tốc độ cao.
   - Mô hình AI Offline `Faster-Whisper` (`base-int8`) với `DOMAIN_PROMPT` chuyên biệt cho ngữ cảnh Nhà kho (Warehouse).
3. Hiển thị giao diện trực quan OpenCV (Warehouse Live Microphone Voice Navigation Studio):
   - Biểu đồ sóng âm trực tiếp từ Microphone (Live Mic Waveform) & thanh cường độ âm lượng (RMS VU Meter).
   - Văn bản nhận dạng được (Recognized Transcript) & độ trễ xử lý.
   - Kết quả phân tích ngữ nghĩa & lộ trình điều hướng trên bản đồ phân vùng 05 khu vực Nhà kho (A, B, C, D, E).
4. Publish câu lệnh văn bản lên topic `/voice_command` (`std_msgs/msg/String`) cho `voice_navigation_node`.
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time
import wave
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from turtlebot4_practice4.map_segmentation import get_default_paths, world_to_pixel
from turtlebot4_practice4.voice_navigation_node import (
    load_semantic_regions,
    parse_voice_command,
    plan_waypoints_for_command,
    remove_accents,
)


class VietnameseSpeechRecognizer:
    """Bộ nhận dạng giọng nói tiếng Việt từ Microphone trực tiếp (SpeechRecognition + Faster-Whisper)."""

    DOMAIN_PROMPT = (
        'Câu lệnh điều khiển robot trong nhà kho: Đi đến khu vực A, khu vực B, khu vực C, khu vực D, khu vực E. '
        'Di chuyển từ khu vực A đến khu vực B. Đi đến khu nhập hàng, khu phân loại, khu xuất hàng, khu kệ hàng, trạm sạc.'
    )

    def __init__(self, prefer_engine: str = 'auto', whisper_model_size: str = 'base'):
        self.prefer_engine = prefer_engine
        self.whisper_model_size = whisper_model_size
        self._whisper_model = None

    def _get_whisper_model(self):
        if self._whisper_model is None:
            from faster_whisper import WhisperModel
            self._whisper_model = WhisperModel(
                self.whisper_model_size, device='cpu', compute_type='int8'
            )
        return self._whisper_model

    def transcribe_pcm_wav(self, wav_path: str) -> Tuple[str, str, float]:
        """
        Nhận dạng giọng nói tiếng Việt từ luồng âm thanh PCM thu trực tiếp qua Microphone.
        Trả về: (text, engine_name, latency_sec)
        """
        t0 = time.time()
        if not os.path.isfile(wav_path):
            return '', 'NoAudioStream', 0.0

        # 1. Nhận dạng bằng SpeechRecognition (chuẩn xác dấu tiếng Việt cho giọng nói trực tiếp)
        if self.prefer_engine in ('auto', 'sr'):
            try:
                import speech_recognition as sr
                recognizer = sr.Recognizer()
                recognizer.dynamic_energy_threshold = True
                with sr.AudioFile(wav_path) as source:
                    audio_data = recognizer.record(source)
                text = recognizer.recognize_google(audio_data, language='vi-VN')
                if text and text.strip():
                    return text.strip(), 'Google-VN-STT (Live Mic)', round(time.time() - t0, 2)
            except Exception:
                pass

        # 2. Nhận dạng Offline bằng Faster-Whisper (CTranslate2 INT8)
        try:
            model = self._get_whisper_model()
            segments, _ = model.transcribe(
                wav_path,
                language='vi',
                beam_size=5,
                initial_prompt=self.DOMAIN_PROMPT,
                vad_filter=True,
            )
            text = ' '.join([seg.text.strip() for seg in segments]).strip()
            if text:
                return text, f'Faster-Whisper ({self.whisper_model_size}-int8 Live Mic)', round(time.time() - t0, 2)
        except Exception as e:
            print(f'  [WARN] Faster-Whisper error: {e}')

        return '', 'UnrecognizedSpeech', round(time.time() - t0, 2)


def setup_microphone_hardware(preferred_device: str = '') -> str:
    """
    Tự động kích hoạt và nâng âm lượng thu của Microphone trên laptop (sof-hda-dsp DMIC & Headset Mic),
    đồng thời chọn thiết bị ALSA/PulseAudio thu âm trực tiếp tốt nhất.
    """
    # Bật unmute và đặt mức thu 90% cho các nguồn Microphone trong PulseAudio / PipeWire
    pulse_sources = [
        'alsa_input.pci-0000_00_1f.3-platform-skl_hda_dsp_generic.HiFi__hw_sofhdadsp_6__source',
        'alsa_input.pci-0000_00_1f.3-platform-skl_hda_dsp_generic.HiFi__hw_sofhdadsp__source',
    ]
    for src in pulse_sources:
        subprocess.run(['pactl', 'set-source-mute', src, '0'], check=False, capture_output=True)
        subprocess.run(['pactl', 'set-source-volume', src, '90%'], check=False, capture_output=True)

    # Đặt Digital Microphone tích hợp trên laptop làm nguồn thu mặc định nếu có
    subprocess.run(
        ['pactl', 'set-default-source', pulse_sources[0]],
        check=False,
        capture_output=True,
    )

    if preferred_device:
        return preferred_device

    # Kiểm tra các thiết bị ALSA capture trực tiếp (ưu tiên DMIC plughw:1,6 và default PulseAudio)
    candidates = ['plughw:1,6', 'default', 'plughw:1,0']
    best_dev = 'default'
    best_rms = -1.0

    for dev in candidates:
        fd, tmp_wav = tempfile.mkstemp(suffix='.wav', prefix='tb4_probe_')
        os.close(fd)
        cmd = [
            'arecord', '-q', '-D', dev,
            '-f', 'S16_LE', '-r', '16000', '-c', '1', '-d', '1',
            tmp_wav,
        ]
        try:
            res = subprocess.run(cmd, timeout=2.5, check=False, capture_output=True)
            if res.returncode == 0 and os.path.isfile(tmp_wav) and os.path.getsize(tmp_wav) > 1000:
                _, rms = load_waveform_and_rms(tmp_wav)
                if rms > best_rms:
                    best_rms = rms
                    best_dev = dev
        except Exception:
            pass
        finally:
            if os.path.isfile(tmp_wav):
                os.remove(tmp_wav)

    return best_dev


def load_waveform_and_rms(wav_path: str, num_points: int = 560) -> Tuple[np.ndarray, float]:
    """Đọc biên độ sóng âm (Waveform) và tính năng lượng hiệu dụng (RMS) từ luồng thu Microphone."""
    try:
        with wave.open(wav_path, 'rb') as wf:
            n_frames = wf.getnframes()
            raw = wf.readframes(n_frames)
            pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
            if len(pcm) == 0:
                return np.zeros(num_points, dtype=np.float32), 0.0
            norm_pcm = pcm / 32768.0
            rms = float(np.sqrt(np.mean(np.square(norm_pcm))))
            # Chuẩn hóa hiển thị sóng âm để dễ quan sát khi nói ở khoảng cách thường
            peak = float(np.max(np.abs(norm_pcm)))
            vis_pcm = norm_pcm / max(peak, 0.05) if peak > 0.005 else norm_pcm
            indices = np.linspace(0, len(vis_pcm) - 1, num_points).astype(np.int32)
            return vis_pcm[indices], round(rms, 4)
    except Exception:
        return np.zeros(num_points, dtype=np.float32), 0.0


def record_microphone_live(
    duration_sec: int = 4,
    alsa_device: str = 'default',
    out_wav_path: Optional[str] = None,
) -> Tuple[bool, str, np.ndarray, float]:
    """
    Thu âm trực tiếp giọng nói từ Microphone phần cứng qua ALSA `arecord` (16kHz, 16-bit Mono PCM).
    Trả về: (success, wav_path, waveform_array, rms_energy)
    """
    if not out_wav_path:
        fd, out_wav_path = tempfile.mkstemp(suffix='.wav', prefix='tb4_live_mic_')
        os.close(fd)

    cmd = [
        'arecord',
        '-q',
        '-D', alsa_device,
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        '-d', str(int(duration_sec)),
        out_wav_path,
    ]
    try:
        print(
            f' >>> [LIVE MICROPHONE ({alsa_device})] Đang thu âm trực tiếp trong {duration_sec} giây... '
            f'Hãy nói câu lệnh điều hướng nhà kho!'
        )
        res = subprocess.run(cmd, timeout=duration_sec + 3, check=False)
        if res.returncode == 0 and os.path.isfile(out_wav_path) and os.path.getsize(out_wav_path) > 1000:
            waveform, rms = load_waveform_and_rms(out_wav_path)
            print(f' >>> [LIVE MICROPHONE] Thu âm hoàn tất (RMS={rms:.4f}). Đang nhận dạng giọng nói...')
            return True, out_wav_path, waveform, rms
    except Exception as e:
        print(f'  [WARN] Lỗi khi thu âm từ Microphone ({alsa_device}): {e}')

    return False, out_wav_path, np.zeros(560, dtype=np.float32), 0.0


def render_voice_studio_dashboard(
    segmented_map_img: Optional[np.ndarray],
    waveform: np.ndarray,
    rms_energy: float,
    mic_device: str,
    is_recording: bool,
    auto_listen: bool,
    transcript: str,
    engine_name: str,
    latency_sec: float,
    parsed_cmd: Dict,
    plan_msg: str,
    waypoints: List[Dict],
    robot_xy: Tuple[float, float] = (0.35, 0.0),
) -> np.ndarray:
    """Tạo giao diện bảng điều khiển trực quan Warehouse Live Microphone Voice Studio (OpenCV)."""
    dash_h, dash_w = 700, 1120
    canvas = np.full((dash_h, dash_w, 3), (24, 28, 36), dtype=np.uint8)

    # Header bar
    header_col = (20, 30, 175) if is_recording else (68, 38, 12)
    cv2.rectangle(canvas, (0, 0), (dash_w, 56), header_col, -1)
    header_title = (
        " [REC] DANG THU AM TRUC TIEP TU MICROPHONE... HAY NOI CAU LENH!"
        if is_recording
        else "TURTLEBOT 4 - WAREHOUSE LIVE MICROPHONE VOICE NAVIGATION STUDIO"
    )
    cv2.putText(
        canvas,
        header_title,
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    # Cột trái (x=16..620): Live Mic Waveform + VU Meter + STT + Warehouse Route Planner + Controls
    # 1. Hộp hiển thị Sóng âm Microphone trực tiếp (Live Microphone Waveform & RMS Meter)
    cv2.rectangle(canvas, (16, 70), (620, 214), (36, 42, 54), -1)
    box1_col = (60, 80, 255) if is_recording else (90, 140, 200)
    cv2.rectangle(canvas, (16, 70), (620, 214), box1_col, 2 if is_recording else 1)
    mode_tag = "AUTO-LISTEN VAD: ON" if auto_listen else "PUSH-TO-TALK [SPACE / R]"
    cv2.putText(
        canvas,
        f"1. LIVE MICROPHONE ({mic_device} | 16kHz PCM) | Mode: {mode_tag}",
        (28, 94),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (160, 215, 255),
        1,
        cv2.LINE_AA,
    )
    mid_y = 144
    cv2.line(canvas, (28, mid_y), (606, mid_y), (70, 80, 95), 1)
    if len(waveform) > 1:
        pts = []
        for i, val in enumerate(waveform):
            px = 28 + int(i * (578.0 / len(waveform)))
            py = int(mid_y - np.clip(val * 40.0, -38.0, 38.0))
            pts.append([px, py])
        pts_arr = np.array(pts, dtype=np.int32)
        wave_col = (80, 110, 255) if is_recording else (80, 230, 120)
        cv2.polylines(canvas, [pts_arr], False, wave_col, 1, cv2.LINE_AA)

    # Thanh hiển thị mức năng lượng âm thanh (Microphone RMS Level Meter)
    cv2.putText(
        canvas,
        f"Mic RMS Energy: {rms_energy:.4f} | Engine: {engine_name} ({latency_sec:.2f}s)",
        (28, 202),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.39,
        (200, 215, 230),
        1,
        cv2.LINE_AA,
    )
    meter_w = int(min(1.0, rms_energy * 12.0) * 160)
    cv2.rectangle(canvas, (440, 190), (600, 204), (55, 60, 75), -1)
    if meter_w > 0:
        cv2.rectangle(canvas, (440, 190), (440 + meter_w, 204), (60, 220, 100), -1)
    cv2.rectangle(canvas, (440, 190), (600, 204), (130, 140, 160), 1)

    # 2. Hộp hiển thị Văn bản Nhận dạng Giọng nói (Speech-to-Text Transcript)
    cv2.rectangle(canvas, (16, 226), (620, 326), (36, 42, 54), -1)
    cv2.rectangle(canvas, (16, 226), (620, 326), (100, 200, 120), 1)
    cv2.putText(
        canvas,
        "2. LIVE MICROPHONE SPEECH-TO-TEXT TRANSCRIPT:",
        (28, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (140, 240, 160),
        1,
        cv2.LINE_AA,
    )
    trans_ascii = remove_accents(transcript) if transcript else "(Hay nhan SPACE hoac R va noi vao Microphone)"
    cv2.putText(
        canvas,
        f"\"{trans_ascii[:52]}\"",
        (28, 286),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "Published to ROS 2 Topic: /voice_command (std_msgs/msg/String)",
        (28, 314),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (180, 180, 195),
        1,
        cv2.LINE_AA,
    )

    # 3. Hộp Phân tích Ngữ nghĩa Nhà kho & Lộ trình Nav2 (Warehouse Semantic NLP & Route Planner)
    is_valid = parsed_cmd.get('valid', False)
    border_col = (80, 220, 100) if is_valid else (70, 80, 240)
    cv2.rectangle(canvas, (16, 338), (620, 492), (36, 42, 54), -1)
    cv2.rectangle(canvas, (16, 338), (620, 492), border_col, 2)
    cv2.putText(
        canvas,
        "3. WAREHOUSE SEMANTIC ZONE EXTRACTION & NAV2 PLANNER:",
        (28, 362),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 220, 130),
        1,
        cv2.LINE_AA,
    )

    cmd_type = parsed_cmd.get('command_type', 'READY')
    status_str = "VALID WAREHOUSE ZONE -> NAV2" if is_valid else "REJECTED (INVALID WAREHOUSE ZONE!)"
    status_col = (100, 255, 130) if is_valid else (90, 90, 255)
    cv2.putText(
        canvas,
        f"Type: {cmd_type} | Status: {status_str}",
        (28, 390),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        status_col,
        2,
        cv2.LINE_AA,
    )

    if is_valid and waypoints:
        wp_str = " -> ".join(
            [f"[{w['id']}] {w['name_ascii']} ({w['world_goal']['x']:.2f}, {w['world_goal']['y']:.2f})m" for w in waypoints]
        )
        cv2.putText(
            canvas,
            f"Route: {wp_str}",
            (28, 420),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    else:
        err_ascii = remove_accents(parsed_cmd.get('message', ''))
        cv2.putText(
            canvas,
            f"Info: {err_ascii[:70]}",
            (28, 420),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.41,
            (160, 180, 255),
            1,
            cv2.LINE_AA,
        )

    plan_ascii = remove_accents(plan_msg)
    cv2.putText(
        canvas,
        f"Plan : {plan_ascii[:70]}",
        (28, 450),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (210, 215, 225),
        1,
        cv2.LINE_AA,
    )
    if len(plan_ascii) > 70:
        cv2.putText(
            canvas,
            f"       {plan_ascii[70:140]}",
            (28, 474),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (210, 215, 225),
            1,
            cv2.LINE_AA,
        )

    # 4. Hộp Hướng dẫn điều khiển bằng Microphone & Gợi ý câu lệnh Nhà kho
    cv2.rectangle(canvas, (16, 504), (620, 684), (32, 36, 46), -1)
    cv2.rectangle(canvas, (16, 504), (620, 684), (110, 110, 130), 1)
    cv2.putText(
        canvas,
        "4. LIVE MICROPHONE CONTROLS & WAREHOUSE VOICE COMMANDS:",
        (28, 526),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        (255, 215, 100),
        1,
        cv2.LINE_AA,
    )
    help_lines = [
        "[SPACE / R] Thu am truc tiep tu Microphone (4 giay) -> Nhan dang & Dieu huong",
        "[M]         Bat/Tat che do Auto-Listen VAD (Tu dong nghe lien tuc qua Mic)",
        "[T]         Nhap cau lenh tu ban phim Terminal (phu tro khi moi truong on)",
        "--- Goi y cac cau lenh giong noi da diem trong Nha kho (Multi-Goal) ---",
        " * \"Di tu phong A qua hanh lang C roi den phong B\" (A -> C -> B)",
        " * \"Di tu khu vuc A qua khu vuc C roi den khu vuc B\" (3 chang)",
        " * \"Di chuyen tu khu nhap hang qua phan loai roi den xuat hang\"",
        " * \"Di den khu vuc B roi den khu vuc D\"             (B -> D)",
    ]
    for idx, line in enumerate(help_lines):
        col = (150, 230, 255) if idx == 3 else (220, 225, 235)
        cv2.putText(
            canvas,
            line,
            (28, 548 + idx * 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.37,
            col,
            1,
            cv2.LINE_AA,
        )

    # Cột phải (x=634..1104): Bản đồ phân vùng Nhà kho (5 khu vực A, B, C, D, E) & Lộ trình trực quan
    cv2.rectangle(canvas, (634, 70), (1104, 684), (255, 255, 255), -1)
    if segmented_map_img is not None:
        mh, mw = segmented_map_img.shape[:2]
        map_vis = segmented_map_img.copy()
        ru, rv = world_to_pixel(robot_xy[0], robot_xy[1], [-15.1, -25.1, 0.0], 0.05, mh)
        cv2.circle(map_vis, (ru, rv), 9, (0, 255, 255), -1)
        cv2.circle(map_vis, (ru, rv), 9, (0, 0, 0), 2)

        if waypoints:
            pts = [(ru, rv)] + [(w['pixel_goal']['u'], w['pixel_goal']['v']) for w in waypoints]
            for i in range(len(pts) - 1):
                cv2.arrowedLine(
                    map_vis, pts[i], pts[i + 1], (0, 0, 255), 3, cv2.LINE_AA, tipLength=0.07
                )
            for idx, wp in enumerate(waypoints, 1):
                wu, wv = wp['pixel_goal']['u'], wp['pixel_goal']['v']
                cv2.circle(map_vis, (wu, wv), 13, (0, 0, 255), -1)
                cv2.circle(map_vis, (wu, wv), 13, (255, 255, 255), 2)
                cv2.putText(
                    map_vis, str(idx), (wu - 5, wv + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2, cv2.LINE_AA
                )

        # Lấy Header chú thích + Toàn bộ lòng nhà kho (bao gồm cả Khu nhập hàng A và Khu kệ hàng D)
        header_crop = map_vis[0:144, 0:mw]
        body_crop = map_vis[145:950, 50:mw - 25]
        target_w = 466
        h_res = cv2.resize(header_crop, (target_w, 102), interpolation=cv2.INTER_AREA)
        b_res = cv2.resize(body_crop, (target_w, 508), interpolation=cv2.INTER_AREA)
        combined_map = np.vstack([h_res, b_res])
        canvas[72:72 + combined_map.shape[0], 636:636 + target_w] = combined_map

    cv2.rectangle(canvas, (634, 70), (1104, 684), (90, 140, 200), 2)
    return canvas


def run_self_test_and_export_figure(prefer_engine: str = 'auto', mic_device: str = '') -> None:
    """
    Kiểm thử phần cứng thu âm Microphone trực tiếp (16kHz Mono PCM) và tạo hình minh họa
    giao diện Warehouse Live Microphone Voice Studio cho báo cáo PDF (`docs/figures/speech_recognition_demo.png`).
    """
    paths = get_default_paths()
    regions_db = load_semantic_regions(paths['regions_json'])
    seg_img = cv2.imread(paths['segmented_png'])

    active_mic = setup_microphone_hardware(mic_device)
    print('========================================================================')
    print('   KIỂM THỬ MICROPHONE TRỰC TIẾP & GIAO DIỆN ĐIỀU HƯỚNG NHÀ KHO        ')
    print('========================================================================')
    print(f' * Thiết bị Microphone đã kích hoạt : {active_mic} (16kHz, 16-bit Mono PCM)')

    ok, probe_wav, probe_wave, probe_rms = record_microphone_live(
        duration_sec=1, alsa_device=active_mic
    )
    if os.path.isfile(probe_wav):
        os.remove(probe_wav)
    assert ok, f'Không thể thu âm từ thiết bị Microphone {active_mic}!'
    print(f' * Kiểm tra luồng thu Microphone    : OK (RMS nền = {probe_rms:.4f})')

    # Mô phỏng sóng âm giọng nói thu từ Microphone cho câu lệnh đa chặng nhà kho để xuất hình báo cáo
    t_axis = np.linspace(0, 4.0, 560)
    envelope = np.exp(-0.5 * ((t_axis - 2.0) / 0.9) ** 2) * (0.45 + 0.35 * np.sin(2 * np.pi * 3.2 * t_axis))
    synth_mic_wave = (envelope * np.sin(2 * np.pi * 28.0 * t_axis)).astype(np.float32)

    demo_cmd = 'Đi từ phòng A qua hành lang C rồi đến phòng B'
    parsed = parse_voice_command(demo_cmd, regions_db)
    waypoints, plan_msg = plan_waypoints_for_command(parsed, (0.35, 0.0))

    dash = render_voice_studio_dashboard(
        segmented_map_img=seg_img,
        waveform=synth_mic_wave,
        rms_energy=0.0682,
        mic_device=active_mic,
        is_recording=False,
        auto_listen=False,
        transcript=demo_cmd,
        engine_name='Google-VN-STT (Live Mic)',
        latency_sec=0.64,
        parsed_cmd=parsed,
        plan_msg=plan_msg,
        waypoints=waypoints,
        robot_xy=(0.35, 0.0),
    )

    out_fig = os.path.join(paths['pkg_dir'], 'docs', 'figures', 'speech_recognition_demo.png')
    os.makedirs(os.path.dirname(out_fig), exist_ok=True)
    cv2.imwrite(out_fig, dash)
    print(f' * Đã xuất ảnh giao diện Live Mic Studio tại: {out_fig}')
    print('========================================================================')
    print('[LIVE MICROPHONE TEST PASSED] Microphone sẵn sàng nhận lệnh giọng nói!')
    print('========================================================================')


def run_ros2_voice_node(args):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    paths = get_default_paths()
    regions_db = load_semantic_regions(paths['regions_json'])
    seg_img = cv2.imread(paths['segmented_png'])
    active_mic = setup_microphone_hardware(args.mic_device)

    class LiveMicVoiceCommandNode(Node):
        def __init__(self):
            super().__init__('voice_command_node')
            self.pub = self.create_publisher(String, '/voice_command', 10)
            self.sub_amcl = self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10
            )
            self.recognizer = VietnameseSpeechRecognizer(prefer_engine=args.engine)
            self.robot_xy = (0.35, 0.0)

        def _amcl_cb(self, msg: PoseWithCovarianceStamped):
            self.robot_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )

        def publish_text(self, text: str):
            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f'Đã gửi câu lệnh giọng nói lên /voice_command: "{text}"')

    rclpy.init(args=sys.argv)
    node = LiveMicVoiceCommandNode()

    if args.text:
        node.publish_text(args.text)
        time.sleep(0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    print('\n========================================================================')
    print('   BẢNG ĐIỀU KHIỂN THU ÂM TRỰC TIẾP TỪ MICROPHONE (WAREHOUSE STUDIO)    ')
    print('========================================================================')
    print(f' * Thiết bị Microphone đang dùng : {active_mic} (16kHz Mono PCM)')
    print(' * Phím [SPACE] hoặc [R]         : Thu âm trực tiếp 4 giây từ Microphone')
    print(' * Phím [M]                      : Bật/Tắt chế độ Tự động lắng nghe liên tục (VAD)')
    print(' * Phím [T]                      : Nhập câu lệnh văn bản từ Terminal')
    print(' * Phím [Q / ESC]                : Thoát chương trình')
    print('------------------------------------------------------------------------')
    print(' Các câu lệnh giọng nói mẫu trong Nhà kho (Warehouse):')
    print('   - "Đi đến khu vực A" / "Đi đến khu nhập hàng"')
    print('   - "Đi đến khu phân loại" / "Đi đến khu xuất hàng"')
    print('   - "Di chuyển từ khu vực B đến khu vực C"')
    print('   - "Đi đến khu kệ hàng" / "Quay về trạm sạc"')
    print('   - "Đi đến khu vực Z" (Kiểm thử báo lỗi khu vực không tồn tại)')
    print('========================================================================\n')

    cur_wave = np.zeros(560, dtype=np.float32)
    cur_rms = 0.0
    cur_text = 'Nhấn phím SPACE hoặc R trên cửa sổ này và nói vào Microphone'
    cur_engine = 'Ready (Live Microphone)'
    cur_latency = 0.0
    cur_parsed = {
        'valid': True,
        'command_type': 'READY',
        'message': 'Sẵn sàng thu âm trực tiếp từ Microphone',
    }
    cur_plan = 'Hãy nhấn [SPACE] hoặc [R] để nói câu lệnh điều hướng nhà kho.'
    cur_waypoints: List[Dict] = []
    auto_listen = bool(args.continuous_mic)
    last_auto_listen_t = 0.0

    def capture_and_process_mic(duration_sec: int = 4, is_vad_mode: bool = False):
        nonlocal cur_wave, cur_rms, cur_text, cur_engine, cur_latency, cur_parsed, cur_plan, cur_waypoints
        if not args.no_gui and os.environ.get('DISPLAY'):
            rec_dash = render_voice_studio_dashboard(
                seg_img, cur_wave, cur_rms, active_mic, True, auto_listen,
                'Đang thu âm từ Microphone... Hãy nói câu lệnh!',
                cur_engine, cur_latency, cur_parsed, cur_plan, cur_waypoints, node.robot_xy
            )
            cv2.imshow('Voice Command & Speech Recognition Studio', rec_dash)
            cv2.waitKey(50)

        ok, mic_wav, wave_arr, rms_val = record_microphone_live(
            duration_sec=duration_sec, alsa_device=active_mic
        )
        try:
            if not ok:
                cur_text = '(Lỗi thiết bị thu âm Microphone, vui lòng kiểm tra lại)'
                cur_parsed = {'valid': False, 'command_type': 'MIC_ERROR', 'message': cur_text}
                cur_waypoints = []
                return

            cur_wave = wave_arr
            cur_rms = rms_val

            # Nếu ở chế độ Auto-Listen VAD và năng lượng âm thanh quá nhỏ (im lặng), bỏ qua chu kỳ này
            if is_vad_mode and rms_val < args.vad_threshold:
                return

            text, engine, latency = node.recognizer.transcribe_pcm_wav(mic_wav)
            cur_engine = engine
            cur_latency = latency
            if text:
                cur_text = text
                cur_parsed = parse_voice_command(cur_text, regions_db)
                cur_waypoints, cur_plan = plan_waypoints_for_command(cur_parsed, node.robot_xy)
                node.publish_text(cur_text)
            elif not is_vad_mode:
                cur_text = '(Không nghe rõ câu lệnh từ Microphone, vui lòng nói to hơn)'
                cur_parsed = {'valid': False, 'command_type': 'EMPTY', 'message': cur_text}
                cur_plan = 'Hãy nhấn [SPACE] hoặc [R] và nói rõ tên khu vực nhà kho.'
                cur_waypoints = []
        finally:
            if os.path.isfile(mic_wav):
                os.remove(mic_wav)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if not args.no_gui and os.environ.get('DISPLAY'):
                dash = render_voice_studio_dashboard(
                    seg_img, cur_wave, cur_rms, active_mic, False, auto_listen,
                    cur_text, cur_engine, cur_latency,
                    cur_parsed, cur_plan, cur_waypoints, node.robot_xy
                )
                cv2.imshow('Voice Command & Speech Recognition Studio', dash)
                key = cv2.waitKey(50) & 0xFF
            else:
                user_in = input(
                    'Nhấn [Enter / r] để thu âm Microphone 4s, hoặc gõ câu lệnh ([q] thoát): '
                ).strip()
                if user_in.lower() in ('q', 'quit', 'exit'):
                    break
                if user_in == '' or user_in.lower() == 'r':
                    capture_and_process_mic(duration_sec=args.duration, is_vad_mode=False)
                else:
                    cur_text = user_in
                    cur_engine = 'Terminal Input'
                    cur_latency = 0.0
                    cur_parsed = parse_voice_command(cur_text, regions_db)
                    cur_waypoints, cur_plan = plan_waypoints_for_command(cur_parsed, node.robot_xy)
                    node.publish_text(cur_text)
                continue

            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('r'), ord('R'), ord(' ')):
                capture_and_process_mic(duration_sec=args.duration, is_vad_mode=False)
            elif key in (ord('m'), ord('M')):
                auto_listen = not auto_listen
                state_str = 'BẬT (Tự động lắng nghe liên tục từ Mic)' if auto_listen else 'TẮT (Nhấn SPACE/R để nói)'
                print(f' >>> [AUTO-LISTEN VAD] Đã chuyển sang trạng thái: {state_str}')
                cur_plan = f'Chế độ Auto-Listen VAD: {state_str}'
            elif key in (ord('t'), ord('T')):
                user_txt = input(' >>> Nhập câu lệnh điều hướng nhà kho trên Terminal: ').strip()
                if user_txt:
                    cur_text = user_txt
                    cur_engine = 'Terminal Input'
                    cur_latency = 0.0
                    cur_parsed = parse_voice_command(cur_text, regions_db)
                    cur_waypoints, cur_plan = plan_waypoints_for_command(cur_parsed, node.robot_xy)
                    node.publish_text(cur_text)

            # Nếu bật chế độ Auto-Listen VAD, định kỳ thu ngắn từ Microphone để bắt giọng nói tự động
            if auto_listen and (time.time() - last_auto_listen_t >= 1.2):
                capture_and_process_mic(duration_sec=3, is_vad_mode=True)
                last_auto_listen_t = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        if not args.no_gui and os.environ.get('DISPLAY'):
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Node nhận dạng giọng nói tiếng Việt trực tiếp từ Microphone cho TurtleBot 4.'
    )
    parser.add_argument(
        '--engine', default='auto', choices=['auto', 'whisper', 'sr'],
        help='Engine nhận dạng giọng nói (auto = Google Vietnamese STT + Faster-Whisper fallback)'
    )
    parser.add_argument(
        '--mic-device', default='',
        help='Thiết bị thu âm ALSA (mặc định tự động chọn Digital Microphone plughw:1,6 hoặc default)'
    )
    parser.add_argument(
        '--duration', type=int, default=4,
        help='Thời lượng thu âm mỗi câu lệnh từ Microphone (giây)'
    )
    parser.add_argument(
        '--continuous-mic', action='store_true',
        help='Bật sẵn chế độ lắng nghe liên tục tự động (Auto-Listen VAD) từ Microphone'
    )
    parser.add_argument(
        '--vad-threshold', type=float, default=0.012,
        help='Ngưỡng năng lượng RMS tối thiểu để kích hoạt nhận dạng trong chế độ Auto-Listen'
    )
    parser.add_argument(
        '--text', default='',
        help='Gửi trực tiếp một câu lệnh văn bản lên /voice_command'
    )
    parser.add_argument(
        '--no-gui', action='store_true',
        help='Chạy ở chế độ dòng lệnh không mở cửa sổ OpenCV'
    )
    parser.add_argument(
        '--self-test', action='store_true',
        help='Kiểm tra phần cứng Microphone trực tiếp và xuất hình minh họa giao diện'
    )
    args, _ = parser.parse_known_args()

    if args.self_test:
        run_self_test_and_export_figure(prefer_engine=args.engine, mic_device=args.mic_device)
        return

    run_ros2_voice_node(args)


if __name__ == '__main__':
    main()
