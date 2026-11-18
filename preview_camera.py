#!/usr/bin/env python3
"""
preview_camera.py
=================
Live visual camera preview window for ESP32-S3 Robot Camera.
Connects over USB Serial, receives live frames, and renders a live HUD window:
  - Live OV2640 camera feed (320x240 scaled to 960x720)
  - Auto Un-Mirroring: un-mirrors the OV2640 sensor readout so ArUco bit patterns match
  - Multi-Dictionary Detection: supports ArUco (DICT_4X4_50) and AprilTag (tag16h5)
  - Interactive Hotkeys:
      [f] Toggle horizontal flip on/off
      [q] or [ESC] Quit preview
"""

import sys
import time
import math
import struct
import serial
import serial.tools.list_ports
import cv2
import numpy as np

# ── Defaults ─────────────────────────────────────────────────────────────────
DEFAULT_PORT = "COM10"
DEFAULT_BAUD = 115200
SYNC_HEADER  = b"\xff\xaa\x55\x00"
WINDOW_NAME  = "ESP32-S3 Live Robot Camera View"

# Calibration Constants
MARKER_REAL_WIDTH_CM = 10.0   # Printed 10x10 cm marker
STOP_DISTANCE_CM     = 25.0   # Stop distance threshold
FOCAL_SCALE          = 0.85   # OV2640 lens factor (272 px on 320w)
CENTER_TOLERANCE_DEG = 5

# ── Auto-Detect Port Helper ──────────────────────────────────────────────────
def find_esp32_port(preferred="COM10"):
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.device.upper() == preferred.upper():
            return p.device
    for p in ports:
        desc = p.description.lower()
        if "ch34" in desc or "cp210" in desc or "usb-serial" in desc or "jtag" in desc:
            return p.device
    if ports:
        return ports[-1].device
    return preferred

# ── Radar Overlay Helper ─────────────────────────────────────────────────────
def draw_radar_overlay(img, origin_x, origin_y, scale):
    h, w = img.shape[:2]
    top_pad = int(50 * scale)

    # 1. Horizontal baseline & vertical center ray
    cv2.line(img, (0, origin_y), (w, origin_y), (0, 0, 200), 1)
    cv2.line(img, (origin_x, top_pad), (origin_x, origin_y), (255, 255, 0), 1)

    # 2. Straight-ahead corridor (85° to 95°)
    ray_len = origin_y - top_pad
    for offset in (-CENTER_TOLERANCE_DEG, CENTER_TOLERANCE_DEG):
        rad = math.radians(90 + offset)
        ex = int(origin_x + ray_len * math.cos(rad))
        ey = int(origin_y - ray_len * math.sin(rad))
        cv2.line(img, (origin_x, origin_y), (ex, ey), (0, 180, 0), 1)

    # 3. Semicircular protractor arc
    arc_radius = min(w // 2 - int(40 * scale), origin_y - top_pad - int(20 * scale))
    if arc_radius > 30:
        cv2.ellipse(img, (origin_x, origin_y), (arc_radius, arc_radius), 0, 180, 360, (80, 80, 80), 1)
        for deg in (0, 45, 90, 135, 180):
            rad = math.radians(deg)
            tx_in  = int(origin_x + (arc_radius - 8 * scale) * math.cos(rad))
            ty_in  = int(origin_y - (arc_radius - 8 * scale) * math.sin(rad))
            tx_out = int(origin_x + (arc_radius + 8 * scale) * math.cos(rad))
            ty_out = int(origin_y - (arc_radius + 8 * scale) * math.sin(rad))
            color = (255, 255, 0) if deg == 90 else (180, 180, 180)
            cv2.line(img, (tx_in, ty_in), (tx_out, ty_out), color, 1)
            lx = int(origin_x + (arc_radius + 20 * scale) * math.cos(rad)) - int(10 * scale)
            ly = int(origin_y - (arc_radius + 20 * scale) * math.sin(rad)) + int(4 * scale)
            cv2.putText(img, f"{deg}°", (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.35 * scale, color, 1)

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_esp32_port(DEFAULT_PORT)
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    print(f"Connecting to ESP32 on {port} @ {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"[ERROR] Could not open port {port}: {e}")
        print("Please check that your ESP32 is plugged in and Arduino Serial Monitor is closed.")
        return

    ser.reset_input_buffer()
    buf = bytearray()

    # Detectors for ArUco 4x4 and AprilTag 16h5
    d_params = cv2.aruco.DetectorParameters()
    d_params.adaptiveThreshWinSizeMin = 3
    d_params.adaptiveThreshWinSizeMax = 55
    d_params.adaptiveThreshWinSizeStep = 4
    d_params.minMarkerPerimeterRate = 0.02

    dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det_aruco  = cv2.aruco.ArucoDetector(dict_aruco, d_params)

    dict_april = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    det_april  = cv2.aruco.ArucoDetector(dict_april, d_params)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 960, 720)

    print("\nVisual Preview Window ACTIVE!")
    print("  - [f] key: Toggle horizontal flip (un-mirroring)")
    print("  - [q] or [ESC] key: Exit preview window\n")

    flip_horizontal = False  # CameraVision.cpp already applies hardware hmirror(1); press [f] to toggle

    # ── Live ESP32 Brain Telemetry State (parsed from serial logs) ───────────
    robot_state  = "WAITING FOR ESP32..."
    robot_action = "Connecting..."
    robot_motors = "L:0 R:0"
    esp32_vision = "Waiting for ESP32 vision log..."

    def parse_esp32_line(line: str):
        nonlocal robot_state, robot_action, robot_motors, esp32_vision
        print(f"[ESP32] {line}")
        if "[FSM:DECISION]" in line:
            # Format: [FSM:DECISION] STATE:... | ACTION:... | L:... R:...
            parts = line.split("|")
            for p in parts:
                p = p.strip()
                if "STATE:" in p:
                    robot_state = p.split("STATE:", 1)[1].strip()
                elif "ACTION:" in p:
                    robot_action = p.split("ACTION:", 1)[1].strip()
                elif p.startswith("L:"):
                    robot_motors = p
        elif "[CAM:DISCOVER]" in line or "[VISION]" in line or "[VISION:DIAG]" in line:
            esp32_vision = line[:78]

    fps_count = 0
    fps_time = time.time()
    fps_display = 0.0

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("f"):
            flip_horizontal = not flip_horizontal
            print(f"[PREVIEW] Horizontal flip set to: {flip_horizontal}")

        waiting = ser.in_waiting
        if waiting > 0:
            incoming = ser.read(waiting)
            buf.extend(incoming)
        else:
            time.sleep(0.005)
            continue

        latest_jpg = None

        while True:
            idx = buf.find(SYNC_HEADER)
            if idx == -1:
                if len(buf) > 4:
                    text_chunk = buf[:-4].decode("utf-8", errors="ignore")
                    for line in text_chunk.splitlines():
                        line = line.strip()
                        if line:
                            parse_esp32_line(line)
                    buf = buf[-4:]
                break

            if idx > 0:
                text_chunk = buf[:idx].decode("utf-8", errors="ignore")
                for line in text_chunk.splitlines():
                    line = line.strip()
                    if line:
                        parse_esp32_line(line)
                del buf[:idx]
                idx = 0

            if len(buf) < 8:
                break

            frame_len = struct.unpack("<I", buf[4:8])[0]
            if frame_len == 0 or frame_len > 300000:
                del buf[:4]
                continue

            if len(buf) < 8 + frame_len:
                break

            latest_jpg = bytes(buf[8 : 8 + frame_len])
            del buf[: 8 + frame_len]

        if latest_jpg is None:
            continue

        raw_frame = cv2.imdecode(np.frombuffer(latest_jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if raw_frame is None:
            continue

        # Un-mirror sensor horizontal flip
        if flip_horizontal:
            frame = cv2.flip(raw_frame, 1)
        else:
            frame = raw_frame

        fps_count += 1
        now = time.time()
        if now - fps_time >= 1.0:
            fps_display = fps_count / (now - fps_time)
            fps_count = 0
            fps_time = now

        h, w = frame.shape[:2]
        scale = 1.5
        display_frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        dh, dw = display_frame.shape[:2]

        origin_x = dw // 2
        origin_y = dh - int(20 * scale)
        focal_px = (w * scale) * FOCAL_SCALE

        # STEP 1: DETECT ON CLEAN FRAME BEFORE DRAWING OVERLAYS
        gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)

        all_corners = []
        all_ids = []
        all_types = []

        # 1. Try ArUco 4x4 (from aruco_a4_id0.png)
        c_ar, id_ar, _ = det_aruco.detectMarkers(gray)
        if id_ar is not None:
            for c, mid in zip(c_ar, id_ar.flatten()):
                all_corners.append(c)
                all_ids.append(mid)
                all_types.append("ArUco")

        # 2. Try AprilTag 16h5
        c_ap, id_ap, _ = det_april.detectMarkers(gray)
        if id_ap is not None:
            for c, mid in zip(c_ap, id_ap.flatten()):
                all_corners.append(c)
                all_ids.append(mid)
                all_types.append("AprilTag")

        # STEP 2: DRAW RADAR OVERLAY
        draw_radar_overlay(display_frame, origin_x, origin_y, scale)

        mean_brightness = np.mean(gray)
        min_b = np.min(gray)
        max_b = np.max(gray)

        if len(all_ids) > 0:
            best_idx = -1
            best_dist = 9999.0

            for i in range(len(all_ids)):
                pts = all_corners[i][0]
                top_w = np.linalg.norm(pts[1] - pts[0])
                bot_w = np.linalg.norm(pts[2] - pts[3])
                apparent_w_px = (top_w + bot_w) / 2.0
                dist_cm = (MARKER_REAL_WIDTH_CM * focal_px) / max(apparent_w_px, 1.0)

                if dist_cm < best_dist:
                    best_dist = dist_cm
                    best_idx = i

                # Draw green polygon around each marker
                int_pts = np.int32(pts).reshape((-1, 1, 2))
                cv2.polylines(display_frame, [int_pts], True, (0, 255, 0), 3)
                for pt in pts:
                    cv2.circle(display_frame, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                cv2.putText(
                    display_frame,
                    f"{all_types[i]} ID:{all_ids[i]}",
                    (cx - 40, cy - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

            # Highlight nearest target
            pts_best = all_corners[best_idx][0]
            cx_best = int(np.mean(pts_best[:, 0]))
            cy_best = int(np.mean(pts_best[:, 1]))
            top_w = np.linalg.norm(pts_best[1] - pts_best[0])
            bot_w = np.linalg.norm(pts_best[2] - pts_best[3])
            app_w = (top_w + bot_w) / 2.0
            dist_cm = (MARKER_REAL_WIDTH_CM * focal_px) / max(app_w, 1.0)

            # True optical angle relative to optical axis (matches ESP32 CameraVision)
            err_x = cx_best - origin_x
            angle_deg = math.degrees(math.atan2(err_x, focal_px))
            polar_angle = int(np.clip(90.0 - angle_deg, 0, 180))

            if dist_cm <= STOP_DISTANCE_CM:
                status_text = f"PC VIEW: WITHIN 25cm ({dist_cm:.1f} cm)"
                status_color = (0, 255, 0)
            else:
                status_text = f"PC VIEW SEES: {all_types[best_idx]} ID:{all_ids[best_idx]} | {dist_cm:.1f} cm | {polar_angle}°"
                status_color = (0, 255, 255)

            cv2.circle(display_frame, (cx_best, cy_best), 6, (0, 0, 255), -1)
            cv2.line(display_frame, (origin_x, origin_y), (cx_best, cy_best), status_color, 2)

            cv2.rectangle(display_frame, (10, 10), (dw - 10, 70), (20, 20, 20), -1)
            cv2.putText(
                display_frame,
                f"TARGET: {all_types[best_idx]} ID:{all_ids[best_idx]} | Dist:{dist_cm:.1f} cm | Angle:{polar_angle}° | Width:{app_w:.0f}px",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (220, 220, 220),
                1,
            )
            cv2.putText(
                display_frame,
                status_text,
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                status_color,
                2,
            )
        else:
            flip_status = "Un-mirrored [f to toggle]" if flip_horizontal else "Raw Sensor [f to toggle]"
            cv2.rectangle(display_frame, (10, 10), (dw - 10, 50), (20, 20, 20), -1)
            cv2.putText(
                display_frame,
                f"SEARCHING FOR MARKER... ({flip_status})",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                1,
            )

        cv2.putText(
            display_frame,
            f"FPS: {fps_display:.1f} | Brightness: {mean_brightness:.0f}",
            (dw - 240, dh - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (180, 180, 180),
            1,
        )

        # ── Append Live ESP32 Decision-Making Panel Below Camera Feed ────────
        panel_h = 95
        panel = np.full((panel_h, dw, 3), 24, dtype=np.uint8)
        cv2.line(panel, (0, 0), (dw, 0), (80, 80, 80), 2)

        # Color-code current FSM state
        if "APPROACH" in robot_state:
            st_color = (0, 255, 255)   # Yellow
        elif "REACHED" in robot_state:
            st_color = (0, 255, 0)     # Green
        elif "CLEARANCE" in robot_state:
            st_color = (255, 165, 0)   # Blue-ish / Cyan
        else:
            st_color = (0, 180, 255)   # Orange (SWEEP_SCAN)

        cv2.putText(panel, f"ESP32 STATE : {robot_state}", (14, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, st_color, 2)
        cv2.putText(panel, f"MOTORS: {robot_motors}", (dw - 190, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(panel, f"DECISION    : {robot_action}", (14, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.54, (235, 235, 235), 1)
        cv2.putText(panel, f"ESP32 CAM   : {esp32_vision}", (14, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (170, 220, 170), 1)

        combined = np.vstack([display_frame, panel])
        cv2.imshow(WINDOW_NAME, combined)

    ser.close()
    cv2.destroyAllWindows()
    print("Preview closed.")

if __name__ == "__main__":
    main()
