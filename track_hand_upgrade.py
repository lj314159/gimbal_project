#!/home/layne/miniconda3/bin/python
#pylint: disable=

import os
import subprocess
import sys
import time

import cv2
import mediapipe as mp
import serial

CAMERA_NAME = "Arducam OV2311 USB Camera"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "hand_landmarker.task")

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

PAN_ID = 1
TILT_ID = 2

KP_PAN = 0.025
KP_TILT = 0.025
DEADBAND_PX = 12
UPDATE_INTERVAL_SEC = 0.05

MAX_PAN_DEG = 90.0
MAX_TILT_DEG = 80.0

PAN_SIGN = 1.0
TILT_SIGN = -1.0

MANUAL_CALIBRATION_WINDOW_SEC = 10.0

# Hold box: if the palm is inside this box, hold position.
BOX_WIDTH = 220
BOX_HEIGHT = 160

# Center zone: once recentering starts, keep moving until the palm reaches this zone.
CENTER_TOLERANCE_PX = 25


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def get_camera_node():
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True)
        lines = out.splitlines()

        for i, line in enumerate(lines):
            if CAMERA_NAME in line:
                for j in range(i + 1, len(lines)):
                    s = lines[j].strip()
                    if s.startswith("/dev/video"):
                        return s
                    if s == "":
                        break
    except Exception:
        pass

    for node in ["/dev/video2", "/dev/video3", "/dev/video0", "/dev/video1"]:
        if os.path.exists(node):
            return node

    return None


def send_cmd(ser, cmd):
    ser.write((cmd + "\n").encode("utf-8"))


def drain_serial(ser, seconds=0.25):
    end_time = time.time() + seconds
    while time.time() < end_time:
        try:
            waiting = ser.in_waiting
        except Exception:
            break

        if waiting > 0:
            try:
                ser.read(waiting)
            except Exception:
                break
        else:
            time.sleep(0.01)


def setup_motors_with_manual_calibration_window(ser):
    time.sleep(2.0)
    drain_serial(ser)

    send_cmd(ser, f"d {PAN_ID}")
    send_cmd(ser, f"d {TILT_ID}")
    time.sleep(0.2)
    drain_serial(ser)

    print()
    print("====================================================")
    print("MANUAL CALIBRATION WINDOW")
    print("Torque is OFF.")
    print("Move the gimbal by hand to the desired zero position.")
    print(f"You have {int(MANUAL_CALIBRATION_WINDOW_SEC)} seconds...")
    print("====================================================")
    print()

    end_time = time.time() + MANUAL_CALIBRATION_WINDOW_SEC
    last_shown = None

    while True:
        remaining = int(end_time - time.time() + 0.999)
        if remaining < 0:
            remaining = 0

        if remaining != last_shown:
            print(f"Starting tracking in {remaining}...")
            last_shown = remaining

        if time.time() >= end_time:
            break

        time.sleep(0.05)

    send_cmd(ser, f"c {PAN_ID}")
    send_cmd(ser, f"c {TILT_ID}")
    time.sleep(0.2)

    send_cmd(ser, f"e {PAN_ID}")
    send_cmd(ser, f"e {TILT_ID}")
    time.sleep(0.1)
    send_cmd(ser, f"p {PAN_ID} 0")
    send_cmd(ser, f"p {TILT_ID} 0")
    time.sleep(0.2)
    drain_serial(ser)

    print()
    print("Calibration captured.")
    print("Tracking is now active.")
    print()


def send_positions(ser, pan_deg, tilt_deg):
    send_cmd(ser, f"p {PAN_ID} {pan_deg:.2f}")
    send_cmd(ser, f"p {TILT_ID} {tilt_deg:.2f}")


def get_palm_center_px(landmarks, w, h):
    palm_ids = [0, 5, 9, 13, 17]
    avg_x = sum(landmarks[i].x for i in palm_ids) / len(palm_ids)
    avg_y = sum(landmarks[i].y for i in palm_ids) / len(palm_ids)
    tx = int(avg_x * w)
    ty = int(avg_y * h)
    return tx, ty


def draw_hand(frame, landmarks):
    h, w, _ = frame.shape

    connections = [
        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (17, 18), (18, 19), (19, 20),
        (0, 17),
    ]

    for lm in landmarks:
        x = int(lm.x * w)
        y = int(lm.y * h)
        cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)

    for a, b in connections:
        x1 = int(landmarks[a].x * w)
        y1 = int(landmarks[a].y * h)
        x2 = int(landmarks[b].x * w)
        y2 = int(landmarks[b].y * h)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    tx, ty = get_palm_center_px(landmarks, w, h)

    cv2.circle(frame, (tx, ty), 8, (0, 255, 255), -1)
    cv2.putText(
        frame,
        "Palm Center",
        (tx + 10, ty - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )

    return tx, ty


def draw_tracking_ui(frame, center_x, center_y, box_width, box_height):
    half_w = box_width // 2
    half_h = box_height // 2

    left = center_x - half_w
    right = center_x + half_w
    top = center_y - half_h
    bottom = center_y + half_h

    cv2.circle(frame, (center_x, center_y), 6, (0, 255, 0), -1)
    cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
    cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)

    cv2.rectangle(frame, (left, top), (right, bottom), (255, 255, 0), 2)
    cv2.putText(
        frame,
        "Hold Box",
        (left, top - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 0),
        2,
        cv2.LINE_AA,
    )

#    c_left = center_x - CENTER_TOLERANCE_PX
#    c_right = center_x + CENTER_TOLERANCE_PX
#    c_top = center_y - CENTER_TOLERANCE_PX
#    c_bottom = center_y + CENTER_TOLERANCE_PX
#
#    cv2.rectangle(frame, (c_left, c_top), (c_right, c_bottom), (0, 165, 255), 2)
#    cv2.putText(
#        frame,
#        "Center Zone",
#        (c_left, c_bottom + 20),
#        cv2.FONT_HERSHEY_SIMPLEX,
#        0.5,
#        (0, 165, 255),
#        1,
#        cv2.LINE_AA,
#    )

    return left, right, top, bottom


def main():
    if not os.path.exists(MODEL_PATH):
        sys.exit(f"Model not found: {MODEL_PATH}")

    video_node = get_camera_node()
    if not video_node:
        sys.exit("Could not find camera.")

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)
    except Exception as exc:
        sys.exit(f"Could not open serial port {SERIAL_PORT}: {exc}")

    cap = None

    try:
        setup_motors_with_manual_calibration_window(ser)

        cap = cv2.VideoCapture(video_node, cv2.CAP_V4L2)
        if not cap.isOpened():
            sys.exit(f"Failed to open {video_node}")

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 50)

        BaseOptions = mp.tasks.BaseOptions
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        desired_pan_deg = 0.0
        desired_tilt_deg = 0.0
        last_update_time = 0.0
        recenter_active = False

        with mp.tasks.vision.HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                frame = cv2.flip(frame, 1)

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                timestamp_ms = int(time.time() * 1000)
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                h, w, _ = frame.shape
                center_x = w // 2
                center_y = h // 2

                left, right, top, bottom = draw_tracking_ui(
                    frame, center_x, center_y, BOX_WIDTH, BOX_HEIGHT
                )

                if result.hand_landmarks:
                    hand_landmarks = result.hand_landmarks[0]
                    target_x, target_y = draw_hand(frame, hand_landmarks)

                    cv2.line(frame, (center_x, center_y), (target_x, target_y), (0, 255, 255), 2)

                    raw_error_x = target_x - center_x
                    raw_error_y = target_y - center_y

                    inside_hold_box = (left <= target_x <= right and top <= target_y <= bottom)
                    centered_now = (
                        abs(raw_error_x) <= CENTER_TOLERANCE_PX
                        and abs(raw_error_y) <= CENTER_TOLERANCE_PX
                    )

                    if not recenter_active:
                        if inside_hold_box:
                            error_x = 0
                            error_y = 0
                            status_text = "HOLD"
                            status_color = (0, 255, 0)
                        else:
                            recenter_active = True
                            error_x = raw_error_x
                            error_y = raw_error_y
                            status_text = "RECENTER"
                            status_color = (0, 165, 255)
                    else:
                        if centered_now:
                            recenter_active = False
                            error_x = 0
                            error_y = 0
                            status_text = "HOLD"
                            status_color = (0, 255, 0)
                        else:
                            error_x = raw_error_x
                            error_y = raw_error_y
                            status_text = "RECENTER"
                            status_color = (0, 165, 255)

                    if abs(error_x) <= DEADBAND_PX:
                        error_x = 0
                    if abs(error_y) <= DEADBAND_PX:
                        error_y = 0

                    now = time.time()
                    if now - last_update_time >= UPDATE_INTERVAL_SEC:
                        desired_pan_deg += PAN_SIGN * KP_PAN * error_x
                        desired_tilt_deg += TILT_SIGN * KP_TILT * error_y

                        desired_pan_deg = clamp(desired_pan_deg, -MAX_PAN_DEG, MAX_PAN_DEG)
                        desired_tilt_deg = clamp(desired_tilt_deg, -MAX_TILT_DEG, MAX_TILT_DEG)

                        send_positions(ser, desired_pan_deg, desired_tilt_deg)
                        last_update_time = now

                    cv2.putText(
                        frame,
                        status_text,
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        status_color,
                        2,
                        cv2.LINE_AA,
                    )

                    cv2.putText(
                        frame,
                        f"err_x={raw_error_x} err_y={raw_error_y}",
                        (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    cv2.putText(
                        frame,
                        f"pan={desired_pan_deg:.1f} tilt={desired_tilt_deg:.1f}",
                        (20, 105),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    recenter_active = False
                    cv2.putText(
                        frame,
                        "NO HAND DETECTED",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

                cv2.imshow("Hand Tracking Follow", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

    finally:
        try:
            send_cmd(ser, f"p {PAN_ID} 0")
            send_cmd(ser, f"p {TILT_ID} 0")
        except Exception:
            pass

        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        ser.close()


if __name__ == "__main__":
    main()
