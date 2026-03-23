#!/home/layne/miniconda3/bin/python
# Main hand-tracking → gimbal control script

import os
import subprocess
import sys
import time

import cv2
import mediapipe as mp
import serial

# ---- CONFIG ----

CAMERA_NAME = "Arducam OV2311 USB Camera"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "hand_landmarker.task")

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

PAN_ID = 1
TILT_ID = 2

# Proportional control gains
KP_PAN = 0.025
KP_TILT = 0.025

# Ignore tiny movements (reduces jitter)
DEADBAND_PX = 12

# How often to send motor updates
UPDATE_INTERVAL_SEC = 0.05

# Mechanical limits
MAX_PAN_DEG = 180.0
MAX_TILT_DEG = 85.0

# Axis direction correction
PAN_SIGN = 1.0
TILT_SIGN = -1.0

# Manual calibration delay
MANUAL_CALIBRATION_WINDOW_SEC = 10.0

# Hold zone (no movement if hand is inside)
BOX_WIDTH = 220
BOX_HEIGHT = 160

# Stop recentering once inside this tighter zone
CENTER_TOLERANCE_PX = 25


# ---- UTILS ----

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def get_camera_node():
    # Find camera device path using v4l2
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

    # fallback guesses
    for node in ["/dev/video2", "/dev/video3", "/dev/video0", "/dev/video1"]:
        if os.path.exists(node):
            return node

    return None


def send_cmd(ser, cmd):
    # Send a single command to firmware
    ser.write((cmd + "\n").encode("utf-8"))


def drain_serial(ser, seconds=0.25):
    # Clear any unread serial data
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


# ---- CALIBRATION ----

def setup_motors_with_manual_calibration_window(ser):
    # Disable torque so user can position gimbal manually
    time.sleep(2.0)
    drain_serial(ser)

    send_cmd(ser, f"d {PAN_ID}")
    send_cmd(ser, f"d {TILT_ID}")
    time.sleep(0.2)
    drain_serial(ser)

    print("\n=== MANUAL CALIBRATION WINDOW ===")
    print("Move gimbal to desired zero position.")

    # countdown timer
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

    # Capture zero position
    send_cmd(ser, f"c {PAN_ID}")
    send_cmd(ser, f"c {TILT_ID}")
    time.sleep(0.2)

    # Enable torque and go to zero
    send_cmd(ser, f"e {PAN_ID}")
    send_cmd(ser, f"e {TILT_ID}")
    time.sleep(0.1)
    send_cmd(ser, f"p {PAN_ID} 0")
    send_cmd(ser, f"p {TILT_ID} 0")

    drain_serial(ser)

    print("Calibration captured. Tracking active.\n")


# ---- MOTION ----

def send_positions(ser, pan_deg, tilt_deg):
    # Send target angles to firmware
    send_cmd(ser, f"p {PAN_ID} {pan_deg:.2f}")
    send_cmd(ser, f"p {TILT_ID} {tilt_deg:.2f}")


# ---- VISION ----

def get_palm_center_px(landmarks, w, h):
    # Average key palm points
    palm_ids = [0, 5, 9, 13, 17]
    avg_x = sum(landmarks[i].x for i in palm_ids) / len(palm_ids)
    avg_y = sum(landmarks[i].y for i in palm_ids) / len(palm_ids)
    return int(avg_x * w), int(avg_y * h)


def draw_hand(frame, landmarks):
    # Draw hand skeleton + palm center
    h, w, _ = frame.shape

    connections = [
        (0,1),(1,2),(2,3),(3,4),
        (0,5),(5,6),(6,7),(7,8),
        (5,9),(9,10),(10,11),(11,12),
        (9,13),(13,14),(14,15),(15,16),
        (13,17),(17,18),(18,19),(19,20),
        (0,17),
    ]

    for lm in landmarks:
        x = int(lm.x * w)
        y = int(lm.y * h)
        cv2.circle(frame, (x, y), 3, (0,255,0), -1)

    for a, b in connections:
        x1 = int(landmarks[a].x * w)
        y1 = int(landmarks[a].y * h)
        x2 = int(landmarks[b].x * w)
        y2 = int(landmarks[b].y * h)
        cv2.line(frame, (x1,y1),(x2,y2),(255,0,0),2)

    tx, ty = get_palm_center_px(landmarks, w, h)

    cv2.circle(frame, (tx, ty), 8, (0,255,255), -1)

    return tx, ty


def draw_tracking_ui(frame, cx, cy, bw, bh):
    # Draw center crosshair + hold box
    half_w = bw // 2
    half_h = bh // 2

    left = cx - half_w
    right = cx + half_w
    top = cy - half_h
    bottom = cy + half_h

    cv2.circle(frame, (cx, cy), 6, (0,255,0), -1)
    cv2.rectangle(frame, (left, top), (right, bottom), (255,255,0), 2)

    return left, right, top, bottom


# ---- MAIN LOOP ----

def main():
    # Validate model
    if not os.path.exists(MODEL_PATH):
        sys.exit("Model not found")

    # Find camera
    video_node = get_camera_node()
    if not video_node:
        sys.exit("Camera not found")

    # Open serial
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)
    except Exception as exc:
        sys.exit(f"Serial error: {exc}")

    cap = None

    try:
        # Manual calibration phase
        setup_motors_with_manual_calibration_window(ser)

        # Open camera
        cap = cv2.VideoCapture(video_node, cv2.CAP_V4L2)

        # Setup mediapipe
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
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

                # Run hand detection
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = landmarker.detect_for_video(mp_image, int(time.time()*1000))

                h, w, _ = frame.shape
                cx, cy = w // 2, h // 2

                left, right, top, bottom = draw_tracking_ui(frame, cx, cy, BOX_WIDTH, BOX_HEIGHT)

                if result.hand_landmarks:
                    # Get target position
                    target_x, target_y = draw_hand(frame, result.hand_landmarks[0])

                    error_x = target_x - cx
                    error_y = target_y - cy

                    # HOLD vs RECENTER logic
                    inside = (left <= target_x <= right and top <= target_y <= bottom)

                    if inside:
                        error_x = 0
                        error_y = 0
                        recenter_active = False
                    else:
                        recenter_active = True

                    # Deadband
                    if abs(error_x) <= DEADBAND_PX:
                        error_x = 0
                    if abs(error_y) <= DEADBAND_PX:
                        error_y = 0

                    # Update motors
                    now = time.time()
                    if now - last_update_time >= UPDATE_INTERVAL_SEC:
                        desired_pan_deg += PAN_SIGN * KP_PAN * error_x
                        desired_tilt_deg += TILT_SIGN * KP_TILT * error_y

                        desired_pan_deg = clamp(desired_pan_deg, -MAX_PAN_DEG, MAX_PAN_DEG)
                        desired_tilt_deg = clamp(desired_tilt_deg, -MAX_TILT_DEG, MAX_TILT_DEG)

                        send_positions(ser, desired_pan_deg, desired_tilt_deg)
                        last_update_time = now

                else:
                    recenter_active = False

                # Display
                cv2.imshow("Hand Tracking Follow", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

    finally:
        # Safe shutdown
        try:
            send_cmd(ser, f"p {PAN_ID} 0")
            send_cmd(ser, f"p {TILT_ID} 0")
        except Exception:
            pass

        if cap:
            cap.release()

        cv2.destroyAllWindows()
        ser.close()


if __name__ == "__main__":
    main()
