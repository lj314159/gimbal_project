#!/home/layne/miniconda3/bin/python
#pylint: disable=C0303,C0301

import os
import subprocess
import sys
import time
import traceback
import cv2
import mediapipe as mp
import serial

# Fix broken/ugly OpenCV Qt GUI behavior from polluted environment vars.
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

# --- Configuration & Constants ---
CAMERA_NAME = "Arducam OV2311 USB Camera"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "hand_landmarker.task")

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

PAN_ID, TILT_ID = 1, 2
KP_PAN, KP_TILT = 0.025, 0.025
DEADBAND_PX = 12
UPDATE_INTERVAL_SEC = 0.05

MAX_PAN_DEG, MAX_TILT_DEG = 90.0, 80.0
PAN_SIGN, TILT_SIGN = 1.0, -1.0

MANUAL_CALIBRATION_WINDOW_SEC = 5.0
BOX_WIDTH, BOX_HEIGHT = 50, 50
CENTER_TOLERANCE_PX = 25

DEBUG = True
WINDOW_NAME = "Hand Tracking Follow"

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]

# --- Helper Functions ---
def dbg(msg):
    """Print debug message if DEBUG enabled."""
    if DEBUG:
        print(f"[DEBUG {time.strftime('%H:%M:%S')}] {msg}", flush=True)

def clamp(x, lo, hi):
    """Clamp x to [lo, hi]."""
    return max(lo, min(hi, x))

def free_serial_port(port):
    """Kill processes using a serial port."""
    subprocess.run(["fuser", "-k", port], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def get_camera_node():
    """Return matching /dev/videoX for camera, else fallback."""
    dbg("Searching for camera node with v4l2-ctl...")
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True)
        lines = out.splitlines()
        for i, line in enumerate(lines):
            if CAMERA_NAME in line:
                for j in range(i + 1, len(lines)):
                    s = lines[j].strip()
                    if s.startswith("/dev/video"):
                        dbg(f"Matched camera name '{CAMERA_NAME}' to node {s}")
                        return s
                    if s == "":
                        break
    except Exception as exc:
        dbg(f"v4l2-ctl lookup failed: {exc}")

    dbg("Falling back to default video nodes...")
    for node in ["/dev/video2", "/dev/video3", "/dev/video0", "/dev/video1"]:
        if os.path.exists(node):
            dbg(f"Using fallback node {node}")
            return node
    return None

def send_cmd(ser, cmd):
    """Send newline-terminated serial command."""
    ser.write((cmd + "\n").encode("utf-8"))

def drain_serial(ser, seconds=0.25):
    """Clear serial input buffer for a short duration."""
    end_time = time.time() + seconds
    while time.time() < end_time:
        try:
            waiting = ser.in_waiting
            if waiting > 0:
                ser.read(waiting)
            else:
                time.sleep(0.01)
        except Exception:
            break

def setup_motors_with_manual_calibration_window(ser):
    """Disable torque, allow manual positioning, capture zero, re-enable."""
    dbg("Starting motor setup/calibration window")
    time.sleep(2.0)
    drain_serial(ser)

    for motor_id in (PAN_ID, TILT_ID):
        send_cmd(ser, f"d {motor_id}")
    time.sleep(0.2)
    drain_serial(ser)

    print(f"\n{'='*52}\nMANUAL CALIBRATION WINDOW\nTorque is OFF.\n"
          f"Move the gimbal by hand to the desired zero position.\n"
          f"You have {int(MANUAL_CALIBRATION_WINDOW_SEC)} seconds...\n{'='*52}\n")

    end_time = time.time() + MANUAL_CALIBRATION_WINDOW_SEC
    last_shown = None

    while (remaining := int(max(0, end_time - time.time() + 0.999))) > 0:
        if remaining != last_shown:
            print(f"Starting tracking in {remaining}...")
            last_shown = remaining
        time.sleep(0.05)

    dbg("Capturing calibration positions and enabling motors")
    for motor_id in (PAN_ID, TILT_ID):
        send_cmd(ser, f"c {motor_id}")
        
    time.sleep(0.2)
    for motor_id in (PAN_ID, TILT_ID):
        send_cmd(ser, f"e {motor_id}")
        
    time.sleep(0.1)
    send_positions(ser, 0, 0)
    time.sleep(0.2)
    drain_serial(ser)
    print("\nCalibration captured.\nTracking is now active.\n")

def send_positions(ser, pan_deg, tilt_deg):
    """Send pan/tilt angles (deg) to motors."""
    send_cmd(ser, f"p {PAN_ID} {pan_deg:.2f}")
    send_cmd(ser, f"p {TILT_ID} {tilt_deg:.2f}")

def get_palm_center_px(landmarks, w, h):
    """Return palm center (x, y) in pixels."""
    palm_ids = [0, 5, 9, 13, 17]
    avg_x = sum(landmarks[i].x for i in palm_ids) / len(palm_ids)
    avg_y = sum(landmarks[i].y for i in palm_ids) / len(palm_ids)
    return int(avg_x * w), int(avg_y * h)

def draw_tracking_ui(frame, center_x, center_y):
    """Draw crosshair + hold box; return box bounds."""
    half_w, half_h = BOX_WIDTH // 2, BOX_HEIGHT // 2
    left, right = center_x - half_w, center_x + half_w
    top, bottom = center_y - half_h, center_y + half_h

    # Center crosshair
    cv2.circle(frame, (center_x, center_y), 6, (0, 255, 0), -1)
    cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
    cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)

    # Hold box
    cv2.rectangle(frame, (left, top), (right, bottom), (255, 255, 0), 2)
    cv2.putText(frame, "Hold Box", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
    
    return left, right, top, bottom

def draw_hand(frame, landmarks, w, h):
    """Draw hand landmarks and connections on frame."""
    for lm in landmarks:
        cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, (0, 255, 0), -1)
    for a, b in HAND_CONNECTIONS:
        x1, y1 = int(landmarks[a].x * w), int(landmarks[a].y * h)
        x2, y2 = int(landmarks[b].x * w), int(landmarks[b].y * h)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

# --- Main Logic ---
def main():
    dbg("Program start")
    free_serial_port(SERIAL_PORT)

    if not os.path.exists(MODEL_PATH):
        sys.exit(f"Model not found: {MODEL_PATH}")

    video_node = get_camera_node()
    if not video_node:
        sys.exit("Could not find camera.")

    ser = None
    cap = None

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)
        setup_motors_with_manual_calibration_window(ser)

        cap = cv2.VideoCapture(video_node, cv2.CAP_V4L2)
        if not cap.isOpened():
            sys.exit(f"Failed to open {video_node}")

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=mp.tasks.vision.RunningMode.IMAGE,
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
                if not ret: continue

                frame = cv2.flip(frame, 1)
                h, w = frame.shape[:2]
                center_x, center_y = w // 2, h // 2

                # Inference
                infer_frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(infer_frame, cv2.COLOR_BGR2RGB))
                result = landmarker.detect(mp_image)

                # UI
                left, right, top, bottom = draw_tracking_ui(frame, center_x, center_y)

                if result.hand_landmarks:
                    hand_landmarks = result.hand_landmarks[0]
                    target_x, target_y = get_palm_center_px(hand_landmarks, w, h)
                    
                    draw_hand(frame, hand_landmarks, w, h)
                    cv2.circle(frame, (target_x, target_y), 8, (0, 255, 255), -1)
                    cv2.line(frame, (center_x, center_y), (target_x, target_y), (0, 255, 255), 2)

                    # Control Logic
                    raw_err_x, raw_err_y = target_x - center_x, target_y - center_y
                    inside_hold = left <= target_x <= right and top <= target_y <= bottom
                    centered = abs(raw_err_x) <= CENTER_TOLERANCE_PX and abs(raw_err_y) <= CENTER_TOLERANCE_PX

                    if not recenter_active and inside_hold:
                        error_x, error_y = 0, 0
                        status, color = "HOLD", (0, 255, 0)
                    elif recenter_active and centered:
                        recenter_active = False
                        error_x, error_y = 0, 0
                        status, color = "HOLD", (0, 255, 0)
                    else:
                        recenter_active = True
                        error_x, error_y = raw_err_x, raw_err_y
                        status, color = "RECENTER", (0, 165, 255)

                    if abs(error_x) <= DEADBAND_PX: error_x = 0
                    if abs(error_y) <= DEADBAND_PX: error_y = 0

                    now = time.time()
                    if now - last_update_time >= UPDATE_INTERVAL_SEC:
                        desired_pan_deg = clamp(desired_pan_deg + (PAN_SIGN * KP_PAN * error_x), -MAX_PAN_DEG, MAX_PAN_DEG)
                        desired_tilt_deg = clamp(desired_tilt_deg + (TILT_SIGN * KP_TILT * error_y), -MAX_TILT_DEG, MAX_TILT_DEG)
                        
                        send_positions(ser, desired_pan_deg, desired_tilt_deg)
                        last_update_time = now

                    cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
                else:
                    recenter_active = False
                    cv2.putText(frame, "NO HAND DETECTED", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

                cv2.imshow(WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

    except KeyboardInterrupt:
        dbg("KeyboardInterrupt received")
    except Exception as exc:
        print(f"\n========== EXCEPTION ==========\n{exc}")
        traceback.print_exc()
        print("================================\n")
    finally:
        dbg("Entering cleanup")
        if ser:
            try:
                send_positions(ser, 0, 0)
                ser.close()
            except Exception as e: dbg(f"Serial cleanup failed: {e}")
        if cap:
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
