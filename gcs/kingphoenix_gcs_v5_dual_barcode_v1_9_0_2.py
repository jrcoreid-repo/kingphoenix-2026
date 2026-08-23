#!/usr/bin/env python3
# ============================================================
# KINGPHOENIX GCS 2026 V5 DUAL BARCODE
# PC SIDE - Windows / Python
#
# FRONT VIDEO : UDP MPEG-TS/H264 14560
# FRONT META  : UDP JSON        14561
# CONTROL     : UDP JSON        14562
# BOTTOM VIDEO: UDP MPEG-TS/H264 14563
# BOTTOM META : UDP JSON        14564
#
# Synchronization:
#   decode 48-cell pixel barcode from each decoded video frame
#   -> frame_id
#   -> exact metadata_buffer[frame_id]
#
# FRONT: AI ACTIVE (YOLOv8 Pose / YOLO26 fallback)
# BOTTOM: AI STANDBY (live stream only for now)
# ============================================================

import math
import json
import socket
import threading
import time
import platform
from collections import OrderedDict
from pathlib import Path

import av
import cv2
import numpy as np
import dearpygui.dearpygui as dpg

# ============================================================
# APP
# ============================================================
APP_TITLE = "Kingphoenix GCS 2026 V5 Dual Barcode - WP Layout 2"
WINDOW_W = 1740
WINDOW_H = 980

# ============================================================
# NETWORK
# ============================================================
FRONT_VIDEO_PORT = 14560
FRONT_METADATA_PORT = 14561
CONTROL_PORT = 14562
BOTTOM_VIDEO_PORT = 14563
BOTTOM_METADATA_PORT = 14564

DEFAULT_DRONE_IP = "192.168.1.50"
HELLO_TIMEOUT_SEC = 2.0
HEARTBEAT_INTERVAL_SEC = 1.0
VIDEO_TIMEOUT_SEC = 2.0
META_TIMEOUT_SEC = 2.0
RECONNECT_DELAY_SEC = 0.35

# ============================================================
# VIDEO PLACEHOLDER / FAILSAFE BACKGROUND
# ============================================================
# Put background.png in the SAME folder as this Python script.
# The same 640x640 image is used by FRONT and BOTTOM whenever:
#   - the stream is still waiting
#   - the video is lost / timed out
#   - barcode/frame synchronization is not valid
# A short grace period prevents a single damaged barcode frame from
# making the display flash rapidly between live video and placeholder.
SCRIPT_DIR = Path(__file__).resolve().parent
BACKGROUND_IMAGE_PATH = SCRIPT_DIR / "background.png"
SYNC_FAIL_GRACE_SEC = 0.30

# ============================================================
# VIDEO
# ============================================================
FRAME_W = 640
FRAME_H = 640
DISPLAY_W = 500
DISPLAY_H = 500

# ============================================================
# BARCODE - MUST MATCH RASPBERRY PI
# ============================================================
BARCODE_PREAMBLE = 0b10101100
BARCODE_PREAMBLE_BITS = 8
BARCODE_FRAME_BITS = 32
BARCODE_CHECKSUM_BITS = 8
BARCODE_TOTAL_BITS = 48
BARCODE_CELL_W = 8
BARCODE_CELL_H = 12
BARCODE_X = 8
BARCODE_Y = FRAME_H - BARCODE_CELL_H - 4
BARCODE_WIDTH = BARCODE_TOTAL_BITS * BARCODE_CELL_W
BARCODE_BORDER = 2

# Sample only the inner area to avoid H264 edge ringing.
BARCODE_INNER_X = 2
BARCODE_INNER_Y = 2
BARCODE_MIN_CONTRAST = 45.0

# Metadata buffering.
META_BUFFER_MAX = 900
META_MATCH_WAIT_SEC = 0.085
META_MATCH_POLL_SEC = 0.001

# ============================================================
# HUD / CAMERA
# ============================================================
CAMERA_HFOV_DEG = 90.0
AXIS_LENGTH_M = 0.55
SYNC_BAR_H = 24
GATE_FILL_ALPHA = 0.12
PANEL_ALPHA = 0.70

# OpenCV BGR
C_TEXT = (235, 238, 242)
C_DIM = (165, 175, 188)
C_PANEL = (12, 15, 19)
C_GATE = (235, 235, 235)
C_ACCENT = (255, 210, 145)
C_GATE_FILL = (160, 95, 25)
C_DET = (80, 225, 95)
C_DET_FILL = (45, 135, 55)
C_AXIS_X = (60, 70, 255)
C_AXIS_Y = (60, 255, 80)
C_AXIS_Z = (255, 130, 55)
C_OK = (80, 225, 110)
C_BAD = (80, 90, 240)
C_ACTIVE = (80, 230, 110)
C_STANDBY = (70, 195, 240)
C_DISABLED = (150, 155, 165)

SNAPSHOT_DIR = Path("snapshots")
SNAPSHOT_DIR.mkdir(exist_ok=True)

# ============================================================
# GLOBAL CONNECTION STATE
# ============================================================
connection_lock = threading.Lock()
connection_state = "DISCONNECTED"
connection_message = "Not connected"
is_connected = False
connection_in_progress = False
drone_ip = DEFAULT_DRONE_IP
stop_event = threading.Event()
shutdown_event = threading.Event()

control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_socket_lock = threading.Lock()

# ============================================================
# CAMERA STATE
# ============================================================
class CameraState:
    def __init__(self, name, video_port, metadata_port):
        self.name = name
        self.video_port = int(video_port)
        self.metadata_port = int(metadata_port)

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_metadata = None
        self.latest_frame_id = None
        self.latest_barcode_crc = None
        self.last_video_time = 0.0
        self.video_fps = 0.0

        self.meta_lock = threading.Lock()
        self.meta_buffer = OrderedDict()
        self.latest_raw_metadata = None
        self.last_meta_time = 0.0
        self.meta_rx_packets = 0
        self.meta_rx_bad_json = 0
        self.meta_rx_wrong_camera = 0
        self.meta_rx_last_sender = None

        self.sync_lock = threading.Lock()
        self.sync_ok = 0
        self.sync_miss = 0
        self.barcode_fail = 0
        self.last_video_frame_id = None
        self.last_meta_frame_id = None
        self.last_sync_diff = None
        self.last_good_sync_time = 0.0

        self.container_lock = threading.Lock()
        self.container = None
        self.video_thread = None
        self.meta_thread = None

front = CameraState("front", FRONT_VIDEO_PORT, FRONT_METADATA_PORT)
bottom = CameraState("bottom", BOTTOM_VIDEO_PORT, BOTTOM_METADATA_PORT)

# ============================================================
# PC INFO
# ============================================================
def get_local_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "Unknown"
    finally:
        sock.close()

PC_IP = get_local_ip()
PC_NAME = platform.node()

# ============================================================
# UTILS
# ============================================================
def valid_ipv4(ip):
    try:
        parts = ip.split(".")
        return len(parts) == 4 and all(p.isdigit() and 0 <= int(p) <= 255 for p in parts)
    except Exception:
        return False


def frame_id_checksum(frame_id):
    frame_id = int(frame_id) & 0xFFFFFFFF
    b0 = frame_id & 0xFF
    b1 = (frame_id >> 8) & 0xFF
    b2 = (frame_id >> 16) & 0xFF
    b3 = (frame_id >> 24) & 0xFF
    return (b0 ^ b1 ^ b2 ^ b3 ^ 0xA5) & 0xFF


def bits_to_integer(bits):
    value = 0
    for bit in bits:
        value = (value << 1) | (1 if bit else 0)
    return value


def decode_frame_id_barcode(frame):
    """Return (frame_id, checksum, diagnostics) or (None, None, diagnostics)."""
    if frame is None or frame.shape[0] < FRAME_H or frame.shape[1] < FRAME_W:
        return None, None, {"reason": "size"}

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    means = []

    for i in range(BARCODE_TOTAL_BITS):
        x0 = BARCODE_X + i * BARCODE_CELL_W + BARCODE_INNER_X
        x1 = BARCODE_X + (i + 1) * BARCODE_CELL_W - BARCODE_INNER_X
        y0 = BARCODE_Y + BARCODE_INNER_Y
        y1 = BARCODE_Y + BARCODE_CELL_H - BARCODE_INNER_Y
        roi = gray[y0:y1, x0:x1]
        if roi.size == 0:
            return None, None, {"reason": "roi"}
        means.append(float(np.mean(roi)))

    lo = float(np.percentile(means, 20))
    hi = float(np.percentile(means, 80))
    contrast = hi - lo
    threshold = (hi + lo) * 0.5

    diagnostics = {
        "contrast": contrast,
        "threshold": threshold,
        "lo": lo,
        "hi": hi,
    }

    if contrast < BARCODE_MIN_CONTRAST:
        diagnostics["reason"] = "contrast"
        return None, None, diagnostics

    bits = [1 if v > threshold else 0 for v in means]
    preamble = bits_to_integer(bits[:BARCODE_PREAMBLE_BITS])

    if preamble != BARCODE_PREAMBLE:
        diagnostics["reason"] = "preamble"
        diagnostics["preamble"] = preamble
        return None, None, diagnostics

    a = BARCODE_PREAMBLE_BITS
    b = a + BARCODE_FRAME_BITS
    frame_id = bits_to_integer(bits[a:b]) & 0xFFFFFFFF
    crc = bits_to_integer(bits[b:b + BARCODE_CHECKSUM_BITS]) & 0xFF
    expected = frame_id_checksum(frame_id)

    if crc != expected:
        diagnostics["reason"] = "checksum"
        diagnostics["crc"] = crc
        diagnostics["expected"] = expected
        return None, None, diagnostics

    diagnostics["reason"] = "ok"
    return frame_id, crc, diagnostics


# ============================================================
# CAMERA MATRIX
# ============================================================
def create_camera_matrix():
    hfov = math.radians(CAMERA_HFOV_DEG)
    fx = FRAME_W / (2.0 * math.tan(hfov / 2.0))
    fy = fx
    cx = FRAME_W / 2.0
    cy = FRAME_H / 2.0
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

CAMERA_MATRIX = create_camera_matrix()
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)


def rpy_to_rotation_vector(rpy_deg):
    if rpy_deg is None or len(rpy_deg) < 3:
        return None
    roll, pitch, yaw = [math.radians(float(v)) for v in rpy_deg[:3]]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    rotation = rz @ ry @ rx
    rvec, _ = cv2.Rodrigues(rotation)
    return rvec

# ============================================================
# METADATA RECEIVER
# ============================================================
def store_metadata(camera, meta):
    frame_id = meta.get("frame_id")
    if frame_id is None:
        return
    frame_id = int(frame_id) & 0xFFFFFFFF

    with camera.meta_lock:
        camera.meta_buffer[frame_id] = meta
        camera.meta_buffer.move_to_end(frame_id)
        while len(camera.meta_buffer) > META_BUFFER_MAX:
            camera.meta_buffer.popitem(last=False)
        camera.latest_raw_metadata = meta
        camera.last_meta_time = time.monotonic()


def metadata_receiver_loop(camera):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", camera.metadata_port))
    sock.settimeout(0.25)
    print(f"[META:{camera.name}] LISTEN :{camera.metadata_port}")

    first_packet_logged = False

    try:
        while not shutdown_event.is_set():
            try:
                data, sender = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception:
                continue

            # V1.9.0.2:
            # Do NOT discard metadata merely because sender[0] differs from
            # drone_ip. Multi-interface Raspberry Pi routing can legitimately
            # use another local source IP. The packet still arrives on the
            # dedicated metadata port and must also identify its camera.
            camera.meta_rx_packets += 1
            camera.meta_rx_last_sender = sender[0]

            if not first_packet_logged:
                first_packet_logged = True
                print(
                    f"[META:{camera.name}] FIRST PACKET "
                    f"from={sender[0]}:{sender[1]} bytes={len(data)} "
                    f"expected_drone_ip={drone_ip}"
                )

            try:
                meta = json.loads(data.decode("utf-8"))
            except Exception as error:
                camera.meta_rx_bad_json += 1
                print(
                    f"[META:{camera.name}] BAD JSON "
                    f"from={sender[0]} error={error}"
                )
                continue

            if str(meta.get("camera", camera.name)).lower() != camera.name:
                camera.meta_rx_wrong_camera += 1
                continue

            store_metadata(camera, meta)
    finally:
        sock.close()

def wait_metadata(camera, frame_id):
    deadline = time.monotonic() + META_MATCH_WAIT_SEC
    frame_id = int(frame_id) & 0xFFFFFFFF
    while time.monotonic() < deadline and not shutdown_event.is_set():
        with camera.meta_lock:
            meta = camera.meta_buffer.get(frame_id)
        if meta is not None:
            return meta
        time.sleep(META_MATCH_POLL_SEC)
    return None

# ============================================================
# MPEG-TS / H264 VIDEO RECEIVER
# ============================================================
def close_camera_container(camera):
    with camera.container_lock:
        container = camera.container
        camera.container = None
    if container is not None:
        try:
            container.close()
        except Exception:
            pass


def video_receiver_loop(camera):
    while not shutdown_event.is_set():
        if not is_connected:
            time.sleep(0.05)
            continue

        container = None
        try:
            url = (
                f"udp://0.0.0.0:{camera.video_port}"
                "?fifo_size=5000000&overrun_nonfatal=1"
            )

            print(f"[VIDEO:{camera.name}] OPEN {url}")

            container = av.open(
                url,
                mode="r",
                format="mpegts",
                options={
                    "fflags": "nobuffer",
                    "flags": "low_delay",
                    "probesize": "131072",
                    "analyzeduration": "500000",
                },
            )

            with camera.container_lock:
                camera.container = container

            video_streams = [s for s in container.streams if s.type == "video"]
            if not video_streams:
                raise RuntimeError("No video stream in MPEG-TS")

            stream = video_streams[0]
            try:
                stream.thread_type = "AUTO"
            except Exception:
                pass

            print(f"[VIDEO:{camera.name}] DECODER OPEN")

            fps_start = time.perf_counter()
            fps_count = 0

            for packet in container.demux(stream):
                if shutdown_event.is_set() or not is_connected:
                    break

                for decoded in packet.decode():
                    if shutdown_event.is_set() or not is_connected:
                        break

                    frame = decoded.to_ndarray(format="bgr24")
                    if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
                        frame = cv2.resize(frame, (FRAME_W, FRAME_H), interpolation=cv2.INTER_LINEAR)

                    frame_id, barcode_crc, diag = decode_frame_id_barcode(frame)

                    if frame_id is None:
                        with camera.sync_lock:
                            camera.barcode_fail += 1
                            camera.last_video_frame_id = None
                            camera.last_meta_frame_id = None
                            camera.last_sync_diff = None

                        # We still display video, but NEVER draw stale AI metadata.
                        with camera.frame_lock:
                            camera.latest_frame = frame
                            camera.latest_metadata = None
                            camera.latest_frame_id = None
                            camera.latest_barcode_crc = None
                            camera.last_video_time = time.monotonic()
                    else:
                        meta = wait_metadata(camera, frame_id)

                        with camera.sync_lock:
                            camera.last_video_frame_id = frame_id
                            if meta is not None:
                                meta_id = int(meta.get("frame_id", -1)) & 0xFFFFFFFF
                                camera.last_meta_frame_id = meta_id
                                camera.last_sync_diff = meta_id - frame_id
                                camera.sync_ok += 1
                                if camera.last_sync_diff == 0:
                                    camera.last_good_sync_time = time.monotonic()
                            else:
                                camera.last_meta_frame_id = None
                                camera.last_sync_diff = None
                                camera.sync_miss += 1

                        with camera.frame_lock:
                            camera.latest_frame = frame
                            camera.latest_metadata = meta
                            camera.latest_frame_id = frame_id
                            camera.latest_barcode_crc = barcode_crc
                            camera.last_video_time = time.monotonic()

                    fps_count += 1
                    now = time.perf_counter()
                    elapsed = now - fps_start
                    if elapsed >= 1.0:
                        camera.video_fps = fps_count / elapsed
                        fps_count = 0
                        fps_start = now

        except Exception as error:
            if is_connected and not shutdown_event.is_set():
                print(f"[VIDEO:{camera.name}] {type(error).__name__}: {error}")
                time.sleep(RECONNECT_DELAY_SEC)
        finally:
            with camera.container_lock:
                if camera.container is container:
                    camera.container = None
            if container is not None:
                try:
                    container.close()
                except Exception:
                    pass

# ============================================================
# CONTROL / HANDSHAKE
# ============================================================
def set_connection_state(state, message):
    global connection_state, connection_message
    with connection_lock:
        connection_state = state
        connection_message = message


def clear_runtime_state(camera):
    with camera.frame_lock:
        camera.latest_frame = None
        camera.latest_metadata = None
        camera.latest_frame_id = None
        camera.latest_barcode_crc = None
        camera.last_video_time = 0.0
        camera.video_fps = 0.0
    with camera.meta_lock:
        camera.meta_buffer.clear()
        camera.latest_raw_metadata = None
        camera.last_meta_time = 0.0
    with camera.sync_lock:
        camera.sync_ok = 0
        camera.sync_miss = 0
        camera.barcode_fail = 0
        camera.last_video_frame_id = None
        camera.last_meta_frame_id = None
        camera.last_sync_diff = None
        camera.last_good_sync_time = 0.0


def perform_handshake(requested_ip):
    global is_connected, connection_in_progress, drone_ip
    connection_in_progress = True
    drone_ip = requested_ip
    set_connection_state("CONNECTING", f"Connecting to {requested_ip}")

    clear_runtime_state(front)
    clear_runtime_state(bottom)

    hello = {
        "type": "HELLO",
        "pc_ip": PC_IP,
        "front_video_port": FRONT_VIDEO_PORT,
        "front_metadata_port": FRONT_METADATA_PORT,
        "bottom_video_port": BOTTOM_VIDEO_PORT,
        "bottom_metadata_port": BOTTOM_METADATA_PORT,
        # Legacy front aliases retained too.
        "video_port": FRONT_VIDEO_PORT,
        "metadata_port": FRONT_METADATA_PORT,
    }

    try:
        with control_socket_lock:
            control_socket.settimeout(HELLO_TIMEOUT_SEC)
            control_socket.sendto(json.dumps(hello).encode("utf-8"), (requested_ip, CONTROL_PORT))
            data, sender = control_socket.recvfrom(4096)

        response = json.loads(data.decode("utf-8"))
        if str(response.get("type", "")).upper() != "HELLO_ACK":
            raise RuntimeError("Invalid HELLO_ACK")

        is_connected = True
        set_connection_state("CONNECTED", f"Connected to {requested_ip}")

        print("\n==============================================")
        print("[CONTROL] CONNECTED")
        print("Drone       :", requested_ip)
        print("Transport   :", response.get("video_transport"))
        print("Sync        :", response.get("sync"))
        print("AI camera   :", response.get("active_ai_camera"))
        print("==============================================")

    except Exception as error:
        is_connected = False
        set_connection_state("ERROR", str(error))
        print("[CONTROL ERROR]", repr(error))
    finally:
        connection_in_progress = False


def disconnect_drone():
    global is_connected
    try:
        with control_socket_lock:
            control_socket.sendto(
                json.dumps({"type": "DISCONNECT"}).encode("utf-8"),
                (drone_ip, CONTROL_PORT),
            )
    except Exception:
        pass

    is_connected = False
    close_camera_container(front)
    close_camera_container(bottom)
    clear_runtime_state(front)
    clear_runtime_state(bottom)
    set_connection_state("DISCONNECTED", "Not connected")


def heartbeat_loop():
    while not shutdown_event.is_set():
        if is_connected:
            try:
                with control_socket_lock:
                    control_socket.sendto(
                        json.dumps({"type": "HEARTBEAT"}).encode("utf-8"),
                        (drone_ip, CONTROL_PORT),
                    )
            except Exception:
                pass
        time.sleep(HEARTBEAT_INTERVAL_SEC)


def connect_button_callback():
    if connection_in_progress:
        return
    if is_connected:
        threading.Thread(target=disconnect_drone, daemon=True).start()
        return
    requested_ip = dpg.get_value("drone_ip_input").strip()
    if not valid_ipv4(requested_ip):
        set_connection_state("ERROR", "Invalid IPv4 address")
        return
    threading.Thread(target=perform_handshake, args=(requested_ip,), daemon=True).start()

# ============================================================
# FRAME ACCESS
# ============================================================
def get_camera_frame(camera):
    with camera.frame_lock:
        if camera.latest_frame is None:
            return None, None, None
        return camera.latest_frame.copy(), camera.latest_metadata, camera.latest_frame_id


def get_latest_raw_metadata(camera):
    with camera.meta_lock:
        meta = camera.latest_raw_metadata
        age = time.monotonic() - camera.last_meta_time if camera.last_meta_time else 999.0
    if meta is None or age > META_TIMEOUT_SEC:
        return None
    return meta

# ============================================================
# DRAWING HELPERS
# ============================================================
def transparent_rect(frame, p1, p2, color, alpha):
    overlay = frame.copy()
    cv2.rectangle(overlay, p1, p2, color, -1)
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)


def transparent_polygon(frame, polygon, color, alpha):
    overlay = frame.copy()
    cv2.fillConvexPoly(overlay, polygon, color, cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)


def draw_center_reticle(frame):
    cx, cy = FRAME_W // 2, FRAME_H // 2
    gap, size = 4, 11
    for p1, p2 in [
        ((cx - size, cy), (cx - gap, cy)),
        ((cx + gap, cy), (cx + size, cy)),
        ((cx, cy - size), (cx, cy - gap)),
        ((cx, cy + gap), (cx, cy + size)),
    ]:
        cv2.line(frame, p1, p2, C_DIM, 1, cv2.LINE_AA)


def draw_corner(frame, point, dx, dy, color):
    x, y = point
    length = 13
    cv2.line(frame, (x, y), (x + dx * length, y), color, 2, cv2.LINE_AA)
    cv2.line(frame, (x, y), (x, y + dy * length), color, 2, cv2.LINE_AA)


def draw_target(frame, target):
    if target is None or len(target) < 2:
        return
    x = int(np.clip(round(float(target[0])), 0, FRAME_W - 1))
    y = int(np.clip(round(float(target[1])), 0, FRAME_H - 1))
    cv2.circle(frame, (x, y), 5, C_TEXT, 1, cv2.LINE_AA)
    cv2.line(frame, (x - 9, y), (x - 4, y), C_TEXT, 1, cv2.LINE_AA)
    cv2.line(frame, (x + 4, y), (x + 9, y), C_TEXT, 1, cv2.LINE_AA)
    cv2.line(frame, (x, y - 9), (x, y - 4), C_TEXT, 1, cv2.LINE_AA)
    cv2.line(frame, (x, y + 4), (x, y + 9), C_TEXT, 1, cv2.LINE_AA)


def draw_pose_axis(frame, metadata):
    xyz = metadata.get("xyz")
    rpy = metadata.get("rpy")
    if xyz is None or rpy is None or len(xyz) < 3 or len(rpy) < 3:
        return

    try:
        tvec = np.asarray(xyz[:3], dtype=np.float64).reshape(3, 1)
        if tvec[2, 0] <= 0.01:
            return
        rvec = rpy_to_rotation_vector(rpy)
        if rvec is None:
            return

        axis = np.array([
            [0.0, 0.0, 0.0],
            [AXIS_LENGTH_M, 0.0, 0.0],
            [0.0, AXIS_LENGTH_M, 0.0],
            [0.0, 0.0, AXIS_LENGTH_M],
        ], dtype=np.float64)

        projected, _ = cv2.projectPoints(axis, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
        pts = [(int(round(p[0])), int(round(p[1]))) for p in projected.reshape(-1, 2)]
        origin = pts[0]
        cv2.arrowedLine(frame, origin, pts[1], C_AXIS_X, 2, cv2.LINE_AA, tipLength=0.18)
        cv2.arrowedLine(frame, origin, pts[2], C_AXIS_Y, 2, cv2.LINE_AA, tipLength=0.18)
        cv2.arrowedLine(frame, origin, pts[3], C_AXIS_Z, 2, cv2.LINE_AA, tipLength=0.18)
        cv2.putText(frame, "X", (pts[1][0] + 4, pts[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.34, C_AXIS_X, 1, cv2.LINE_AA)
        cv2.putText(frame, "Y", (pts[2][0] + 4, pts[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.34, C_AXIS_Y, 1, cv2.LINE_AA)
        cv2.putText(frame, "Z", (pts[3][0] + 4, pts[3][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.34, C_AXIS_Z, 1, cv2.LINE_AA)
    except Exception:
        return


def draw_pose_info(frame, metadata, polygon):
    confidence = metadata.get("confidence", metadata.get("conf"))
    xyz = metadata.get("xyz")
    rpy = metadata.get("rpy")
    distance = None
    if xyz is not None and len(xyz) >= 3:
        try:
            x, y, z = [float(v) for v in xyz[:3]]
            distance = math.sqrt(x * x + y * y + z * z)
        except Exception:
            xyz = None

    min_x = int(np.min(polygon[:, 0]))
    min_y = int(np.min(polygon[:, 1]))
    panel_w, panel_h = 215, 76
    px = int(np.clip(min_x, 8, FRAME_W - panel_w - 8))
    py = min_y - panel_h - 8
    if py < 8:
        py = min_y + 10
    py = int(np.clip(py, 8, FRAME_H - panel_h - SYNC_BAR_H - 8))

    transparent_rect(frame, (px, py), (px + panel_w, py + panel_h), C_PANEL, PANEL_ALPHA)
    conf_text = "--" if confidence is None else f"{float(confidence) * 100:.0f}%"
    cv2.putText(frame, f"GATE POSE  {conf_text}", (px + 12, py + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_ACCENT, 1, cv2.LINE_AA)
    if distance is not None:
        cv2.putText(frame, f"DIST {distance:.2f} m", (px + 12, py + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_TEXT, 1, cv2.LINE_AA)
    if xyz is not None:
        cv2.putText(frame, f"X {float(xyz[0]):+.2f}  Y {float(xyz[1]):+.2f}  Z {float(xyz[2]):.2f}", (px + 12, py + 57), cv2.FONT_HERSHEY_SIMPLEX, 0.35, C_DIM, 1, cv2.LINE_AA)
    if rpy is not None and len(rpy) >= 3:
        cv2.putText(frame, f"R {float(rpy[0]):+.1f}  P {float(rpy[1]):+.1f}  Y {float(rpy[2]):+.1f}", (px + 12, py + 71), cv2.FONT_HERSHEY_SIMPLEX, 0.29, C_DIM, 1, cv2.LINE_AA)


def draw_pose_hud(frame, metadata):
    keypoints = metadata.get("keypoints")
    if keypoints is None or len(keypoints) < 4:
        return

    polygon = []
    for point in keypoints[:4]:
        if point is None or len(point) < 2:
            return
        x = int(np.clip(round(float(point[0])), 0, FRAME_W - 1))
        y = int(np.clip(round(float(point[1])), 0, FRAME_H - 1))
        polygon.append([x, y])

    polygon = np.asarray(polygon, dtype=np.int32)
    transparent_polygon(frame, polygon, C_GATE_FILL, GATE_FILL_ALPHA)
    cv2.polylines(frame, [polygon], True, C_GATE, 1, cv2.LINE_AA)
    draw_corner(frame, tuple(polygon[0]), +1, +1, C_ACCENT)
    draw_corner(frame, tuple(polygon[1]), -1, +1, C_ACCENT)
    draw_corner(frame, tuple(polygon[2]), -1, -1, C_ACCENT)
    draw_corner(frame, tuple(polygon[3]), +1, -1, C_ACCENT)
    draw_target(frame, metadata.get("target"))
    draw_pose_axis(frame, metadata)
    draw_pose_info(frame, metadata, polygon)


def draw_detector_hud(frame, metadata):
    bbox = metadata.get("bbox")
    if bbox is None or len(bbox) < 4:
        return

    x1 = int(np.clip(round(float(bbox[0])), 0, FRAME_W - 1))
    y1 = int(np.clip(round(float(bbox[1])), 0, FRAME_H - 1))
    x2 = int(np.clip(round(float(bbox[2])), 0, FRAME_W - 1))
    y2 = int(np.clip(round(float(bbox[3])), 0, FRAME_H - 1))
    transparent_rect(frame, (x1, y1), (x2, y2), C_DET_FILL, 0.09)

    corner = int(max(12, min(30, min(max(1, x2 - x1), max(1, y2 - y1)) * 0.12)))
    for p1, p2 in [
        ((x1, y1), (x1 + corner, y1)), ((x1, y1), (x1, y1 + corner)),
        ((x2, y1), (x2 - corner, y1)), ((x2, y1), (x2, y1 + corner)),
        ((x2, y2), (x2 - corner, y2)), ((x2, y2), (x2, y2 - corner)),
        ((x1, y2), (x1 + corner, y2)), ((x1, y2), (x1, y2 - corner)),
    ]:
        cv2.line(frame, p1, p2, C_DET, 2, cv2.LINE_AA)

    conf = metadata.get("confidence", metadata.get("conf"))
    label = "YOLO26 FALLBACK" if conf is None else f"YOLO26 FALLBACK {float(conf) * 100:.0f}%"
    py = max(5, y1 - 30)
    transparent_rect(frame, (x1, py), (min(FRAME_W - 5, x1 + 190), py + 25), C_PANEL, PANEL_ALPHA)
    cv2.putText(frame, label, (x1 + 8, py + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.37, C_DET, 1, cv2.LINE_AA)
    draw_target(frame, metadata.get("target"))


def draw_camera_badge(frame, camera, metadata):
    title = "FRONT" if camera.name == "front" else "BOTTOM"
    ai_state = "NO META"
    ai_task = "--"
    if metadata is not None:
        ai_state = str(metadata.get("ai_state", "--")).upper()
        ai_task = str(metadata.get("ai_task", "--")).upper()

    color = C_ACTIVE if ai_state == "ACTIVE" else C_STANDBY if ai_state == "STANDBY" else C_DISABLED
    transparent_rect(frame, (9, 9), (280, 42), C_PANEL, 0.72)
    cv2.circle(frame, (21, 25), 4, color, -1, cv2.LINE_AA)
    cv2.putText(frame, f"{title}  AI {ai_state}  {ai_task}", (32, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)


def draw_sync_strip(camera, frame):
    with camera.sync_lock:
        video_id = camera.last_video_frame_id
        meta_id = camera.last_meta_frame_id
        diff = camera.last_sync_diff
        ok_count = camera.sync_ok
        miss_count = camera.sync_miss
        barcode_fail = camera.barcode_fail

    sync_ok = video_id is not None and meta_id is not None and diff == 0
    y1 = FRAME_H - SYNC_BAR_H

    # Intentionally covers the barcode in the GCS display.
    transparent_rect(frame, (0, y1), (FRAME_W - 1, FRAME_H - 1), C_PANEL, 0.96)
    color = C_OK if sync_ok else C_BAD
    cv2.circle(frame, (12, FRAME_H - 12), 3, color, -1, cv2.LINE_AA)

    if sync_ok:
        text = f"BARCODE SYNC OK  F {video_id}  META {meta_id}"
    else:
        text = f"SYNC WAIT  VIDEO {video_id}  META {meta_id}  BF {barcode_fail}"
    cv2.putText(frame, text, (22, FRAME_H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.31, C_DIM, 1, cv2.LINE_AA)


def draw_camera_overlay(camera, frame, metadata):
    draw_center_reticle(frame)
    draw_camera_badge(frame, camera, metadata)
    if metadata is not None:
        ai_state = str(metadata.get("ai_state", "")).upper()
        mode = str(metadata.get("vision_mode", metadata.get("mode", ""))).upper()
        if ai_state == "ACTIVE":
            if mode == "POSE":
                draw_pose_hud(frame, metadata)
            elif mode == "YOLO26":
                draw_detector_hud(frame, metadata)
    draw_sync_strip(camera, frame)
    return frame

# ============================================================
# DEARPYGUI
# ============================================================
def load_background_frame():
    """Load the shared 640x640 placeholder image for both camera views.

    If background.png is missing or unreadable, return a pure-black frame so
    the GCS can still start safely instead of crashing.
    """
    try:
        image = cv2.imread(str(BACKGROUND_IMAGE_PATH), cv2.IMREAD_COLOR)

        if image is None:
            raise RuntimeError(f"Cannot read {BACKGROUND_IMAGE_PATH}")

        if image.shape[1] != FRAME_W or image.shape[0] != FRAME_H:
            image = cv2.resize(
                image,
                (FRAME_W, FRAME_H),
                interpolation=cv2.INTER_AREA,
            )

        print(f"[GUI] Background loaded: {BACKGROUND_IMAGE_PATH}")
        return np.ascontiguousarray(image, dtype=np.uint8)

    except Exception as error:
        print(f"[GUI] Background fallback to black: {error}")
        return np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)


BACKGROUND_FRAME = load_background_frame()


def make_empty_texture():
    # Initial texture is the same background used for WAIT/LOST/SYNC FAIL.
    return bgr_to_texture(BACKGROUND_FRAME.copy())

def bgr_to_texture(frame):
    rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    return (rgba.astype(np.float32) / 255.0).ravel()


def create_theme():
    with dpg.theme(tag="theme_global"):
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (17, 19, 23, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (22, 25, 30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Border, (54, 60, 69, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (221, 226, 233, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (31, 35, 42, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Button, (42, 47, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (57, 64, 75, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (68, 78, 92, 255))
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 8, 8)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 7, 5)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 4)

def create_dark_high_contrast_theme():

    with dpg.theme(
        tag="theme_dark_high_contrast"
    ):

        with dpg.theme_component(
            dpg.mvAll
        ):

            # =================================================
            # MAIN BACKGROUND
            # PURE BLACK
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_WindowBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ChildBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_PopupBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )


            # =================================================
            # TEXT
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_Text,
                (
                    255,
                    255,
                    255,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TextDisabled,
                (
                    135,
                    135,
                    135,
                    255
                )
            )


            # =================================================
            # INPUT / FRAME
            # Slight differentiation from pure-black background
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_FrameBg,
                (
                    10,
                    10,
                    10,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_FrameBgHovered,
                (
                    25,
                    25,
                    25,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_FrameBgActive,
                (
                    40,
                    40,
                    40,
                    255
                )
            )


            # =================================================
            # BORDER / SEPARATOR
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_Border,
                (
                    70,
                    70,
                    70,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_BorderShadow,
                (
                    0,
                    0,
                    0,
                    0
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_Separator,
                (
                    65,
                    65,
                    65,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_SeparatorHovered,
                (
                    110,
                    110,
                    110,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_SeparatorActive,
                (
                    160,
                    160,
                    160,
                    255
                )
            )


            # =================================================
            # BUTTON
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_Button,
                (
                    15,
                    15,
                    15,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ButtonHovered,
                (
                    35,
                    35,
                    35,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ButtonActive,
                (
                    55,
                    55,
                    55,
                    255
                )
            )


            # =================================================
            # TABLE / HEADER
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_Header,
                (
                    12,
                    12,
                    12,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_HeaderHovered,
                (
                    30,
                    30,
                    30,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_HeaderActive,
                (
                    45,
                    45,
                    45,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TableHeaderBg,
                (
                    5,
                    5,
                    5,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TableBorderStrong,
                (
                    70,
                    70,
                    70,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TableBorderLight,
                (
                    35,
                    35,
                    35,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TableRowBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TableRowBgAlt,
                (
                    5,
                    5,
                    5,
                    255
                )
            )


            # =================================================
            # SCROLLBAR
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_ScrollbarBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ScrollbarGrab,
                (
                    60,
                    60,
                    60,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ScrollbarGrabHovered,
                (
                    90,
                    90,
                    90,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ScrollbarGrabActive,
                (
                    125,
                    125,
                    125,
                    255
                )
            )


            # =================================================
            # CHECKBOX / SLIDER / ACCENT
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_CheckMark,
                (
                    60,
                    255,
                    100,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_SliderGrab,
                (
                    220,
                    220,
                    220,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_SliderGrabActive,
                (
                    255,
                    255,
                    255,
                    255
                )
            )


            # =================================================
            # TITLE BAR
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_TitleBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TitleBgActive,
                (
                    0,
                    0,
                    0,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TitleBgCollapsed,
                (
                    0,
                    0,
                    0,
                    255
                )
            )


            # =================================================
            # MENU BAR
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_MenuBarBg,
                (
                    0,
                    0,
                    0,
                    255
                )
            )


            # =================================================
            # TABS
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_Tab,
                (
                    10,
                    10,
                    10,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TabHovered,
                (
                    30,
                    30,
                    30,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TabActive,
                (
                    45,
                    45,
                    45,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TabUnfocused,
                (
                    5,
                    5,
                    5,
                    255
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_TabUnfocusedActive,
                (
                    25,
                    25,
                    25,
                    255
                )
            )


            # =================================================
            # RESIZE GRIP
            # =================================================

            dpg.add_theme_color(
                dpg.mvThemeCol_ResizeGrip,
                (
                    70,
                    70,
                    70,
                    80
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ResizeGripHovered,
                (
                    140,
                    140,
                    140,
                    180
                )
            )

            dpg.add_theme_color(
                dpg.mvThemeCol_ResizeGripActive,
                (
                    255,
                    255,
                    255,
                    220
                )
            )


            # =================================================
            # STYLE
            # =================================================

            dpg.add_theme_style(
                dpg.mvStyleVar_WindowRounding,
                2
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_ChildRounding,
                2
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_FrameRounding,
                3
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_PopupRounding,
                2
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_ScrollbarRounding,
                2
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_GrabRounding,
                2
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_TabRounding,
                2
            )


            # =================================================
            # SPACING
            # =================================================

            dpg.add_theme_style(
                dpg.mvStyleVar_WindowPadding,
                8,
                8
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_FramePadding,
                8,
                5
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_ItemSpacing,
                8,
                6
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_ItemInnerSpacing,
                6,
                4
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_IndentSpacing,
                18
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_ScrollbarSize,
                12
            )


            # =================================================
            # BORDER SIZE
            # =================================================

            dpg.add_theme_style(
                dpg.mvStyleVar_WindowBorderSize,
                1
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_ChildBorderSize,
                1
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_FrameBorderSize,
                1
            )

            dpg.add_theme_style(
                dpg.mvStyleVar_PopupBorderSize,
                1
            )
            
def section_title(text):
    dpg.add_text(text, color=(135, 170, 220, 255))
    dpg.add_separator()


def add_value_row(label, value, tag):
    with dpg.table(header_row=False, policy=dpg.mvTable_SizingStretchProp,
                   borders_innerH=False, borders_outerH=False,
                   borders_innerV=False, borders_outerV=False):
        dpg.add_table_column(init_width_or_weight=0.50)
        dpg.add_table_column(init_width_or_weight=0.50)
        with dpg.table_row():
            dpg.add_text(label)
            dpg.add_text(value, tag=tag)


def snapshot(camera):
    frame, meta, frame_id = get_camera_frame(camera)
    if frame is None:
        return
    draw_camera_overlay(camera, frame, meta)
    name = f"{camera.name}_{time.strftime('%Y%m%d_%H%M%S')}_{frame_id}.jpg"
    cv2.imwrite(str(SNAPSHOT_DIR / name), frame)


# ============================================================
# WAYPOINT EDIT CONTROL
# ============================================================
edit_mode_unlocked = False

WP_OPTIONS = [str(i) for i in range(1, 13)]
POS_STRAT_OPTIONS = ["LEFT", "RIGHT"]


def unlock_edit_mode_callback():
    """Toggle waypoint edit protection.

    SAVE WAYPOINT stays disabled until the operator explicitly
    unlocks edit mode. Raspberry Pi parameter writes will be wired
    into this protected path later.
    """
    global edit_mode_unlocked

    edit_mode_unlocked = not edit_mode_unlocked

    if edit_mode_unlocked:
        dpg.enable_item("save_waypoint_button")
        dpg.set_item_label("unlock_edit_button", "LOCK EDIT MODE")
        dpg.bind_item_theme("unlock_edit_button", "theme_button_unlocked")
        dpg.set_value("waypoint_edit_state", "EDIT MODE UNLOCKED")
        dpg.configure_item("waypoint_edit_state", color=(255, 90, 90, 255))
        dpg.set_value("waypoint_save_status", "EDIT ENABLED - WAYPOINT CAN BE SAVED")
    else:
        dpg.disable_item("save_waypoint_button")
        dpg.set_item_label("unlock_edit_button", "UNLOCK EDIT MODE")
        dpg.bind_item_theme("unlock_edit_button", "theme_button_unlock")
        dpg.set_value("waypoint_edit_state", "EDIT MODE LOCKED")
        dpg.configure_item("waypoint_edit_state", color=(255, 170, 70, 255))
        dpg.set_value("waypoint_save_status", "LOCKED - UNLOCK EDIT MODE TO SAVE")


def save_waypoint_callback():
    """Protected placeholder for future Raspberry Pi waypoint writes.

    No flight-control or Raspberry Pi parameter write is performed yet.
    The UI already enforces the future safety gate: edit mode must be
    unlocked before SAVE WAYPOINT can be used.
    """
    if not edit_mode_unlocked:
        dpg.set_value("waypoint_save_status", "BLOCKED - EDIT MODE IS LOCKED")
        return

    selected_wp = dpg.get_value("waypoint_selector")
    pos_strat = dpg.get_value("pos_strat_selector")
    current_wp = dpg.get_value("waypoint_current_value")

    print(
        f"[WAYPOINT] SAVE requested | WP={selected_wp} | "
        f"POS_STRAT={pos_strat} | current={current_wp} | "
        f"PLACEHOLDER ONLY"
    )

    dpg.set_value(
        "waypoint_save_status",
        f"READY FOR PI WRITE - WP {selected_wp} / {pos_strat}"
    )


def mission_strategy_callback(strategy):
    """Placeholder strategy selector; no command is sent to the drone yet."""
    print(f"[MISSION] STRATEGY button pressed: {strategy} | PLACEHOLDER ONLY")
    dpg.set_value("mission_state", f"STRAT {strategy} SELECTED")


def create_waypoint_button_themes():
    """High-contrast button colors for outdoor operation."""

    # STRAT L = bright cyan
    with dpg.theme(tag="theme_strat_l"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 150, 210, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (0, 195, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (70, 220, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    # STRAT R = bright magenta
    with dpg.theme(tag="theme_strat_r"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (195, 35, 150, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (235, 55, 185, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255, 90, 210, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    # SAVE = green
    with dpg.theme(tag="theme_button_save"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (18, 125, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (28, 170, 75, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (40, 210, 95, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    # UNLOCK = red (danger/protected action)
    with dpg.theme(tag="theme_button_unlock"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (150, 20, 20, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (205, 28, 28, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (245, 45, 45, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    # When unlocked, the same button becomes amber so the operator can
    # immediately see that edit protection is currently OFF.
    with dpg.theme(tag="theme_button_unlocked"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (175, 90, 5, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (220, 120, 10, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (250, 145, 20, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)


def create_gui():
    dpg.create_context()
    #create_theme()
    create_dark_high_contrast_theme()
    create_waypoint_button_themes()

    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(
            FRAME_W,
            FRAME_H,
            make_empty_texture(),
            tag="texture_front",
        )
        dpg.add_dynamic_texture(
            FRAME_W,
            FRAME_H,
            make_empty_texture(),
            tag="texture_bottom",
        )

    # ========================================================
    # MAIN WINDOW
    # ========================================================
    with dpg.window(
        tag="main_window",
        no_title_bar=True,
        no_move=True,
        no_resize=True,
        no_collapse=True,
        no_scrollbar=True,
        no_scroll_with_mouse=True,
    ):

        # ----------------------------------------------------
        # TOP HEADER
        # ----------------------------------------------------
        with dpg.group(horizontal=True, tag="header_group"):
            dpg.add_text("KINGPHOENIX GCS", color=(150, 205, 255, 255))
            dpg.add_text("2026 V5")
            dpg.add_spacer(tag="header_spacer", width=500)
            dpg.add_text("DUAL VIDEO / BARCODE SYNC", color=(145, 180, 220, 255))

        dpg.add_separator()

        # ----------------------------------------------------
        # TOP / MAIN AREA
        # ----------------------------------------------------
        with dpg.group(horizontal=True, tag="main_top_group"):

            # FRONT CAMERA CARD
            with dpg.child_window(
                tag="front_camera_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                with dpg.group(horizontal=True):
                    dpg.add_text("FRONT CAMERA", color=(155, 205, 255, 255))
                    dpg.add_spacer(width=8)
                    dpg.add_text("LIVE", tag="front_live_badge", color=(95, 230, 120, 255))

                dpg.add_separator()

                dpg.add_image(
                    "texture_front",
                    tag="front_image",
                    width=500,
                    height=500,
                )

                with dpg.group(horizontal=True, tag="front_status_row"):
                    dpg.add_text("VIDEO --", tag="front_status_video")
                    dpg.add_text("|")
                    dpg.add_text("AI --", tag="front_status_ai")
                    dpg.add_text("|")
                    dpg.add_text("SYNC --", tag="front_status_sync")

                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Snapshot Front",
                        width=125,
                        callback=lambda: snapshot(front),
                    )
                    dpg.add_text("F -- / M --", tag="front_compact_sync", color=(170, 180, 190, 255))

            # BOTTOM CAMERA CARD
            with dpg.child_window(
                tag="bottom_camera_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                with dpg.group(horizontal=True):
                    dpg.add_text("BOTTOM CAMERA", color=(155, 205, 255, 255))
                    dpg.add_spacer(width=8)
                    dpg.add_text("LIVE", tag="bottom_live_badge", color=(95, 230, 120, 255))

                dpg.add_separator()

                dpg.add_image(
                    "texture_bottom",
                    tag="bottom_image",
                    width=500,
                    height=500,
                )

                with dpg.group(horizontal=True, tag="bottom_status_row"):
                    dpg.add_text("VIDEO --", tag="bottom_status_video")
                    dpg.add_text("|")
                    dpg.add_text("AI --", tag="bottom_status_ai")
                    dpg.add_text("|")
                    dpg.add_text("SYNC --", tag="bottom_status_sync")

                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Snapshot Bottom",
                        width=125,
                        callback=lambda: snapshot(bottom),
                    )
                    dpg.add_text("F -- / M --", tag="bottom_compact_sync", color=(170, 180, 190, 255))

            # RIGHT INFORMATION PANEL
            with dpg.child_window(
                tag="right_info_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                # Connection area kept at the top and compact.
                with dpg.group(horizontal=True):
                    dpg.add_text(
                        "DRONE CONNECTION",
                        color=(135, 190, 245, 255)
                    )
                    dpg.add_spacer(width=8)
                    dpg.add_text(
                        "DISCONNECTED",
                        tag="connection_badge",
                        color=(230, 120, 95, 255)
                    )

                dpg.add_separator()

                with dpg.group(horizontal=True):
                    dpg.add_input_text(
                        tag="drone_ip_input",
                        default_value=DEFAULT_DRONE_IP,
                        width=180,
                    )
                    dpg.add_button(
                        label="Connect",
                        tag="connect_button",
                        width=105,
                        callback=connect_button_callback,
                    )

                dpg.add_text(
                    "Not connected",
                    tag="connection_message"
                )

                # ------------------------------------------------
                # RIGHT PANEL 2-COLUMN GRID
                #
                # Row 1 : Vision / System
                # Row 2 : Pose / Frame Sync
                # Row 3 : MAVLink / Flight Telemetry
                #
                # MAVLink is intentionally moved here so the empty
                # area below ACTIVE POSE / FRAME SYNC is fully used.
                # ------------------------------------------------
                with dpg.table(
                    tag="right_metrics_table",
                    header_row=False,
                    policy=dpg.mvTable_SizingStretchProp,
                    borders_innerV=True,
                    borders_outerV=False,
                    borders_innerH=False,
                    borders_outerH=False,
                    resizable=False,
                ):
                    dpg.add_table_column(init_width_or_weight=0.50)
                    dpg.add_table_column(init_width_or_weight=0.50)

                    # ROW 1
                    with dpg.table_row():
                        with dpg.table_cell():
                            section_title("VISION / AI")
                            add_value_row(
                                "Active Camera",
                                "FRONT",
                                "value_active_camera"
                            )
                            add_value_row(
                                "AI Task",
                                "--",
                                "value_ai_task"
                            )
                            add_value_row(
                                "AI Model",
                                "--",
                                "value_ai_model"
                            )
                            add_value_row(
                                "Vision Mode",
                                "--",
                                "value_vision_mode"
                            )

                        with dpg.table_cell():
                            section_title("SYSTEM")
                            add_value_row(
                                "CPU Temp",
                                "--",
                                "value_cpu_temp"
                            )
                            add_value_row(
                                "Front Video",
                                "--",
                                "value_front_fps"
                            )
                            add_value_row(
                                "Bottom Video",
                                "--",
                                "value_bottom_fps"
                            )
                            add_value_row(
                                "Model FPS",
                                "--",
                                "value_model_fps"
                            )
                            add_value_row(
                                "Model Time",
                                "--",
                                "value_model_ms"
                            )

                    # ROW 2
                    with dpg.table_row():
                        with dpg.table_cell():
                            section_title("ACTIVE POSE")
                            add_value_row(
                                "Distance",
                                "--",
                                "value_distance"
                            )
                            add_value_row(
                                "Position XYZ",
                                "--",
                                "value_xyz"
                            )
                            add_value_row(
                                "Roll Pitch Yaw",
                                "--",
                                "value_rpy"
                            )
                            add_value_row(
                                "Confidence",
                                "--",
                                "value_confidence"
                            )

                        with dpg.table_cell():
                            section_title("FRAME SYNC")
                            add_value_row(
                                "Front",
                                "--",
                                "value_front_sync"
                            )
                            add_value_row(
                                "Bottom",
                                "--",
                                "value_bottom_sync"
                            )
                            add_value_row(
                                "PC IP",
                                PC_IP,
                                "value_pc_ip"
                            )
                            add_value_row(
                                "Drone IP",
                                "--",
                                "value_drone_ip"
                            )

                    # ROW 3 - MOVED FROM THE BOTTOM PANEL
                    with dpg.table_row():
                        with dpg.table_cell():
                            section_title("MAVLINK / MAVPROXY")
                            add_value_row(
                                "Link",
                                "NOT CONNECTED",
                                "mav_link"
                            )
                            add_value_row(
                                "Flight Mode",
                                "--",
                                "mav_mode"
                            )
                            add_value_row(
                                "Armed",
                                "--",
                                "mav_armed"
                            )
                            add_value_row(
                                "Altitude",
                                "--",
                                "mav_altitude"
                            )
                            add_value_row(
                                "Ground Speed",
                                "--",
                                "mav_speed"
                            )

                        with dpg.table_cell():
                            section_title("FLIGHT TELEMETRY")
                            add_value_row(
                                "Battery",
                                "--",
                                "mav_battery"
                            )
                            add_value_row(
                                "GPS",
                                "--",
                                "mav_gps"
                            )
                            add_value_row(
                                "Position",
                                "--",
                                "mav_position"
                            )
                            add_value_row(
                                "Attitude",
                                "--",
                                "mav_attitude"
                            )
                            add_value_row(
                                "Waypoint",
                                "--",
                                "mav_waypoint"
                            )

        # ----------------------------------------------------
        # BOTTOM MISSION / WAYPOINT AREA
        # ----------------------------------------------------
        dpg.add_spacer(height=4)

        with dpg.child_window(
            tag="bottom_status_card",
            border=True,
            no_scrollbar=True,
            no_scroll_with_mouse=True,
        ):
            with dpg.table(
                tag="bottom_status_table",
                header_row=False,
                policy=dpg.mvTable_SizingStretchProp,
                borders_innerV=True,
                borders_outerV=False,
                borders_innerH=False,
                borders_outerH=False,
                resizable=False,
            ):
                dpg.add_table_column(init_width_or_weight=0.44)
                dpg.add_table_column(init_width_or_weight=0.56)

                with dpg.table_row():

                    # ------------------------------------------------
                    # MISSION SUMMARY
                    # ------------------------------------------------
                    with dpg.table_cell():
                        section_title("MISSION")
                        add_value_row(
                            "Mission Phase",
                            "TEST_DUAL_VIDEO",
                            "mission_phase"
                        )
                        add_value_row(
                            "AI Camera",
                            "FRONT",
                            "mission_ai_camera"
                        )
                        add_value_row(
                            "Vision Task",
                            "GATE_NAV",
                            "mission_ai_task"
                        )
                        add_value_row(
                            "Mission State",
                            "WAITING MAVLINK",
                            "mission_state"
                        )

                        # Strategy controls (placeholder for future mission logic).
                        dpg.add_spacer(height=8)
                        with dpg.group(horizontal=True):
                            dpg.add_button(
                                label="STRAT L",
                                tag="mission_strat_l_button",
                                width=145,
                                height=42,
                                callback=lambda: mission_strategy_callback("LEFT"),
                            )
                            dpg.add_button(
                                label="STRAT R",
                                tag="mission_strat_r_button",
                                width=145,
                                height=42,
                                callback=lambda: mission_strategy_callback("RIGHT"),
                            )
                        dpg.bind_item_theme("mission_strat_l_button", "theme_strat_l")
                        dpg.bind_item_theme("mission_strat_r_button", "theme_strat_r")

                    # ------------------------------------------------
                    # WAYPOINT CONTROL
                    # ------------------------------------------------
                    with dpg.table_cell():
                        section_title("WAYPOINT CONTROL")

                        # WP target + position strategy on one compact row.
                        with dpg.group(horizontal=True):
                            dpg.add_text("WP:")
                            dpg.add_combo(
                                WP_OPTIONS,
                                default_value="1",
                                tag="waypoint_selector",
                                width=105,
                            )

                            dpg.add_spacer(width=12)

                            dpg.add_text("POS STRAT:")
                            dpg.add_combo(
                                POS_STRAT_OPTIONS,
                                default_value="LEFT",
                                tag="pos_strat_selector",
                                width=125,
                            )

                        dpg.add_spacer(height=5)

                        # Only current mission waypoint is shown here.
                        # XYZ and Mission Step have intentionally been removed.
                        add_value_row(
                            "Current Waypoint",
                            "--",
                            "waypoint_current_value"
                        )

                        dpg.add_spacer(height=6)

                        # SAVE is protected by edit-mode lock. It starts disabled.
                        dpg.add_button(
                            label="SAVE WAYPOINT",
                            tag="save_waypoint_button",
                            width=-1,
                            height=50,
                            callback=save_waypoint_callback,
                            enabled=False,
                        )
                        dpg.bind_item_theme(
                            "save_waypoint_button",
                            "theme_button_save"
                        )

                        dpg.add_spacer(height=5)

                        # Explicit red protected-action button.
                        dpg.add_button(
                            label="UNLOCK EDIT MODE",
                            tag="unlock_edit_button",
                            width=-1,
                            height=42,
                            callback=unlock_edit_mode_callback,
                        )
                        dpg.bind_item_theme(
                            "unlock_edit_button",
                            "theme_button_unlock"
                        )

                        with dpg.group(horizontal=True):
                            dpg.add_text(
                                "EDIT MODE LOCKED",
                                tag="waypoint_edit_state",
                                color=(255, 170, 70, 255),
                            )
                            dpg.add_text(" | ")
                            dpg.add_text(
                                "LOCKED - UNLOCK EDIT MODE TO SAVE",
                                tag="waypoint_save_status",
                                color=(180, 190, 200, 255),
                            )

        # Thin footer: useful for seeing the architecture without taking
        # vertical room from the video cards.
        with dpg.group(horizontal=True, tag="footer_group"):
            dpg.add_text("GCS STATUS:", color=(130, 180, 230, 255))
            dpg.add_text("READY", tag="footer_gcs_status", color=(100, 220, 120, 255))
            dpg.add_spacer(tag="footer_spacer_1", width=150)
            dpg.add_text("SYNC: PIXEL_FRAME_ID_V1 / 48-bit")
            dpg.add_spacer(tag="footer_spacer_2", width=150)
            dpg.add_text("PORTS 14560 / 14561 / 14562 / 14563 / 14564")

    # Start maximized. DearPyGUI will use the user's desktop resolution.
    dpg.create_viewport(
        title=APP_TITLE,
        width=WINDOW_W,
        height=WINDOW_H,
        resizable=True,
    )
    dpg.setup_dearpygui()
    #dpg.bind_theme("theme_global")
    dpg.bind_theme("theme_dark_high_contrast")
    dpg.set_primary_window("main_window", True)
    dpg.show_viewport()

    try:
        dpg.maximize_viewport()
    except Exception:
        pass

    apply_responsive_layout(force=True)


# ============================================================
# RESPONSIVE FULL-SCREEN LAYOUT
# ============================================================
_last_layout_size = (0, 0)


def apply_responsive_layout(force=False):
    """
    Resize all major cards from the actual viewport client area.

    Goals:
      * no vertical scrolling in full-screen mode
      * both video status rows are always visible
      * video stays square because source is 640x640
      * right telemetry panel uses available horizontal space
      * bottom mission/waypoint block remains permanently visible
    """
    global _last_layout_size

    try:
        vw = int(dpg.get_viewport_client_width())
        vh = int(dpg.get_viewport_client_height())
    except Exception:
        return

    if vw <= 100 or vh <= 100:
        return

    if not force and (vw, vh) == _last_layout_size:
        return

    _last_layout_size = (vw, vh)

    # Keep the primary window exactly pinned to the viewport.
    try:
        dpg.configure_item("main_window", pos=(0, 0), width=vw, height=vh)
    except Exception:
        pass

    # Overall geometry.
    outer_margin = 10
    gap = 8
    header_h = 34
    footer_h = 24

    # Reserve more vertical space for the bottom Mission / Waypoint block.
    # This intentionally pulls the camera + telemetry cards slightly upward
    # so the large SAVE WAYPOINT button and status stay fully visible.
    bottom_h = int(np.clip(vh * 0.23, 215, 250))

    top_h = (
        vh
        - header_h
        - footer_h
        - bottom_h
        - outer_margin * 2
        - gap * 3
    )
    top_h = max(400, top_h)

    # Camera cards get ~31% each, right panel gets the remainder.
    usable_w = vw - outer_margin * 2 - gap * 2
    camera_w = int(usable_w * 0.315)
    right_w = usable_w - camera_w * 2

    # Keep sensible minimums for laptop resolutions.
    camera_w = max(390, camera_w)
    right_w = max(430, right_w)

    # If minimums overflow, distribute proportionally.
    total_w = camera_w * 2 + right_w
    if total_w > usable_w:
        scale = usable_w / total_w
        camera_w = int(camera_w * scale)
        right_w = usable_w - camera_w * 2

    # Space inside each camera card:
    # title+separator ~35, status ~26, snapshot row ~34, padding ~20.
    camera_chrome_h = 118
    max_image_by_h = max(250, top_h - camera_chrome_h)
    max_image_by_w = max(250, camera_w - 24)
    image_size = int(min(max_image_by_h, max_image_by_w))

    # Main cards.
    dpg.configure_item("front_camera_card", width=camera_w, height=top_h)
    dpg.configure_item("bottom_camera_card", width=camera_w, height=top_h)
    dpg.configure_item("right_info_card", width=right_w, height=top_h)

    # Camera images. Square 640x640 source remains undistorted.
    dpg.configure_item("front_image", width=image_size, height=image_size)
    dpg.configure_item("bottom_image", width=image_size, height=image_size)

    # Bottom area always visible without scroll.
    dpg.configure_item("bottom_status_card", height=bottom_h)

    # Header/footer spacers adapt to available width.
    header_spacer = max(20, vw - 520)
    dpg.configure_item("header_spacer", width=header_spacer)

    footer_space = max(20, int((vw - 850) / 2))
    dpg.configure_item("footer_spacer_1", width=footer_space)
    dpg.configure_item("footer_spacer_2", width=footer_space)

# ============================================================
# GUI UPDATE
# ============================================================
last_gui_connection_state = None

def update_connection_ui():
    global last_gui_connection_state
    with connection_lock:
        state = connection_state
        message = connection_message

    if state != last_gui_connection_state:
        last_gui_connection_state = state
        if state == "CONNECTED":
            dpg.set_item_label("connect_button", "Disconnect")
            dpg.disable_item("drone_ip_input")
        elif state == "CONNECTING":
            dpg.set_item_label("connect_button", "Connecting...")
        else:
            dpg.set_item_label("connect_button", "Connect")
            dpg.enable_item("drone_ip_input")
        dpg.set_value("connection_message", message)

        if state == "CONNECTED":
            dpg.set_value("connection_badge", "CONNECTED")
            dpg.configure_item("connection_badge", color=(95, 230, 120, 255))
        elif state == "CONNECTING":
            dpg.set_value("connection_badge", "CONNECTING")
            dpg.configure_item("connection_badge", color=(240, 195, 90, 255))
        else:
            dpg.set_value("connection_badge", "DISCONNECTED")
            dpg.configure_item("connection_badge", color=(230, 120, 95, 255))

    dpg.set_value("value_drone_ip", drone_ip if is_connected else "--")


def update_texture(camera, texture_tag):
    """Render live video only while the stream is healthy and synchronized.

    FRONT and BOTTOM both fall back to background.png when:
      * disconnected / waiting for first frame
      * no decoded frame has arrived recently
      * barcode/metadata synchronization is invalid beyond the grace period

    The short sync grace keeps a single damaged barcode frame from causing
    distracting flicker outdoors. No stale AI overlay is ever drawn.
    """
    now = time.monotonic()
    frame, meta, frame_id = get_camera_frame(camera)

    # --------------------------------------------------------
    # VIDEO HEALTH
    # --------------------------------------------------------
    video_live = (
        is_connected
        and frame is not None
        and camera.last_video_time > 0.0
        and (now - camera.last_video_time) < VIDEO_TIMEOUT_SEC
    )

    # --------------------------------------------------------
    # EXACT BARCODE/METADATA SYNC HEALTH
    # --------------------------------------------------------
    with camera.sync_lock:
        exact_sync = (
            camera.last_video_frame_id is not None
            and camera.last_meta_frame_id is not None
            and camera.last_sync_diff == 0
        )
        last_good_sync_time = camera.last_good_sync_time

    if exact_sync:
        sync_healthy = True
    elif last_good_sync_time > 0.0:
        sync_healthy = (
            now - last_good_sync_time
        ) <= SYNC_FAIL_GRACE_SEC
    else:
        sync_healthy = False

    # --------------------------------------------------------
    # PLACEHOLDER
    # --------------------------------------------------------
    if not video_live or not sync_healthy:
        dpg.set_value(
            texture_tag,
            bgr_to_texture(BACKGROUND_FRAME)
        )
        return

    # --------------------------------------------------------
    # LIVE FRAME
    # --------------------------------------------------------
    # During the tiny sync grace window meta may be None. In that case we
    # show the current video without AI graphics rather than reusing stale
    # metadata from another frame.
    display_frame = frame.copy()
    draw_camera_overlay(camera, display_frame, meta)

    dpg.set_value(
        texture_tag,
        bgr_to_texture(display_frame)
    )


def update_camera_status(camera, prefix):
    now = time.monotonic()
    video_live = camera.last_video_time > 0 and (now - camera.last_video_time) < VIDEO_TIMEOUT_SEC
    raw_meta = get_latest_raw_metadata(camera)

    dpg.set_value(f"{prefix}_status_video", "VIDEO LIVE" if video_live else "VIDEO WAIT")
    ai_state = "--" if raw_meta is None else str(raw_meta.get("ai_state", "--")).upper()
    dpg.set_value(f"{prefix}_status_ai", f"AI {ai_state}")

    with camera.sync_lock:
        sync = (
            camera.last_video_frame_id is not None
            and camera.last_meta_frame_id is not None
            and camera.last_sync_diff == 0
        )
        video_id = camera.last_video_frame_id
        meta_id = camera.last_meta_frame_id
        ok = camera.sync_ok
        miss = camera.sync_miss
        fail = camera.barcode_fail

    dpg.set_value(f"{prefix}_status_sync", "SYNC OK" if sync else "SYNC WAIT")
    dpg.set_value(
        f"value_{prefix}_sync",
        f"F {video_id} / M {meta_id} | {ok} OK {miss} MISS {fail} BF",
    )

    compact = f"F {video_id} / M {meta_id}"
    dpg.set_value(f"{prefix}_compact_sync", compact)


def get_active_metadata():
    # Front only in this version. Prefer the EXACTLY MATCHED metadata so
    # values in the panel correspond to the actual displayed frame.
    with front.frame_lock:
        matched = front.latest_metadata
    if matched is not None:
        return matched
    return get_latest_raw_metadata(front)


def update_info_panel():
    meta = get_active_metadata()
    if meta is None:
        for tag in [
            "value_ai_task", "value_ai_model", "value_vision_mode",
            "value_distance", "value_xyz", "value_rpy", "value_confidence",
            "value_cpu_temp", "value_model_fps", "value_model_ms",
        ]:
            dpg.set_value(tag, "--")
        return

    dpg.set_value("value_active_camera", str(meta.get("camera", "front")).upper())
    dpg.set_value("value_ai_task", str(meta.get("ai_task", "--")))
    dpg.set_value("value_ai_model", str(meta.get("ai_model", "--")))
    dpg.set_value("value_vision_mode", str(meta.get("vision_mode", meta.get("mode", "--"))))

    conf = meta.get("confidence", meta.get("conf"))
    dpg.set_value("value_confidence", "--" if conf is None else f"{float(conf) * 100:.1f}%")

    xyz = meta.get("xyz")
    if xyz is not None and len(xyz) >= 3:
        x, y, z = [float(v) for v in xyz[:3]]
        dist = math.sqrt(x * x + y * y + z * z)
        dpg.set_value("value_distance", f"{dist:.2f} m")
        dpg.set_value("value_xyz", f"{x:+.2f}, {y:+.2f}, {z:+.2f} m")
    else:
        dpg.set_value("value_distance", "--")
        dpg.set_value("value_xyz", "--")

    rpy = meta.get("rpy")
    if rpy is not None and len(rpy) >= 3:
        dpg.set_value("value_rpy", f"{float(rpy[0]):+.1f}, {float(rpy[1]):+.1f}, {float(rpy[2]):+.1f} deg")
    else:
        dpg.set_value("value_rpy", "--")

    cpu = meta.get("cpu_temp")
    model_fps = meta.get("model_fps")
    model_ms = meta.get("model_ms")
    dpg.set_value("value_cpu_temp", "--" if cpu is None else f"{float(cpu):.1f} C")
    dpg.set_value("value_model_fps", "--" if model_fps is None else f"{float(model_fps):.1f} FPS")
    dpg.set_value("value_model_ms", "--" if model_ms is None else f"{float(model_ms):.1f} ms")

    dpg.set_value("mission_ai_camera", "FRONT")
    dpg.set_value("mission_ai_task", str(meta.get("ai_task", "GATE_NAV")))


def update_gui():
    apply_responsive_layout()
    update_connection_ui()
    update_texture(front, "texture_front")
    update_texture(bottom, "texture_bottom")
    update_camera_status(front, "front")
    update_camera_status(bottom, "bottom")
    update_info_panel()

    dpg.set_value("value_front_fps", "--" if front.video_fps <= 0 else f"{front.video_fps:.1f} FPS")
    dpg.set_value("value_bottom_fps", "--" if bottom.video_fps <= 0 else f"{bottom.video_fps:.1f} FPS")

# ============================================================
# THREAD STARTUP / SHUTDOWN
# ============================================================
def metadata_diagnostics_loop():
    while not shutdown_event.is_set():
        if is_connected:
            for camera in (front, bottom):
                print(
                    f"[META-RX:{camera.name}] "
                    f"packets={camera.meta_rx_packets} "
                    f"last_sender={camera.meta_rx_last_sender} "
                    f"bad_json={camera.meta_rx_bad_json} "
                    f"wrong_camera={camera.meta_rx_wrong_camera} "
                    f"latest={'YES' if camera.latest_raw_metadata is not None else 'NO'}"
                )
        time.sleep(2.0)


def start_background_threads():
    for camera in (front, bottom):
        camera.meta_thread = threading.Thread(target=metadata_receiver_loop, args=(camera,), daemon=True)
        camera.video_thread = threading.Thread(target=video_receiver_loop, args=(camera,), daemon=True)
        camera.meta_thread.start()
        camera.video_thread.start()

    threading.Thread(target=heartbeat_loop, daemon=True).start()
    threading.Thread(target=metadata_diagnostics_loop, daemon=True).start()


def shutdown():
    global is_connected
    if is_connected:
        try:
            disconnect_drone()
        except Exception:
            pass
    is_connected = False
    shutdown_event.set()
    close_camera_container(front)
    close_camera_container(bottom)
    try:
        control_socket.close()
    except Exception:
        pass

# ============================================================
# MAIN
# ============================================================
def main():
    print("\n======================================================")
    print(" KINGPHOENIX GCS 2026 V5 DUAL BARCODE")
    print("======================================================")
    print(f"PC Name          : {PC_NAME}")
    print(f"PC IP            : {PC_IP}")
    print(f"FRONT Video      : UDP MPEG-TS :{FRONT_VIDEO_PORT}")
    print(f"FRONT Metadata   : UDP JSON    :{FRONT_METADATA_PORT}")
    print(f"Control          : UDP JSON    :{CONTROL_PORT}")
    print(f"BOTTOM Video     : UDP MPEG-TS :{BOTTOM_VIDEO_PORT}")
    print(f"BOTTOM Metadata  : UDP JSON    :{BOTTOM_METADATA_PORT}")
    print("Sync             : 48-cell PIXEL FRAME ID")
    print("AI               : FRONT ONLY")
    print(f"Background       : {BACKGROUND_IMAGE_PATH}")
    print(f"Sync fail grace  : {SYNC_FAIL_GRACE_SEC:.2f} s")
    print("======================================================")

    start_background_threads()
    create_gui()

    try:
        while dpg.is_dearpygui_running():
            update_gui()
            dpg.render_dearpygui_frame()
    finally:
        shutdown()
        dpg.destroy_context()


if __name__ == "__main__":
    main()
