#!/usr/bin/env python3
# ============================================================
# KINGPHOENIX GCS V2.0.2 - WIDE OPERATOR LAYOUT
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
FRAME_H = 400
DISPLAY_W = 500
DISPLAY_H = 312

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

gui_runtime_error_count = 0
last_gui_runtime_error = None
disconnect_in_progress = False

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
    """Disconnect the drone link without closing the GCS application."""
    global is_connected, disconnect_in_progress

    if disconnect_in_progress:
        return

    disconnect_in_progress = True
    try:
        # Mark disconnected first so RX loops stop trying to consume/reopen
        # stream resources while we clear the current session.
        is_connected = False
        set_connection_state("DISCONNECTING", "Disconnecting...")

        try:
            with control_socket_lock:
                control_socket.sendto(
                    json.dumps({"type": "DISCONNECT"}).encode("utf-8"),
                    (drone_ip, CONTROL_PORT),
                )
        except Exception as error:
            print(f"[CONTROL] DISCONNECT send warning: {error}")

        # Closing PyAV containers can race with demux/decode threads.
        # They already check is_connected and will leave their loops.
        close_camera_container(front)
        close_camera_container(bottom)

        # Clear state under each CameraState lock only after the session has
        # been marked disconnected. GCS remains alive and shows placeholders.
        clear_runtime_state(front)
        clear_runtime_state(bottom)

        set_connection_state("DISCONNECTED", "Not connected")
        print("[CONTROL] DISCONNECTED - GCS remains running")

    finally:
        disconnect_in_progress = False


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
    distance = metadata.get("distance_m")
    if xyz is not None and len(xyz) >= 3:
        try:
            x, y, z = [float(v) for v in xyz[:3]]
            if distance is None:
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
    ex = metadata.get("ex")
    ey = metadata.get("ey")
    label = "YOLO26" if conf is None else f"YOLO26 {float(conf) * 100:.0f}%"
    if ex is not None and ey is not None:
        label += f"  EX {float(ex):+.2f} EY {float(ey):+.2f}"
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
        # V1.9.2.2:
        # Draw from the actual per-frame geometry payload. Router labels are
        # informational only and must never suppress valid keypoints/bboxes.
        bbox = metadata.get("bbox")
        if bbox is not None and len(bbox) >= 4:
            draw_detector_hud(frame, metadata)
    draw_sync_strip(camera, frame)
    return frame

# ============================================================
# DEARPYGUI
# ============================================================
def load_background_frame():
    """Load the shared placeholder image for both 640x400 wide camera views.

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
edit_mode_unlocked = False  # True = SETUP MODE

WP_OPTIONS = [str(i) for i in range(1, 13)]
POS_STRAT_OPTIONS = ["LEFT", "RIGHT"]


def _set_operator_mode(setup_mode):
    """Apply the GCS safety interlock between MISSION and SETUP modes.

    SETUP MODE:
      - START L/R, HOLD, ABORT disabled
      - waypoint editing enabled
      - servo OPEN/CLOSE/TEST enabled

    MISSION MODE:
      - START L/R, PRE-FLIGHT CHECK, ABORT enabled
      - waypoint editing locked
      - servo bench/setup controls disabled

    This is currently a GUI interlock only; no vehicle command is sent.
    """
    setup_mode = bool(setup_mode)

    mission_tags = (
        "mission_start_l_button",
        "mission_start_r_button",
        "mission_preflight_button",
        "mission_abort_button",
    )
    servo_tags = (
        "servo_open_button",
        "servo_close_button",
        "servo_test_button",
    )
    setup_edit_tags = (
        "waypoint_selector",
        "pos_strat_selector",
        "save_waypoint_button",
    )

    for tag in mission_tags:
        if setup_mode:
            dpg.disable_item(tag)
        else:
            dpg.enable_item(tag)

    for tag in servo_tags:
        if setup_mode:
            dpg.enable_item(tag)
        else:
            dpg.disable_item(tag)

    for tag in setup_edit_tags:
        if setup_mode:
            dpg.enable_item(tag)
        else:
            dpg.disable_item(tag)

    if setup_mode:
        dpg.set_item_label("unlock_edit_button", "EXIT SETUP MODE")
        dpg.bind_item_theme("unlock_edit_button", "theme_button_unlocked")
        dpg.set_value("waypoint_edit_state", "SETUP MODE ACTIVE")
        dpg.configure_item("waypoint_edit_state", color=(255, 90, 90, 255))
        dpg.set_value(
            "waypoint_save_status",
            "SETUP ACTIVE - WAYPOINT + SERVO ENABLED / MISSION LOCKED"
        )
        dpg.set_value("servo_state", "READY - SETUP MODE")
        dpg.set_value("mission_state", "LOCKED BY SETUP MODE")
        dpg.set_value("operator_mode", "SETUP")
    else:
        dpg.set_item_label("unlock_edit_button", "ENTER SETUP MODE")
        dpg.bind_item_theme("unlock_edit_button", "theme_button_unlock")
        dpg.set_value("waypoint_edit_state", "MISSION MODE")
        dpg.configure_item("waypoint_edit_state", color=(95, 230, 120, 255))
        dpg.set_value(
            "waypoint_save_status",
            "SETUP LOCKED - MISSION CONTROLS ENABLED"
        )
        dpg.set_value("servo_state", "LOCKED - ENTER SETUP MODE")
        dpg.set_value("mission_state", "READY")
        dpg.set_value("operator_mode", "MISSION")


def unlock_edit_mode_callback():
    """Toggle mutually exclusive MISSION MODE <-> SETUP MODE."""
    global edit_mode_unlocked
    edit_mode_unlocked = not edit_mode_unlocked
    _set_operator_mode(edit_mode_unlocked)



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



# ============================================================
# V2.0.2 OPERATOR CONTROL PLACEHOLDERS
# ============================================================
# Layout first. These buttons intentionally DO NOT transmit any MAVLink,
# servo PWM, mission command, or Raspberry Pi actuator command yet.
# They are UI/control-protocol placeholders for the next integration step.


# ============================================================
# PRE-FLIGHT CHECK
# ============================================================
# Current build uses a MOCK SUCCESS result so the final GUI flow can be
# validated before GCS-VCT / Raspberry Pi readiness checks are wired in.

PREFLIGHT_MOCK_ITEMS = [
    ("MAVLink Link", "PASS", "Link healthy"),
    ("FC Heartbeat", "PASS", "Heartbeat fresh"),
    ("Vehicle State", "PASS", "Telemetry fresh"),
    ("GPS", "PASS", "Ready"),
    ("LiDAR / Rangefinder", "PASS", "Fresh"),
    ("Front Camera", "PASS", "Streaming"),
    ("Bottom Camera", "PASS", "Streaming"),
    ("YOLO26 / Hailo", "PASS", "Runtime ready"),
    ("Mission File", "PASS", "Loaded"),
    ("Strategy Data", "PASS", "LEFT / RIGHT available"),
    ("Battery", "PASS", "Voltage healthy"),
    ("Setup Mode", "PASS", "OFF / mission-safe"),
    ("Pi Thermal", "PASS", "Waiting live health"),
    ("Pi Power", "PASS", "Waiting live health"),
    ("Pi Throttling", "PASS", "Waiting live health"),
    ("Pi CPU Load", "PASS", "Waiting live health"),
    ("Pi RAM", "PASS", "Waiting live health"),
    ("Pi Disk", "PASS", "Waiting live health"),
]


def _center_modal(tag, width, height):
    try:
        vw = int(dpg.get_viewport_client_width())
        vh = int(dpg.get_viewport_client_height())
        x = max(0, (vw - width) // 2)
        y = max(0, (vh - height) // 2)
        dpg.configure_item(tag, pos=(x, y), width=width, height=height)
    except Exception:
        pass


def close_preflight_popup():
    dpg.configure_item("preflight_modal", show=False)


def run_preflight_check():
    """PRE-FLIGHT popup.

    Raspberry Pi health rows are LIVE from Pi metadata.
    Remaining flight/mission items stay as current UI placeholders until the
    GCS-VCT control backend is wired.
    """
    if edit_mode_unlocked:
        dpg.set_value("mission_state", "PREFLIGHT BLOCKED - EXIT SETUP MODE")
        return

    print("[PREFLIGHT] CHECK START")
    dpg.set_value("mission_state", "PRE-FLIGHT CHECK RUNNING")

    items = [list(item) for item in PREFLIGHT_MOCK_ITEMS]

    meta = get_active_metadata()
    pi_health = {}
    if isinstance(meta, dict):
        pi_health = meta.get("pi_health") or {}

    checks = pi_health.get("checks", {}) if isinstance(pi_health, dict) else {}
    pi_map = {
        "Pi Thermal": "pi_thermal",
        "Pi Power": "pi_power",
        "Pi Throttling": "pi_throttling",
        "Pi CPU Load": "pi_cpu",
        "Pi RAM": "pi_ram",
        "Pi Disk": "pi_disk",
    }

    for row in items:
        key = pi_map.get(row[0])
        if not key:
            continue
        check = checks.get(key)
        if isinstance(check, dict):
            row[1] = str(check.get("status", "WARN")).upper()
            row[2] = str(check.get("detail", "--"))
        else:
            row[1] = "FAIL"
            row[2] = "No live Pi health telemetry"

    all_ready = True
    has_warn = False

    for i, (_, status, detail) in enumerate(items):
        status = str(status).upper()
        status_tag = f"preflight_status_{i}"
        detail_tag = f"preflight_detail_{i}"

        dpg.set_value(status_tag, status)
        dpg.set_value(detail_tag, detail)

        if status == "PASS":
            dpg.configure_item(status_tag, color=(95, 230, 120, 255))
        elif status == "WARN":
            has_warn = True
            dpg.configure_item(status_tag, color=(245, 190, 70, 255))
        else:
            all_ready = False
            dpg.configure_item(status_tag, color=(245, 80, 80, 255))

    if all_ready:
        verdict = "STATUS: READY" if not has_warn else "STATUS: READY WITH WARNING"
        message = (
            "READY TO ROCK AND ROLL"
            if not has_warn
            else "CHECK WARNINGS BEFORE START"
        )
        color = (
            (95, 230, 120, 255)
            if not has_warn
            else (245, 190, 70, 255)
        )

        dpg.set_value("preflight_verdict", verdict)
        dpg.configure_item("preflight_verdict", color=color)
        dpg.set_value("preflight_message", message)
        dpg.configure_item("preflight_message", color=color)
        dpg.set_value("mission_state", "PREFLIGHT READY")
    else:
        dpg.set_value("preflight_verdict", "STATUS: NOT READY")
        dpg.configure_item(
            "preflight_verdict",
            color=(245, 80, 80, 255),
        )
        dpg.set_value(
            "preflight_message",
            "CHECK FAILED ITEMS BEFORE START",
        )
        dpg.configure_item(
            "preflight_message",
            color=(245, 80, 80, 255),
        )
        dpg.set_value("mission_state", "PREFLIGHT NOT READY")

    dpg.configure_item("preflight_modal", show=True)
    _center_modal("preflight_modal", 710, 720)
    print("[PREFLIGHT] CHECK COMPLETE")



def mission_control_callback(action):
    if edit_mode_unlocked:
        print("[MISSION-UI] BLOCKED - SETUP MODE ACTIVE")
        dpg.set_value("mission_state", "BLOCKED - EXIT SETUP MODE")
        return

    action = str(action).upper()
    print(f"[MISSION-UI] {action} pressed | UI ONLY / NO TX")

    if action == "START_L":
        dpg.set_value("mission_state", "START L REQUESTED - UI ONLY")
        dpg.set_value("mission_selected_strategy", "LEFT")
    elif action == "START_R":
        dpg.set_value("mission_state", "START R REQUESTED - UI ONLY")
        dpg.set_value("mission_selected_strategy", "RIGHT")
    elif action == "PREFLIGHT":
        run_preflight_check()
    elif action == "ABORT":
        dpg.set_value("mission_state", "ABORT REQUESTED - UI ONLY")


def servo_control_callback(action):
    if not edit_mode_unlocked:
        print("[SERVO-UI] BLOCKED - ENTER SETUP MODE")
        dpg.set_value("servo_state", "BLOCKED - ENTER SETUP MODE")
        return

    action = str(action).upper()
    print(f"[SERVO-UI] {action} pressed | SETUP MODE / UI ONLY / NO PWM TX")

    if action == "OPEN":
        dpg.set_value("servo_state", "OPEN REQUESTED - UI ONLY")
    elif action == "CLOSE":
        dpg.set_value("servo_state", "CLOSE REQUESTED - UI ONLY")
    elif action == "TEST":
        dpg.set_value("servo_state", "TEST CYCLE REQUESTED - UI ONLY")


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


    # HOLD = amber
    with dpg.theme(tag="theme_hold"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (165, 100, 10, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (215, 140, 20, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (245, 170, 35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)

    # ABORT = strong red
    with dpg.theme(tag="theme_abort"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (175, 20, 28, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (230, 28, 38, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255, 55, 60, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)

    # Servo controls = neutral blue/green family
    with dpg.theme(tag="theme_servo"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (40, 88, 135, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (55, 120, 180, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (70, 145, 210, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)


def create_gui():
    dpg.create_context()
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

    with dpg.window(
        tag="main_window",
        no_title_bar=True,
        no_move=True,
        no_resize=True,
        no_collapse=True,
        no_scrollbar=True,
        no_scroll_with_mouse=True,
    ):

        # ====================================================
        # HEADER
        # ====================================================
        with dpg.group(horizontal=True, tag="header_group"):
            dpg.add_text(
                "KINGPHOENIX GCS",
                color=(150, 205, 255, 255)
            )
            dpg.add_text("2026 GCS FINAL")
            dpg.add_spacer(tag="header_spacer", width=300)

            dpg.add_text(
                "YOLO26 / WIDE / LOW LATENCY / FULLSCREEN",
                color=(145, 180, 220, 255),
            )

            dpg.add_spacer(tag="header_spacer_2", width=30)

            dpg.add_text("LINK:")
            dpg.add_text(
                "DISCONNECTED",
                tag="connection_badge",
                color=(230, 120, 95, 255),
            )

        dpg.add_separator()

        # ====================================================
        # CONNECTION STRIP
        # ====================================================
        with dpg.group(horizontal=True, tag="connection_strip"):
            dpg.add_text("DRONE IP")
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
            dpg.add_spacer(width=10)
            dpg.add_text(
                "Not connected",
                tag="connection_message",
                color=(170, 180, 190, 255),
            )

        dpg.add_spacer(height=1)

        # ====================================================
        # WIDE VIDEO ROW + PER-CAMERA INFO SIDEBARS
        # ====================================================
        with dpg.group(horizontal=True, tag="video_row"):

            # ------------------------------------------------
            # FRONT CAMERA CARD
            # ------------------------------------------------
            with dpg.child_window(
                tag="front_camera_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                with dpg.group(horizontal=True):
                    dpg.add_text(
                        "FRONT / GATE",
                        color=(155, 205, 255, 255)
                    )
                    dpg.add_spacer(width=10)
                    dpg.add_text(
                        "LIVE",
                        tag="front_live_badge",
                        color=(95, 230, 120, 255),
                    )

                with dpg.group(horizontal=True, tag="front_camera_content"):
                    dpg.add_image(
                        "texture_front",
                        tag="front_image",
                        width=640,
                        height=400,
                    )

                    with dpg.child_window(
                        tag="front_camera_info",
                        border=False,
                        no_scrollbar=True,
                        no_scroll_with_mouse=True,
                    ):
                        section_title("FRONT VISION")

                        add_value_row(
                            "Video",
                            "--",
                            "front_status_video",
                        )
                        add_value_row(
                            "AI",
                            "--",
                            "front_status_ai",
                        )
                        add_value_row(
                            "Sync",
                            "--",
                            "front_status_sync",
                        )
                        add_value_row(
                            "Video FPS",
                            "--",
                            "front_panel_video_fps",
                        )
                        add_value_row(
                            "Model FPS",
                            "--",
                            "front_panel_model_fps",
                        )
                        add_value_row(
                            "Confidence",
                            "--",
                            "front_panel_confidence",
                        )
                        add_value_row(
                            "Error X",
                            "--",
                            "front_panel_ex",
                        )
                        add_value_row(
                            "Error Y",
                            "--",
                            "front_panel_ey",
                        )
                        add_value_row(
                            "BBox Area",
                            "--",
                            "front_panel_bbox_area",
                        )
                        add_value_row(
                            "Phase",
                            "--",
                            "front_panel_phase",
                        )
                        add_value_row(
                            "Frame Sync",
                            "--",
                            "front_compact_sync",
                        )


            # ------------------------------------------------
            # BOTTOM CAMERA CARD
            # ------------------------------------------------
            with dpg.child_window(
                tag="bottom_camera_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                with dpg.group(horizontal=True):
                    dpg.add_text(
                        "BOTTOM / DROP",
                        color=(155, 205, 255, 255)
                    )
                    dpg.add_spacer(width=10)
                    dpg.add_text(
                        "LIVE",
                        tag="bottom_live_badge",
                        color=(95, 230, 120, 255),
                    )

                with dpg.group(horizontal=True, tag="bottom_camera_content"):
                    dpg.add_image(
                        "texture_bottom",
                        tag="bottom_image",
                        width=640,
                        height=400,
                    )

                    with dpg.child_window(
                        tag="bottom_camera_info",
                        border=False,
                        no_scrollbar=True,
                        no_scroll_with_mouse=True,
                    ):
                        section_title("BOTTOM VISION")

                        add_value_row(
                            "Video",
                            "--",
                            "bottom_status_video",
                        )
                        add_value_row(
                            "AI",
                            "--",
                            "bottom_status_ai",
                        )
                        add_value_row(
                            "Sync",
                            "--",
                            "bottom_status_sync",
                        )
                        add_value_row(
                            "Video FPS",
                            "--",
                            "bottom_panel_video_fps",
                        )
                        add_value_row(
                            "Model FPS",
                            "--",
                            "bottom_panel_model_fps",
                        )
                        add_value_row(
                            "Confidence",
                            "--",
                            "bottom_panel_confidence",
                        )
                        add_value_row(
                            "Error X",
                            "--",
                            "bottom_panel_ex",
                        )
                        add_value_row(
                            "Error Y",
                            "--",
                            "bottom_panel_ey",
                        )
                        add_value_row(
                            "BBox Area",
                            "--",
                            "bottom_panel_bbox_area",
                        )
                        add_value_row(
                            "Phase",
                            "--",
                            "bottom_panel_phase",
                        )
                        add_value_row(
                            "Frame Sync",
                            "--",
                            "bottom_compact_sync",
                        )


        dpg.add_spacer(height=4)

        # ====================================================
        # COMPACT FLIGHT / SYSTEM STRIP
        # Vision-specific values moved into each camera card.
        # ====================================================
        with dpg.child_window(
            tag="telemetry_strip",
            border=True,
            no_scrollbar=True,
            no_scroll_with_mouse=True,
        ):
            with dpg.table(
                tag="telemetry_table",
                header_row=False,
                policy=dpg.mvTable_SizingStretchProp,
                borders_innerV=True,
                borders_outerV=False,
                resizable=False,
            ):
                dpg.add_table_column(init_width_or_weight=0.50)
                dpg.add_table_column(init_width_or_weight=0.50)

                with dpg.table_row():
                    with dpg.table_cell():
                        section_title("FLIGHT / MAVLINK")
                        add_value_row("Link", "NOT CONNECTED", "mav_link")
                        add_value_row("Mode", "--", "mav_mode")
                        add_value_row("Armed", "--", "mav_armed")
                        add_value_row("Battery", "--", "mav_battery")
                        dpg.add_spacer(height=5)
                        add_value_row("Altitude", "--", "mav_altitude")
                        add_value_row("Speed", "--", "mav_speed")

                    with dpg.table_cell():
                        section_title("SYSTEM / RANGE")
                        add_value_row("LiDAR", "--", "value_rangefinder")
                        dpg.add_spacer(height=5)
                        add_value_row("Pi Health", "--", "value_pi_health")
                        add_value_row("CPU Temp", "--", "value_cpu_temp")
                        add_value_row("Power / Throttle", "--", "value_pi_power")
                        add_value_row("Active Camera", "--", "value_active_camera")

        dpg.add_spacer(height=4)

        # ====================================================
        # OPERATOR CONTROL ROW
        # ====================================================
        with dpg.group(horizontal=True, tag="operator_row"):

            # MISSION CONTROL
            with dpg.child_window(
                tag="mission_control_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                section_title("MISSION CONTROL")

                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="START L",
                        tag="mission_start_l_button",
                        width=125,
                        height=48,
                        callback=lambda: mission_control_callback("START_L"),
                    )
                    dpg.add_button(
                        label="START R",
                        tag="mission_start_r_button",
                        width=125,
                        height=48,
                        callback=lambda: mission_control_callback("START_R"),
                    )
                    dpg.add_button(
                        label="PRE-FLIGHT CHECK",
                        tag="mission_preflight_button",
                        width=170,
                        height=48,
                        callback=lambda: mission_control_callback("PREFLIGHT"),
                    )
                    dpg.add_button(
                        label="ABORT",
                        tag="mission_abort_button",
                        width=120,
                        height=48,
                        callback=lambda: mission_control_callback("ABORT"),
                    )

                dpg.bind_item_theme("mission_start_l_button", "theme_strat_l")
                dpg.bind_item_theme("mission_start_r_button", "theme_strat_r")
                dpg.bind_item_theme("mission_preflight_button", "theme_hold")
                dpg.bind_item_theme("mission_abort_button", "theme_abort")

                add_value_row("Operator Mode", "MISSION", "operator_mode")
                add_value_row("Selected", "--", "mission_selected_strategy")
                add_value_row("Mission Phase", "BENCH", "mission_phase")
                add_value_row("AI Camera", "FRONT", "mission_ai_camera")
                add_value_row("Vision Task", "GATE_NAV", "mission_ai_task")
                add_value_row("Mission State", "WAITING", "mission_state")

            # DROP MECHANISM
            with dpg.child_window(
                tag="drop_control_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                section_title("SETUP SERVO / DROP MECHANISM")

                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="SERVO OPEN",
                        tag="servo_open_button",
                        enabled=False,
                        width=135,
                        height=44,
                        callback=lambda: servo_control_callback("OPEN"),
                    )
                    dpg.add_button(
                        label="SERVO CLOSE",
                        tag="servo_close_button",
                        enabled=False,
                        width=135,
                        height=44,
                        callback=lambda: servo_control_callback("CLOSE"),
                    )
                    dpg.add_button(
                        label="TEST CYCLE",
                        tag="servo_test_button",
                        enabled=False,
                        width=135,
                        height=44,
                        callback=lambda: servo_control_callback("TEST"),
                    )

                for tag in (
                    "servo_open_button",
                    "servo_close_button",
                    "servo_test_button",
                ):
                    dpg.bind_item_theme(tag, "theme_servo")

                add_value_row("Servo State", "LOCKED - ENTER SETUP MODE", "servo_state")
                dpg.add_text(
                    "Actuation is intentionally disabled in this layout build.",
                    color=(170, 180, 190, 255),
                )

            # WAYPOINT / EDIT
            with dpg.child_window(
                tag="waypoint_control_card",
                border=True,
                no_scrollbar=True,
                no_scroll_with_mouse=True,
            ):
                section_title("SETUP / WAYPOINT")

                with dpg.group(horizontal=True):
                    dpg.add_text("WP")
                    dpg.add_combo(
                        WP_OPTIONS,
                        default_value="1",
                        tag="waypoint_selector",
                        enabled=False,
                        width=90,
                    )
                    dpg.add_text("STRAT")
                    dpg.add_combo(
                        POS_STRAT_OPTIONS,
                        default_value="LEFT",
                        tag="pos_strat_selector",
                        enabled=False,
                        width=110,
                    )

                add_value_row(
                    "Current Waypoint",
                    "--",
                    "waypoint_current_value"
                )

                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="SAVE WAYPOINT",
                        tag="save_waypoint_button",
                        width=155,
                        height=42,
                        callback=save_waypoint_callback,
                        enabled=False,
                    )
                    dpg.add_button(
                        label="ENTER SETUP MODE",
                        tag="unlock_edit_button",
                        width=145,
                        height=42,
                        callback=unlock_edit_mode_callback,
                    )

                dpg.bind_item_theme(
                    "save_waypoint_button",
                    "theme_button_save"
                )
                dpg.bind_item_theme(
                    "unlock_edit_button",
                    "theme_button_unlock"
                )

                dpg.add_text(
                    "MISSION MODE",
                    tag="waypoint_edit_state",
                    color=(95, 230, 120, 255),
                )
                dpg.add_text(
                    "SETUP LOCKED - MISSION CONTROLS ENABLED",
                    tag="waypoint_save_status",
                    color=(180, 190, 200, 255),
                )

        # ====================================================
        # HIDDEN COMPATIBILITY VALUES
        # ====================================================
        # Old update functions may still reference these fields. They remain
        # available but are not shown in the operator layout.
        with dpg.group(show=False, tag="compatibility_hidden_values"):
            for tag in [
                "value_vision_mode",
                "value_distance",
                "value_xyz",
                "value_rpy",
                "value_keypoints",
                "value_pose_source",
                "value_reproj_error",
                "mav_gps",
                "mav_position",
                "mav_attitude",
                "mav_waypoint",
                "value_front_sync",
                "value_bottom_sync",
                "value_drone_ip",
                "value_model_ms",
                "value_confidence",
                "value_control_phase",
                "value_error_x",
                "value_error_y",
                "value_bbox_area",
                "value_target_2d",
                "value_velocity_intent",
                "value_front_fps",
                "value_bottom_fps",
                "value_model_fps",
            ]:
                dpg.add_text("--", tag=tag)

        dpg.add_spacer(height=2)

        with dpg.group(horizontal=True, tag="footer_group"):
            dpg.add_text(
                "GCS STATUS:",
                color=(130, 180, 230, 255)
            )
            dpg.add_text(
                "READY",
                tag="footer_gcs_status",
                color=(100, 220, 120, 255),
            )
            dpg.add_spacer(tag="footer_spacer_1", width=18)
            dpg.add_text("GUI Made with love from jrcore-lab ft Kingphoenix Indonesia 2026")
            dpg.add_spacer(tag="footer_spacer_2", width=18)
            dpg.add_text("GCS FINAL / PI HEALTH INTEGRATION / CONTROL TX DISABLED")

    # ========================================================
    # V2.0.4 DESKTOP MODE
    # ========================================================
    # Start borderless and maximized so the Windows taskbar/title bar do not
    # steal layout space. DearPyGUI's "always_on_top" keeps the operator GCS
    # above normal desktop windows.

    # ========================================================
    # PRE-FLIGHT MODAL
    # ========================================================
    with dpg.window(
        tag="preflight_modal",
        label="PRE-FLIGHT CHECK",
        modal=True,
        show=False,
        no_resize=True,
        no_move=False,
        no_collapse=True,
        width=690,
        height=720,
    ):
        dpg.add_text(
            "KINGPHOENIX PRE-FLIGHT READINESS",
            color=(150, 205, 255, 255),
        )
        dpg.add_text(
            "Readiness view: live Raspberry Pi health + current mission/vision readiness",
            color=(170, 180, 190, 255),
        )
        dpg.add_separator()

        with dpg.table(
            header_row=True,
            policy=dpg.mvTable_SizingStretchProp,
            borders_innerH=True,
            borders_innerV=True,
            borders_outerH=True,
            borders_outerV=True,
        ):
            dpg.add_table_column(label="CHECK", init_width_or_weight=0.40)
            dpg.add_table_column(label="STATUS", init_width_or_weight=0.16)
            dpg.add_table_column(label="DETAIL", init_width_or_weight=0.44)

            for i, (name, status, detail) in enumerate(PREFLIGHT_MOCK_ITEMS):
                with dpg.table_row():
                    dpg.add_text(name)
                    dpg.add_text(
                        status,
                        tag=f"preflight_status_{i}",
                        color=(95, 230, 120, 255),
                    )
                    dpg.add_text(
                        detail,
                        tag=f"preflight_detail_{i}",
                    )

        dpg.add_separator()
        dpg.add_text(
            "STATUS: READY",
            tag="preflight_verdict",
            color=(95, 230, 120, 255),
        )
        dpg.add_text(
            "READY TO ROCK AND ROLL",
            tag="preflight_message",
            color=(95, 230, 120, 255),
        )
        dpg.add_spacer(height=6)
        dpg.add_button(
            label="OK - READY",
            width=-1,
            height=42,
            callback=close_preflight_popup,
        )

    dpg.create_viewport(
        title=APP_TITLE,
        width=WINDOW_W,
        height=WINDOW_H,
        resizable=False,
        decorated=False,
        always_on_top=True,
    )
    dpg.setup_dearpygui()
    dpg.bind_theme("theme_dark_high_contrast")
    dpg.set_primary_window("main_window", True)
    dpg.show_viewport()

    # Use the full monitor work area immediately.
    try:
        dpg.maximize_viewport()
    except Exception:
        pass

    apply_responsive_layout(force=True)
    _set_operator_mode(False)

    # ESC exits the GCS. F11 toggles top-most/borderless mode when supported.
    with dpg.handler_registry(tag="global_keyboard_handlers"):
        dpg.add_key_press_handler(
            key=dpg.mvKey_Escape,
            callback=lambda: dpg.stop_dearpygui(),
        )


# ============================================================
# RESPONSIVE FULL-SCREEN LAYOUT
# ============================================================
_last_layout_size = (0, 0)


def apply_responsive_layout(force=False):
    """V2.0.7: consume the full vertical screen without clipping panels."""
    global _last_layout_size

    try:
        vw = int(dpg.get_viewport_client_width())
        vh = int(dpg.get_viewport_client_height())
    except Exception:
        return

    if vw <= 200 or vh <= 200:
        return

    if not force and (vw, vh) == _last_layout_size:
        return

    _last_layout_size = (vw, vh)

    try:
        dpg.configure_item(
            "main_window",
            pos=(0, 0),
            width=vw,
            height=vh,
        )
    except Exception:
        pass

    margin = 10
    gap = 8
    header_h = 32
    connection_h = 34
    footer_h = 18

    # --------------------------------------------------------
    # WIDTH FIRST: determine actual wide-video size.
    # --------------------------------------------------------
    usable_w = vw - margin * 2 - gap
    camera_w = max(520, int((usable_w - gap) / 2))

    side_w = int(max(195, min(240, camera_w * 0.25)))
    content_w = camera_w - 24
    max_image_w = max(300, content_w - side_w - 8)

    # Side panel contains the per-camera vision/status rows only.
    # Snapshot is intentionally removed in V2.0.8.
    info_min_h = 315

    # Keep the image at native 16:10 and let width determine its natural size.
    image_w = int(max_image_w)
    image_h = int(round(image_w * FRAME_H / FRAME_W))

    # On shorter screens, video may shrink, but sidebar still gets enough room.
    max_camera_content_h = max(
        info_min_h,
        int(vh * 0.43),
    )
    if image_h > max_camera_content_h:
        image_h = max_camera_content_h
        image_w = int(round(image_h * FRAME_W / FRAME_H))

    camera_content_h = max(image_h, info_min_h)
    camera_chrome_h = 42
    camera_card_h = camera_content_h + camera_chrome_h

    dpg.configure_item(
        "front_camera_card",
        width=camera_w,
        height=camera_card_h,
    )
    dpg.configure_item(
        "bottom_camera_card",
        width=camera_w,
        height=camera_card_h,
    )

    dpg.configure_item(
        "front_image",
        width=image_w,
        height=image_h,
    )
    dpg.configure_item(
        "bottom_image",
        width=image_w,
        height=image_h,
    )

    dpg.configure_item(
        "front_camera_info",
        width=side_w,
        height=camera_content_h,
    )
    dpg.configure_item(
        "bottom_camera_info",
        width=side_w,
        height=camera_content_h,
    )

    # --------------------------------------------------------
    # CRITICAL TELEMETRY
    # --------------------------------------------------------
    # Six MAVLink rows including Battery + five System rows including LiDAR.
    # Give it a real minimum height instead of squeezing it.
    telemetry_h = 196
    dpg.configure_item(
        "telemetry_strip",
        width=vw - margin * 2,
        height=telemetry_h,
    )

    # --------------------------------------------------------
    # USE ALL REMAINING VERTICAL SPACE FOR OPERATOR CONTROLS.
    # --------------------------------------------------------
    # This is the key V2.0.7 change. Previously the layout used fixed heights;
    # when the video was width-limited the unused pixels accumulated below the
    # footer. Now that spare height is assigned to Mission/Servo/Waypoint.
    #
    # extra_overhead covers separators, group spacing and DearPyGUI window
    # padding not represented by explicit child-window heights.
    extra_overhead = 66

    control_h = (
        vh
        - header_h
        - connection_h
        - camera_card_h
        - telemetry_h
        - footer_h
        - extra_overhead
    )
    control_h = int(max(232, min(330, control_h)))

    operator_usable = vw - margin * 2 - gap * 2
    mission_w = int(operator_usable * 0.40)
    drop_w = int(operator_usable * 0.27)
    waypoint_w = operator_usable - mission_w - drop_w

    dpg.configure_item(
        "mission_control_card",
        width=mission_w,
        height=control_h,
    )
    dpg.configure_item(
        "drop_control_card",
        width=drop_w,
        height=control_h,
    )
    dpg.configure_item(
        "waypoint_control_card",
        width=waypoint_w,
        height=control_h,
    )

    # Header/footer compact horizontal spacing.
    header_spacer = max(20, vw - 760)
    dpg.configure_item("header_spacer", width=header_spacer)

    footer_space = max(6, int((vw - 1280) / 5))
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
    video_live = (
        camera.last_video_time > 0
        and (now - camera.last_video_time) < VIDEO_TIMEOUT_SEC
    )
    raw_meta = get_latest_raw_metadata(camera)

    dpg.set_value(
        f"{prefix}_status_video",
        "LIVE" if video_live else "WAIT",
    )

    ai_state = (
        "--"
        if raw_meta is None
        else str(raw_meta.get("ai_state", "--")).upper()
    )
    dpg.set_value(
        f"{prefix}_status_ai",
        ai_state,
    )

    with camera.sync_lock:
        sync = (
            camera.last_video_frame_id is not None
            and camera.last_meta_frame_id is not None
            and camera.last_sync_diff == 0
        )
        video_id = camera.last_video_frame_id
        meta_id = camera.last_meta_frame_id

    dpg.set_value(
        f"{prefix}_status_sync",
        "OK" if sync else "WAIT",
    )
    dpg.set_value(
        f"{prefix}_compact_sync",
        f"F {video_id} / M {meta_id}",
    )

    # --------------------------------------------------------
    # Per-camera vision sidebar
    # --------------------------------------------------------
    dpg.set_value(
        f"{prefix}_panel_video_fps",
        f"{camera.video_fps:.1f}" if video_live else "--",
    )

    if raw_meta is None:
        for suffix in [
            "model_fps",
            "confidence",
            "ex",
            "ey",
            "bbox_area",
            "phase",
        ]:
            dpg.set_value(
                f"{prefix}_panel_{suffix}",
                "--",
            )
        return

    model_fps = raw_meta.get("model_fps")
    confidence = raw_meta.get(
        "confidence",
        raw_meta.get("conf"),
    )
    ex = raw_meta.get("ex")
    ey = raw_meta.get("ey")
    area = raw_meta.get("bbox_area_ratio")
    phase = raw_meta.get("control_phase")

    if phase is None:
        ctrl = raw_meta.get("bbox_control") or {}
        phase = ctrl.get("phase")

    dpg.set_value(
        f"{prefix}_panel_model_fps",
        "--" if model_fps is None else f"{float(model_fps):.1f}",
    )
    dpg.set_value(
        f"{prefix}_panel_confidence",
        "--" if confidence is None else f"{float(confidence)*100:.1f}%",
    )
    dpg.set_value(
        f"{prefix}_panel_ex",
        "--" if ex is None else f"{float(ex):+.3f}",
    )
    dpg.set_value(
        f"{prefix}_panel_ey",
        "--" if ey is None else f"{float(ey):+.3f}",
    )
    dpg.set_value(
        f"{prefix}_panel_bbox_area",
        "--" if area is None else f"{float(area)*100:.2f}%",
    )
    dpg.set_value(
        f"{prefix}_panel_phase",
        "--" if phase is None else str(phase),
    )



def get_active_metadata():
    # Prefer the camera currently reporting ACTIVE AI.
    front_meta = get_latest_raw_metadata(front)
    bottom_meta = get_latest_raw_metadata(bottom)

    candidates = []
    if front_meta is not None:
        candidates.append((front, front_meta))
    if bottom_meta is not None:
        candidates.append((bottom, bottom_meta))

    active = [
        item for item in candidates
        if str(item[1].get("ai_state", "")).upper() == "ACTIVE"
    ]
    selected = active[0] if active else (candidates[0] if candidates else None)

    if selected is None:
        return None

    camera, raw = selected
    with camera.frame_lock:
        matched = camera.latest_metadata
    return matched if matched is not None else raw


def update_info_panel():
    meta = get_active_metadata()

    if meta is None:
        for tag in [
            "value_active_camera",
            "value_ai_task",
            "value_ai_model",
            "value_confidence",
            "value_control_phase",
            "value_error_x",
            "value_error_y",
            "value_bbox_area",
            "value_target_2d",
            "value_velocity_intent",
            "value_rangefinder",
            "value_pi_health",
            "value_cpu_temp",
            "value_pi_power",
            "value_model_fps",
            "value_model_ms",
        ]:
            try:
                dpg.set_value(tag, "--")
            except Exception:
                pass
        return

    dpg.set_value(
        "value_active_camera",
        str(meta.get("camera", "--")).upper(),
    )
    dpg.set_value(
        "value_ai_task",
        str(meta.get("ai_task", "--")),
    )
    dpg.set_value(
        "value_ai_model",
        str(meta.get("ai_model", "--")),
    )

    conf = meta.get("confidence", meta.get("conf"))
    dpg.set_value(
        "value_confidence",
        "--" if conf is None else f"{float(conf) * 100:.1f}%",
    )

    phase = meta.get("control_phase")
    if phase is None:
        ctrl = meta.get("bbox_control") or {}
        phase = ctrl.get("phase")
    dpg.set_value(
        "value_control_phase",
        "--" if phase is None else str(phase),
    )

    ex = meta.get("ex")
    ey = meta.get("ey")
    dpg.set_value(
        "value_error_x",
        "--" if ex is None else f"{float(ex):+.3f}",
    )
    dpg.set_value(
        "value_error_y",
        "--" if ey is None else f"{float(ey):+.3f}",
    )

    area = meta.get("bbox_area_ratio")
    dpg.set_value(
        "value_bbox_area",
        "--" if area is None else f"{float(area) * 100:.2f}%",
    )

    target = meta.get("target")
    if target is not None and len(target) >= 2:
        dpg.set_value(
            "value_target_2d",
            f"{float(target[0]):.1f}, {float(target[1]):.1f}",
        )
    else:
        dpg.set_value("value_target_2d", "--")

    intent = meta.get("velocity_intent")
    if isinstance(intent, dict):
        parts = []
        for key in (
            "forward_mps",
            "lateral_mps",
            "right_mps",
            "yaw_rate_dps",
        ):
            if key in intent and intent[key] is not None:
                parts.append(f"{key}={float(intent[key]):+.2f}")
        intent_text = " | ".join(parts) if parts else "--"
    else:
        intent_text = "--"

    dpg.set_value("value_velocity_intent", intent_text)

    # Rangefinder may be included in future metadata. Keep UI ready now.
    range_value = (
        meta.get("rangefinder_m")
        if meta.get("rangefinder_m") is not None
        else meta.get("distance_sensor_m")
    )
    dpg.set_value(
        "value_rangefinder",
        "--" if range_value is None else f"{float(range_value):.2f} m",
    )

    pi_health = meta.get("pi_health") or {}
    cpu = (
        pi_health.get("cpu_temp_c")
        if isinstance(pi_health, dict)
        else meta.get("cpu_temp")
    )
    if cpu is None:
        cpu = meta.get("cpu_temp")

    health_status = (
        pi_health.get("status", meta.get("pi_health_status", "--"))
        if isinstance(pi_health, dict)
        else meta.get("pi_health_status", "--")
    )
    dpg.set_value("value_pi_health", str(health_status).upper())

    uv_now = bool(
        pi_health.get("undervoltage_now", meta.get("undervoltage_now", False))
        if isinstance(pi_health, dict)
        else meta.get("undervoltage_now", False)
    )
    throttle_now = bool(
        (
            pi_health.get("throttling_now", False)
            or pi_health.get("frequency_capped_now", False)
            or pi_health.get("soft_temp_limit_now", False)
        )
        if isinstance(pi_health, dict)
        else meta.get("throttling_now", False)
    )
    throttled_hex = (
        pi_health.get("throttled_hex", meta.get("throttled_hex"))
        if isinstance(pi_health, dict)
        else meta.get("throttled_hex")
    )

    if uv_now:
        power_text = "UNDERVOLTAGE NOW"
    elif throttle_now:
        power_text = "THROTTLING NOW"
    elif throttled_hex:
        power_text = f"OK / {throttled_hex}"
    else:
        power_text = "--"

    dpg.set_value("value_pi_power", power_text)

    model_fps = meta.get("model_fps")
    model_ms = meta.get("model_ms")

    dpg.set_value(
        "value_cpu_temp",
        "--" if cpu is None else f"{float(cpu):.1f} C",
    )
    dpg.set_value(
        "value_model_fps",
        "--" if model_fps is None else f"{float(model_fps):.1f} FPS",
    )
    dpg.set_value(
        "value_model_ms",
        "--" if model_ms is None else f"{float(model_ms):.1f} ms",
    )

    dpg.set_value(
        "mission_ai_camera",
        str(meta.get("camera", "--")).upper(),
    )
    dpg.set_value(
        "mission_ai_task",
        str(meta.get("ai_task", "--")),
    )



def update_gui():
    """One GUI frame. Disconnect/reconnect must never terminate the app."""
    apply_responsive_layout()
    update_connection_ui()

    # Every section is deliberately tolerant of a stream disappearing between
    # two Python statements. This is normal during DISCONNECT/reconnect.
    try:
        update_texture(front, "texture_front")
    except Exception as error:
        print(f"[GUI] front texture update warning: {error}")

    try:
        update_texture(bottom, "texture_bottom")
    except Exception as error:
        print(f"[GUI] bottom texture update warning: {error}")

    try:
        update_camera_status(front, "front")
    except Exception as error:
        print(f"[GUI] front status warning: {error}")

    try:
        update_camera_status(bottom, "bottom")
    except Exception as error:
        print(f"[GUI] bottom status warning: {error}")

    try:
        update_info_panel()
    except Exception as error:
        print(f"[GUI] info panel warning: {error}")

    front_fps = float(front.video_fps or 0.0)
    bottom_fps = float(bottom.video_fps or 0.0)

    dpg.set_value(
        "value_front_fps",
        "--" if front_fps <= 0 else f"{front_fps:.1f} FPS",
    )
    dpg.set_value(
        "value_bottom_fps",
        "--" if bottom_fps <= 0 else f"{bottom_fps:.1f} FPS",
    )

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
    print(" KINGPHOENIX GCS V2.0.2 - WIDE OPERATOR LAYOUT")
    print("======================================================")
    print(f"PC Name          : {PC_NAME}")
    print(f"PC IP            : {PC_IP}")
    print(f"FRONT Video      : UDP MPEG-TS :{FRONT_VIDEO_PORT}")
    print(f"FRONT Metadata   : UDP JSON    :{FRONT_METADATA_PORT}")
    print(f"Control          : UDP JSON    :{CONTROL_PORT}")
    print(f"BOTTOM Video     : UDP MPEG-TS :{BOTTOM_VIDEO_PORT}")
    print(f"BOTTOM Metadata  : UDP JSON    :{BOTTOM_METADATA_PORT}")
    print("Sync             : 48-cell PIXEL FRAME ID")
    print("AI               : YOLO26 / ROUTED FRONT-BOTTOM")
    print(f"Background       : {BACKGROUND_IMAGE_PATH}")
    print(f"Sync fail grace  : {SYNC_FAIL_GRACE_SEC:.2f} s")
    print("======================================================")

    start_background_threads()
    create_gui()

    global gui_runtime_error_count, last_gui_runtime_error

    try:
        while dpg.is_dearpygui_running():
            try:
                update_gui()
            except Exception as error:
                gui_runtime_error_count += 1
                last_gui_runtime_error = repr(error)
                print(
                    f"[GUI] RUNTIME WARNING #{gui_runtime_error_count}: "
                    f"{last_gui_runtime_error}"
                )

            # A disconnect is a connection-state event, NOT an application
            # shutdown event. Only DearPyGUI window close ends this loop.
            dpg.render_dearpygui_frame()

    finally:
        print("[GUI] Window close requested - shutting down GCS")
        shutdown()
        dpg.destroy_context()


if __name__ == "__main__":
    main()
