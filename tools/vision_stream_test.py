#!/usr/bin/env python3
"""Kingphoenix V1.9.0 vision/stream bench test.

No flight controller is required.

Flow:
  Windows GCS HELLO -> Pi HELLO_ACK
  -> front + bottom test-video capture
  -> frame barcode
  -> MPEG-TS/H264 UDP
  -> exact frame_id JSON metadata

No MAVLink movement commands are sent.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import load_json
from core.shared_state import SharedState
from core.event_bus import EventBus
from mission.waypoint_manager import WaypointManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from streaming.video_streamer import VideoStreamer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--front", default=None, help="front video/camera source")
    ap.add_argument("--bottom", default=None, help="bottom video/camera source")
    args = ap.parse_args()

    cfg = load_json("system.json")
    camera_cfg = cfg["camera"]

    # Optional CLI source overrides are legacy bench/debug only.
    if args.front is not None or args.bottom is not None:
        fallback = camera_cfg.setdefault("legacy_file_fallback", {})
        fallback["enabled"] = True

        if args.front is not None:
            fallback["front_source"] = args.front
        if args.bottom is not None:
            fallback["bottom_source"] = args.bottom

        # Explicit source override means use legacy OpenCV/file backend
        # for this run only. Real Picamera2 role mapping is preserved.
        camera_cfg["backend"] = "opencv"

    if not VideoStreamer.ffmpeg_available():
        print("ERROR: ffmpeg not found in PATH")
        print("Install ffmpeg first, then re-run this test.")
        return 2

    state = SharedState()
    events = EventBus()
    stop = threading.Event()
    waypoints = WaypointManager(state)

    gcs = GcsServer(state, events, waypoints, stop)
    vision = VisionStreamSession(cfg, state, stop)

    print("======================================================")
    print(" KINGPHOENIX V1.9.5 - REAL VISION / COLOR CONTRACT BENCH")
    print(" NO FC REQUIRED / NO MAVLINK MOVEMENT")
    print("======================================================")
    backend = str(camera_cfg.get("backend", "opencv")).lower()
    print("CAMERA BACKEND:", backend)

    if backend == "picamera2":
        front_cfg = camera_cfg["front"]
        bottom_cfg = camera_cfg["bottom"]

        print(
            "FRONT CAMERA :",
            f"camera_num={front_cfg.get('camera_num')} "
            f"role={front_cfg.get('role', 'FRONT')} "
            f"sensor={front_cfg.get('sensor_mode', {})} "
            f"output={front_cfg.get('output', {})}",
        )
        print(
            "BOTTOM CAMERA:",
            f"camera_num={bottom_cfg.get('camera_num')} "
            f"role={bottom_cfg.get('role', 'BOTTOM')} "
            f"sensor={bottom_cfg.get('sensor_mode', {})} "
            f"output={bottom_cfg.get('output', {})}",
        )
    else:
        fallback = camera_cfg.get("legacy_file_fallback", camera_cfg)
        print("FRONT SOURCE :", fallback.get("front_source", "--"))
        print("BOTTOM SOURCE:", fallback.get("bottom_source", "--"))
    print("CONTROL PORT :", cfg["stream"]["control_port"])
    print("Waiting for Windows GCS HELLO...")

    try:
        gcs.start()
        vision.start()

        while not stop.is_set():
            time.sleep(1.0)

            front = state.vision["front"]
            bottom = state.vision["bottom"]

            if state.system.get("gcs_connected"):
                route = state.vision.get("ai_route", {})
                print(
                    "[STREAM] "
                    f"ROUTE={route.get('camera')}/"
                    f"{route.get('model_mode')}/"
                    f"C{route.get('detector_class_id')} | "
                    f"front={front['video_state']} F={front['frame_id']} "
                    f"AI={front.get('ai_model') or '--'} "
                    f"VALID={front.get('vision_valid', False)} | "
                    f"bottom={bottom['video_state']} F={bottom['frame_id']} "
                    f"AI={bottom.get('ai_model') or '--'} "
                    f"VALID={bottom.get('vision_valid', False)}"
                )

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop.set()
        gcs.join(timeout=1.0)
        vision.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
