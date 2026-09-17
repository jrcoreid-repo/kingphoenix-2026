#!/usr/bin/env python3
"""Headless wrapper for the FRONT gate PD + cruise tuner.

Runs the existing run_vision_follow_gate_pd_cruise_tuning controller without
requiring the Windows GCS application. The normal vision pipeline still runs on
the Pi; its video/metadata sink is redirected to localhost UDP so the shared
vision state keeps updating exactly as in the GCS-backed path.

All tuning/flight CLI arguments are inherited from the base tuner.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import tools.run_vision_follow_gate_pd_cruise_tuning as base


class HeadlessLoopbackGcs:
    """Minimal GCS stand-in used only by the field-tuning wrapper."""

    def __init__(self, state, events, waypoints, stop_event, command_manager=None):
        self.state = state
        self.stop_event = stop_event

    def start(self):
        # VisionStreamSession gates capture/inference on these fields. Point the
        # stream at localhost so no Windows GCS/HELLO handshake is required.
        with self.state.lock:
            self.state.system["gcs_connected"] = True
            self.state.system["gcs_endpoint"] = {
                "ip": "127.0.0.1",
                "front_video_port": 14560,
                "front_metadata_port": 14561,
                "control_port": 14562,
                "bottom_video_port": 14563,
                "bottom_metadata_port": 14564,
            }
        print("[HEADLESS] Windows GCS not required")
        print("[HEADLESS] FRONT vision sink -> 127.0.0.1 loopback")

    def is_alive(self):
        return False

    def join(self, timeout=None):
        return None


def wait_for_headless_front(state, timeout_s: float):
    print("[HEADLESS] waiting for FRONT camera + YOLO26 ...")
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        with state.lock:
            front = dict(state.vision.get("front") or {})
        video_state = str(front.get("video_state", "")).upper()
        ai_state = str(front.get("ai_state", "UNKNOWN")).upper()
        if video_state == "LIVE" and ai_state not in {"DISABLED", "ERROR"}:
            print(
                f"[HEADLESS] FRONT LIVE | AI={ai_state} "
                f"mode={front.get('vision_mode', 'SEARCH')}"
            )
            return
        time.sleep(0.10)
    raise base.FlightTestError("HEADLESS FRONT vision not ready before timeout")


def main() -> int:
    # Preserve the tested controller and only replace the external GCS
    # dependency. Safety-pilot takeover, LiDAR guards, target-loss ZERO,
    # adaptive-forward behavior, and end-of-test LAND remain unchanged.
    base.GcsServer = HeadlessLoopbackGcs
    base.wait_for_gcs = wait_for_headless_front
    print("============================================================")
    print(" KINGPHOENIX GATE TUNING - HEADLESS MODE")
    print(" No Windows GCS needed; all base tuner arguments still apply")
    print("============================================================")
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
