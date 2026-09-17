#!/usr/bin/env python3
"""Headless + axis-select wrapper for bottom-camera PD tuning.

Default remains SHADOW ONLY. Add --flight only after physical mapping is
verified. Use --axis x, --axis y, then --axis xy for staged field tuning.

Physical mount contract inherited from run_bottom_target_pd_tuning.py:
  image X left->right  = drone FRONT->REAR  => BODY X error = -ex
  image Y top->bottom  = drone RIGHT->LEFT  => BODY Y error = -ey

No Windows GCS is required. Vision video/metadata are looped to localhost so
the normal VisionStreamSession continues updating SharedState.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import tools.run_bottom_target_pd_tuning as base


class HeadlessLoopbackGcs:
    def __init__(self, state, events, waypoints, stop_event, command_manager=None):
        self.state = state
        self.stop_event = stop_event

    def start(self):
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
        print("[HEADLESS] BOTTOM vision sink -> 127.0.0.1 loopback")

    def is_alive(self):
        return False

    def join(self, timeout=None):
        return None


def wait_bottom_headless(state, timeout_s: float):
    print("[HEADLESS] waiting for BOTTOM camera + YOLO26 ...")
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        with state.lock:
            bottom = dict(state.vision.get("bottom") or {})
        video_state = str(bottom.get("video_state", "")).upper()
        ai_state = str(bottom.get("ai_state", "UNKNOWN")).upper()
        if video_state == "LIVE" and ai_state not in {"DISABLED", "ERROR"}:
            print(
                f"[HEADLESS] BOTTOM LIVE | AI={ai_state} "
                f"mode={bottom.get('vision_mode', 'SEARCH')}"
            )
            return
        time.sleep(0.10)
    raise base.FlightTestError("HEADLESS BOTTOM vision not ready before timeout")


def main() -> int:
    own = argparse.ArgumentParser(add_help=False)
    own.add_argument("--axis", choices=["x", "y", "xy"], default="xy")
    own_args, remaining = own.parse_known_args(sys.argv[1:])
    axis = own_args.axis

    # Strip wrapper-only arguments before the base parser runs.
    sys.argv = [sys.argv[0], *remaining]

    base.GcsServer = HeadlessLoopbackGcs
    base.wait_bottom_stream = wait_bottom_headless

    # Filter the inactive axis BEFORE vector capping/slew. This makes the
    # printed BODY VX/VY values match the actual axis being tuned.
    original_vector_cap = base.vector_cap

    def axis_filtered_vector_cap(x: float, y: float, cap: float):
        xx = float(x)
        yy = float(y)
        if axis == "x":
            yy = 0.0
        elif axis == "y":
            xx = 0.0
        return original_vector_cap(xx, yy, cap)

    base.vector_cap = axis_filtered_vector_cap

    # Defense in depth: also force the inactive MAVLink component to zero.
    original_send = base.send_body_xy

    def axis_filtered_send(test, vx_mps: float, vy_mps: float):
        vx = float(vx_mps)
        vy = float(vy_mps)
        if axis == "x":
            vy = 0.0
        elif axis == "y":
            vx = 0.0
        return original_send(test, vx, vy)

    base.send_body_xy = axis_filtered_send

    print("============================================================")
    print(" KINGPHOENIX BOTTOM PD - HEADLESS AXIS TUNER")
    print("============================================================")
    print(f"[AXIS] {axis.upper()}")
    if axis == "x":
        print("[ACTIVE] BODY X only: forward/back | BODY Y forced ZERO")
    elif axis == "y":
        print("[ACTIVE] BODY Y only: right/left | BODY X forced ZERO")
    else:
        print("[ACTIVE] BODY X + BODY Y combined")
    print("[MAP] image LEFT -> drone FORWARD | image RIGHT -> REARWARD")
    print("[MAP] image TOP -> drone RIGHT | image BOTTOM -> LEFT")
    print("[NOTE] default remains SHADOW ONLY; add --flight explicitly")
    print("------------------------------------------------------------")

    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
