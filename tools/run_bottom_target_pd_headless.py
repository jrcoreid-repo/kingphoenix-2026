#!/usr/bin/env python3
"""Headless + axis-select wrapper for bottom-camera PD tuning.

Default remains SHADOW ONLY. Add --flight only after physical mapping is
verified. Use --axis x, --axis y, then --axis xy for staged field tuning.

Field correction
----------------
The first BODY-X flight test showed the original image-X -> BODY-X command sign
was reversed in the actual aircraft response, so X_SIGN=-1 by default.
The dedicated BODY-Y flight test then showed the same for Y, so Y_SIGN=-1 by
default as well. These signs are field-validated for the current bottom-camera
mounting and aircraft response.

No Windows GCS is required. Vision video/metadata are looped to localhost so
the normal VisionStreamSession continues updating SharedState.

Field landing policy
--------------------
For this headless wrapper AUTO LAND is DISABLED by default because the bottom
camera may be looking at / flying above the payload container. At normal test
completion, Ctrl+C, or a guarded abort, horizontal commands are zeroed and the
operator lands manually with the RC. Pass --auto-land only when an automatic
LAND command is explicitly desired.
"""

from __future__ import annotations

import argparse
import builtins
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
    own.add_argument(
        "--x-sign",
        type=int,
        choices=[-1, 1],
        default=-1,
        help="BODY-X field sign correction; default -1 from flight validation",
    )
    own.add_argument(
        "--y-sign",
        type=int,
        choices=[-1, 1],
        default=-1,
        help="BODY-Y field sign correction; default -1 from flight validation",
    )
    own.add_argument(
        "--auto-land",
        action="store_true",
        help="opt in to the base tuner's automatic LAND at test end/abort",
    )
    own_args, remaining = own.parse_known_args(sys.argv[1:])
    axis = own_args.axis
    x_sign = int(own_args.x_sign)
    y_sign = int(own_args.y_sign)
    auto_land = bool(own_args.auto_land)

    # Strip wrapper-only arguments before the base parser runs.
    sys.argv = [sys.argv[0], *remaining]

    base.GcsServer = HeadlessLoopbackGcs
    base.wait_bottom_stream = wait_bottom_headless

    # Apply field sign correction and filter inactive axes BEFORE vector
    # capping/slew. This keeps printed BODY VX/VY consistent with transmitted
    # MAVLink commands.
    original_vector_cap = base.vector_cap

    def axis_filtered_vector_cap(x: float, y: float, cap: float):
        xx = float(x) * x_sign
        yy = float(y) * y_sign
        if axis == "x":
            yy = 0.0
        elif axis == "y":
            xx = 0.0
        return original_vector_cap(xx, yy, cap)

    base.vector_cap = axis_filtered_vector_cap

    # Defense in depth: force the inactive MAVLink component to zero. Sign has
    # already been corrected upstream, so do not invert it again here.
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

    if not auto_land:
        # Keep the base tuner untouched, but suppress every software-initiated
        # LAND path for this field wrapper. The runner sends XY ZERO and then
        # leaves landing/mode takeover to the safety pilot on the RC.
        def manual_land_and_wait(test):
            try:
                base.zero_xy(test)
            except Exception:
                pass
            builtins.print("[MANUAL LAND] AUTO LAND suppressed; take over with RC and land manually")

        def manual_land_best_effort(test, reason=""):
            try:
                base.zero_xy(test)
            except Exception:
                pass
            suffix = f" | reason={reason}" if reason else ""
            builtins.print(
                "[MANUAL LAND] software LAND suppressed; take over with RC and land manually"
                + suffix
            )

        base.DirectFlightTest.land_and_wait = manual_land_and_wait
        base.DirectFlightTest.land_best_effort = manual_land_best_effort

        # Rewrite the one base status line that otherwise says ZERO -> LAND,
        # so the field terminal cannot imply that LAND will be commanded.
        def base_print(*args, **kwargs):
            if len(args) == 1 and args[0] == "[BOTTOM] duration complete -> ZERO -> LAND":
                builtins.print(
                    "[BOTTOM] duration complete -> ZERO; AUTO LAND DISABLED -> RC TAKEOVER",
                    **kwargs,
                )
                return
            builtins.print(*args, **kwargs)

        base.print = base_print

    print("============================================================")
    print(" KINGPHOENIX BOTTOM PD - HEADLESS AXIS TUNER")
    print("============================================================")
    print(f"[AXIS] {axis.upper()}")
    print(f"[FIELD SIGN] X={x_sign:+d} | Y={y_sign:+d}")
    if axis == "x":
        print("[ACTIVE] BODY X only: forward/back | BODY Y forced ZERO")
    elif axis == "y":
        print("[ACTIVE] BODY Y only: right/left | BODY X forced ZERO")
    else:
        print("[ACTIVE] BODY X + BODY Y combined")
    print("[FIELD FIX] flight validation: X reversed and Y reversed -> defaults X=-1, Y=-1")
    print("[NOTE] default remains SHADOW ONLY; add --flight explicitly")
    if auto_land:
        print("[LANDING] AUTO LAND ENABLED by --auto-land")
    else:
        print("[LANDING] AUTO LAND DISABLED | test end/abort -> XY ZERO -> RC MANUAL LAND")
    print("------------------------------------------------------------")

    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
