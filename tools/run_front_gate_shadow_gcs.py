#!/usr/bin/env python3
"""Kingphoenix FRONT gate SHADOW runtime with normal GCS streaming.

Purpose
-------
Run the real FRONT camera + YOLO26/Hailo pipeline and expose the normal
Kingphoenix video/metadata/control handshake to the Windows GCS, while opening
NO MAVLink connection and sending NO aircraft movement command.

Use this during the perception/decision phase before vision is allowed to
control GUIDED flight.

Safety contract
---------------
- does NOT import/start the Kingphoenix flight runtime
- does NOT open /dev/ttyAMA0
- does NOT create a MavlinkConnection/MavlinkTx
- forces vision_control.movement_tx_enabled = False
- GCS bench FC commands have no command backend in this runtime

The existing streaming stack is reused so the GCS sees the same 640x400
MPEG-TS/H264 stream, frame-id barcode, exact-frame metadata, YOLO26 bbox,
confidence and EX/EY overlay that the final runtime uses.
"""

from __future__ import annotations

import copy
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from config import load_json
from core.event_bus import EventBus
from core.shared_state import SharedState
from mission.waypoint_manager import WaypointManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession


def shadow_config() -> dict:
    """Return a front-gate-only AI routing config without mutating JSON files."""
    cfg = copy.deepcopy(load_json("system.json"))

    ai = cfg.setdefault("ai", {})
    ai["enabled"] = True
    ai["engine"] = "YOLO26_ONLY"

    # Keep the active AI permanently on FRONT during this shadow test.
    route = ai.setdefault("route", {})
    route.update(
        {
            "camera": "front",
            "model_mode": "YOLO26_ONLY",
            "task": "GATE_NAV",
            "detector_class_id": int(ai.get("front", {}).get("detector_class_id", 1)),
            "source": "SHADOW_GCS",
        }
    )

    # The normal config contains a front/bottom AI test switcher. Disable it
    # here so gate AI does not jump to the bottom camera every few seconds.
    ai.setdefault("test_switch", {})["enabled"] = False

    vision_control = cfg.setdefault("vision_control", {})
    vision_control["enabled"] = True
    vision_control["movement_tx_enabled"] = False

    run_mode = cfg.setdefault("run_mode", {})
    run_mode["profile"] = "VISION_AI_TEST"

    return cfg


def fmt(value, digits=3):
    if value is None:
        return "--"
    try:
        return f"{float(value):+.{digits}f}"
    except Exception:
        return str(value)


def main() -> int:
    cfg = shadow_config()
    stop_event = threading.Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)

    # No CommandManager is supplied. Therefore the GCS can handshake and
    # monitor streams, but FC ARM/DISARM/MODE requests have no backend here.
    gcs = GcsServer(
        state,
        events,
        waypoints,
        stop_event,
        command_manager=None,
    )
    vision = VisionStreamSession(cfg, state, stop_event)

    with state.lock:
        state.system["running"] = True
        state.system["startup_state"] = "VISION_SHADOW_GCS"
        state.system["ready_for_mission"] = False
        state.system["vision_shadow_only"] = True

    print("============================================================")
    print(" KINGPHOENIX FRONT GATE SHADOW + GCS STREAM")
    print("============================================================")
    print("[MODE] FRONT camera -> YOLO26/Hailo -> GCS overlay")
    print("[MODE] camera route fixed to FRONT / GATE_NAV / class 1")
    print("[SAFE] MAVLink runtime NOT STARTED")
    print("[SAFE] movement_tx_enabled = False")
    print(f"[GCS] control UDP :{int(cfg['stream']['control_port'])}")
    print(
        f"[GCS] FRONT video/meta :{int(cfg['stream']['front_video_port'])}/"
        f"{int(cfg['stream']['front_metadata_port'])}"
    )
    print("[GCS] open Windows GCS and CONNECT to this Raspberry Pi IP")
    print("[NOTE] do NOT run main.py or run_front_gate_shadow.py at the same time")

    try:
        gcs.start()
        vision.start()

        last_line = 0.0
        while not stop_event.is_set():
            now = time.monotonic()
            if now - last_line >= 0.50:
                last_line = now
                with state.lock:
                    front = dict(state.vision.get("front") or {})
                    connected = bool(state.system.get("gcs_connected", False))

                valid = bool(front.get("vision_valid", False))
                conf = front.get("confidence")
                ex = front.get("ex")
                ey = front.get("ey")
                area = front.get("bbox_area_ratio")
                phase = str(front.get("control_phase") or "SEARCH")
                bbox = front.get("bbox")
                intent = front.get("velocity_intent") or {}

                if valid and bbox is not None:
                    print(
                        "[SHADOW] "
                        f"GCS={'ON' if connected else 'WAIT'} "
                        f"conf={fmt(conf, 2)} ex={fmt(ex)} ey={fmt(ey)} "
                        f"area={fmt(area, 4)} phase={phase} "
                        f"lat={fmt(intent.get('lateral_mps'), 2)}m/s "
                        f"yaw={fmt(intent.get('yaw_rate_dps'), 1)}deg/s "
                        "TX=OFF"
                    )
                else:
                    print(
                        "[SHADOW] "
                        f"GCS={'ON' if connected else 'WAIT'} "
                        f"video={front.get('video_state', 'WAIT')} "
                        f"AI={front.get('ai_state', 'WAIT')} "
                        f"mode={front.get('vision_mode', 'SEARCH')} "
                        "target=SEARCH TX=OFF"
                    )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[STOP] operator requested shutdown")
    finally:
        stop_event.set()
        with state.lock:
            state.system["running"] = False
            state.system["gcs_connected"] = False
            state.system["gcs_endpoint"] = None
            state.system["startup_state"] = "STOPPED"

        if gcs.is_alive():
            gcs.join(timeout=1.0)
        vision.join(timeout=1.0)
        print("[STOP] vision/GCS shadow runtime closed")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
