#!/usr/bin/env python3
"""Read-only Kingphoenix rangefinder diagnostic.

Run with the aircraft DISARMED. This tool owns the FC serial connection while
active, so stop main.py/MAVProxy/other serial consumers first.

It requests and prints both DISTANCE_SENSOR and legacy RANGEFINDER messages so
we can identify exactly which MAVLink source Mission Planner is displaying.
This script sends no arm, mode, takeoff, mission, servo, or movement command.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "system.json"


def main():
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    mc = cfg["mavlink"]
    conn = mc["connection"]
    kwargs = {
        "source_system": int(mc.get("source_system", 1)),
        "source_component": int(mc.get("source_component", 191)),
        "autoreconnect": True,
    }
    if str(conn).startswith("/dev/"):
        kwargs["baud"] = int(mc.get("baud", 921600))

    master = mavutil.mavlink_connection(conn, **kwargs)
    print("============================================================")
    print(" KINGPHOENIX RANGEFINDER DIAGNOSTIC - READ ONLY")
    print("============================================================")
    print(f"[MAVLINK] connecting {conn} ...")
    hb = master.wait_heartbeat(timeout=5.0)
    if hb is None:
        raise SystemExit("heartbeat timeout")
    print(
        f"[MAVLINK] heartbeat src={hb.get_srcSystem()}:{hb.get_srcComponent()} "
        f"target={master.target_system}:{master.target_component}"
    )

    # Explicitly address the FC component from config if pymavlink did not
    # populate it from the heartbeat (some serial setups leave component=0).
    target_system = int(mc.get("target_system", master.target_system or 1))
    target_component = int(mc.get("target_component", 1))

    def request(message_id: int, hz: float):
        interval_us = int(round(1_000_000.0 / hz))
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id), float(interval_us), 0, 0, 0, 0, 0,
        )

    request(132, 20.0)  # DISTANCE_SENSOR
    request(173, 10.0)  # RANGEFINDER legacy
    print("[INFO] requested DISTANCE_SENSOR=20Hz and RANGEFINDER=10Hz")
    print("[INFO] move/lift the DISARMED aircraft by hand and compare with Mission Planner")
    print("[INFO] Ctrl+C to stop\n")

    last_dist_print = {}
    last_rf_print = 0.0
    try:
        while True:
            msg = master.recv_match(
                type=["DISTANCE_SENSOR", "RANGEFINDER", "HEARTBEAT"],
                blocking=True,
                timeout=1.0,
            )
            if msg is None:
                print("[WARN] no rangefinder message in last 1s")
                continue
            now = time.monotonic()
            t = msg.get_type()
            if t == "DISTANCE_SENSOR":
                sid = int(getattr(msg, "id", 0))
                orientation = int(getattr(msg, "orientation", -1))
                key = (sid, orientation)
                if now - last_dist_print.get(key, 0.0) < 0.20:
                    continue
                last_dist_print[key] = now
                current_cm = int(getattr(msg, "current_distance", 0))
                min_cm = int(getattr(msg, "min_distance", 0))
                max_cm = int(getattr(msg, "max_distance", 0))
                quality = getattr(msg, "signal_quality", None)
                covariance = getattr(msg, "covariance", None)
                print(
                    f"DISTANCE_SENSOR src={msg.get_srcSystem()}:{msg.get_srcComponent()} "
                    f"id={sid} orient={orientation} current={current_cm}cm "
                    f"({current_cm/100.0:.2f}m) min={min_cm}cm max={max_cm}cm "
                    f"quality={quality} covariance={covariance}"
                )
            elif t == "RANGEFINDER":
                if now - last_rf_print < 0.20:
                    continue
                last_rf_print = now
                distance = float(getattr(msg, "distance", 0.0))
                voltage = getattr(msg, "voltage", None)
                print(
                    f"RANGEFINDER     src={msg.get_srcSystem()}:{msg.get_srcComponent()} "
                    f"distance={distance:.3f}m voltage={voltage}"
                )
    except KeyboardInterrupt:
        print("\n[DONE] diagnostic stopped")
    finally:
        try:
            master.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
