#!/usr/bin/env python3
"""Bench-safe VCT mode-transition test.

This tool does NOT arm and does NOT command takeoff.
It uses the same production MAVLink RX, FreshnessPolicy, TX and CommandManager.

Example:
    python tools/vct_mode_test.py --mode STABILIZE

You may test another disarmed-safe mode only if you understand the FC state.
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
from flight.mavlink_connection import MavlinkConnection
from flight.mavlink_rx import MavlinkRxWorker
from flight.mavlink_tx import MavlinkTx
from flight.mavlink_stream_config import MavlinkStreamConfigurator
from flight.freshness_policy import FreshnessPolicy
from flight.command_manager import CommandManager


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--wait-heartbeat", type=float, default=5.0)
    args = ap.parse_args()

    cfg = load_json("system.json")
    mav_cfg = cfg["mavlink"]

    state = SharedState()
    events = EventBus()
    stop = threading.Event()

    conn = MavlinkConnection(mav_cfg)
    tx = MavlinkTx(conn)
    freshness = FreshnessPolicy(state.vehicle, mav_cfg.get("freshness_s", {}))
    command = CommandManager(
        tx,
        state,
        events,
        freshness_policy=freshness,
        vct_cfg=mav_cfg.get("vct", {}),
    )

    rx = MavlinkRxWorker(
        conn,
        state,
        events,
        stop,
        heartbeat_timeout_s=mav_cfg.get("heartbeat_timeout_s", 2.5),
    )

    stream_cfg = MavlinkStreamConfigurator(
        tx,
        state,
        stop,
        mav_cfg.get("message_rates_hz", {}),
        retry_interval_s=mav_cfg.get("rate_retry_interval_s", 10.0),
    )

    try:
        conn.open()
        rx.start()
        stream_cfg.start()

        deadline = time.monotonic() + args.wait_heartbeat
        while time.monotonic() < deadline:
            if state.vehicle.connected:
                break
            time.sleep(0.05)

        if not state.vehicle.connected:
            print("[VCT TEST] FAIL: no autopilot heartbeat")
            return 2

        print(
            f"[VCT TEST] FC={state.vehicle.autopilot_system_id}:"
            f"{state.vehicle.autopilot_component_id} "
            f"current_mode={state.vehicle.mode} armed={state.vehicle.armed}"
        )

        if state.vehicle.armed:
            print("[VCT TEST] REFUSED: vehicle is ARMED")
            return 3

        target = args.mode.upper()
        print(f"[VCT TEST] requesting mode {target}")
        result = command.set_mode(target)

        print(
            f"[VCT TEST] ok={result.ok} attempts={result.attempts} "
            f"reason={result.reason} ack={result.ack_result} "
            f"ack_source={result.ack_source} elapsed={result.elapsed_s:.3f}s"
        )
        print(f"[VCT TEST] final_mode={state.vehicle.mode}")

        return 0 if result.ok else 4

    finally:
        stop.set()
        rx.join(timeout=1.0)
        stream_cfg.join(timeout=1.0)
        conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
