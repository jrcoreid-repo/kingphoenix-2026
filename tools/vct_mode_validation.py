#!/usr/bin/env python3
"""
Kingphoenix V1.6 - MODE VCT Validation

Bench-safe validation of the command path:
    RX -> freshness -> identity -> TX -> mode state verification -> retry

Default behavior:
    current mode -> GUIDED -> original mode

Safety:
- refuses to start if vehicle is armed
- never sends ARM
- never sends TAKEOFF
- never sends velocity/position setpoints
- restores the original mode after the GUIDED test when possible

Usage:
    python tools/vct_mode_validation.py

Optional:
    python tools/vct_mode_validation.py --target GUIDED
    python tools/vct_mode_validation.py --target LOITER
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


def yn(v):
    return "YES" if v else "NO"


def line(label, value):
    print(f"{label:<18}: {value}")


def make_trace(vehicle):
    def trace(event, data):
        if event == "START":
            print("\n[VCT] transaction started")
        elif event == "PRECHECK":
            print(f"[VCT] PRECHECK     : {'PASS' if data['ok'] else 'FAIL'}"
                  + (f" ({data['reason']})" if data.get("reason") else ""))
        elif event == "ALREADY_SATISFIED":
            print("[VCT] STATE        : already satisfied")
        elif event == "ATTEMPT":
            print(f"[VCT] ATTEMPT      : {data['attempt']}/{data['total']}")
        elif event == "SEND":
            print(f"[VCT] TX           : {'SENT' if data['ok'] else 'FAILED'}")
        elif event == "ACK":
            if data["received"]:
                print(f"[VCT] ACK          : result={data['result']} source={data['source']}")
            else:
                print("[VCT] ACK          : none/optional for SET_MODE")
        elif event == "STATE_VERIFY":
            print(f"[VCT] MODE STATE   : {vehicle.mode}")
            print(f"[VCT] VERIFY       : {'PASS' if data['ok'] else 'FAIL'}")
        elif event == "BACKOFF":
            print(f"[VCT] RETRY        : {data['seconds']:.2f}s ({data['reason']})")
        elif event == "FAILED":
            print(f"[VCT] FAILED       : {data['reason']}")
    return trace


def wait_connected(state, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if state.vehicle.connected:
            return True
        time.sleep(0.05)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="GUIDED")
    ap.add_argument("--wait-heartbeat", type=float, default=6.0)
    ap.add_argument("--no-restore", action="store_true",
                    help="Do not return to the original mode after target-mode test")
    args = ap.parse_args()

    cfg = load_json("system.json")
    mav_cfg = cfg["mavlink"]
    vct_cfg = mav_cfg.get("vct", {})

    state = SharedState()
    events = EventBus()
    stop = threading.Event()

    conn = MavlinkConnection(mav_cfg)
    tx = MavlinkTx(conn)
    freshness = FreshnessPolicy(state.vehicle, mav_cfg.get("freshness_s", {}))
    command = CommandManager(
        tx, state, events,
        freshness_policy=freshness,
        vct_cfg=vct_cfg,
    )

    rx = MavlinkRxWorker(
        conn, state, events, stop,
        heartbeat_timeout_s=mav_cfg.get("heartbeat_timeout_s", 2.5),
    )
    streams = MavlinkStreamConfigurator(
        tx, state, stop,
        mav_cfg.get("message_rates_hz", {}),
        retry_interval_s=mav_cfg.get("rate_retry_interval_s", 10.0),
    )

    try:
        print("======================================================")
        print(" KINGPHOENIX V1.7.1 - MODE VCT VALIDATION")
        print(" DISARMED BENCH TEST ONLY")
        print(" VCT API SELF-CHECK: run() present")
        print("======================================================")

        # Catch packaging/API regressions before touching the FC.
        from flight.vct import VerifiedCommandTransaction
        required_vct_methods = ("run", "_wait_for_state", "_wait_for_ack")
        missing = [
            name for name in required_vct_methods
            if not callable(getattr(VerifiedCommandTransaction, name, None))
        ]
        if missing:
            print(f"\nRESULT: FAIL - VCT API incomplete: {missing}")
            return 10

        conn.open()
        rx.start()
        streams.start()

        if not wait_connected(state, args.wait_heartbeat):
            print("\nRESULT: FAIL - no autopilot heartbeat")
            return 2

        # Give heartbeat/mode state one more cycle to settle.
        time.sleep(0.3)
        v = state.vehicle

        print("\n--- PRE-FLIGHT STATE ---")
        line("CONNECTED", yn(v.connected))
        line("ARMED", yn(v.armed))
        hb_limit = float(mav_cfg.get("freshness_s", {}).get("HEARTBEAT", 2.5))
        line("HEARTBEAT", state.vehicle.message_status("HEARTBEAT", hb_limit))
        line("HB AGE", f"{v.heartbeat_age_s():.3f} s")
        line("AUTOPILOT", f"{v.autopilot_system_id}:{v.autopilot_component_id}")
        line("KINGPHOENIX", f"{conn.source_system}:{conn.source_component}")
        line("TARGET", f"{conn.target_system}:{conn.target_component}")
        line("CURRENT MODE", v.mode)

        if v.armed:
            print("\nRESULT: REFUSED - vehicle is ARMED")
            return 3

        original = str(v.mode).upper()
        target = str(args.target).upper()

        if original == "UNKNOWN":
            print("\nRESULT: FAIL - original mode is UNKNOWN")
            return 4

        print("\n--- TRANSACTION A ---")
        line("FROM", original)
        line("TO", target)

        result_a = command.set_mode(
            target,
            trace_fn=make_trace(v),
            require_disarmed=True,
        )

        time.sleep(float(vct_cfg.get("mode_settle_s", 0.35)))

        print("\n--- RESULT A ---")
        line("SUCCESS", yn(result_a.ok))
        line("ATTEMPTS", result_a.attempts)
        line("REASON", result_a.reason)
        line("FINAL MODE", v.mode)
        line("ELAPSED", f"{result_a.elapsed_s:.3f} s")

        if not result_a.ok:
            print("\nFINAL RESULT: FAIL")
            return 5

        if args.no_restore or original == target:
            print("\nFINAL RESULT: PASS")
            return 0

        time.sleep(float(vct_cfg.get("mode_roundtrip_pause_s", 0.75)))

        print("\n--- TRANSACTION B / RESTORE ---")
        line("FROM", v.mode)
        line("TO", original)

        result_b = command.set_mode(
            original,
            trace_fn=make_trace(v),
            require_disarmed=True,
        )

        time.sleep(float(vct_cfg.get("mode_settle_s", 0.35)))

        print("\n--- RESULT B ---")
        line("SUCCESS", yn(result_b.ok))
        line("ATTEMPTS", result_b.attempts)
        line("REASON", result_b.reason)
        line("FINAL MODE", v.mode)
        line("ELAPSED", f"{result_b.elapsed_s:.3f} s")

        passed = result_b.ok and str(v.mode).upper() == original

        print("\n======================================================")
        print(" FINAL RESULT:", "PASS" if passed else "FAIL")
        print(" ROUND TRIP  :", f"{original} -> {target} -> {original}")
        print("======================================================")
        return 0 if passed else 6

    except KeyboardInterrupt:
        print("\n[VCT TEST] interrupted")
        return 130
    finally:
        stop.set()
        rx.join(timeout=1.0)
        streams.join(timeout=1.0)
        conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
