#!/usr/bin/env python3
"""Kingphoenix V1.7 ARM/DISARM VCT validation."""

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
from flight.mavlink_result import mav_result_name, format_mav_result



def line(label, value):
    print(f"{label:<20}: {value}")


def yn(value):
    return "YES" if bool(value) else "NO"


def wait_connected(state, timeout_s):
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if state.vehicle.connected:
            return True
        time.sleep(0.05)
    return False


def make_trace(vehicle):
    def trace(event, data):
        if event == "START":
            print("\n[VCT] transaction started")
        elif event == "PRECHECK":
            print(
                f"[VCT] PRECHECK       : {'PASS' if data['ok'] else 'FAIL'}"
                + (f" ({data['reason']})" if data.get("reason") else "")
            )
        elif event == "ALREADY_SATISFIED":
            print("[VCT] STATE          : already satisfied")
        elif event == "ATTEMPT":
            print(f"[VCT] ATTEMPT        : {data['attempt']}/{data['total']}")
        elif event == "SEND":
            print(f"[VCT] TX             : {'SENT' if data['ok'] else 'FAILED'}")
        elif event == "ACK":
            if data["received"]:
                result = data["result"]
                print(
                    f"[VCT] ACK            : {result} "
                    f"({mav_result_name(result)}) "
                    f"source={data['source']}"
                )
            else:
                print("[VCT] ACK            : TIMEOUT")
        elif event == "REJECTED":
            result = data["result"]
            print(
                f"[VCT] FC REJECTION   : {result} "
                f"({mav_result_name(result)}) "
                f"source={data['source']}"
            )
        elif event == "STATE_VERIFY":
            print(f"[VCT] ARMED STATE    : {vehicle.armed}")
            print(f"[VCT] VERIFY         : {'PASS' if data['ok'] else 'FAIL'}")
        elif event == "BACKOFF":
            print(
                f"[VCT] RETRY          : {data['seconds']:.2f}s "
                f"({data['reason']})"
            )
        elif event == "FAILED":
            print(f"[VCT] FAILED         : {data['reason']}")
    return trace


def print_status_texts(vehicle, since_ns, target_system, target_component):
    time.sleep(0.35)
    items = vehicle.recent_status_texts(
        since_ns=since_ns,
        source_system=target_system,
        source_component=target_component,
    )

    print("\n--- FC STATUSTEXT SINCE ARM REQUEST ---")
    if not items:
        print("(none captured)")
        return

    seen = set()
    for item in items:
        key = (item.severity, item.text)
        if key in seen:
            continue
        seen.add(key)
        print(f"severity={item.severity:<3} | {item.text}")


def main():
    ap = argparse.ArgumentParser()
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--expect-reject",
        action="store_true",
        help="Incomplete bench hardware: explicit FC rejection is expected.",
    )
    group.add_argument(
        "--expect-success",
        action="store_true",
        help="Later complete hardware: ARM then verified DISARM is expected.",
    )
    ap.add_argument("--wait-heartbeat", type=float, default=6.0)
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
    streams = MavlinkStreamConfigurator(
        tx,
        state,
        stop,
        mav_cfg.get("message_rates_hz", {}),
        retry_interval_s=mav_cfg.get("rate_retry_interval_s", 10.0),
    )

    try:
        print("======================================================")
        print(" KINGPHOENIX V1.7.1 - ARM/DISARM VCT VALIDATION")
        print(" NO TAKEOFF / NO VELOCITY / NO POSITION COMMANDS")
        print("======================================================")
        line("EXPECTATION", "REJECTION" if args.expect_reject else "SUCCESS")

        conn.open()
        rx.start()
        streams.start()

        if not wait_connected(state, args.wait_heartbeat):
            print("\nFINAL RESULT: FAIL - no autopilot heartbeat")
            return 2

        time.sleep(0.35)
        v = state.vehicle

        print("\n--- PRE-FLIGHT STATE ---")
        line("CONNECTED", yn(v.connected))
        line("ARMED", yn(v.armed))
        line("HEARTBEAT", freshness.check("HEARTBEAT").reason.upper())
        line("AUTOPILOT", f"{v.autopilot_system_id}:{v.autopilot_component_id}")
        line("KINGPHOENIX", f"{conn.source_system}:{conn.source_component}")
        line("TARGET", f"{conn.target_system}:{conn.target_component}")
        line("MODE", v.mode)

        if v.armed:
            print("\nFINAL RESULT: REFUSED - vehicle already ARMED")
            return 3

        print("\n*** SAFETY ***")
        print("This command WILL request ARM from the FC.")
        print("Use only with the aircraft physically safe for an ARM request.")
        print("No TAKEOFF or movement target is sent by this tool.")

        arm_start_ns = time.monotonic_ns()

        print("\n--- ARM TRANSACTION ---")
        arm_result = command.arm(trace_fn=make_trace(v))

        print("\n--- ARM RESULT ---")
        line("OUTCOME", arm_result.outcome)
        line("OK", yn(arm_result.ok))
        line("ATTEMPTS", arm_result.attempts)
        line(
            "ACK",
            "--" if arm_result.ack_result is None else (
                f"{arm_result.ack_result} "
                f"({mav_result_name(arm_result.ack_result)})"
            ),
        )
        line("ACK SOURCE", arm_result.ack_source or "--")
        line("ARMED STATE", yn(v.armed))
        line("REASON", arm_result.reason)
        line("ELAPSED", f"{arm_result.elapsed_s:.3f} s")

        print_status_texts(
            v,
            arm_start_ns,
            conn.target_system,
            conn.target_component,
        )

        if args.expect_reject:
            if arm_result.outcome == "REJECTED" and not v.armed:
                print("\n======================================================")
                print(" FINAL RESULT: PASS")
                print(" EXPECTED FC REJECTION CORRECTLY DETECTED")
                print(" VEHICLE REMAINED DISARMED")
                print("======================================================")
                return 0

            if v.armed:
                print("\n!!! UNEXPECTED ARM SUCCESS - REQUESTING VCT DISARM !!!")
                disarm_result = command.disarm(trace_fn=make_trace(v))
                time.sleep(0.3)
                line("DISARM OUTCOME", disarm_result.outcome)
                line("FINAL ARMED", yn(v.armed))
                print("\nFINAL RESULT: FAIL - ARM succeeded while rejection was expected")
                return 5

            print("\nFINAL RESULT: FAIL - rejection was not cleanly classified")
            return 6

        # Future complete-aircraft path.
        if not arm_result.ok or not v.armed:
            print("\nFINAL RESULT: FAIL - ARM success was expected")
            return 7

        print("\n--- DISARM TRANSACTION ---")
        disarm_result = command.disarm(trace_fn=make_trace(v))
        time.sleep(0.3)

        print("\n--- DISARM RESULT ---")
        line("OUTCOME", disarm_result.outcome)
        line("OK", yn(disarm_result.ok))
        line("ATTEMPTS", disarm_result.attempts)
        line(
            "ACK",
            "--" if disarm_result.ack_result is None else (
                f"{disarm_result.ack_result} "
                f"({mav_result_name(disarm_result.ack_result)})"
            ),
        )
        line("ARMED STATE", yn(v.armed))
        line("REASON", disarm_result.reason)

        passed = disarm_result.ok and not v.armed

        print("\n======================================================")
        print(" FINAL RESULT:", "PASS" if passed else "FAIL")
        print(" ARM/DISARM VCT:", "VALIDATED" if passed else "NOT VALIDATED")
        print("======================================================")
        return 0 if passed else 8

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
