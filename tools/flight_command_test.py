#!/usr/bin/env python3
"""Kingphoenix standalone flight-command validation tool.

Development policy: prove each flight primitive here before the final GCS
mission recipe is allowed to depend on it. Do not run this tool at the same
time as main_v2_3_2.py because both would own the same FC serial link.
"""

from __future__ import annotations

import argparse
import time

from config import load_json
from core.runtime import KingphoenixRuntime


def _trace(event, data):
    details = " ".join(f"{k}={v}" for k, v in data.items())
    print(f"[VCT] {event}" + (f" | {details}" if details else ""))


def _wait_heartbeat(runtime, timeout_s):
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        v = runtime.state.vehicle
        if v.connected and v.heartbeat_age_s() <= 2.5:
            return True
        time.sleep(0.05)
    return False


def _state(runtime):
    v = runtime.state.vehicle
    alt, alt_source, alt_fresh = v.altitude_reference(0.30)
    print("\n=== KINGPHOENIX FLIGHT STATE ===")
    print(f"connected : {v.connected}")
    print(f"heartbeat : {v.heartbeat_age_s():.3f} s")
    print(f"mode      : {v.mode}")
    print(f"armed     : {v.armed}")
    print(f"altitude  : {alt} m ({alt_source}, fresh={alt_fresh})")
    print(f"rel_alt   : {v.relative_alt_m} m")
    print(f"LiDAR     : {v.rangefinder_distance_m} m")
    print(f"GPS       : fix={v.gps_fix_type} sats={v.gps_satellites}")
    print(f"battery   : {v.battery_v} V / {v.battery_remaining}%")
    try:
        with v.status_text_lock:
            if v.status_text_history:
                last = v.status_text_history[-1]
                print(f"FC message: [{last.severity}] {last.text}")
    except Exception:
        pass


def _result(result):
    print("\n=== VCT RESULT ===")
    print(f"outcome    : {result.outcome}")
    print(f"ok         : {result.ok}")
    print(f"attempts   : {result.attempts}")
    print(f"reason     : {result.reason}")
    print(f"ack_result : {result.ack_result}")
    print(f"ack_source : {result.ack_source}")
    print(f"elapsed_s  : {result.elapsed_s:.3f}")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    parser = argparse.ArgumentParser(
        description="Kingphoenix standalone VCT flight primitive test"
    )
    parser.add_argument(
        "action",
        choices=["status", "arm", "disarm", "mode", "takeoff", "land"],
    )
    parser.add_argument("--mode", dest="flight_mode", default=None,
                        help="Flight mode for action=mode, e.g. GUIDED or LOITER")
    parser.add_argument("--altitude", type=float, default=nominal_alt,
                        help=f"Takeoff altitude in metres (default {nominal_alt:.1f})")
    parser.add_argument("--heartbeat-timeout", type=float, default=8.0)
    parser.add_argument("--confirm-props-removed", action="store_true",
                        help="Required for standalone bench ARM")
    parser.add_argument("--confirm-flight-area-clear", action="store_true",
                        help="Required for real TAKEOFF; confirms props/airframe/area are flight-ready")
    args = parser.parse_args()

    if args.action == "arm" and not args.confirm_props_removed:
        raise SystemExit("REFUSED: bench ARM requires --confirm-props-removed")
    if args.action == "takeoff" and not args.confirm_flight_area_clear:
        raise SystemExit(
            "REFUSED: TAKEOFF requires --confirm-flight-area-clear. "
            "Use only after physical pre-flight inspection and a clear test area."
        )
    if args.action == "mode" and not args.flight_mode:
        raise SystemExit("REFUSED: action=mode requires --mode=<MODE>")

    runtime = KingphoenixRuntime()
    try:
        runtime.start()
        if not _wait_heartbeat(runtime, args.heartbeat_timeout):
            raise SystemExit("FAIL: no fresh FC heartbeat")

        _state(runtime)

        if args.action == "status":
            print("\nRESULT: READ-ONLY STATUS PASS")
            return
        if args.action == "arm":
            result = runtime.command.arm(trace_fn=_trace)
        elif args.action == "disarm":
            result = runtime.command.disarm(trace_fn=_trace)
        elif args.action == "mode":
            result = runtime.command.set_mode(args.flight_mode, trace_fn=_trace)
        elif args.action == "takeoff":
            result = runtime.command.takeoff(
                args.altitude,
                trace_fn=_trace,
                min_alt_m=float(defaults.get("takeoff_min_alt_m", 0.5)),
                max_alt_m=float(defaults.get("takeoff_max_alt_m", 3.0)),
                verify_ratio=float(defaults.get("takeoff_verify_ratio", 0.8)),
                state_timeout_s=float(defaults.get("takeoff_state_timeout_s", 10.0)),
            )
        else:
            result = runtime.command.land(trace_fn=_trace)

        _result(result)
        time.sleep(0.4)
        _state(runtime)
        if not result.ok:
            raise SystemExit(2)
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
