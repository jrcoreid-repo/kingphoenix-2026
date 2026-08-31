from __future__ import annotations

import argparse
import time

from core.runtime import KingphoenixRuntime


def _trace(event, data):
    details = " ".join(f"{k}={v}" for k, v in data.items())
    print(f"[VCT] {event}" + (f" | {details}" if details else ""))


def _wait_heartbeat(runtime, timeout_s):
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if runtime.state.vehicle.connected and runtime.state.vehicle.heartbeat_age_s() <= 2.5:
            return True
        time.sleep(0.05)
    return False


def _print_state(runtime):
    v = runtime.state.vehicle
    print("\n=== KINGPHOENIX MAVLINK BENCH STATE ===")
    print(f"connected : {v.connected}")
    print(f"heartbeat : {v.heartbeat_age_s():.3f} s")
    print(f"mode      : {v.mode}")
    print(f"armed     : {v.armed}")
    print(f"battery   : {getattr(v, 'battery_v', None)} V")
    print(f"current   : {getattr(v, 'battery_current_a', None)} A")
    print(f"GPS       : fix={getattr(v, 'gps_fix_type', None)} sat={getattr(v, 'gps_satellites', None)}")
    print(f"LiDAR     : {getattr(v, 'rangefinder_distance_m', None)} m")


def main():
    parser = argparse.ArgumentParser(
        description="Kingphoenix V2.2.2 explicit VCT ARM/DISARM bench tool"
    )
    parser.add_argument("action", choices=["status", "arm", "disarm"])
    parser.add_argument("--heartbeat-timeout", type=float, default=8.0)
    parser.add_argument(
        "--confirm-props-removed",
        action="store_true",
        help="Required for ARM. Confirms the vehicle is physically safe for a bench arm test.",
    )
    args = parser.parse_args()

    if args.action == "arm" and not args.confirm_props_removed:
        raise SystemExit(
            "REFUSED: ARM bench test requires --confirm-props-removed. "
            "Remove propellers / make propulsion physically safe first."
        )

    runtime = KingphoenixRuntime()
    try:
        runtime.start()

        if not _wait_heartbeat(runtime, args.heartbeat_timeout):
            raise SystemExit("FAIL: no fresh FC heartbeat")

        _print_state(runtime)

        if args.action == "status":
            print("\nRESULT: READ-ONLY STATUS PASS")
            return

        if args.action == "arm":
            print("\n[ACTION] ARM via CommandManager -> VCT")
            result = runtime.command.arm(trace_fn=_trace)
        else:
            print("\n[ACTION] DISARM via CommandManager -> VCT")
            result = runtime.command.disarm(trace_fn=_trace)

        print("\n=== VCT RESULT ===")
        print(f"outcome    : {result.outcome}")
        print(f"ok         : {result.ok}")
        print(f"attempts   : {result.attempts}")
        print(f"reason     : {result.reason}")
        print(f"ack_result : {result.ack_result}")
        print(f"ack_source : {result.ack_source}")
        print(f"elapsed_s  : {result.elapsed_s:.3f}")

        time.sleep(0.3)
        _print_state(runtime)

        if not result.ok:
            raise SystemExit(2)
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
