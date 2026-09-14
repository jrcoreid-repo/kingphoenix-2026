#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


def _fmt(value):
    return "--" if value is None else f"{float(value):.2f}"


def takeoff_with_diagnostics(test: DirectFlightTest, target_m: float, *, margin: float,
                             tolerance: float, hold_s: float, timeout: float):
    target_m = float(target_m)
    if not 0.50 <= target_m <= 3.0:
        raise FlightTestError("takeoff altitude must be 0.50-3.00 m")

    command_m = target_m * (1.0 + float(margin))
    threshold = target_m - float(tolerance)

    print(f"[TAKEOFF] requested={target_m:.2f}m command={command_m:.2f}m (+{margin*100:.0f}%)")
    test.set_mode("GUIDED")
    test.arm()

    # Send ArduCopter GUIDED takeoff. Completion is still LiDAR-only, but this
    # runner now shows the FC altitude estimators and COMMAND_ACK for diagnosis.
    test.master.mav.command_long_send(
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, float("nan"), 0, 0, command_m,
    )

    deadline = time.monotonic() + float(timeout)
    reached_since = None
    lidar_loss_since = None
    last_print = 0.0
    takeoff_ack = None

    while time.monotonic() < deadline:
        msg = test.poll(0.10)
        now = time.monotonic()

        if msg is not None and msg.get_type() == "COMMAND_ACK":
            command = int(getattr(msg, "command", -1))
            if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                takeoff_ack = int(getattr(msg, "result", -1))
                print(f"\n[ACK] NAV_TAKEOFF result={takeoff_ack}")
                if takeoff_ack not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                ):
                    raise FlightTestError(f"NAV_TAKEOFF rejected/result={takeoff_ack}")

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during takeoff")

        # If the FC leaves GUIDED during the climb, stop the test immediately.
        # A spontaneous transition to LAND/ALT_HOLD/etc. is more important than
        # waiting for the LiDAR timeout.
        if test.mode != "GUIDED":
            raise FlightTestError(f"flight mode changed during takeoff: {test.mode}")

        rel_alt = None if test.global_pos is None else test.global_pos[2]
        local_up = None if test.local is None else -float(test.local[2])

        if now - last_print >= 0.20:
            print(
                f"[LIVE] mode={test.mode:<8} armed={str(test.armed):<5} "
                f"LiDAR={_fmt(test.lidar)}m relAlt={_fmt(rel_alt)}m "
                f"localUp={_fmt(local_up)}m target>={threshold:.2f}m "
                f"ACK={takeoff_ack if takeoff_ack is not None else '--'}"
            )
            last_print = now

        if test.lidar_ok():
            lidar_loss_since = None
            if test.lidar >= threshold:
                reached_since = reached_since or now
                if now - reached_since >= float(hold_s):
                    print(f"[TAKEOFF] COMPLETE LiDAR={test.lidar:.2f}m stable={hold_s:.2f}s")
                    return
            else:
                reached_since = None
        else:
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR invalid/stale >0.60 s during takeoff")

    raise FlightTestError(
        "takeoff timeout; "
        f"LiDAR={_fmt(test.lidar)}m relAlt={_fmt(None if test.global_pos is None else test.global_pos[2])}m "
        f"localUp={_fmt(None if test.local is None else -float(test.local[2]))}m "
        f"mode={test.mode} ACK={takeoff_ack if takeoff_ack is not None else '--'}"
    )


def main():
    ap = argparse.ArgumentParser(description="Kingphoenix DEV guarded takeoff -> LAND test")
    ap.add_argument("--alt", type=float, default=1.2, help="true takeoff target from LiDAR, metres")
    ap.add_argument("--margin", type=float, default=0.15, help="takeoff command compensation; default +15%%")
    ap.add_argument("--tolerance", type=float, default=0.05, help="LiDAR completion tolerance, metres")
    ap.add_argument("--hold", type=float, default=0.50, help="LiDAR stable time at target")
    ap.add_argument("--timeout", type=float, default=12.0, help="takeoff timeout")
    args = ap.parse_args()

    test = DirectFlightTest()
    print("=================================================")
    print(" KINGPHOENIX DEV TAKEOFF -> LAND TEST")
    print("=================================================")
    print(f"[PLAN] true target={args.alt:.2f}m | FC command={args.alt*(1+args.margin):.2f}m")

    try:
        test.open()
        test.collect_preflight(require_auto=False)
        rel_alt = None if test.global_pos is None else test.global_pos[2]
        local_up = None if test.local is None else -float(test.local[2])
        print(
            f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m mode={test.mode} armed={test.armed} "
            f"relAlt={_fmt(rel_alt)}m localUp={_fmt(local_up)}m"
        )
        takeoff_with_diagnostics(
            test,
            args.alt,
            margin=args.margin,
            tolerance=args.tolerance,
            hold_s=args.hold,
            timeout=args.timeout,
        )
        test.land_and_wait()
        print("[RESULT] TAKEOFF -> LAND TEST PASS")
        return 0
    except KeyboardInterrupt:
        test.land_best_effort("operator interrupted")
        return 130
    except FlightTestError as exc:
        test.land_best_effort(str(exc))
        print(f"[RESULT] BLOCKED/FAILED: {exc}")
        return 2
    except Exception as exc:
        test.land_best_effort(f"unexpected {type(exc).__name__}: {exc}")
        print(f"[RESULT] UNEXPECTED ERROR: {type(exc).__name__}: {exc}")
        return 3
    finally:
        test.close()


if __name__ == "__main__":
    raise SystemExit(main())
