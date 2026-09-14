#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


def land_and_wait(test: DirectFlightTest, timeout: float = 20.0):
    """Switch to LAND after a successful takeoff test and wait for disarm."""
    print("[LAND] takeoff test complete; switching GUIDED -> LAND")
    test.set_mode("LAND")

    deadline = time.monotonic() + float(timeout)
    last_print = 0.0
    while time.monotonic() < deadline:
        test.poll(0.15)
        now = time.monotonic()

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost while waiting for LAND completion")

        if now - last_print >= 0.5:
            lidar_text = f"{test.lidar:.2f}m" if test.lidar_ok() else "--"
            print(f"[LAND] mode={test.mode} armed={test.armed} LiDAR={lidar_text}")
            last_print = now

        if not test.armed:
            print("[LAND] COMPLETE vehicle DISARMED")
            return

    raise FlightTestError(
        f"LAND timeout: vehicle still armed after {float(timeout):.1f} s"
    )


def main():
    ap = argparse.ArgumentParser(
        description="Kingphoenix guarded takeoff + automatic LAND test"
    )
    ap.add_argument(
        "--alt",
        type=float,
        default=1.0,
        help="target LiDAR altitude in metres (default 1.0)",
    )
    ap.add_argument(
        "--margin",
        type=float,
        default=0.15,
        help="takeoff command overshoot margin (default 0.15 = +15%%)",
    )
    ap.add_argument(
        "--tolerance",
        type=float,
        default=0.05,
        help="LiDAR completion tolerance below target (default 0.05 m)",
    )
    ap.add_argument(
        "--hold",
        type=float,
        default=0.50,
        help="time LiDAR must remain above threshold (default 0.50 s)",
    )
    ap.add_argument("--timeout", type=float, default=12.0, help="takeoff timeout seconds")
    ap.add_argument(
        "--land-timeout",
        type=float,
        default=20.0,
        help="maximum wait for LAND + automatic disarm (default 20 s)",
    )
    args = ap.parse_args()

    test = DirectFlightTest()
    print("=================================================")
    print(" KINGPHOENIX TAKEOFF + LAND TEST - LiDAR CONFIRMED")
    print("=================================================")
    print(f"[PLAN] takeoff target : {args.alt:.2f} m")
    print("[PLAN] finish         : automatic LAND -> DISARM")

    try:
        test.open()
        test.collect_preflight(require_auto=False)
        print(
            f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m "
            f"mode={test.mode} armed={test.armed}"
        )

        test.takeoff_lidar_confirmed(
            args.alt,
            margin=args.margin,
            tolerance=args.tolerance,
            hold_s=args.hold,
            timeout=args.timeout,
        )

        print("[TAKEOFF] target confirmed by LiDAR; test objective complete")
        land_and_wait(test, timeout=args.land_timeout)
        print("[RESULT] TAKEOFF + LAND TEST PASS")
        return 0

    except KeyboardInterrupt:
        test.land_best_effort("operator interrupted test")
        return 130
    except FlightTestError as e:
        if test.armed:
            test.land_best_effort(str(e))
        print(f"[RESULT] TAKEOFF TEST BLOCKED/FAILED: {e}")
        return 2
    except Exception as e:
        if test.armed:
            test.land_best_effort(f"unexpected error: {e}")
        print(f"[RESULT] UNEXPECTED ERROR: {type(e).__name__}: {e}")
        return 3
    finally:
        test.close()


if __name__ == "__main__":
    raise SystemExit(main())
