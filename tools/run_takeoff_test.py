#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


def main():
    ap = argparse.ArgumentParser(description="Kingphoenix guarded takeoff-only test")
    ap.add_argument("--alt", type=float, default=1.0, help="target LiDAR altitude in metres (default 1.0)")
    ap.add_argument("--margin", type=float, default=0.15, help="takeoff command overshoot margin (default 0.15 = +15%%)")
    ap.add_argument("--tolerance", type=float, default=0.05, help="LiDAR completion tolerance below target (default 0.05 m)")
    ap.add_argument("--hold", type=float, default=0.50, help="time LiDAR must remain above threshold (default 0.50 s)")
    ap.add_argument("--timeout", type=float, default=12.0, help="takeoff timeout seconds")
    args = ap.parse_args()

    test = DirectFlightTest()
    print("=================================================")
    print(" KINGPHOENIX TAKEOFF TEST - LiDAR CONFIRMED")
    print("=================================================")
    try:
        test.open()
        test.collect_preflight(require_auto=False)
        print(f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m mode={test.mode} armed={test.armed}")
        test.takeoff_lidar_confirmed(
            args.alt,
            margin=args.margin,
            tolerance=args.tolerance,
            hold_s=args.hold,
            timeout=args.timeout,
        )
        print("[RESULT] TAKEOFF TEST PASS - holding in GUIDED")
        print("[NOTE] Pilot retains RC takeover. Land manually or use LAND mode when finished.")
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
