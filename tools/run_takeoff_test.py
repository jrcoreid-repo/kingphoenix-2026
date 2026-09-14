#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


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
        print(f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m mode={test.mode} armed={test.armed}")
        test.takeoff_lidar_confirmed(
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
