#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


def main():
    ap = argparse.ArgumentParser(description="Kingphoenix guarded takeoff + one AUTO waypoint test")
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=int, default=1, help="waypoint number from config/waypoints.json")
    ap.add_argument("--alt", type=float, default=1.0, help="takeoff and AUTO altitude in metres (default 1.0)")
    ap.add_argument("--margin", type=float, default=0.15, help="takeoff command overshoot margin (default +15%%)")
    ap.add_argument("--tolerance", type=float, default=0.05, help="LiDAR takeoff completion tolerance (m)")
    ap.add_argument("--hold", type=float, default=0.50, help="LiDAR stable time before AUTO transition (s)")
    ap.add_argument("--takeoff-timeout", type=float, default=12.0)
    ap.add_argument("--auto-timeout", type=float, default=30.0)
    ap.add_argument("--accept-radius", type=float, default=0.5, help="AUTO WP acceptance radius (m)")
    args = ap.parse_args()

    print("=================================================")
    print(" KINGPHOENIX TAKEOFF + AUTO WAYPOINT TEST")
    print("=================================================")

    # Hard-block before opening/arming if the requested waypoint is not populated.
    try:
        waypoint = DirectFlightTest.validate_waypoint(args.strategy, args.wp)
    except FlightTestError as e:
        print(f"[BLOCKED] {e}")
        print("[SAFE] No MAVLink command sent. Populate the waypoint first.")
        return 2

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        print(
            f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m "
            f"GPSfix={test.gps_fix} sats={test.gps_sats} "
            f"local=({test.local[0]:.2f},{test.local[1]:.2f},{test.local[2]:.2f})"
        )

        lat, lon, horizontal_m = test.local_waypoint_to_global(waypoint)
        if horizontal_m < 0.30:
            raise FlightTestError(
                f"{args.strategy}/WP{args.wp} is only {horizontal_m:.2f} m from current position; "
                "AUTO test blocked to avoid an ambiguous no-movement test"
            )
        if horizontal_m > 100.0:
            raise FlightTestError(
                f"{args.strategy}/WP{args.wp} is {horizontal_m:.1f} m away; "
                "AUTO test runner is limited to <=100 m"
            )

        print(
            f"[WAYPOINT] {args.strategy}/WP{args.wp} local=({waypoint['x']:.2f},{waypoint['y']:.2f}) "
            f"distance={horizontal_m:.2f}m"
        )
        print(f"[WAYPOINT] converted target lat={lat:.7f} lon={lon:.7f} alt={args.alt:.2f}m REL_HOME")

        # Upload/ACK the mission while still DISARMED. If upload fails, takeoff never happens.
        test.upload_single_auto_waypoint(
            lat,
            lon,
            args.alt,
            acceptance_radius_m=args.accept_radius,
        )

        test.takeoff_lidar_confirmed(
            args.alt,
            margin=args.margin,
            tolerance=args.tolerance,
            hold_s=args.hold,
            timeout=args.takeoff_timeout,
        )

        print("[AUTO] takeoff confirmed by LiDAR; transitioning GUIDED -> AUTO")
        test.start_auto_and_wait(timeout=args.auto_timeout)
        print("[RESULT] TAKEOFF + AUTO WAYPOINT TEST PASS")
        print("[NOTE] Waypoint reached. Pilot retains RC takeover; choose LOITER/LAND as appropriate.")
        return 0

    except KeyboardInterrupt:
        test.land_best_effort("operator interrupted test")
        return 130
    except FlightTestError as e:
        if test.armed:
            test.land_best_effort(str(e))
        print(f"[RESULT] AUTO TEST BLOCKED/FAILED: {e}")
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
