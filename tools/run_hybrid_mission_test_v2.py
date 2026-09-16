#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_hybrid_mission_test import (
    FC_WP1_SEQ,
    FC_WP2_SEQ,
    distance_m,
    fly_auto_to_wp,
    guided_lateral_cruise,
    load_waypoint,
    send_body_velocity,
    upload_two_wp_route,
    verify_two_wp_route,
)


def main():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description=(
            "Kingphoenix low-alt hybrid V2: profiled LiDAR GUIDED takeoff -> "
            "AUTO WP1 -> GUIDED lateral -> AUTO WP2 -> LAND"
        )
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp1", type=int, default=1)
    ap.add_argument("--wp2", type=int, default=2)
    ap.add_argument("--alt", type=float, default=nominal_alt)
    ap.add_argument("--climb-rate", type=float, default=0.45, help="maximum/far climb rate")
    ap.add_argument("--near-rate", type=float, default=0.20, help="climb rate inside final taper zone")
    ap.add_argument("--taper-zone", type=float, default=0.25, help="meters before LiDAR trigger to slow climb")
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0, help="fixed post-ARM NAV_TAKEOFF launch kick")
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15, help="zero-velocity command flush; not a hover hold")
    ap.add_argument("--cruise-speed", type=float, default=0.20)
    ap.add_argument("--cruise-duration", type=float, default=2.0)
    ap.add_argument("--accept-radius", type=float, default=0.5)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--wp-timeout", type=float, default=60.0)
    ap.add_argument("--preload-only", action="store_true")
    args = ap.parse_args()

    if args.wp1 < 1 or args.wp2 < 1 or args.wp1 == args.wp2:
        raise SystemExit("--wp1 and --wp2 must be different positive waypoint numbers")
    if not 0.50 <= float(args.alt) <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if not 0.10 <= float(args.climb_rate) <= 0.80:
        raise SystemExit("--climb-rate must be 0.10..0.80m/s")
    if not 0.08 <= float(args.near_rate) <= float(args.climb_rate):
        raise SystemExit("--near-rate must be 0.08m/s..--climb-rate")
    if not 0.05 <= float(args.taper_zone) <= 0.60:
        raise SystemExit("--taper-zone must be 0.05..0.60m")
    if not 0.05 <= float(args.cruise_speed) <= 0.50:
        raise SystemExit("--cruise-speed must be 0.05..0.50m/s")
    if not 0.5 <= float(args.cruise_duration) <= 5.0:
        raise SystemExit("--cruise-duration must be 0.5..5.0s")
    if args.auto_floor >= args.auto_ceiling:
        raise SystemExit("--auto-floor must be below --auto-ceiling")
    if args.takeoff_ceiling <= args.alt:
        raise SystemExit("--takeoff-ceiling must be above --alt")

    try:
        wp1 = load_waypoint(args.strategy, args.wp1)
        wp2 = load_waypoint(args.strategy, args.wp2)
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    route_distance = distance_m(wp1["lat"], wp1["lon"], wp2["lat"], wp2["lon"])
    if route_distance < 0.50:
        print(f"[BLOCKED] WP1 -> WP2 only {route_distance:.2f}m; need >=0.50m")
        return 2

    print("============================================================")
    print(" KINGPHOENIX LOW-ALT HYBRID CONTROL TEST V2")
    print("============================================================")
    print(
        f"[PLAN] GUIDED LiDAR TAKEOFF {args.alt:.2f}m | max {args.climb_rate:.2f}m/s "
        f"-> taper {args.near_rate:.2f}m/s -> AUTO {wp1['label']} "
        f"-> GUIDED R/L -> AUTO {wp2['label']} -> LAND"
    )
    print("[PLAN] fixed post-ARM kick; PREARM relAlt NEVER changes launch target")
    print("[PLAN] NO 2s hold; only short zero-command flush before AUTO")
    print("[PLAN] mission contains HOME + WP1 + WP2; AUTO never owns takeoff completion")

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")

        cur_lat, cur_lon, prearm_rel = test.global_pos
        for wp in (wp1, wp2):
            d = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
            if d > 100.0:
                raise FlightTestError(f"{wp['label']} is {d:.1f}m away; test limit 100m")
            print(
                f"[WP] {wp['label']} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={d:.2f}m relAlt={wp['alt_m']:.2f}m"
            )
        print(f"[PREARM] FCrelAlt={prearm_rel:.2f}m DIAGNOSTIC ONLY")

        upload_two_wp_route(test, wp1, wp2, acceptance_radius_m=float(args.accept_radius))
        verify_two_wp_route(test, wp1, wp2)
        print("[SAFE] HOME -> WP1 -> WP2 verified while DISARMED")

        if args.preload_only:
            print("[PRELOAD-ONLY] verified; vehicle remains DISARMED")
            return 0

        profiled_lidar_takeoff(
            test,
            float(args.alt),
            tolerance_m=float(args.tolerance),
            handoff_m=float(args.handoff),
            fast_rate_mps=float(args.climb_rate),
            near_rate_mps=float(args.near_rate),
            taper_zone_m=float(args.taper_zone),
            kick_alt_m=float(args.kick_alt),
            timeout_s=float(args.takeoff_timeout),
            ceiling_m=float(args.takeoff_ceiling),
            zero_flush_s=float(args.zero_flush),
        )

        rel = None if test.global_pos is None else test.global_pos[2]
        print(
            f"[POST-TAKEOFF] physical LiDAR={test.lidar:.2f}m "
            f"FCrelAlt={rel if rel is not None else float('nan'):.2f}m -> AUTO WP1"
        )

        fly_auto_to_wp(
            test,
            seq=FC_WP1_SEQ,
            label=wp1["label"],
            floor_m=float(args.auto_floor),
            ceiling_m=float(args.auto_ceiling),
            timeout=float(args.wp_timeout),
        )

        guided_lateral_cruise(
            test,
            speed_mps=float(args.cruise_speed),
            duration_s=float(args.cruise_duration),
            floor_m=float(args.auto_floor),
            ceiling_m=float(args.auto_ceiling),
        )

        fly_auto_to_wp(
            test,
            seq=FC_WP2_SEQ,
            label=wp2["label"],
            floor_m=float(args.auto_floor),
            ceiling_m=float(args.auto_ceiling),
            timeout=float(args.wp_timeout),
        )

        test.land_and_wait()
        print("[RESULT] HYBRID V2 GUIDED TAKEOFF -> AUTO WP1 -> GUIDED R/L -> AUTO WP2 -> LAND PASS")
        return 0

    except KeyboardInterrupt:
        try:
            send_body_velocity(test, vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
        except Exception:
            pass
        test.land_best_effort("operator interrupted")
        return 130
    except FlightTestError as exc:
        try:
            send_body_velocity(test, vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
        except Exception:
            pass
        test.land_best_effort(str(exc))
        print(f"[RESULT] BLOCKED/FAILED: {exc}")
        return 2
    except Exception as exc:
        try:
            send_body_velocity(test, vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
        except Exception:
            pass
        test.land_best_effort(f"unexpected {type(exc).__name__}: {exc}")
        print(f"[RESULT] UNEXPECTED ERROR: {type(exc).__name__}: {exc}")
        return 3
    finally:
        test.close()


if __name__ == "__main__":
    raise SystemExit(main())
