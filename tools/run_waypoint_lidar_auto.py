#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.check_waypoints import request_fc_mission
from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_hybrid_mission_test import fly_auto_to_wp, send_body_velocity

FC_HOME_SEQ = 0
FC_WP1_SEQ = 1


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(float(lat1))
    p2 = math.radians(float(lat2))
    dp = math.radians(float(lat2) - float(lat1))
    dl = math.radians(float(lon2) - float(lon1))
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def load_waypoint(strategy: str, wp_num: int) -> dict:
    data = load_json("waypoints.json")
    defaults = load_json("flight_defaults.json")
    default_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))
    strategy = str(strategy).upper()
    key = f"WP{int(wp_num)}"

    if strategy not in data or key not in data[strategy]:
        raise FlightTestError(f"unknown waypoint {strategy}/{key}")
    item = data[strategy][key]
    if item.get("lat") is None or item.get("lon") is None:
        raise FlightTestError(f"{strategy}/{key} has no saved lat/lon")

    lat = float(item["lat"])
    lon = float(item["lon"])
    alt_m = float(item.get("alt_m", default_alt))
    if not all(math.isfinite(v) for v in (lat, lon, alt_m)):
        raise FlightTestError(f"{strategy}/{key} contains invalid numeric data")
    return {"label": f"{strategy}/{key}", "lat": lat, "lon": lon, "alt_m": alt_m}


def upload_single_wp(test: DirectFlightTest, wp: dict, acceptance_radius_m: float, timeout: float = 10.0):
    if test.global_pos is None:
        raise FlightTestError("GLOBAL_POSITION_INT unavailable")

    home_lat = float(test.global_pos[0])
    home_lon = float(test.global_pos[1])
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = 2

    test.clear_mission()
    try:
        test.master.mav.mission_count_send(test.master.target_system, test.master.target_component, count, mtype)
    except TypeError:
        test.master.mav.mission_count_send(test.master.target_system, test.master.target_component, count)

    sent = set()
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        msg = test.poll(0.4)
        if msg is None:
            continue
        t = msg.get_type()

        if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            if seq == FC_HOME_SEQ:
                label = "HOME_SLOT"
                lat, lon, alt_m = home_lat, home_lon, 0.0
                p2, p4 = 0.0, 0.0
            elif seq == FC_WP1_SEQ:
                label = wp["label"]
                lat, lon, alt_m = float(wp["lat"]), float(wp["lon"]), float(wp["alt_m"])
                p2, p4 = float(acceptance_radius_m), float("nan")
            else:
                raise FlightTestError(f"FC requested invalid mission seq={seq}")

            args = [
                test.master.target_system,
                test.master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0.0,
                p2,
                0.0,
                p4,
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                alt_m,
            ]
            try:
                test.master.mav.mission_item_int_send(*args, mtype)
            except TypeError:
                test.master.mav.mission_item_int_send(*args)

            sent.add(seq)
            print(
                f"[MISSION] sent seq={seq} {label} lat={lat:.7f} lon={lon:.7f} relAlt={alt_m:.2f}m"
            )

        elif t == "MISSION_ACK":
            ack = int(getattr(msg, "type", -1))
            if ack != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission rejected ACK={ack}")
            if len(sent) != count:
                raise FlightTestError(f"mission ACK before all items sent ({len(sent)}/{count})")
            print("[MISSION] HOME + WP1 uploaded and ACKed")
            return

    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


def verify_single_wp(test: DirectFlightTest, wp: dict, position_tol_m: float = 0.30, alt_tol_m: float = 0.05):
    print("[MISSION] readback verification while DISARMED ...")
    items = request_fc_mission(test)
    if len(items) != 2:
        raise FlightTestError(f"FC mission count mismatch raw={len(items)} expected=2")

    actual = items[FC_WP1_SEQ]
    if int(actual["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
        raise FlightTestError("FC seq1 is not NAV_WAYPOINT")

    pos_err = distance_m(wp["lat"], wp["lon"], actual["lat"], actual["lon"])
    alt_err = abs(float(wp["alt_m"]) - float(actual["alt_m"]))
    print(
        f"[MISSION] verify {wp['label']} -> seq1 pos_err={pos_err:.3f}m "
        f"relAlt={actual['alt_m']:.2f}m alt_err={alt_err:.3f}m"
    )
    if pos_err > position_tol_m or alt_err > alt_tol_m:
        raise FlightTestError(
            f"mission mismatch {wp['label']}: pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
        )
    print("[MISSION] PRELOAD VERIFIED - HOME -> WP1")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description="Kingphoenix low-alt waypoint test: GUIDED LiDAR takeoff -> AUTO WP -> LAND"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=int, default=1)
    ap.add_argument("--alt", type=float, default=nominal_alt)
    ap.add_argument("--climb-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.25)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15)
    ap.add_argument("--accept-radius", type=float, default=0.5)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--wp-timeout", type=float, default=60.0)
    ap.add_argument("--preload-only", action="store_true")
    args = ap.parse_args()

    if args.wp < 1:
        raise SystemExit("--wp must be >=1")

    try:
        wp = load_waypoint(args.strategy, args.wp)
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    print("============================================================")
    print(" KINGPHOENIX LIDAR TAKEOFF -> AUTO WAYPOINT TEST")
    print("============================================================")
    print(
        f"[PLAN] GUIDED LiDAR TAKEOFF {args.alt:.2f}m -> AUTO {wp['label']} -> LAND"
    )
    print(
        f"[PLAN] climb max={args.climb_rate:.2f}m/s, taper={args.near_rate:.2f}m/s, "
        f"NO 2s hold, fixed kick={args.kick_alt:.2f}m"
    )
    print("[PLAN] AUTO mission owns horizontal WP only; it does NOT decide takeoff completion")

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")

        cur_lat, cur_lon, prearm_rel = test.global_pos
        d = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
        if d > 100.0:
            raise FlightTestError(f"{wp['label']} is {d:.1f}m away; test limit 100m")
        if d < 0.30:
            raise FlightTestError(f"{wp['label']} only {d:.2f}m away; test blocked")

        print(
            f"[WP] {wp['label']} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
            f"distance={d:.2f}m relAlt={wp['alt_m']:.2f}m"
        )
        print(f"[PREARM] FCrelAlt={prearm_rel:.2f}m DIAGNOSTIC ONLY")

        upload_single_wp(test, wp, float(args.accept_radius))
        verify_single_wp(test, wp)
        print("[SAFE] HOME -> WP1 verified while DISARMED")

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
            f"FCrelAlt={rel if rel is not None else float('nan'):.2f}m -> AUTO WP"
        )

        fly_auto_to_wp(
            test,
            seq=FC_WP1_SEQ,
            label=wp["label"],
            floor_m=float(args.auto_floor),
            ceiling_m=float(args.auto_ceiling),
            timeout=float(args.wp_timeout),
        )

        test.land_and_wait()
        print("[RESULT] GUIDED LIDAR TAKEOFF -> AUTO WP -> LAND PASS")
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
