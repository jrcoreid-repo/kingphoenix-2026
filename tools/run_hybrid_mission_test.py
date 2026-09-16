#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json
from tools.check_waypoints import request_fc_mission
from tools.run_takeoff_lidar_test import lidar_authoritative_takeoff

# Hybrid mission layout:
#   FC seq0 = reserved/internal HOME
#   FC seq1 = WP1
#   FC seq2 = WP2
#
# Physical launch is performed in GUIDED by the proven LiDAR-authoritative
# takeoff controller, so there is intentionally NO NAV_TAKEOFF mission item.
FC_HOME_SEQ = 0
FC_WP1_SEQ = 1
FC_WP2_SEQ = 2


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

    try:
        lat = float(item["lat"])
        lon = float(item["lon"])
        alt_m = float(item.get("alt_m", default_alt))
    except Exception as exc:
        raise FlightTestError(f"invalid {strategy}/{key}: {exc}")

    if not all(math.isfinite(v) for v in (lat, lon, alt_m)):
        raise FlightTestError(f"{strategy}/{key} contains non-finite values")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise FlightTestError(f"{strategy}/{key} lat/lon out of range")
    if lat == 0.0 and lon == 0.0:
        raise FlightTestError(f"{strategy}/{key} is 0,0")
    if not 0.30 <= alt_m <= 5.0:
        raise FlightTestError(f"{strategy}/{key} altitude {alt_m:.2f}m outside test limits")

    return {"label": f"{strategy}/{key}", "lat": lat, "lon": lon, "alt_m": alt_m}


def upload_two_wp_route(
    test: DirectFlightTest,
    wp1: dict,
    wp2: dict,
    *,
    acceptance_radius_m: float,
    timeout: float = 10.0,
):
    """Upload reserved HOME + WP1 + WP2 as REL_HOME MISSION_ITEM_INT."""
    if test.global_pos is None:
        raise FlightTestError("GLOBAL_POSITION_INT unavailable for mission upload")

    home_lat = float(test.global_pos[0])
    home_lon = float(test.global_pos[1])
    route = [wp1, wp2]

    test.clear_mission()
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = 3

    try:
        test.master.mav.mission_count_send(
            test.master.target_system,
            test.master.target_component,
            count,
            mtype,
        )
    except TypeError:
        test.master.mav.mission_count_send(
            test.master.target_system,
            test.master.target_component,
            count,
        )

    sent = set()
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        msg = test.poll(0.4)
        if msg is None:
            continue

        t = msg.get_type()
        if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            if seq < 0 or seq >= count:
                raise FlightTestError(f"FC requested invalid mission seq={seq}")

            if seq == FC_HOME_SEQ:
                label = "HOME_SLOT"
                lat, lon, alt_m = home_lat, home_lon, 0.0
                p2 = 0.0
            else:
                wanted = route[seq - 1]
                label = wanted["label"]
                lat = float(wanted["lat"])
                lon = float(wanted["lon"])
                alt_m = float(wanted["alt_m"])
                p2 = float(acceptance_radius_m)

            args = [
                test.master.target_system,
                test.master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # current selected explicitly before each AUTO leg
                1,  # autocontinue
                0.0,
                p2,
                0.0,
                float("nan") if seq != FC_HOME_SEQ else 0.0,
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                alt_m,
            ]
            try:
                test.master.mav.mission_item_int_send(*args, mtype)
            except TypeError:
                test.master.mav.mission_item_int_send(*args)

            sent.add(seq)
            req = "INT" if t == "MISSION_REQUEST_INT" else "LEGACY->INT"
            print(
                f"[MISSION] sent seq={seq} {label} lat={lat:.7f} lon={lon:.7f} "
                f"relAlt={alt_m:.2f}m request={req}"
            )

        elif t == "MISSION_ACK":
            ack = int(getattr(msg, "type", -1))
            if ack != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission rejected ACK={ack}")
            if len(sent) != count:
                raise FlightTestError(f"mission ACK before all items sent ({len(sent)}/{count})")
            print("[MISSION] HOME + WP1 + WP2 uploaded and ACKed")
            return

    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


def verify_two_wp_route(test: DirectFlightTest, wp1: dict, wp2: dict, pos_tol_m=0.30, alt_tol_m=0.05):
    print("[MISSION] readback verification while DISARMED ...")
    items = request_fc_mission(test)
    if len(items) != 3:
        raise FlightTestError(f"FC mission count mismatch raw={len(items)} expected=3")

    for seq, wanted in ((FC_WP1_SEQ, wp1), (FC_WP2_SEQ, wp2)):
        actual = items[seq]
        if int(actual["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            raise FlightTestError(f"FC seq={seq} is not NAV_WAYPOINT")
        pos_err = distance_m(wanted["lat"], wanted["lon"], actual["lat"], actual["lon"])
        alt_err = abs(float(wanted["alt_m"]) - float(actual["alt_m"]))
        print(
            f"[MISSION] verify {wanted['label']} -> seq={seq} "
            f"pos_err={pos_err:.3f}m relAlt={actual['alt_m']:.2f}m alt_err={alt_err:.3f}m"
        )
        if pos_err > float(pos_tol_m) or alt_err > float(alt_tol_m):
            raise FlightTestError(
                f"mission mismatch {wanted['label']}: pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
            )

    print("[MISSION] PRELOAD VERIFIED - HOME -> WP1 -> WP2")


def set_mission_current(test: DirectFlightTest, seq: int, label: str, timeout: float = 2.0):
    test.mission_current = None
    test.master.mav.mission_set_current_send(
        test.master.target_system,
        test.master.target_component,
        int(seq),
    )

    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        test.poll(0.10)
        if test.mission_current == int(seq):
            print(f"[MISSION] current -> seq={seq} {label}")
            return
    raise FlightTestError(f"MISSION_SET_CURRENT seq={seq} {label} not confirmed")


def check_flight_health(test: DirectFlightTest):
    if not test.heartbeat_ok():
        raise FlightTestError("heartbeat lost")
    if test.gps_fix is not None and test.gps_fix < 3:
        raise FlightTestError(f"GPS fix degraded (fix={test.gps_fix})")


def check_lidar_guard(
    test: DirectFlightTest,
    *,
    floor_m: float,
    ceiling_m: float,
    outside_since: float | None,
    max_outside_s: float = 0.35,
) -> float | None:
    now = time.monotonic()
    if not test.lidar_ok():
        raise FlightTestError("LiDAR missing/stale during hybrid flight")

    agl = float(test.lidar)
    if agl < float(floor_m) or agl > float(ceiling_m):
        outside_since = outside_since or now
        if now - outside_since > float(max_outside_s):
            raise FlightTestError(
                f"LiDAR AGL out of guard: {agl:.2f}m guard={floor_m:.2f}..{ceiling_m:.2f}m"
            )
    else:
        outside_since = None
    return outside_since


def fly_auto_to_wp(
    test: DirectFlightTest,
    *,
    seq: int,
    label: str,
    floor_m: float,
    ceiling_m: float,
    timeout: float,
):
    set_mission_current(test, seq, label)
    test.reached_seq = None
    test.set_mode("AUTO")
    print(f"[AUTO] flying to {label} seq={seq}")

    outside_since = None
    last_print = 0.0
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        test.poll(0.05)
        check_flight_health(test)
        outside_since = check_lidar_guard(
            test,
            floor_m=floor_m,
            ceiling_m=ceiling_m,
            outside_since=outside_since,
        )

        now = time.monotonic()
        if now - last_print >= 0.50:
            rel = None if test.global_pos is None else test.global_pos[2]
            print(
                f"[AUTO:{label}] current={test.mission_current} "
                f"LiDAR={test.lidar:.2f}m relAlt={rel if rel is not None else float('nan'):.2f}m"
            )
            last_print = now

        if test.reached_seq == int(seq):
            print(f"[AUTO] REACHED {label} seq={seq} | LiDAR={test.lidar:.2f}m")
            return

    raise FlightTestError(f"AUTO timeout before reaching {label}")


def send_body_velocity(test: DirectFlightTest, *, vx_mps: float, vy_mps: float, vz_mps: float = 0.0):
    """Velocity in BODY_OFFSET_NED: +X forward, +Y right, +Z down.

    type_mask 1479 = velocity XYZ + yaw-rate enabled, position/accel/yaw ignored.
    yaw_rate=0 keeps this test from commanding a yaw change.
    """
    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        1479,
        0.0,
        0.0,
        0.0,
        float(vx_mps),
        float(vy_mps),
        float(vz_mps),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def guided_lateral_cruise(
    test: DirectFlightTest,
    *,
    speed_mps: float,
    duration_s: float,
    floor_m: float,
    ceiling_m: float,
):
    test.set_mode("GUIDED")
    send_body_velocity(test, vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
    print(
        f"[GUIDED-CRUISE] BODY velocity test: RIGHT {speed_mps:.2f}m/s x {duration_s:.1f}s "
        f"-> LEFT {speed_mps:.2f}m/s x {duration_s:.1f}s"
    )

    phases = (("RIGHT", abs(float(speed_mps))), ("LEFT", -abs(float(speed_mps))))
    outside_since = None

    for phase_name, vy in phases:
        phase_end = time.monotonic() + float(duration_s)
        next_send = 0.0
        last_print = 0.0

        while time.monotonic() < phase_end:
            now = time.monotonic()
            if now >= next_send:
                send_body_velocity(test, vx_mps=0.0, vy_mps=vy, vz_mps=0.0)
                next_send = now + 0.10  # 10Hz refresh

            test.poll(0.05)
            check_flight_health(test)
            if test.mode != "GUIDED":
                raise FlightTestError(f"mode changed during GUIDED cruise: {test.mode}")

            outside_since = check_lidar_guard(
                test,
                floor_m=floor_m,
                ceiling_m=ceiling_m,
                outside_since=outside_since,
            )

            now = time.monotonic()
            if now - last_print >= 0.25:
                rel = None if test.global_pos is None else test.global_pos[2]
                remaining = max(0.0, phase_end - now)
                print(
                    f"[GUIDED:{phase_name}] vy={vy:+.2f}m/s LiDAR={test.lidar:.2f}m "
                    f"relAlt={rel if rel is not None else float('nan'):.2f}m remaining={remaining:.1f}s"
                )
                last_print = now

    send_body_velocity(test, vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
    print("[GUIDED-CRUISE] COMPLETE -> ZERO BODY VELOCITY")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description="Kingphoenix hybrid test: GUIDED takeoff -> AUTO WP1 -> GUIDED lateral -> AUTO WP2 -> LAND"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp1", type=int, default=1)
    ap.add_argument("--wp2", type=int, default=2)
    ap.add_argument("--alt", type=float, default=nominal_alt, help="LiDAR takeoff target AGL")
    ap.add_argument("--climb-rate", type=float, default=0.45, help="GUIDED LiDAR climb speed m/s")
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--cruise-speed", type=float, default=0.20, help="body-right/left velocity m/s")
    ap.add_argument("--cruise-duration", type=float, default=2.0, help="seconds per direction")
    ap.add_argument("--accept-radius", type=float, default=0.5)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--wp-timeout", type=float, default=60.0)
    ap.add_argument("--preload-only", action="store_true", help="upload/verify route then exit DISARMED")
    args = ap.parse_args()

    if args.wp1 < 1 or args.wp2 < 1 or args.wp1 == args.wp2:
        raise SystemExit("--wp1 and --wp2 must be different positive waypoint numbers")
    if not 0.50 <= float(args.alt) <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if not 0.10 <= float(args.climb_rate) <= 0.80:
        raise SystemExit("--climb-rate must be 0.10..0.80m/s")
    if not 0.05 <= float(args.cruise_speed) <= 0.50:
        raise SystemExit("--cruise-speed must be 0.05..0.50m/s for this test")
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
        print(f"[BLOCKED] WP1 -> WP2 only {route_distance:.2f}m; need >=0.50m for hybrid test")
        return 2

    print("============================================================")
    print(" KINGPHOENIX HYBRID CONTROL TEST")
    print("============================================================")
    print(
        f"[PLAN] GUIDED LiDAR TAKEOFF {args.alt:.2f}m @ {args.climb_rate:.2f}m/s "
        f"-> AUTO {wp1['label']} -> GUIDED RIGHT/LEFT -> AUTO {wp2['label']} -> LAND"
    )
    print("[PLAN] no deliberate takeoff hold; target reached -> immediate WP1 transition")
    print(
        f"[PLAN] GUIDED cruise BODY frame: RIGHT {args.cruise_speed:.2f}m/s x {args.cruise_duration:.1f}s, "
        f"LEFT {args.cruise_speed:.2f}m/s x {args.cruise_duration:.1f}s"
    )
    print("[PLAN] LiDAR is hard safety monitor during AUTO and GUIDED cruise")

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
                raise FlightTestError(f"{wp['label']} is {d:.1f}m away; field-test limit is 100m")
            print(
                f"[WP] {wp['label']} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={d:.2f}m relAlt={wp['alt_m']:.2f}m"
            )
        print(f"[PREARM] FCrelAlt={prearm_rel:.2f}m diagnostic only; not used for mission altitude")

        upload_two_wp_route(
            test,
            wp1,
            wp2,
            acceptance_radius_m=float(args.accept_radius),
        )
        verify_two_wp_route(test, wp1, wp2)
        print("[SAFE] HOME -> WP1 -> WP2 verified while DISARMED")

        if args.preload_only:
            print("[PRELOAD-ONLY] verified; vehicle remains DISARMED; exiting")
            return 0

        # Proven physical launch controller, now with no deliberate hold.
        lidar_authoritative_takeoff(
            test,
            float(args.alt),
            tolerance_m=float(args.tolerance),
            handoff_m=float(args.handoff),
            climb_rate_mps=float(args.climb_rate),
            kick_alt_m=float(args.kick_alt),
            hold_s=0.0,
            timeout_s=float(args.takeoff_timeout),
            ceiling_m=float(args.takeoff_ceiling),
        )

        if test.global_pos is not None:
            print(
                f"[POST-TAKEOFF] LiDAR={test.lidar:.2f}m FCrelAlt={test.global_pos[2]:.2f}m "
                "-> immediate AUTO WP1"
            )

        fly_auto_to_wp(
            test,
            seq=FC_WP1_SEQ,
            label=wp1["label"],
            floor_m=float(args.auto_floor),
            ceiling_m=float(args.auto_ceiling),
            timeout=float(args.wp_timeout),
        )

        # Intercept the mission at WP1 and exercise the exact GUIDED body-velocity
        # primitive that vision steering will later use.
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
        print("[RESULT] GUIDED TAKEOFF -> AUTO WP1 -> GUIDED R/L -> AUTO WP2 -> LAND PASS")
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
