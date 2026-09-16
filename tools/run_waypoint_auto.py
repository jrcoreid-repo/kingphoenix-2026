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


def parse_wp_range(value: str) -> list[int]:
    raw = str(value).strip()
    try:
        if "-" not in raw:
            n = int(raw)
            if n < 1:
                raise ValueError
            return [n]
        a, b = raw.split("-", 1)
        start, end = int(a), int(b)
        if start < 1 or end < start:
            raise ValueError
        return list(range(start, end + 1))
    except Exception:
        raise argparse.ArgumentTypeError("--wp must be like 1, 1-2, or 2-5")


def load_global_waypoint(strategy: str, wp: int) -> dict:
    data = load_json("waypoints.json")
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))
    strategy = str(strategy).upper()
    key = f"WP{int(wp)}"

    if strategy not in data or key not in data[strategy]:
        raise FlightTestError(f"unknown waypoint {strategy}/{key}")

    item = data[strategy][key]
    lat, lon = item.get("lat"), item.get("lon")
    if lat is None or lon is None:
        raise FlightTestError(
            f"{strategy}/{key} has no lat/lon coordinate; capture it from main.py first"
        )

    try:
        lat = float(lat)
        lon = float(lon)
        alt_m = float(item.get("alt_m", nominal_alt))
    except Exception as exc:
        raise FlightTestError(f"{strategy}/{key} contains non-numeric data: {exc}")

    if not all(math.isfinite(v) for v in (lat, lon, alt_m)):
        raise FlightTestError(f"{strategy}/{key} contains invalid numeric data")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise FlightTestError(f"{strategy}/{key} lat/lon out of range")
    if lat == 0.0 and lon == 0.0:
        raise FlightTestError(f"{strategy}/{key} lat/lon is 0,0")

    return {"lat": lat, "lon": lon, "alt_m": alt_m}


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(float(lat1))
    p2 = math.radians(float(lat2))
    dp = math.radians(float(lat2) - float(lat1))
    dl = math.radians(float(lon2) - float(lon1))
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def upload_auto_waypoints(test: DirectFlightTest, items: list[dict], acceptance_radius_m=0.5, timeout=10.0):
    if not items:
        raise FlightTestError("AUTO mission is empty")
    test.clear_mission()
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = len(items)
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
            if seq < 0 or seq >= count:
                raise FlightTestError(f"FC requested invalid mission seq={seq}")
            item = items[seq]
            lat, lon, alt_m = float(item["lat"]), float(item["lon"]), float(item["alt_m"])
            if t == "MISSION_REQUEST_INT":
                args = [
                    test.master.target_system, test.master.target_component, seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1 if seq == 0 else 0,
                    0.0, float(acceptance_radius_m), 0.0, float("nan"),
                    int(round(lat * 1e7)), int(round(lon * 1e7)), alt_m,
                ]
                try:
                    test.master.mav.mission_item_int_send(*args, mtype)
                except TypeError:
                    test.master.mav.mission_item_int_send(*args)
            else:
                args = [
                    test.master.target_system, test.master.target_component, seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1 if seq == 0 else 0,
                    0.0, float(acceptance_radius_m), 0.0, float("nan"),
                    lat, lon, alt_m,
                ]
                try:
                    test.master.mav.mission_item_send(*args, mtype)
                except TypeError:
                    test.master.mav.mission_item_send(*args)
            sent.add(seq)
            print(f"[MISSION] sent seq={seq} {item['label']} lat={lat:.7f} lon={lon:.7f} alt={alt_m:.2f}m")
        elif t == "MISSION_ACK":
            ack_type = int(getattr(msg, "type", -1))
            if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission rejected ACK={ack_type}")
            if len(sent) != count:
                raise FlightTestError(f"mission ACK before all items sent ({len(sent)}/{count})")
            print(f"[MISSION] {count} waypoint(s) uploaded and ACKed")
            return
    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


def verify_uploaded_mission(test: DirectFlightTest, items: list[dict], position_tol_m=0.30, alt_tol_m=0.05):
    print("[MISSION] downloading FC mission for readback verification ...")
    fc_items = request_fc_mission(test)
    if len(fc_items) != len(items):
        raise FlightTestError(
            f"FC mission readback count mismatch: FC={len(fc_items)} expected={len(items)}"
        )

    for seq, (expected, actual) in enumerate(zip(items, fc_items)):
        if int(actual["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            raise FlightTestError(
                f"FC mission seq={seq} command={actual['command']} is not NAV_WAYPOINT"
            )
        pos_err = distance_m(
            expected["lat"], expected["lon"], actual["lat"], actual["lon"]
        )
        alt_err = abs(float(expected["alt_m"]) - float(actual["alt_m"]))
        print(
            f"[MISSION] verify seq={seq} {expected['label']} "
            f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
        )
        if pos_err > float(position_tol_m) or alt_err > float(alt_tol_m):
            raise FlightTestError(
                f"FC mission readback mismatch at seq={seq}: "
                f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
            )

    print("[MISSION] READBACK VERIFIED - FC mission matches requested JSON route")


def run_auto(test: DirectFlightTest, labels: list[str], timeout=60.0):
    test.reached_seq = None
    try:
        test.master.mav.mission_set_current_send(test.master.target_system, test.master.target_component, 0)
    except Exception as exc:
        raise FlightTestError(f"cannot select mission item 0: {exc}")
    time.sleep(0.15)
    test.set_mode("AUTO")
    print(f"[AUTO] {' -> '.join(labels)}")

    final_seq = len(labels) - 1
    reached = set()
    lidar_loss_since = None
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        test.poll(0.15)
        now = time.monotonic()
        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during AUTO")
        if test.gps_fix is not None and test.gps_fix < 3:
            raise FlightTestError(f"GPS fix degraded during AUTO (fix={test.gps_fix})")
        if test.lidar_ok():
            lidar_loss_since = None
        else:
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 1.0:
                raise FlightTestError("LiDAR stale >1.0s during low-altitude AUTO")
        if test.reached_seq is not None:
            seq = int(test.reached_seq)
            if 0 <= seq < len(labels) and seq not in reached:
                reached.add(seq)
                print(f"[AUTO] reached {labels[seq]}")
            if seq == final_seq:
                print(f"[AUTO] final waypoint reached: {labels[final_seq]}")
                return
    raise FlightTestError("AUTO mission timeout")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_wp_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(description="Kingphoenix DEV LiDAR takeoff -> AUTO WP range -> LAND")
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=parse_wp_range, default=parse_wp_range("1"), help="1, 1-2, 2-5, etc")
    ap.add_argument("--alt", type=float, default=1.2, help="raw LiDAR takeoff AGL target; AUTO WP altitude comes from waypoint config")
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--handoff", type=float, default=0.25, help="LiDAR AGL for NAV_TAKEOFF -> velocity handoff")
    ap.add_argument("--climb-rate", type=float, default=0.30, help="LiDAR-authoritative Guided climb rate after handoff")
    ap.add_argument("--kick-alt", type=float, default=3.0, help="temporary NAV_TAKEOFF launch target only")
    ap.add_argument("--hold", type=float, default=2.0, help="zero-velocity hold before AUTO handoff")
    ap.add_argument("--ceiling", type=float, default=1.40, help="hard raw-LiDAR takeoff ceiling")
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--auto-timeout", type=float, default=60.0)
    ap.add_argument("--accept-radius", type=float, default=0.5)
    # Retained only so older field commands with '--margin 0' do not fail.
    # It has no effect in the LiDAR-authoritative takeoff path.
    ap.add_argument("--margin", type=float, default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()

    wp_numbers = list(args.wp)
    labels = [f"{args.strategy}/WP{n}" for n in wp_numbers]
    print("=================================================")
    print(" KINGPHOENIX DEV LIDAR TAKEOFF -> AUTO -> LAND")
    print("=================================================")
    print(f"[PLAN] raw-LiDAR takeoff target={args.alt:.2f}m")
    print(f"[PLAN] takeoff handoff={args.handoff:.2f}m climb={args.climb_rate:.2f}m/s")
    print(f"[PLAN] nominal AUTO altitude={nominal_wp_alt:.2f}m")
    print(f"[PLAN] route={' -> '.join(labels)} -> LAND")
    print("[PLAN] waypoint source=GLOBAL lat/lon from config/waypoints.json")
    print("[PLAN] safety=upload -> FC readback verify -> LiDAR takeoff -> AUTO")
    if args.margin is not None:
        print("[NOTE] --margin is ignored; takeoff completion is raw LiDAR-authoritative")

    saved = []
    try:
        for n in wp_numbers:
            saved.append(load_global_waypoint(args.strategy, n))
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        print("[SAFE] no MAVLink command sent")
        return 2

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        print(f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m GPSfix={test.gps_fix} sats={test.gps_sats}")

        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")
        cur_lat, cur_lon, _ = test.global_pos

        mission_items = []
        for label, wp in zip(labels, saved):
            horizontal_m = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
            if horizontal_m > 100.0:
                raise FlightTestError(f"{label} is {horizontal_m:.1f}m away; test limit is 100m")
            mission_items.append({
                "label": label,
                "lat": float(wp["lat"]),
                "lon": float(wp["lon"]),
                "alt_m": float(wp.get("alt_m", nominal_wp_alt)),
            })
            print(
                f"[WP] {label} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={horizontal_m:.2f}m alt={mission_items[-1]['alt_m']:.2f}m"
            )

        if len(mission_items) == 1:
            d = distance_m(cur_lat, cur_lon, saved[0]["lat"], saved[0]["lon"])
            if d < 0.30:
                raise FlightTestError(f"{labels[0]} only {d:.2f}m away; single-WP test blocked")

        upload_auto_waypoints(test, mission_items, acceptance_radius_m=args.accept_radius)
        verify_uploaded_mission(test, mission_items)

        # This is the exact takeoff controller validated in the dedicated
        # LiDAR field test: NAV_TAKEOFF only gets airborne, raw downward LiDAR
        # hands off to Guided velocity and is the sole takeoff completion source.
        lidar_authoritative_takeoff(
            test,
            args.alt,
            tolerance_m=args.tolerance,
            handoff_m=args.handoff,
            climb_rate_mps=args.climb_rate,
            kick_alt_m=args.kick_alt,
            hold_s=args.hold,
            timeout_s=args.takeoff_timeout,
            ceiling_m=args.ceiling,
        )
        print("[AUTO] LiDAR-authoritative takeoff complete; GUIDED -> AUTO")

        run_auto(test, labels, timeout=args.auto_timeout)
        test.land_and_wait()
        print("[RESULT] LIDAR TAKEOFF -> AUTO -> LAND PASS")
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
