#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json


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
            print(f"[MISSION] sent seq={seq} {item['label']} alt={alt_m:.2f}m")
        elif t == "MISSION_ACK":
            ack_type = int(getattr(msg, "type", -1))
            if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission rejected ACK={ack_type}")
            if len(sent) != count:
                raise FlightTestError(f"mission ACK before all items sent ({len(sent)}/{count})")
            print(f"[MISSION] {count} waypoint(s) uploaded and ACKed")
            return
    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


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

    ap = argparse.ArgumentParser(description="Kingphoenix DEV takeoff -> AUTO WP range -> LAND")
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=parse_wp_range, default=parse_wp_range("1"), help="1, 1-2, 2-5, etc")
    ap.add_argument("--alt", type=float, default=1.2, help="takeoff target only; AUTO WP altitude comes from DEV waypoint/default config")
    ap.add_argument("--margin", type=float, default=0.15, help="takeoff command compensation, default +15%%")
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--hold", type=float, default=0.50)
    ap.add_argument("--takeoff-timeout", type=float, default=12.0)
    ap.add_argument("--auto-timeout", type=float, default=60.0)
    ap.add_argument("--accept-radius", type=float, default=0.5)
    args = ap.parse_args()

    wp_numbers = list(args.wp)
    labels = [f"{args.strategy}/WP{n}" for n in wp_numbers]
    print("=================================================")
    print(" KINGPHOENIX DEV TAKEOFF -> AUTO -> LAND")
    print("=================================================")
    print(f"[PLAN] takeoff target={args.alt:.2f}m")
    print(f"[PLAN] nominal AUTO altitude={nominal_wp_alt:.2f}m")
    print(f"[PLAN] route={' -> '.join(labels)} -> LAND")

    saved = []
    try:
        for n in wp_numbers:
            saved.append(DirectFlightTest.validate_waypoint(args.strategy, n))
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        print("[SAFE] no MAVLink command sent")
        return 2

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        print(f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m GPSfix={test.gps_fix} sats={test.gps_sats}")

        mission_items = []
        for label, wp in zip(labels, saved):
            lat, lon, distance_m = test.local_waypoint_to_global(wp)
            if distance_m > 100.0:
                raise FlightTestError(f"{label} is {distance_m:.1f}m away; test limit is 100m")
            mission_items.append({"label": label, "lat": lat, "lon": lon, "alt_m": float(wp.get("alt_m", nominal_wp_alt))})
            print(f"[WP] {label} distance={distance_m:.2f}m alt={mission_items[-1]['alt_m']:.2f}m")

        if len(mission_items) == 1:
            _, _, d = test.local_waypoint_to_global(saved[0])
            if d < 0.30:
                raise FlightTestError(f"{labels[0]} only {d:.2f}m away; single-WP test blocked")

        upload_auto_waypoints(test, mission_items, acceptance_radius_m=args.accept_radius)
        test.takeoff_lidar_confirmed(args.alt, margin=args.margin, tolerance=args.tolerance, hold_s=args.hold, timeout=args.takeoff_timeout)
        print("[AUTO] LiDAR-confirmed takeoff complete; GUIDED -> AUTO")
        run_auto(test, labels, timeout=args.auto_timeout)
        test.land_and_wait()
        print("[RESULT] TAKEOFF -> AUTO -> LAND PASS")
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
