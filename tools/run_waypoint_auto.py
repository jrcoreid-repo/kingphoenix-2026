#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


AUTO_WP_ALT_M = 1.20


def parse_wp_range(value: str) -> list[int]:
    """Accept `1`, `1-2`, `2-5`, etc. and return an inclusive ascending list."""
    raw = str(value).strip()
    try:
        if "-" not in raw:
            n = int(raw)
            if n < 1:
                raise ValueError
            return [n]

        parts = raw.split("-")
        if len(parts) != 2:
            raise ValueError
        start = int(parts[0].strip())
        end = int(parts[1].strip())
        if start < 1 or end < 1 or end < start:
            raise ValueError
        return list(range(start, end + 1))
    except ValueError:
        raise argparse.ArgumentTypeError(
            "--wp must be a waypoint number or ascending range, e.g. 1, 1-2, 2-5"
        )


def upload_auto_waypoints(test: DirectFlightTest, mission_items: list[dict],
                          acceptance_radius_m: float = 0.5, timeout: float = 10.0):
    """Upload an ordered list of AUTO waypoints and require MISSION_ACK."""
    if not mission_items:
        raise FlightTestError("AUTO mission contains no waypoint")

    test.clear_mission()
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = len(mission_items)

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

        msg_type = msg.get_type()
        if msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            if seq < 0 or seq >= count:
                raise FlightTestError(f"FC requested invalid mission seq={seq}")

            item = mission_items[seq]
            lat = float(item["lat"])
            lon = float(item["lon"])
            alt_m = float(item["alt_m"])

            if msg_type == "MISSION_REQUEST_INT":
                args = [
                    test.master.target_system,
                    test.master.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,
                    1 if seq == 0 else 0,
                    0.0,
                    float(acceptance_radius_m),
                    0.0,
                    float("nan"),
                    int(round(lat * 1e7)),
                    int(round(lon * 1e7)),
                    alt_m,
                ]
                try:
                    test.master.mav.mission_item_int_send(*args, mtype)
                except TypeError:
                    test.master.mav.mission_item_int_send(*args)
            else:
                args = [
                    test.master.target_system,
                    test.master.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,
                    1 if seq == 0 else 0,
                    0.0,
                    float(acceptance_radius_m),
                    0.0,
                    float("nan"),
                    lat,
                    lon,
                    alt_m,
                ]
                try:
                    test.master.mav.mission_item_send(*args, mtype)
                except TypeError:
                    test.master.mav.mission_item_send(*args)

            sent.add(seq)
            print(
                f"[MISSION] sent seq={seq} {item['label']} "
                f"lat={lat:.7f} lon={lon:.7f} alt={alt_m:.2f}m"
            )
            continue

        if msg_type == "MISSION_ACK":
            ack_type = int(getattr(msg, "type", -1))
            if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission upload rejected, ACK type={ack_type}")
            if len(sent) != count:
                raise FlightTestError(
                    f"mission ACK arrived before all items were sent ({len(sent)}/{count})"
                )
            print(f"[MISSION] {count} AUTO waypoint(s) uploaded and ACKed")
            return

    raise FlightTestError(f"mission upload timed out ({len(sent)}/{count} item sent)")


def start_auto_and_wait_sequence(test: DirectFlightTest, labels: list[str],
                                 timeout: float = 60.0,
                                 lidar_loss_abort_s: float = 1.0):
    """Run the uploaded mission in AUTO and wait for every waypoint in order."""
    if not labels:
        raise FlightTestError("no AUTO waypoint labels supplied")

    final_seq = len(labels) - 1
    reached = set()
    test.reached_seq = None

    try:
        test.master.mav.mission_set_current_send(
            test.master.target_system,
            test.master.target_component,
            0,
        )
    except Exception as e:
        raise FlightTestError(f"failed to select AUTO mission item 0: {e}")

    time.sleep(0.15)
    test.set_mode("AUTO")
    print(f"[AUTO] mission started: {' -> '.join(labels)}")

    deadline = time.monotonic() + float(timeout)
    lidar_loss_since = None
    last_announced_current = None

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
            if now - lidar_loss_since > float(lidar_loss_abort_s):
                raise FlightTestError("LiDAR invalid/stale during low-altitude AUTO")

        if test.mission_current is not None and test.mission_current != last_announced_current:
            seq = int(test.mission_current)
            if 0 <= seq < len(labels):
                print(f"[AUTO] current seq={seq} target={labels[seq]}")
            last_announced_current = seq

        if test.reached_seq is not None:
            seq = int(test.reached_seq)
            if seq not in reached and 0 <= seq < len(labels):
                reached.add(seq)
                print(f"[AUTO] reached seq={seq} {labels[seq]}")
            if seq == final_seq:
                print(f"[AUTO] final waypoint reached: {labels[final_seq]}")
                return

    missing = [labels[i] for i in range(len(labels)) if i not in reached]
    raise FlightTestError(
        f"AUTO mission timeout; unreached={','.join(missing) if missing else 'final confirmation missing'}"
    )


def land_and_wait(test: DirectFlightTest, timeout: float = 20.0):
    """Switch to LAND after successful AUTO completion and wait for disarm."""
    print("[LAND] final waypoint complete; switching AUTO -> LAND")
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

    raise FlightTestError("LAND timeout: vehicle still armed after 20 s")


def main():
    ap = argparse.ArgumentParser(
        description="Kingphoenix guarded takeoff + AUTO waypoint/range test"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument(
        "--wp",
        type=parse_wp_range,
        default=parse_wp_range("1"),
        help="waypoint or inclusive range, e.g. 1, 1-2, 2-5",
    )
    ap.add_argument(
        "--alt",
        type=float,
        default=1.0,
        help="takeoff target altitude only, in metres (AUTO WPs are fixed at 1.20 m)",
    )
    ap.add_argument("--margin", type=float, default=0.15,
                    help="takeoff command overshoot margin (default +15%%)")
    ap.add_argument("--tolerance", type=float, default=0.05,
                    help="LiDAR takeoff completion tolerance (m)")
    ap.add_argument("--hold", type=float, default=0.50,
                    help="LiDAR stable time before AUTO transition (s)")
    ap.add_argument("--takeoff-timeout", type=float, default=12.0)
    ap.add_argument("--auto-timeout", type=float, default=60.0,
                    help="timeout for the whole AUTO waypoint sequence")
    ap.add_argument("--accept-radius", type=float, default=0.5,
                    help="AUTO waypoint acceptance radius (m)")
    args = ap.parse_args()

    wp_numbers = list(args.wp)
    labels = [f"{args.strategy}/WP{n}" for n in wp_numbers]

    print("=================================================")
    print(" KINGPHOENIX TAKEOFF + AUTO WAYPOINT TEST")
    print("=================================================")
    print(f"[PLAN] takeoff target : {args.alt:.2f} m")
    print(f"[PLAN] AUTO altitude  : {AUTO_WP_ALT_M:.2f} m FIXED")
    print(f"[PLAN] route          : {' -> '.join(labels)} -> LAND")

    saved_waypoints = []
    try:
        for wp_num in wp_numbers:
            saved_waypoints.append(
                DirectFlightTest.validate_waypoint(args.strategy, wp_num)
            )
    except FlightTestError as e:
        print(f"[BLOCKED] {e}")
        print("[SAFE] No MAVLink command sent. Populate every requested waypoint first.")
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

        mission_items = []
        for wp_num, waypoint in zip(wp_numbers, saved_waypoints):
            lat, lon, horizontal_m = test.local_waypoint_to_global(waypoint)
            if horizontal_m > 100.0:
                raise FlightTestError(
                    f"{args.strategy}/WP{wp_num} is {horizontal_m:.1f} m from current position; "
                    "AUTO test runner is limited to <=100 m"
                )

            label = f"{args.strategy}/WP{wp_num}"
            print(
                f"[WAYPOINT] {label} local=({waypoint['x']:.2f},{waypoint['y']:.2f}) "
                f"distance_from_home={horizontal_m:.2f}m -> "
                f"lat={lat:.7f} lon={lon:.7f} alt={AUTO_WP_ALT_M:.2f}m"
            )
            mission_items.append(
                {
                    "label": label,
                    "lat": lat,
                    "lon": lon,
                    "alt_m": AUTO_WP_ALT_M,
                }
            )

        if len(mission_items) == 1:
            _, _, distance_m = test.local_waypoint_to_global(saved_waypoints[0])
            if distance_m < 0.30:
                raise FlightTestError(
                    f"{labels[0]} is only {distance_m:.2f} m from current position; "
                    "single-WP AUTO test blocked to avoid an ambiguous no-movement test"
                )

        upload_auto_waypoints(
            test,
            mission_items,
            acceptance_radius_m=args.accept_radius,
        )

        test.takeoff_lidar_confirmed(
            args.alt,
            margin=args.margin,
            tolerance=args.tolerance,
            hold_s=args.hold,
            timeout=args.takeoff_timeout,
        )

        print(
            f"[AUTO] takeoff confirmed by LiDAR; transitioning GUIDED -> AUTO "
            f"at fixed WP altitude {AUTO_WP_ALT_M:.2f}m"
        )
        start_auto_and_wait_sequence(
            test,
            labels,
            timeout=args.auto_timeout,
        )

        land_and_wait(test)
        print("[RESULT] TAKEOFF + AUTO WAYPOINT + LAND TEST PASS")
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