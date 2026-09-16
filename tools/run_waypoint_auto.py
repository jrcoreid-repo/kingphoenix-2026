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


# ArduPilot exposes HOME as virtual FC mission seq=0 when reading a mission
# back. Real uploaded mission items therefore appear as FC seq=1..N.
FC_HOME_SEQ_OFFSET = 1


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
        agl_m = float(item.get("alt_m", nominal_alt))
    except Exception as exc:
        raise FlightTestError(f"{strategy}/{key} contains non-numeric data: {exc}")

    if not all(math.isfinite(v) for v in (lat, lon, agl_m)):
        raise FlightTestError(f"{strategy}/{key} contains invalid numeric data")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise FlightTestError(f"{strategy}/{key} lat/lon out of range")
    if lat == 0.0 and lon == 0.0:
        raise FlightTestError(f"{strategy}/{key} lat/lon is 0,0")

    return {"lat": lat, "lon": lon, "agl_m": agl_m}


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(float(lat1))
    p2 = math.radians(float(lat2))
    dp = math.radians(float(lat2) - float(lat1))
    dl = math.radians(float(lon2) - float(lon1))
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def upload_auto_waypoints(
    test: DirectFlightTest,
    items: list[dict],
    acceptance_radius_m=0.5,
    timeout=10.0,
):
    """Upload mission using MISSION_ITEM_INT only."""
    if not items:
        raise FlightTestError("AUTO mission is empty")

    test.clear_mission()
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = len(items)
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

            item = items[seq]
            lat = float(item["lat"])
            lon = float(item["lon"])
            alt_m = float(item["alt_m"])

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

            sent.add(seq)
            request_label = "INT" if t == "MISSION_REQUEST_INT" else "LEGACY->INT"
            print(
                f"[MISSION] sent INT seq={seq} {item['label']} "
                f"lat={lat:.7f} lon={lon:.7f} FCrelAlt={alt_m:.2f}m "
                f"request={request_label}"
            )

        elif t == "MISSION_ACK":
            ack_type = int(getattr(msg, "type", -1))
            if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission rejected ACK={ack_type}")
            if len(sent) != count:
                raise FlightTestError(
                    f"mission ACK before all items sent ({len(sent)}/{count})"
                )
            print(
                f"[MISSION] {count} waypoint(s) uploaded as MISSION_ITEM_INT and ACKed"
            )
            return

    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


def verify_uploaded_mission(
    test: DirectFlightTest,
    items: list[dict],
    position_tol_m=0.30,
    alt_tol_m=0.05,
):
    print("[MISSION] downloading FC mission for readback verification ...")
    fc_items = request_fc_mission(test)

    expected_fc_count = len(items) + FC_HOME_SEQ_OFFSET
    if len(fc_items) != expected_fc_count:
        raise FlightTestError(
            f"FC mission readback count mismatch: raw FC={len(fc_items)} "
            f"expected={expected_fc_count} (HOME + {len(items)} real item(s))"
        )

    print(
        f"[MISSION] FC seq=0 is HOME; verifying {len(items)} real waypoint(s) "
        f"at seq=1..{len(items)}"
    )

    for index, expected in enumerate(items):
        fc_seq = index + FC_HOME_SEQ_OFFSET
        actual = fc_items[fc_seq]

        if int(actual["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            raise FlightTestError(
                f"FC mission seq={fc_seq} command={actual['command']} is not NAV_WAYPOINT"
            )

        pos_err = distance_m(
            expected["lat"], expected["lon"], actual["lat"], actual["lon"]
        )
        alt_err = abs(float(expected["alt_m"]) - float(actual["alt_m"]))
        print(
            f"[MISSION] verify FC seq={fc_seq} {expected['label']} "
            f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
        )
        if pos_err > float(position_tol_m) or alt_err > float(alt_tol_m):
            raise FlightTestError(
                f"FC mission readback mismatch at seq={fc_seq}: "
                f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
            )

    print("[MISSION] READBACK VERIFIED - FC real mission items match requested route")


def run_auto(
    test: DirectFlightTest,
    labels: list[str],
    *,
    desired_agl_m: float,
    floor_m: float,
    ceiling_m: float,
    timeout=60.0,
):
    test.reached_seq = None
    first_fc_seq = FC_HOME_SEQ_OFFSET
    final_fc_seq = len(labels) - 1 + FC_HOME_SEQ_OFFSET

    try:
        test.master.mav.mission_set_current_send(
            test.master.target_system,
            test.master.target_component,
            first_fc_seq,
        )
    except Exception as exc:
        raise FlightTestError(
            f"cannot select first real mission item seq={first_fc_seq}: {exc}"
        )

    time.sleep(0.15)
    test.set_mode("AUTO")
    print(
        f"[AUTO] {' -> '.join(labels)} | FC seq={first_fc_seq}..{final_fc_seq} "
        f"| LiDAR guard={floor_m:.2f}..{ceiling_m:.2f}m"
    )

    reached = set()
    lidar_loss_since = None
    outside_since = None
    deadline = time.monotonic() + float(timeout)

    while time.monotonic() < deadline:
        test.poll(0.10)
        now = time.monotonic()

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during AUTO")
        if test.gps_fix is not None and test.gps_fix < 3:
            raise FlightTestError(
                f"GPS fix degraded during AUTO (fix={test.gps_fix})"
            )

        if test.lidar_ok():
            lidar_loss_since = None
            agl = float(test.lidar)
            if agl < float(floor_m) or agl > float(ceiling_m):
                outside_since = outside_since or now
                if now - outside_since > 0.35:
                    raise FlightTestError(
                        f"AUTO LiDAR AGL out of guard band: {agl:.2f}m "
                        f"(desired={desired_agl_m:.2f}m, guard={floor_m:.2f}..{ceiling_m:.2f}m)"
                    )
            else:
                outside_since = None
        else:
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR stale >0.60s during low-altitude AUTO")

        if test.reached_seq is not None:
            fc_seq = int(test.reached_seq)
            route_index = fc_seq - FC_HOME_SEQ_OFFSET
            if 0 <= route_index < len(labels) and fc_seq not in reached:
                reached.add(fc_seq)
                print(f"[AUTO] reached FC seq={fc_seq} {labels[route_index]}")
            if fc_seq == final_fc_seq:
                print(
                    f"[AUTO] final waypoint reached: FC seq={fc_seq} {labels[-1]}"
                )
                return

    raise FlightTestError("AUTO mission timeout")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_agl = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description="Kingphoenix DEV LiDAR takeoff -> altitude-bridged AUTO WP -> LAND"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument(
        "--wp",
        type=parse_wp_range,
        default=parse_wp_range("1"),
        help="1, 1-2, 2-5, etc",
    )
    ap.add_argument("--alt", type=float, default=1.2, help="raw LiDAR AGL target")
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--climb-rate", type=float, default=0.30)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--ceiling", type=float, default=1.40, help="takeoff hard ceiling")
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--auto-timeout", type=float, default=60.0)
    ap.add_argument("--accept-radius", type=float, default=0.5)
    ap.add_argument("--margin", type=float, default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.auto_floor >= args.alt:
        raise SystemExit("--auto-floor must be below --alt")
    if args.auto_ceiling <= args.alt:
        raise SystemExit("--auto-ceiling must be above --alt")

    wp_numbers = list(args.wp)
    labels = [f"{args.strategy}/WP{n}" for n in wp_numbers]

    print("=================================================")
    print(" KINGPHOENIX DEV LIDAR TAKEOFF -> AUTO -> LAND")
    print("=================================================")
    print(f"[PLAN] physical AGL target={args.alt:.2f}m from raw LiDAR")
    print(f"[PLAN] JSON nominal AGL={nominal_agl:.2f}m")
    print(f"[PLAN] route={' -> '.join(labels)} -> LAND")
    print("[PLAN] AUTO altitude is bridged from FC relative_alt after LiDAR hover")
    print("[PLAN] mission upload happens AFTER takeoff; MISSION_ITEM_INT only")
    print("[PLAN] FC seq0=HOME; real mission begins at seq1")
    if args.margin is not None:
        print("[NOTE] --margin ignored; takeoff completion is raw LiDAR-authoritative")

    saved = []
    try:
        for n in wp_numbers:
            saved.append(load_global_waypoint(args.strategy, n))
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        print(
            f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m "
            f"GPSfix={test.gps_fix} sats={test.gps_sats}"
        )

        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")
        cur_lat, cur_lon, _ = test.global_pos

        for label, wp in zip(labels, saved):
            horizontal_m = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
            if horizontal_m > 100.0:
                raise FlightTestError(
                    f"{label} is {horizontal_m:.1f}m away; test limit is 100m"
                )
            print(
                f"[WP] {label} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={horizontal_m:.2f}m desiredAGL={wp['agl_m']:.2f}m"
            )

        if len(saved) == 1:
            d = distance_m(cur_lat, cur_lon, saved[0]["lat"], saved[0]["lon"])
            if d < 0.30:
                raise FlightTestError(
                    f"{labels[0]} only {d:.2f}m away; single-WP test blocked"
                )

        # Phase 1: validated LiDAR-authoritative vertical launch.
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

        # Phase 2: bridge the FC's current altitude estimate into AUTO mission
        # altitude so GUIDED->AUTO cannot command a large vertical step.
        bridge_deadline = time.monotonic() + 1.0
        while time.monotonic() < bridge_deadline:
            test.poll(0.10)

        if not test.lidar_ok():
            raise FlightTestError("LiDAR invalid before AUTO altitude bridge")
        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable after takeoff")

        fc_rel_alt = float(test.global_pos[2])
        if not math.isfinite(fc_rel_alt) or fc_rel_alt < -2.0 or fc_rel_alt > 10.0:
            raise FlightTestError(
                f"FC relative_alt unsafe after LiDAR hover: {fc_rel_alt:.2f}m; AUTO blocked"
            )

        print(
            f"[ALT-BRIDGE] LiDAR AGL={test.lidar:.2f}m -> "
            f"FC relative_alt={fc_rel_alt:.2f}m"
        )
        print(
            "[ALT-BRIDGE] All AUTO WPs will use current FC relative_alt, "
            "preventing vertical target step at GUIDED->AUTO"
        )

        mission_items = []
        for label, wp in zip(labels, saved):
            mission_items.append(
                {
                    "label": label,
                    "lat": float(wp["lat"]),
                    "lon": float(wp["lon"]),
                    "alt_m": fc_rel_alt,
                }
            )

        upload_auto_waypoints(
            test,
            mission_items,
            acceptance_radius_m=args.accept_radius,
        )
        verify_uploaded_mission(test, mission_items)

        if not test.lidar_ok():
            raise FlightTestError("LiDAR invalid immediately before AUTO")
        if not (args.auto_floor <= float(test.lidar) <= args.auto_ceiling):
            raise FlightTestError(
                f"LiDAR={test.lidar:.2f}m outside AUTO guard before mode switch"
            )

        print("[AUTO] Mission verified; switching GUIDED -> AUTO")
        run_auto(
            test,
            labels,
            desired_agl_m=args.alt,
            floor_m=args.auto_floor,
            ceiling_m=args.auto_ceiling,
            timeout=args.auto_timeout,
        )
        test.land_and_wait()
        print("[RESULT] LIDAR TAKEOFF -> ALT-BRIDGED AUTO -> LAND PASS")
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
