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

# Mission layout intentionally mirrors Mission Planner / ArduPilot:
#   seq 0 = reserved HOME
#   seq 1 = MAV_CMD_NAV_TAKEOFF
#   seq 2 = first route waypoint
#   seq 3 = second route waypoint, ...
FC_HOME_SEQ = 0
FC_TAKEOFF_SEQ = 1
FC_FIRST_WP_SEQ = 2


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


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(float(lat1))
    p2 = math.radians(float(lat2))
    dp = math.radians(float(lat2) - float(lat1))
    dl = math.radians(float(lon2) - float(lon1))
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


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
        raise FlightTestError(f"{strategy}/{key} has no lat/lon coordinate")

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
    if not 0.30 <= alt_m <= 5.0:
        raise FlightTestError(f"{strategy}/{key} altitude {alt_m:.2f}m outside test limits")

    return {"lat": lat, "lon": lon, "alt_m": alt_m}


def upload_mission(
    test: DirectFlightTest,
    items: list[dict],
    acceptance_radius_m: float = 0.5,
    timeout: float = 10.0,
):
    """Upload HOME + TAKEOFF + route with Mission-Planner-like semantics."""
    if not items:
        raise FlightTestError("mission is empty")
    if test.global_pos is None:
        raise FlightTestError("GLOBAL_POSITION_INT unavailable for HOME slot")

    home_lat = float(test.global_pos[0])
    home_lon = float(test.global_pos[1])

    test.clear_mission()
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = len(items) + 1  # reserved HOME + TAKEOFF + route

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
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                lat, lon, alt_m = home_lat, home_lon, 0.0
                p1 = p2 = p3 = p4 = 0.0
            else:
                item = items[seq - 1]
                label = str(item["label"])
                command = int(item["command"])
                lat = float(item.get("lat", 0.0))
                lon = float(item.get("lon", 0.0))
                alt_m = float(item["alt_m"])

                if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    p1 = p2 = p3 = p4 = 0.0
                else:
                    p1 = 0.0
                    p2 = float(acceptance_radius_m)
                    p3 = 0.0
                    p4 = float("nan")

            # Important: autocontinue=1.  The previous runner used 0, which did
            # not match a normal Mission Planner mission.
            args = [
                test.master.target_system,
                test.master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command,
                0,  # current; selected explicitly after upload
                1,  # autocontinue
                p1,
                p2,
                p3,
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
            req = "INT" if t == "MISSION_REQUEST_INT" else "LEGACY->INT"
            print(
                f"[MISSION] sent seq={seq} {label} cmd={command} "
                f"lat={lat:.7f} lon={lon:.7f} relAlt={alt_m:.2f}m "
                f"autocontinue=1 request={req}"
            )

        elif t == "MISSION_ACK":
            ack = int(getattr(msg, "type", -1))
            if ack != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise FlightTestError(f"mission rejected ACK={ack}")
            if len(sent) != count:
                raise FlightTestError(
                    f"mission ACK before all items sent ({len(sent)}/{count})"
                )
            print(
                f"[MISSION] HOME + TAKEOFF + {len(items)-1} waypoint(s) uploaded and ACKed"
            )
            return

    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


def verify_uploaded_mission(
    test: DirectFlightTest,
    items: list[dict],
    position_tol_m: float = 0.30,
    alt_tol_m: float = 0.05,
):
    print("[MISSION] readback verification while DISARMED ...")
    fc_items = request_fc_mission(test)
    expected_count = len(items) + 1
    if len(fc_items) != expected_count:
        raise FlightTestError(
            f"FC mission count mismatch: raw={len(fc_items)} expected={expected_count}"
        )

    takeoff = fc_items[FC_TAKEOFF_SEQ]
    wanted_takeoff = items[0]
    if int(takeoff["command"]) != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        raise FlightTestError(
            f"FC seq1 command={takeoff['command']}, expected NAV_TAKEOFF"
        )
    takeoff_alt_err = abs(float(wanted_takeoff["alt_m"]) - float(takeoff["alt_m"]))
    print(
        f"[MISSION] verify TAKEOFF seq1 relAlt={takeoff['alt_m']:.2f}m "
        f"alt_err={takeoff_alt_err:.3f}m"
    )
    if takeoff_alt_err > float(alt_tol_m):
        raise FlightTestError("TAKEOFF readback altitude mismatch")

    for route_index, wanted in enumerate(items[1:]):
        fc_seq = FC_FIRST_WP_SEQ + route_index
        actual = fc_items[fc_seq]
        if int(actual["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            raise FlightTestError(f"FC seq={fc_seq} is not NAV_WAYPOINT")

        pos_err = distance_m(
            wanted["lat"], wanted["lon"], actual["lat"], actual["lon"]
        )
        alt_err = abs(float(wanted["alt_m"]) - float(actual["alt_m"]))
        print(
            f"[MISSION] verify {wanted['label']} -> FC seq={fc_seq} "
            f"pos_err={pos_err:.3f}m relAlt={actual['alt_m']:.2f}m "
            f"alt_err={alt_err:.3f}m"
        )
        if pos_err > float(position_tol_m) or alt_err > float(alt_tol_m):
            raise FlightTestError(
                f"mission readback mismatch {wanted['label']}: "
                f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
            )

    print("[MISSION] PRELOAD VERIFIED - Mission Planner style layout stored")


def select_takeoff_and_start_auto(test: DirectFlightTest):
    """Arm outside AUTO, select TAKEOFF seq1, then enter AUTO."""
    test.set_mode("GUIDED")
    test.arm()

    test.master.mav.mission_set_current_send(
        test.master.target_system,
        test.master.target_component,
        FC_TAKEOFF_SEQ,
    )
    time.sleep(0.20)
    print("[MISSION] current set to seq1 TAKEOFF")

    test.set_mode("AUTO")
    print("[AUTO] native mission start at TAKEOFF seq1; ArduPilot owns TAKEOFF -> WP transition")


def run_native_auto(
    test: DirectFlightTest,
    labels: list[str],
    *,
    desired_agl_m: float,
    takeoff_ceiling_m: float,
    floor_m: float,
    ceiling_m: float,
    takeoff_timeout: float,
    auto_timeout: float,
):
    final_fc_seq = FC_FIRST_WP_SEQ + len(labels) - 1
    reached = set()
    lidar_loss_since = None
    high_since = None
    outside_since = None
    wp_phase = False
    takeoff_deadline = time.monotonic() + float(takeoff_timeout)
    mission_deadline = time.monotonic() + float(auto_timeout)

    while time.monotonic() < mission_deadline:
        test.poll(0.10)
        now = time.monotonic()

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during AUTO")
        if test.gps_fix is not None and test.gps_fix < 3:
            raise FlightTestError(f"GPS fix degraded during AUTO (fix={test.gps_fix})")

        if test.mission_current is not None and int(test.mission_current) >= FC_FIRST_WP_SEQ:
            if not wp_phase:
                wp_phase = True
                print(
                    f"[AUTO] TAKEOFF completed natively; now FC seq={test.mission_current} "
                    f"| LiDAR={test.lidar if test.lidar is not None else float('nan'):.2f}m"
                )

        if not wp_phase and now > takeoff_deadline:
            raise FlightTestError("native AUTO TAKEOFF timeout before WP phase")

        if test.lidar_ok():
            lidar_loss_since = None
            agl = float(test.lidar)

            if not wp_phase:
                # During climb, only protect against excessive height.  A floor
                # guard would of course trip while the aircraft is taking off.
                if agl > float(takeoff_ceiling_m):
                    high_since = high_since or now
                    if now - high_since > 0.35:
                        raise FlightTestError(
                            f"native TAKEOFF exceeded LiDAR ceiling: {agl:.2f}m > "
                            f"{takeoff_ceiling_m:.2f}m"
                        )
                else:
                    high_since = None
            else:
                if agl < float(floor_m) or agl > float(ceiling_m):
                    outside_since = outside_since or now
                    if now - outside_since > 0.35:
                        raise FlightTestError(
                            f"AUTO LiDAR AGL out of guard: {agl:.2f}m "
                            f"desired={desired_agl_m:.2f}m "
                            f"guard={floor_m:.2f}..{ceiling_m:.2f}m"
                        )
                else:
                    outside_since = None
        else:
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR stale >0.60s during AUTO")

        if test.reached_seq is not None:
            fc_seq = int(test.reached_seq)
            if fc_seq not in reached:
                reached.add(fc_seq)
                if fc_seq == FC_TAKEOFF_SEQ:
                    print(f"[AUTO] reached TAKEOFF seq1 | LiDAR={test.lidar:.2f}m")
                elif fc_seq >= FC_FIRST_WP_SEQ:
                    route_index = fc_seq - FC_FIRST_WP_SEQ
                    if 0 <= route_index < len(labels):
                        print(
                            f"[AUTO] reached {labels[route_index]} "
                            f"(FC seq={fc_seq}, LiDAR={test.lidar:.2f}m)"
                        )

            if fc_seq == final_fc_seq:
                print(f"[AUTO] final waypoint reached: {labels[-1]}")
                return

    raise FlightTestError("AUTO mission timeout")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description="Kingphoenix Mission-Planner-style native AUTO test"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=parse_wp_range, default=parse_wp_range("1"))
    ap.add_argument(
        "--alt",
        type=float,
        default=nominal_alt,
        help="native AUTO TAKEOFF relative altitude; default 1.2m",
    )
    ap.add_argument("--accept-radius", type=float, default=0.5)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.70)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--auto-timeout", type=float, default=60.0)

    # Compatibility with older field commands.  These no longer control this
    # Mission-Planner-style native AUTO path.
    ap.add_argument("--bridge-tol", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--tolerance", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--handoff", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--climb-rate", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--kick-alt", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--hold", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--ceiling", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--margin", type=float, default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if not 0.30 <= float(args.alt) <= 5.0:
        raise SystemExit("--alt must be 0.30..5.00m")
    if args.auto_floor >= args.auto_ceiling:
        raise SystemExit("--auto-floor must be below --auto-ceiling")
    if args.takeoff_ceiling <= args.alt:
        raise SystemExit("--takeoff-ceiling must be above --alt")

    wp_numbers = list(args.wp)
    labels = [f"{args.strategy}/WP{n}" for n in wp_numbers]

    print("============================================================")
    print(" KINGPHOENIX MISSION-PLANNER-STYLE NATIVE AUTO")
    print("============================================================")
    print(f"[PLAN] route=TAKEOFF({args.alt:.2f}m REL) -> {' -> '.join(labels)} -> LAND")
    print("[PLAN] preload while DISARMED: seq0 HOME, seq1 TAKEOFF, seq2 WP1 ...")
    print("[PLAN] AUTO starts at seq1 TAKEOFF; TAKEOFF is NOT skipped")
    print("[PLAN] waypoint altitude is written directly as JSON relative altitude")
    print("[PLAN] LiDAR is safety monitor only, not the vertical controller")

    try:
        saved = [load_global_waypoint(args.strategy, n) for n in wp_numbers]
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=True)
        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")

        cur_lat, cur_lon, _ = test.global_pos
        for label, wp in zip(labels, saved):
            d = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
            if d > 100.0:
                raise FlightTestError(f"{label} is {d:.1f}m away; test limit is 100m")
            print(
                f"[WP] {label} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={d:.2f}m relativeAlt={wp['alt_m']:.2f}m"
            )

        if len(saved) == 1:
            d = distance_m(cur_lat, cur_lon, saved[0]["lat"], saved[0]["lon"])
            if d < 0.30:
                raise FlightTestError(f"{labels[0]} only {d:.2f}m away; test blocked")

        mission_items = [
            {
                "label": "TAKEOFF",
                "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                "lat": 0.0,
                "lon": 0.0,
                "alt_m": float(args.alt),
            }
        ]
        for label, wp in zip(labels, saved):
            mission_items.append(
                {
                    "label": label,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "lat": float(wp["lat"]),
                    "lon": float(wp["lon"]),
                    "alt_m": float(wp["alt_m"]),
                }
            )

        upload_mission(
            test,
            mission_items,
            acceptance_radius_m=args.accept_radius,
        )
        verify_uploaded_mission(test, mission_items)
        print("[SAFE] Mission verified while DISARMED")

        select_takeoff_and_start_auto(test)
        run_native_auto(
            test,
            labels,
            desired_agl_m=float(args.alt),
            takeoff_ceiling_m=float(args.takeoff_ceiling),
            floor_m=float(args.auto_floor),
            ceiling_m=float(args.auto_ceiling),
            takeoff_timeout=float(args.takeoff_timeout),
            auto_timeout=float(args.auto_timeout),
        )

        test.land_and_wait()
        print("[RESULT] NATIVE AUTO TAKEOFF -> WP -> LAND PASS")
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
