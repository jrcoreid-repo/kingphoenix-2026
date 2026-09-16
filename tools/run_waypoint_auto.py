#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json
from tools.check_waypoints import request_fc_mission
from tools.run_takeoff_lidar_test import lidar_authoritative_takeoff

# ArduPilot mission storage:
#   FC seq 0 = reserved/internal HOME
#   FC seq 1 = MAV_CMD_NAV_TAKEOFF marker
#   FC seq 2 = first real route waypoint (WP1 for --wp 1...)
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
    nominal_agl = float(defaults.get("nominal_waypoint_alt_m", 1.2))
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
        agl_m = float(item.get("alt_m", nominal_agl))
    except Exception as exc:
        raise FlightTestError(f"{strategy}/{key} contains non-numeric data: {exc}")
    if not all(math.isfinite(v) for v in (lat, lon, agl_m)):
        raise FlightTestError(f"{strategy}/{key} contains invalid numeric data")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise FlightTestError(f"{strategy}/{key} lat/lon out of range")
    if lat == 0.0 and lon == 0.0:
        raise FlightTestError(f"{strategy}/{key} lat/lon is 0,0")
    return {"lat": lat, "lon": lon, "agl_m": agl_m}


def capture_ground_baseline(test: DirectFlightTest, duration_s=1.5) -> tuple[float, float]:
    lidar_samples: list[float] = []
    rel_samples: list[float] = []
    deadline = time.monotonic() + float(duration_s)
    while time.monotonic() < deadline:
        test.poll(0.05)
        if test.lidar_ok() and test.global_pos is not None:
            lidar = float(test.lidar)
            rel_alt = float(test.global_pos[2])
            if math.isfinite(lidar) and math.isfinite(rel_alt):
                lidar_samples.append(lidar)
                rel_samples.append(rel_alt)
    if len(lidar_samples) < 8 or len(rel_samples) < 8:
        raise FlightTestError("insufficient stable ground baseline samples")
    ground_lidar = float(statistics.median(lidar_samples))
    ground_rel = float(statistics.median(rel_samples))
    if ground_lidar > 0.30:
        raise FlightTestError(
            f"ground baseline LiDAR={ground_lidar:.2f}m >0.30m; place drone on ground"
        )
    if not -10.0 <= ground_rel <= 10.0:
        raise FlightTestError(f"ground FC relative_alt unsafe: {ground_rel:.2f}m")
    return ground_lidar, ground_rel


def upload_mission(test: DirectFlightTest, items: list[dict], acceptance_radius_m=0.5, timeout=10.0):
    """Upload reserved HOME slot + TAKEOFF + route as MISSION_ITEM_INT.

    ArduPilot reserves mission index 0 for HOME. The upload count therefore
    includes that slot. A valid HOME placeholder is sent at upload seq 0;
    ArduPilot ignores its supplied contents and keeps AHRS HOME there. TAKEOFF
    is sent at seq 1 and WP1 starts at seq 2.
    """
    if not items:
        raise FlightTestError("mission is empty")
    if test.global_pos is None:
        raise FlightTestError("GLOBAL_POSITION_INT unavailable for HOME upload slot")

    home_lat = float(test.global_pos[0])
    home_lon = float(test.global_pos[1])

    test.clear_mission()
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = len(items) + 1  # seq0 HOME + TAKEOFF + route WPs
    try:
        test.master.mav.mission_count_send(
            test.master.target_system, test.master.target_component, count, mtype
        )
    except TypeError:
        test.master.mav.mission_count_send(
            test.master.target_system, test.master.target_component, count
        )

    sent = set()
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        msg = test.poll(0.4)
        if msg is None:
            continue
        t = msg.get_type()
        if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            upload_seq = int(msg.seq)
            if upload_seq < 0 or upload_seq >= count:
                raise FlightTestError(f"FC requested invalid upload seq={upload_seq}")

            if upload_seq == FC_HOME_SEQ:
                label = "HOME_SLOT"
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                lat = home_lat
                lon = home_lon
                alt_m = 0.0
                p1 = p2 = p3 = p4 = 0.0
            else:
                item = items[upload_seq - 1]
                label = str(item["label"])
                command = int(item["command"])
                alt_m = float(item["alt_m"])
                lat = float(item.get("lat", 0.0))
                lon = float(item.get("lon", 0.0))

                if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    # Structural TAKEOFF marker. Physical launch is performed
                    # separately by the LiDAR-authoritative GUIDED controller.
                    p1 = p2 = p3 = p4 = 0.0
                else:
                    p1 = 0.0
                    p2 = float(acceptance_radius_m)
                    p3 = 0.0
                    p4 = float("nan")

            args = [
                test.master.target_system,
                test.master.target_component,
                upload_seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command,
                0,
                0,
                p1, p2, p3, p4,
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                alt_m,
            ]
            try:
                test.master.mav.mission_item_int_send(*args, mtype)
            except TypeError:
                test.master.mav.mission_item_int_send(*args)

            sent.add(upload_seq)
            req = "INT" if t == "MISSION_REQUEST_INT" else "LEGACY->INT"
            print(
                f"[MISSION] sent uploadSeq={upload_seq} {label} "
                f"cmd={command} lat={lat:.7f} lon={lon:.7f} "
                f"alt={alt_m:.2f}m request={req}"
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
                f"[MISSION] HOME slot + TAKEOFF + {len(items)-1} waypoint(s) uploaded and ACKed"
            )
            return

    raise FlightTestError(f"mission upload timeout ({len(sent)}/{count})")


def verify_uploaded_mission(test: DirectFlightTest, items: list[dict], position_tol_m=0.30, alt_tol_m=0.05):
    print("[MISSION] readback verification while DISARMED ...")
    fc_items = request_fc_mission(test)
    expected_count = len(items) + 1  # HOME + TAKEOFF + route WPs
    if len(fc_items) != expected_count:
        raise FlightTestError(
            f"FC mission count mismatch: raw={len(fc_items)} expected={expected_count} "
            f"(HOME + TAKEOFF + {len(items)-1} WP)"
        )

    home = fc_items[FC_HOME_SEQ]
    takeoff = fc_items[FC_TAKEOFF_SEQ]
    if int(takeoff["command"]) != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        raise FlightTestError(
            f"FC seq={FC_TAKEOFF_SEQ} is command={takeoff['command']}, expected NAV_TAKEOFF"
        )
    takeoff_alt_err = abs(float(items[0]["alt_m"]) - float(takeoff["alt_m"]))
    print(
        f"[MISSION] verify HOME seq=0 lat={home['lat']:.7f} lon={home['lon']:.7f} "
        f"(internal only)"
    )
    print(
        f"[MISSION] verify TAKEOFF -> FC seq=1 alt_err={takeoff_alt_err:.3f}m"
    )
    if takeoff_alt_err > float(alt_tol_m):
        raise FlightTestError(
            f"TAKEOFF readback altitude mismatch: {takeoff_alt_err:.3f}m"
        )

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
            f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
        )
        if pos_err > float(position_tol_m) or alt_err > float(alt_tol_m):
            raise FlightTestError(
                f"mission readback mismatch {wanted['label']}: "
                f"pos_err={pos_err:.3f}m alt_err={alt_err:.3f}m"
            )

    print("[MISSION] PRELOAD VERIFIED - HOME -> TAKEOFF -> WP route stored before ARM")


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
    first_fc_seq = FC_FIRST_WP_SEQ
    final_fc_seq = FC_FIRST_WP_SEQ + len(labels) - 1

    # LiDAR already completed takeoff in GUIDED, therefore explicitly skip the
    # mission TAKEOFF item at FC seq 1 and start AUTO from WP1 at FC seq 2.
    test.master.mav.mission_set_current_send(
        test.master.target_system,
        test.master.target_component,
        first_fc_seq,
    )
    time.sleep(0.15)
    test.set_mode("AUTO")
    print(
        f"[AUTO] skipped FC seq=1 TAKEOFF; {' -> '.join(labels)} | "
        f"WP FC seq={first_fc_seq}..{final_fc_seq}"
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
            raise FlightTestError(f"GPS fix degraded during AUTO (fix={test.gps_fix})")

        if test.lidar_ok():
            lidar_loss_since = None
            agl = float(test.lidar)
            if agl < float(floor_m) or agl > float(ceiling_m):
                outside_since = outside_since or now
                if now - outside_since > 0.35:
                    raise FlightTestError(
                        f"AUTO LiDAR AGL out of guard: {agl:.2f}m "
                        f"desired={desired_agl_m:.2f}m guard={floor_m:.2f}..{ceiling_m:.2f}m"
                    )
            else:
                outside_since = None
        else:
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR stale >0.60s during AUTO")

        if test.reached_seq is not None:
            fc_seq = int(test.reached_seq)
            route_index = fc_seq - FC_FIRST_WP_SEQ
            if 0 <= route_index < len(labels) and fc_seq not in reached:
                reached.add(fc_seq)
                print(f"[AUTO] reached {labels[route_index]} (FC seq={fc_seq})")
            if fc_seq == final_fc_seq:
                print(f"[AUTO] final waypoint reached: {labels[-1]}")
                return

    raise FlightTestError("AUTO mission timeout")


def main():
    defaults = load_json("flight_defaults.json")
    nominal_agl = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description="Kingphoenix DEV preload HOME/TAKEOFF/WP mission -> LiDAR takeoff -> AUTO"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=parse_wp_range, default=parse_wp_range("1"))
    ap.add_argument("--alt", type=float, default=1.2, help="LiDAR takeoff AGL target")
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--climb-rate", type=float, default=0.30)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--ceiling", type=float, default=1.40)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--bridge-tol", type=float, default=0.45)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)
    ap.add_argument("--auto-timeout", type=float, default=60.0)
    ap.add_argument("--accept-radius", type=float, default=0.5)
    ap.add_argument("--margin", type=float, default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.auto_floor >= args.alt or args.auto_ceiling <= args.alt:
        raise SystemExit("AUTO LiDAR guard must bracket --alt")

    wp_numbers = list(args.wp)
    labels = [f"{args.strategy}/WP{n}" for n in wp_numbers]

    print("============================================================")
    print(" KINGPHOENIX PRELOAD HOME -> TAKEOFF -> WP, THEN LIDAR LAUNCH")
    print("============================================================")
    print(f"[PLAN] route={' -> '.join(labels)} -> LAND")
    print(f"[PLAN] physical takeoff target={args.alt:.2f}m raw LiDAR AGL")
    print("[PLAN] mission upload/readback happens while DISARMED")
    print("[PLAN] FC layout: seq0 HOME, seq1 TAKEOFF, seq2 WP1 ...")
    print("[PLAN] LiDAR GUIDED takeoff executes physically; AUTO starts at seq2 WP1")
    if args.margin is not None:
        print("[NOTE] --margin ignored; LiDAR remains takeoff authority")

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
                f"distance={d:.2f}m desiredAGL={wp['agl_m']:.2f}m"
            )
        if len(saved) == 1:
            d = distance_m(cur_lat, cur_lon, saved[0]["lat"], saved[0]["lon"])
            if d < 0.30:
                raise FlightTestError(
                    f"{labels[0]} only {d:.2f}m away; single-WP test blocked"
                )

        ground_lidar, ground_rel = capture_ground_baseline(test)
        print(
            f"[GROUND-BASELINE] LiDAR={ground_lidar:.2f}m FCrelAlt={ground_rel:.2f}m"
        )

        takeoff_target_rel = ground_rel + (float(args.alt) - ground_lidar)
        if not math.isfinite(takeoff_target_rel) or not -10.0 <= takeoff_target_rel <= 10.0:
            raise FlightTestError(
                f"computed TAKEOFF FC-relative altitude unsafe: {takeoff_target_rel:.2f}m"
            )

        mission_items = [
            {
                "label": "TAKEOFF",
                "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                "lat": 0.0,
                "lon": 0.0,
                "alt_m": takeoff_target_rel,
            }
        ]
        print(
            f"[ALT-PRELOAD] TAKEOFF desiredAGL={args.alt:.2f}m -> "
            f"FCrelAlt target={takeoff_target_rel:.2f}m"
        )

        for label, wp in zip(labels, saved):
            delta_agl = float(wp["agl_m"]) - ground_lidar
            fc_target_rel = ground_rel + delta_agl
            if not math.isfinite(fc_target_rel) or not -10.0 <= fc_target_rel <= 10.0:
                raise FlightTestError(
                    f"computed FC target altitude unsafe for {label}: {fc_target_rel:.2f}m"
                )
            mission_items.append(
                {
                    "label": label,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "lat": float(wp["lat"]),
                    "lon": float(wp["lon"]),
                    "alt_m": fc_target_rel,
                }
            )
            print(
                f"[ALT-PRELOAD] {label}: desiredAGL={wp['agl_m']:.2f}m -> "
                f"FCrelAlt target={fc_target_rel:.2f}m"
            )

        upload_mission(
            test,
            mission_items,
            acceptance_radius_m=args.accept_radius,
        )
        verify_uploaded_mission(test, mission_items)
        print("[SAFE] HOME -> TAKEOFF -> WP mission verified while vehicle is DISARMED")

        # Physical launch remains the already field-validated controller.
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

        settle_deadline = time.monotonic() + 0.8
        while time.monotonic() < settle_deadline:
            test.poll(0.08)
        if not test.lidar_ok() or test.global_pos is None:
            raise FlightTestError("telemetry invalid before AUTO handoff")

        first_wp_target_rel = float(mission_items[1]["alt_m"])
        current_rel = float(test.global_pos[2])
        rel_err = abs(current_rel - first_wp_target_rel)
        print(
            f"[AUTO-BRIDGE-CHECK] LiDAR={test.lidar:.2f}m "
            f"current FCrelAlt={current_rel:.2f}m "
            f"WP1 target={first_wp_target_rel:.2f}m error={rel_err:.2f}m"
        )
        if rel_err > float(args.bridge_tol):
            raise FlightTestError(
                f"FC altitude bridge mismatch {rel_err:.2f}m > {args.bridge_tol:.2f}m; AUTO blocked"
            )
        if not (args.auto_floor <= float(test.lidar) <= args.auto_ceiling):
            raise FlightTestError(
                f"LiDAR={test.lidar:.2f}m outside AUTO guard before switch"
            )

        run_auto(
            test,
            labels,
            desired_agl_m=args.alt,
            floor_m=args.auto_floor,
            ceiling_m=args.auto_ceiling,
            timeout=args.auto_timeout,
        )
        test.land_and_wait()
        print("[RESULT] PRELOADED HOME/TAKEOFF/WP -> LIDAR TAKEOFF -> AUTO -> LAND PASS")
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
