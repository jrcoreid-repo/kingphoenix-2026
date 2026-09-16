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


def read_json_waypoints(strategy: str):
    data = load_json("waypoints.json")
    strategy = str(strategy).upper()
    if strategy not in data:
        raise FlightTestError(f"strategy {strategy} not found in waypoints.json")
    return data[strategy]


def waypoint_saved(item: dict) -> bool:
    lat = item.get("lat")
    lon = item.get("lon")
    if lat is None or lon is None:
        return False
    try:
        lat = float(lat)
        lon = float(lon)
        return (
            math.isfinite(lat)
            and math.isfinite(lon)
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
            and not (lat == 0.0 and lon == 0.0)
        )
    except Exception:
        return False


def command_name(command: int) -> str:
    try:
        return mavutil.mavlink.enums["MAV_CMD"][int(command)].name
    except Exception:
        return str(int(command))


def frame_name(frame: int) -> str:
    names = {
        mavutil.mavlink.MAV_FRAME_GLOBAL: "GLOBAL/AMSL",
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT: "REL_HOME",
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT: "GLOBAL_INT/AMSL",
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: "REL_HOME_INT",
        getattr(mavutil.mavlink, "MAV_FRAME_GLOBAL_TERRAIN_ALT", 10): "TERRAIN",
        getattr(mavutil.mavlink, "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT", 11): "TERRAIN_INT",
    }
    return names.get(int(frame), str(int(frame)))


def request_fc_mission(test: DirectFlightTest, timeout=5.0) -> list[dict]:
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    try:
        test.master.mav.mission_request_list_send(
            test.master.target_system, test.master.target_component, mtype
        )
    except TypeError:
        test.master.mav.mission_request_list_send(
            test.master.target_system, test.master.target_component
        )

    count = None
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        msg = test.poll(0.25)
        if msg is not None and msg.get_type() == "MISSION_COUNT":
            count = int(msg.count)
            break
    if count is None:
        raise FlightTestError("FC did not return MISSION_COUNT")
    if count == 0:
        return []

    items = []
    for seq in range(count):
        try:
            test.master.mav.mission_request_int_send(
                test.master.target_system, test.master.target_component, seq, mtype
            )
        except TypeError:
            test.master.mav.mission_request_int_send(
                test.master.target_system, test.master.target_component, seq
            )

        item = None
        item_deadline = time.monotonic() + float(timeout)
        while time.monotonic() < item_deadline:
            msg = test.poll(0.25)
            if msg is None:
                continue
            t = msg.get_type()
            if t not in ("MISSION_ITEM_INT", "MISSION_ITEM"):
                continue
            if int(getattr(msg, "seq", -1)) != seq:
                continue
            if t == "MISSION_ITEM_INT":
                lat = float(msg.x) / 1e7
                lon = float(msg.y) / 1e7
            else:
                lat = float(msg.x)
                lon = float(msg.y)
            item = {
                "seq": seq,
                "command": int(msg.command),
                "frame": int(msg.frame),
                "lat": lat,
                "lon": lon,
                "alt_m": float(msg.z),
            }
            break
        if item is None:
            raise FlightTestError(f"FC mission item seq={seq} download timeout")
        items.append(item)
    return items


def print_json_table(strategy: str, entries: dict):
    print("\nCURRENT JSON")
    print(f"Strategy : {strategy}")
    print("WP     STATUS    LATITUDE       LONGITUDE       DESIRED AGL")
    print("--------------------------------------------------------------------")

    def wp_number(name: str):
        try:
            return int(str(name).upper().replace("WP", ""))
        except Exception:
            return 9999

    for key in sorted(entries, key=wp_number):
        item = entries[key]
        if waypoint_saved(item):
            alt = float(item.get("alt_m", 0.0))
            print(
                f"{key:<6} SAVED     {float(item['lat']):>11.7f}  "
                f"{float(item['lon']):>12.7f}  {alt:>5.2f}m"
            )
        else:
            print(f"{key:<6} EMPTY     --             --              --")


def print_fc_table(items: list[dict]):
    print("\nCURRENT FC MISSION")
    if not items:
        print("Mission count : 0 (EMPTY)")
        return

    print(f"Raw FC count  : {len(items)}")
    print("Expected layout: seq0 HOME -> seq1 TAKEOFF -> seq2 WP1 -> seq3 WP2 ...")
    print("NOTE: HOME altitude is AMSL. TAKEOFF/WP altitude is FC REL_HOME, not raw LiDAR AGL.")
    print("SEQ  ROLE     FRAME           COMMAND                LATITUDE       LONGITUDE       ALT")
    print("------------------------------------------------------------------------------------------------")
    for item in items:
        seq = int(item["seq"])
        cmd = int(item["command"])
        if seq == FC_HOME_SEQ:
            role = "HOME"
        elif seq == FC_TAKEOFF_SEQ and cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            role = "TAKEOFF"
        else:
            role = "WP"
        print(
            f"{seq:<4} {role:<8} {frame_name(item['frame']):<15} "
            f"{command_name(cmd):<22} {item['lat']:>11.7f}  "
            f"{item['lon']:>12.7f}  {item['alt_m']:>6.2f}m"
        )


def compare(
    entries: dict,
    strategy: str,
    wp_numbers: list[int],
    fc_items: list[dict],
    position_tol_m: float,
) -> bool:
    print("\nCOMPARE JSON <-> FC")
    print(f"Expected route : {' -> '.join(f'{strategy}/WP{n}' for n in wp_numbers)}")
    print("Mapping: seq0 HOME, seq1 TAKEOFF, first route waypoint at seq2.")
    print("Strict route MATCH uses LAT/LON; JSON altitude is desired physical AGL.")

    expected_raw_count = len(wp_numbers) + 2
    if len(fc_items) != expected_raw_count:
        print(
            f"[MISMATCH] FC raw mission count={len(fc_items)} expected={expected_raw_count} "
            f"(HOME + TAKEOFF + {len(wp_numbers)} WP)"
        )
        return False
    if int(fc_items[0].get("seq", -1)) != FC_HOME_SEQ:
        print("[MISMATCH] FC HOME seq0 missing")
        return False
    if int(fc_items[FC_TAKEOFF_SEQ]["command"]) != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        print(
            f"[MISMATCH] FC seq1 command={command_name(fc_items[1]['command'])}, expected NAV_TAKEOFF"
        )
        return False

    print(
        f"[MATCH] TAKEOFF marker at FC seq1, FC_ALT={fc_items[1]['alt_m']:.2f}m "
        f"frame={frame_name(fc_items[1]['frame'])}"
    )

    all_match = True
    for route_index, wp_num in enumerate(wp_numbers):
        key = f"WP{wp_num}"
        item = entries.get(key)
        if not isinstance(item, dict) or not waypoint_saved(item):
            print(f"[MISMATCH] {strategy}/{key} JSON is EMPTY/INVALID")
            all_match = False
            continue

        fc_seq = FC_FIRST_WP_SEQ + route_index
        fc = fc_items[fc_seq]
        if int(fc["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            print(
                f"[MISMATCH] {strategy}/{key} -> FC seq={fc_seq} "
                f"command={command_name(fc['command'])}, expected NAV_WAYPOINT"
            )
            all_match = False
            continue

        pos_err = distance_m(
            float(item["lat"]), float(item["lon"]), fc["lat"], fc["lon"]
        )
        ok = pos_err <= float(position_tol_m)
        state = "MATCH" if ok else "MISMATCH"
        print(
            f"[{state}] {strategy}/{key} -> FC seq={fc_seq} "
            f"position_error={pos_err:.3f}m | JSON_AGL={float(item.get('alt_m', 0.0)):.2f}m "
            f"| FC_ALT={fc['alt_m']:.2f}m frame={frame_name(fc['frame'])}"
        )
        all_match = all_match and ok
    return all_match


def main():
    ap = argparse.ArgumentParser(
        description="Read-only checker for HOME -> TAKEOFF -> WP Kingphoenix mission layout"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp", type=parse_wp_range, default=None)
    ap.add_argument("--position-tol", type=float, default=0.30)
    args = ap.parse_args()

    print("============================================================")
    print(" KINGPHOENIX WAYPOINT / FC MISSION CHECKER")
    print("============================================================")
    print("READ ONLY: this tool does not upload, clear, arm, or change mode.")
    print("IMPORTANT: stop main.py/MAVProxy first; this tool needs direct UART ownership.")

    try:
        entries = read_json_waypoints(args.strategy)
        print_json_table(args.strategy, entries)
    except Exception as exc:
        print(f"[BLOCKED] cannot read waypoint JSON: {exc}")
        return 2

    test = DirectFlightTest()
    try:
        test.open()
        fc_items = request_fc_mission(test)
        print_fc_table(fc_items)
        if args.wp is None:
            print("\nRESULT: JSON and FC snapshots displayed.")
            print("For strict comparison use, for example:")
            print("  python3 tools/check_waypoints.py --strategy LEFT --wp 1-2")
            return 0

        ok = compare(entries, args.strategy, list(args.wp), fc_items, args.position_tol)
        print(
            "\nRESULT: "
            + (
                "MATCH - HOME/TAKEOFF layout and route positions are consistent"
                if ok
                else "MISMATCH - do not fly AUTO until mission layout is corrected"
            )
        )
        return 0 if ok else 3
    except KeyboardInterrupt:
        print("\n[STOP] operator interrupted")
        return 130
    except FlightTestError as exc:
        print(f"\n[FAILED] {exc}")
        return 2
    except Exception as exc:
        print(f"\n[FAILED] {type(exc).__name__}: {exc}")
        return 2
    finally:
        test.close()


if __name__ == "__main__":
    raise SystemExit(main())
