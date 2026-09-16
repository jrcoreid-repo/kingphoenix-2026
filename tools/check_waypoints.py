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


def request_fc_mission(test: DirectFlightTest, timeout=5.0) -> list[dict]:
    """Download the raw ArduPilot mission list.

    ArduPilot exposes HOME as mission sequence 0. Real uploaded mission items
    therefore begin at sequence 1 on readback, even though the upload protocol
    requested GCS item sequence 0 for the first real waypoint.
    """
    mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

    try:
        test.master.mav.mission_request_list_send(
            test.master.target_system,
            test.master.target_component,
            mtype,
        )
    except TypeError:
        test.master.mav.mission_request_list_send(
            test.master.target_system,
            test.master.target_component,
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
                test.master.target_system,
                test.master.target_component,
                seq,
                mtype,
            )
        except TypeError:
            test.master.mav.mission_request_int_send(
                test.master.target_system,
                test.master.target_component,
                seq,
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
                "is_home": seq == 0,
            }
            break

        if item is None:
            raise FlightTestError(f"FC mission item seq={seq} download timeout")
        items.append(item)

    return items


def print_json_table(strategy: str, entries: dict):
    print("\nCURRENT JSON")
    print(f"Strategy : {strategy}")
    print("WP     STATUS    LATITUDE       LONGITUDE       ALT")
    print("---------------------------------------------------------------")

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
    real_count = max(0, len(items) - 1)
    print(f"Raw FC count  : {len(items)} (includes HOME seq=0)")
    print(f"Real mission  : {real_count} item(s)")
    print("SEQ  ROLE   COMMAND                LATITUDE       LONGITUDE       ALT")
    print("----------------------------------------------------------------------------")
    for item in items:
        name = command_name(item["command"])
        role = "HOME" if item.get("is_home") else "MISSION"
        print(
            f"{item['seq']:<4} {role:<6} {name:<22} {item['lat']:>11.7f}  "
            f"{item['lon']:>12.7f}  {item['alt_m']:>5.2f}m"
        )


def compare(
    entries: dict,
    strategy: str,
    wp_numbers: list[int],
    fc_items: list[dict],
    position_tol_m: float,
    alt_tol_m: float,
) -> bool:
    print("\nCOMPARE JSON <-> FC")
    print(f"Expected route : {' -> '.join(f'{strategy}/WP{n}' for n in wp_numbers)}")
    print("ArduPilot mapping: FC seq=0 HOME, first real waypoint starts at seq=1")

    expected_raw_count = len(wp_numbers) + 1
    if len(fc_items) != expected_raw_count:
        print(
            f"[MISMATCH] FC raw mission count={len(fc_items)} "
            f"expected={expected_raw_count} (HOME + {len(wp_numbers)} waypoint(s))"
        )
        return False

    if not fc_items or int(fc_items[0].get("seq", -1)) != 0:
        print("[MISMATCH] FC HOME seq=0 missing")
        return False

    all_match = True
    for route_index, wp_num in enumerate(wp_numbers):
        key = f"WP{wp_num}"
        item = entries.get(key)
        if not isinstance(item, dict) or not waypoint_saved(item):
            print(f"[MISMATCH] {strategy}/{key} JSON is EMPTY/INVALID")
            all_match = False
            continue

        fc_seq = route_index + 1
        fc = fc_items[fc_seq]
        if int(fc["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            print(
                f"[MISMATCH] {strategy}/{key} -> FC seq={fc_seq} command="
                f"{command_name(fc['command'])}, expected MAV_CMD_NAV_WAYPOINT"
            )
            all_match = False
            continue

        pos_err = distance_m(
            float(item["lat"]),
            float(item["lon"]),
            fc["lat"],
            fc["lon"],
        )
        alt_json = float(item.get("alt_m", 0.0))
        alt_err = abs(alt_json - float(fc["alt_m"]))
        ok = pos_err <= float(position_tol_m) and alt_err <= float(alt_tol_m)
        state = "MATCH" if ok else "MISMATCH"
        print(
            f"[{state}] {strategy}/{key} -> FC seq={fc_seq} "
            f"position_error={pos_err:.3f}m alt_error={alt_err:.3f}m"
        )
        all_match = all_match and ok

    return all_match


def main():
    ap = argparse.ArgumentParser(
        description="Read-only Kingphoenix checker: config/waypoints.json versus current mission stored in FC"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument(
        "--wp",
        type=parse_wp_range,
        default=None,
        help="optional expected FC route, e.g. 1, 1-2, or 2-5",
    )
    ap.add_argument("--position-tol", type=float, default=0.30, help="MATCH tolerance in meters")
    ap.add_argument("--alt-tol", type=float, default=0.05, help="MATCH altitude tolerance in meters")
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

        ok = compare(
            entries,
            args.strategy,
            list(args.wp),
            fc_items,
            args.position_tol,
            args.alt_tol,
        )
        print(
            "\nRESULT: "
            + (
                "MATCH - JSON and FC mission are consistent"
                if ok
                else "MISMATCH - do not assume FC mission matches JSON"
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
