#!/usr/bin/env python3
"""Read-only waypoint JSON inspector for standalone mission-part tests."""

from __future__ import annotations

import argparse

from config import load_json


def main():
    parser = argparse.ArgumentParser(description="Inspect Kingphoenix saved waypoint JSON")
    parser.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    parser.add_argument("--wp", type=int, default=None, help="Optional waypoint number 1..12")
    args = parser.parse_args()

    data = load_json("waypoints.json")
    defaults = load_json("flight_defaults.json")
    nominal = float(defaults.get("nominal_waypoint_alt_m", 1.2))
    strategy = args.strategy.upper()

    print(f"KINGPHOENIX WAYPOINTS | strategy={strategy} | nominal_alt={nominal:.2f}m")
    print("source: config/waypoints.json")

    if args.wp is not None:
        key = f"WP{int(args.wp)}"
        item = dict(data[strategy][key])
        item.setdefault("alt_m", nominal)
        print(f"{key}: {item}")
        return

    for key, raw in data[strategy].items():
        item = dict(raw)
        item.setdefault("alt_m", nominal)
        state = "SAVED" if item.get("x") is not None and item.get("y") is not None else "EMPTY"
        print(f"{key:>4}  {state:5}  x={item.get('x')} y={item.get('y')} alt_m={item.get('alt_m')}")


if __name__ == "__main__":
    main()
