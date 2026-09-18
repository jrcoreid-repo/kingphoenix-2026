#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import load_json
from mission.full_mission_config import load_full_mission_config


def main():
    full = load_full_mission_config()
    system = load_json("system.json")
    waypoints = load_json("waypoints.json")

    assert full["mission"]["total_waypoints"] == 11
    assert full["mission"]["ai_enable_wp"] == 6
    assert full["mission"]["drop_wp"] == 7
    assert full["mission"]["final_wp"] == 11
    assert abs(float(full["takeoff"]["target_alt_m"]) - 0.50) < 1e-9
    assert abs(float(full["takeoff"]["fast_vz_mps"]) - 0.80) < 1e-9
    assert full["waypoint"]["override_auto_speed"] is False
    assert full["servo"]["channels"] == [7, 8]
    assert int(full["servo"]["open_pwm"]) == 1100
    assert int(full["servo"]["close_pwm"]) == 2000
    assert full["servo"]["require_closed_before_start"] is True
    assert float(full["landing"]["timeout_s"]) >= 5.0

    mav = system["mavlink"]
    assert mav["transport"] == "MAVPROXY_UDP"
    assert mav["connection"] == "udpin:127.0.0.1:14550"

    route = system["ai"]["route"]
    assert route["camera"] == "none"
    assert route["model_mode"] == "OFF"
    assert system["ai"]["test_switch"]["enabled"] is False
    assert system["run_mode"]["profile"] == "FULL_MISSION"

    for strategy in ("LEFT", "RIGHT"):
        assert strategy in waypoints
        assert set(waypoints[strategy]) == {f"WP{n}" for n in range(1, 12)}
        for n in range(1, 12):
            assert f"WP{n}" in waypoints[strategy]
            assert abs(float(waypoints[strategy][f"WP{n}"]["alt_m"]) - 1.0) < 1e-9

    print("[SELF-CHECK] full mission contract PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
