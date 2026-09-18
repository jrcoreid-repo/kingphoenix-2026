#!/usr/bin/env python3
"""Hardware-free constructor/wiring smoke check for the full race runtime.

This intentionally does NOT open MAVLink, cameras, Hailo, servos, or threads.
It verifies that production modules import together, constructor signatures
match, and the final mission contract reaches every critical runtime component.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.runtime import KingphoenixRuntime


def main():
    runtime = KingphoenixRuntime()

    assert runtime.cfg["mavlink"]["connection"] == "udpin:127.0.0.1:14550"
    assert runtime.cfg["mission_planner_gateway"]["enabled"] is False
    mission_conf = min(
        float(runtime.full_cfg["vision"]["confidence_threshold"]),
        float(runtime.full_cfg["guided_fuzzy"]["confidence_threshold"]),
    )
    assert abs(float(runtime.cfg["ai"]["detector"]["confidence_threshold"]) - mission_conf) < 1e-9
    assert abs(float(runtime.cfg["vision_control"]["confidence_min"]) - mission_conf) < 1e-9

    assert runtime.full_cfg["mission"]["total_waypoints"] == 11
    assert runtime.full_cfg["mission"]["ai_enable_wp"] == 6
    assert runtime.full_cfg["mission"]["drop_wp"] == 7
    assert runtime.full_cfg["mission"]["final_wp"] == 11

    assert runtime.waypoints.total_waypoints == 11
    assert runtime.waypoints.final_wp == 11

    assert runtime.mission.fc_sync is runtime.fc_sync
    assert runtime.mission.vision is runtime.vision_stream
    assert runtime.mission.servo is runtime.servo
    assert runtime.mission.takeoff is runtime.takeoff
    assert runtime.mission.fuzzy is runtime.fuzzy

    assert runtime.servo.channels == (7, 8)
    assert runtime.servo.open_pwm == 1100
    assert runtime.servo.close_pwm == 2000

    assert runtime.takeoff.cfg["target_alt_m"] == 0.50
    assert runtime.takeoff.cfg["fast_vz_mps"] == 0.80

    assert runtime.fuzzy.cfg["enabled"] is True
    assert runtime.fuzzy.cfg["x_sign"] == -1
    assert runtime.fuzzy.cfg["y_sign"] == -1

    assert runtime.vision_stream.ai_router.snapshot().camera == "none"
    assert runtime.vision_stream.ai_router.snapshot().model_mode == "OFF"
    assert runtime.vision_stream.shared_ai is not None

    assert runtime.mp_gateway is None
    assert runtime.mission not in runtime.workers

    print("[WIRING-CHECK] production module integration PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
