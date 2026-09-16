#!/usr/bin/env python3
"""Higher-range tuning wrapper for the validated front-gate follow field tool.

This wrapper changes ONLY CLI validation limits for lateral P tuning. The
underlying flight logic, LiDAR takeoff, target-loss zeroing, GCS requirement,
altitude guards, and hard-disabled forward/backward + vertical velocity remain
exactly those in run_vision_follow_gate_test.py.

Use the conservative base tool first. This wrapper exists so field tuning can
try stronger lateral gains/speed caps without editing source between runs.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from tools import run_vision_follow_gate_test as base


def relaxed_validate_args(args):
    if not 1.0 <= args.duration <= 120.0:
        raise SystemExit("--duration must be 1..120s")
    if not 5.0 <= args.control_hz <= 20.0:
        raise SystemExit("--control-hz must be 5..20Hz")

    # Expanded ONLY for field tuning. These are parser limits, not recommended
    # starting values. Begin around Kp=1.2 / cap=0.6 and increase gradually.
    if not 0.05 <= args.kp_lat <= 3.0:
        raise SystemExit("--kp-lat must be 0.05..3.0")
    if not 1.0 <= args.kp_yaw <= 30.0:
        raise SystemExit("--kp-yaw must be 1..30 deg/s per EX")
    if not 0.05 <= args.lat_cap <= 2.0:
        raise SystemExit("--lat-cap must be 0.05..2.00m/s")
    if not 1.0 <= args.yaw_cap <= 10.0:
        raise SystemExit("--yaw-cap must be 1..10deg/s")
    if not 0.01 <= args.deadband <= 0.15:
        raise SystemExit("--deadband must be 0.01..0.15")
    if not 0.05 <= args.filter_alpha <= 1.0:
        raise SystemExit("--filter-alpha must be 0.05..1.0")
    if not 0.25 <= args.confidence <= 0.95:
        raise SystemExit("--confidence must be 0.25..0.95")
    if not 1 <= args.lock_frames <= 20:
        raise SystemExit("--lock-frames must be 1..20")
    if not 0.0 <= args.lost_timeout <= 1.0:
        raise SystemExit("--lost-timeout must be 0..1.0s")

    if not 0.50 <= args.alt <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if not 0.10 <= args.climb_rate <= 0.80:
        raise SystemExit("--climb-rate must be 0.10..0.80m/s")
    if not 0.08 <= args.near_rate <= args.climb_rate:
        raise SystemExit("--near-rate must be 0.08..climb-rate")
    if args.takeoff_ceiling <= args.alt:
        raise SystemExit("--takeoff-ceiling must be above --alt")
    if not 0.30 <= args.guard_floor < args.guard_ceiling:
        raise SystemExit("invalid --guard-floor/--guard-ceiling")
    if args.guard_ceiling > 2.0:
        raise SystemExit("--guard-ceiling max for this field tool is 2.0m")
    if not 0.10 <= args.guard_outside_s <= 1.0:
        raise SystemExit("--guard-outside-s must be 0.10..1.0")
    if not 0.20 <= args.lidar_stale_s <= 1.0:
        raise SystemExit("--lidar-stale-s must be 0.20..1.0")


# Replace only argument validation. All flight/control implementation remains
# in the original tested tool.
base.validate_args = relaxed_validate_args


if __name__ == "__main__":
    print("[TUNING] expanded lateral range enabled: Kp <= 3.0, cap <= 2.0 m/s")
    print("[TUNING] recommended next step: Kp=1.20, cap=0.60 m/s")
    raise SystemExit(base.main())
