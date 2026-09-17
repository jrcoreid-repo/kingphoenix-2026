#!/usr/bin/env python3
"""Kingphoenix hybrid drop V4: V3 GCS flow with 1.7m race altitude profile.

Changes relative to V3:
- force mission waypoint altitude to 1.70m for every waypoint loaded by this runner
- force drop/search timeout to 10.0s
- keep enough LiDAR guard headroom for 1.70m flight
- preserve V3 GCS streaming + preflight servo flow + terminal GO behavior

V1/V2/V3 are intentionally left untouched.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import tools.run_hybrid_drop_race_launch_v2 as race_v2
import tools.run_hybrid_drop_race_launch_gcs_v3 as race_v3

RACE_WP_ALT_M = 1.70
DROP_TIMEOUT_S = 10.0
AUTO_CEILING_M = 2.05
DROP_GUARD_CEILING_M = 2.00

_orig_load_waypoint = race_v3.load_waypoint


def parse_args_v4():
    a = race_v2.parse_args()

    # V4 profile is intentionally fixed for today's field test so an old
    # command line cannot accidentally keep the previous 1.2m/1.55m profile.
    a.alt = RACE_WP_ALT_M
    a.drop_timeout = DROP_TIMEOUT_S

    if float(a.auto_ceiling) < AUTO_CEILING_M:
        print(
            f"[V4 PROFILE] auto-ceiling {a.auto_ceiling:.2f}m -> "
            f"{AUTO_CEILING_M:.2f}m for 1.70m cruise"
        )
        a.auto_ceiling = AUTO_CEILING_M

    if float(a.guard_ceiling) < DROP_GUARD_CEILING_M:
        print(
            f"[V4 PROFILE] drop guard ceiling {a.guard_ceiling:.2f}m -> "
            f"{DROP_GUARD_CEILING_M:.2f}m for 1.70m drop phase"
        )
        a.guard_ceiling = DROP_GUARD_CEILING_M

    print(
        f"[V4 PROFILE] WP altitude={RACE_WP_ALT_M:.2f}m | "
        f"drop/search timeout={DROP_TIMEOUT_S:.1f}s | "
        f"AUTO ceiling={a.auto_ceiling:.2f}m | drop ceiling={a.guard_ceiling:.2f}m"
    )
    return a


def validate_v4(a):
    # Reuse the existing V2 validation for everything except its old 5s
    # drop-timeout cap, then apply V4's 10s limit explicitly.
    requested_timeout = float(a.drop_timeout)
    a.drop_timeout = min(requested_timeout, 5.0)
    try:
        race_v2.validate(a)
    finally:
        a.drop_timeout = requested_timeout

    if not 0.50 <= requested_timeout <= 10.0:
        raise SystemExit("--drop-timeout must be 0.50..10.00s in V4")


def load_waypoint_v4(strategy: str, wp_num: int) -> dict:
    wp = _orig_load_waypoint(strategy, wp_num)
    stored_alt = float(wp["alt_m"])
    wp["alt_m"] = RACE_WP_ALT_M
    if abs(stored_alt - RACE_WP_ALT_M) > 1e-6:
        print(
            f"[V4 WP ALT] {wp['label']} stored={stored_alt:.2f}m -> "
            f"mission={RACE_WP_ALT_M:.2f}m"
        )
    return wp


def main() -> int:
    # Patch only the symbols V3 resolves at runtime. No persistent mutation to
    # waypoints.json is required, so saved RTK coordinates remain untouched.
    race_v3.parse_args = parse_args_v4
    race_v3.validate = validate_v4
    race_v3.load_waypoint = load_waypoint_v4
    return race_v3.main()


if __name__ == "__main__":
    raise SystemExit(main())
