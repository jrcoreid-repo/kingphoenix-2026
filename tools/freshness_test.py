#!/usr/bin/env python3
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flight.vehicle_state import VehicleState
from flight.freshness_policy import FreshnessPolicy


def main():
    v = VehicleState()
    policy = FreshnessPolicy(v, {
        "HEARTBEAT": 2.5,
        "ATTITUDE": 0.1,
    })

    print("[1] Before messages:")
    print(policy.check("HEARTBEAT"))

    now = time.monotonic_ns()
    v.mark_message("HEARTBEAT", now)
    v.mark_message("ATTITUDE", now)

    print("\n[2] Immediately after messages:")
    print(policy.check("HEARTBEAT"))
    print(policy.check("ATTITUDE"))

    time.sleep(0.15)

    print("\n[3] After 150 ms:")
    print(policy.check("HEARTBEAT"))
    print(policy.check("ATTITUDE"))


if __name__ == "__main__":
    main()
