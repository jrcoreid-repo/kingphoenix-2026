#!/usr/bin/env python3
"""Read-only TFmini Plus / rangefinder MAVLink bench test.

Expected architecture:
    TFmini Plus -> I2C -> Kakute/ArduPilot -> MAVLink -> Kingphoenix

This tool never opens Raspberry Pi I2C directly and never sends movement
commands. It uses the same production MAVLink RX and stream configurator.
"""

from __future__ import annotations

from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import load_json
from core.shared_state import SharedState
from core.event_bus import EventBus
from flight.mavlink_connection import MavlinkConnection
from flight.mavlink_rx import MavlinkRxWorker
from flight.mavlink_tx import MavlinkTx
from flight.mavlink_stream_config import MavlinkStreamConfigurator
from flight.rangefinder_state import RangefinderStatePolicy


def main():
    cfg = load_json("system.json")
    mav_cfg = cfg["mavlink"]
    rf_cfg = cfg.get("rangefinder", {})

    state = SharedState()
    events = EventBus()
    stop = threading.Event()

    conn = MavlinkConnection(mav_cfg)
    tx = MavlinkTx(conn)

    rx = MavlinkRxWorker(
        conn, state, events, stop,
        heartbeat_timeout_s=mav_cfg.get("heartbeat_timeout_s", 2.5),
    )

    stream_cfg = MavlinkStreamConfigurator(
        tx, state, stop,
        mav_cfg.get("message_rates_hz", {}),
        retry_interval_s=mav_cfg.get("rate_retry_interval_s", 10.0),
    )

    policy = RangefinderStatePolicy(
        state.vehicle,
        max_age_s=rf_cfg.get("valid_age_s", 0.15),
        expected_orientation=rf_cfg.get("expected_orientation", 25),
        allow_legacy_fallback=rf_cfg.get("allow_legacy_fallback", True),
    )

    try:
        conn.open()
        rx.start()
        stream_cfg.start()

        print("======================================================")
        print(" KINGPHOENIX V1.5 RANGEFINDER TEST")
        print(" TFmini Plus -> I2C FC -> MAVLink")
        print(" Ctrl+C to stop")
        print("======================================================")

        while True:
            v = state.vehicle
            check = policy.check()

            def rate(name):
                r = v.message_rate_hz(name)
                return "--" if r <= 0 else f"{r:.1f} Hz"

            age = check.age_s
            age_s = "--" if age == float("inf") else f"{age:.3f} s"
            dist = "--" if check.distance_m is None else f"{check.distance_m:.3f} m"

            print(
                f"distance={dist:>10}  "
                f"source={check.source:<15} "
                f"age={age_s:>8} "
                f"status={'VALID' if check.ok else 'INVALID':<7} "
                f"reason={check.reason}"
            )
            print(
                f"  DISTANCE_SENSOR={rate('DISTANCE_SENSOR')} | "
                f"RANGEFINDER={rate('RANGEFINDER')} | "
                f"orientation={v.rangefinder_orientation} | id={v.rangefinder_id}"
            )

            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        rx.join(timeout=1.0)
        stream_cfg.join(timeout=1.0)
        conn.close()


if __name__ == "__main__":
    main()
