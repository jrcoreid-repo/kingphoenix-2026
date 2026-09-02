#!/usr/bin/env python3
"""Kingphoenix V2.3.0 guarded FC bench-control runtime.

Keeps all V2.2.3 telemetry/readiness monkeypatches and starts the updated core
runtime where the GCS server is wired to CommandManager/VCT. Real command TX
is intentionally limited to ARM, DISARM and DISARMED flight-mode changes.
Movement, takeoff, mission velocity and servo PWM TX remain disabled.
"""

from __future__ import annotations

# Import installs the validated V2.2.3 telemetry/readiness patches.
import main_v2_2  # noqa: F401

from core.runtime import KingphoenixRuntime


def main():
    print("============================================================")
    print(" KINGPHOENIX V2.3.0 - GUARDED FC BENCH CONTROL")
    print("============================================================")
    print("[V2.3] Real GCS TX: ARM / DISARM / DISARMED mode selection")
    print("[V2.3] Existing CommandManager/VCT performs ACK + state verification")
    print("[V2.3] Stream profile: 640x400 @20fps, 2000/1700 kbps")
    print("[V2.3] Servo PWM + movement/takeoff/mission TX remain DISABLED")

    runtime = KingphoenixRuntime()
    try:
        runtime.start()
        runtime.wait()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
