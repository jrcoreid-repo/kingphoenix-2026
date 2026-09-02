#!/usr/bin/env python3
"""Kingphoenix V2.3.2 guarded FC bench-control + FC message telemetry.

Extends the validated V2.3 runtime. Publishes the latest ArduPilot STATUSTEXT
record inside the existing camera telemetry metadata so the GCS can show the
actual FC reason for pre-arm failures, warnings, failsafes, etc.

No new flight command authority is introduced here.
"""

from __future__ import annotations

import time

# Preserve V2.3 guarded ARM/DISARM/mode command path and all V2.2.3 telemetry.
import main_v2_3  # noqa: F401

from core.runtime import KingphoenixRuntime
from streaming.vision_stream_session import CameraStreamWorker


_original_telemetry_snapshot = CameraStreamWorker._telemetry_snapshot


def _telemetry_snapshot_v232(self):
    out = dict(_original_telemetry_snapshot(self) or {})
    v = self.state.vehicle

    latest = None
    try:
        with v.status_text_lock:
            if v.status_text_history:
                latest = v.status_text_history[-1]
    except Exception:
        latest = None

    if latest is None:
        out.update({
            "fc_message": None,
            "fc_message_severity": None,
            "fc_message_age_s": None,
            "fc_message_source": None,
        })
        return out

    age_s = max(0.0, (time.monotonic_ns() - int(latest.timestamp_ns)) / 1e9)
    out.update({
        "fc_message": str(latest.text or "").strip() or None,
        "fc_message_severity": int(latest.severity),
        "fc_message_age_s": age_s,
        "fc_message_source": f"{int(latest.source_system)}:{int(latest.source_component)}",
    })
    return out


CameraStreamWorker._telemetry_snapshot = _telemetry_snapshot_v232


def main():
    print("============================================================")
    print(" KINGPHOENIX V2.3.2 - FC MESSAGE DEBUG TELEMETRY")
    print("============================================================")
    print("[V2.3.2] Latest ArduPilot STATUSTEXT published to GCS metadata")
    print("[V2.3.2] ARM / DISARM / DISARMED mode TX remains guarded by VCT")
    print("[V2.3.2] Servo PWM + movement/takeoff/mission TX remain DISABLED")

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
