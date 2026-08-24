#!/usr/bin/env python3
"""Kingphoenix V2.1.6 runtime wrapper.

Adds low-overhead wlan1 RSSI telemetry and a selective read-only MAVLink
telemetry snapshot for the GCS. Flight movement/control TX remains disabled.
"""

from __future__ import annotations

from pathlib import Path

from core.runtime import KingphoenixRuntime
from system.health import PiHealthMonitor
from streaming.vision_stream_session import CameraStreamWorker


WIFI_INTERFACE = "wlan1"


def _read_wifi_rssi_dbm(interface=WIFI_INTERFACE):
    """Read RSSI from /proc/net/wireless. Returns None if unavailable."""
    try:
        text = Path("/proc/net/wireless").read_text()
        for line in text.splitlines():
            if ":" not in line:
                continue
            name, rest = line.split(":", 1)
            if name.strip() != interface:
                continue
            fields = rest.split()
            if len(fields) < 3:
                return None
            return float(fields[2].rstrip("."))
    except Exception:
        return None
    return None


def _wifi_quality(rssi):
    if rssi is None:
        return "OFFLINE", "FAIL"
    if rssi >= -60:
        return "EXCELLENT", "PASS"
    if rssi >= -67:
        return "GOOD", "PASS"
    if rssi >= -74:
        return "FAIR", "WARN"
    if rssi >= -82:
        return "WEAK", "WARN"
    return "CRITICAL", "FAIL"


_original_sample = PiHealthMonitor.sample


def _sample_with_wifi(self):
    health = _original_sample(self)
    rssi = _read_wifi_rssi_dbm(WIFI_INTERFACE)
    quality, wifi_status = _wifi_quality(rssi)

    health["wifi_interface"] = WIFI_INTERFACE
    health["wifi_rssi_dbm"] = rssi
    health["wifi_quality"] = quality
    health["wifi_connected"] = rssi is not None
    health.setdefault("checks", {})["wifi_link"] = {
        "status": wifi_status,
        "detail": (
            f"{rssi:.0f} dBm / {quality}"
            if rssi is not None
            else f"{WIFI_INTERFACE} offline"
        ),
    }
    return health


PiHealthMonitor.sample = _sample_with_wifi


# ---------------------------------------------------------------------------
# Selective MAVLink telemetry -> GCS
# ---------------------------------------------------------------------------
# We intentionally do NOT forward every MAVLink message/parameter to the GUI.
# The Pi remains the single FC serial owner and only exports an operator-facing
# snapshot derived from VehicleState.

def _telemetry_snapshot_v216(self):
    v = self.state.vehicle
    with self.state.lock:
        pi_health = dict(self.state.system.get("pi_health") or {})

    heartbeat_age = v.heartbeat_age_s()
    if heartbeat_age == float("inf"):
        heartbeat_age = None

    return {
        "connected": bool(v.connected),
        "heartbeat_age_s": heartbeat_age,
        "mode": str(v.mode or "UNKNOWN"),
        "armed": bool(v.armed),
        "battery_v": v.battery_v,
        "battery_remaining": v.battery_remaining,
        "relative_alt_m": v.relative_alt_m,
        "groundspeed_mps": v.groundspeed_mps,
        "gps_satellites": v.gps_satellites,
        "gps_fix_type": v.gps_fix_type,
        "rangefinder_m": v.rangefinder_distance_m,
        "rangefinder_source": v.rangefinder_source,
        "rangefinder_age_s": (
            None if v.rangefinder_last_ns == 0 else v.rangefinder_age_s()
        ),
        "mission_item": v.current_mission_item,
        "autopilot_system_id": v.autopilot_system_id,
        "autopilot_component_id": v.autopilot_component_id,
        "pi_health": pi_health,
    }


CameraStreamWorker._telemetry_snapshot = _telemetry_snapshot_v216


def main():
    print("[RUNTIME] V2.1.6 wlan1 RSSI + selective MAVLink telemetry enabled")
    print("[MAVLINK] READ-ONLY GCS telemetry: link/mode/armed/battery/alt/speed/GPS/LiDAR")
    print("[MAVLINK] Flight movement/control TX remains DISABLED")
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
