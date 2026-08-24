#!/usr/bin/env python3
"""Kingphoenix V2.1.3 runtime wrapper.

Adds low-overhead wlan1 RSSI telemetry to the existing Pi Health payload without
changing the frozen video pipeline or MAVLink behavior.
"""

from __future__ import annotations

from pathlib import Path

from core.runtime import KingphoenixRuntime
from system.health import PiHealthMonitor


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
            # /proc/net/wireless columns after status: quality, level, noise.
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


def main():
    print("[RUNTIME] V2.1.3 wlan1 RSSI telemetry enabled")
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
