#!/usr/bin/env python3
"""Kingphoenix V2.2.3 MAVLink telemetry + readiness bench runtime."""

from __future__ import annotations

import math
import time

# Preserve validated V2.1.6 RSSI/selective telemetry patches.
import main_v2_1_3  # noqa: F401

from core.runtime import KingphoenixRuntime
from streaming.vision_stream_session import VisionStreamSession, CameraStreamWorker
from system.vehicle_monitor import VehicleMonitor


_original_system_metadata_snapshot = CameraStreamWorker._system_metadata_snapshot
_original_vehicle_monitor_run = VehicleMonitor.run


def _finite_or_none(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except Exception:
        return None


def _telemetry_snapshot_v223(self):
    """Single complete read-only FC snapshot for both camera metadata streams."""
    v = self.state.vehicle
    with self.state.lock:
        pi_health = dict(self.state.system.get("pi_health") or {})

    rf_age = _finite_or_none(v.rangefinder_age_s())
    hb_age = _finite_or_none(v.heartbeat_age_s())

    return {
        "connected": bool(v.connected),
        "heartbeat_age_s": hb_age,
        "mode": v.mode,
        "armed": bool(v.armed),
        "battery_v": v.battery_v,
        "battery_current_a": v.battery_current_a,
        "battery_remaining": v.battery_remaining,
        "relative_alt_m": v.relative_alt_m,
        "groundspeed_mps": v.groundspeed_mps,
        "gps_satellites": v.gps_satellites,
        "gps_fix_type": v.gps_fix_type,
        "gps_lat_deg": v.gps_lat_deg,
        "gps_lon_deg": v.gps_lon_deg,
        "gps_alt_m": v.gps_alt_m,
        "gps_hdop": v.gps_hdop,
        "gps_vdop": v.gps_vdop,
        "gps_cog_deg": v.gps_cog_deg,
        "current_mission_item": v.current_mission_item,
        "rangefinder_m": v.rangefinder_distance_m,
        "rangefinder_age_s": rf_age,
        "rangefinder_fresh": bool(v.rangefinder_is_fresh(0.20)),
        "rangefinder_source": v.rangefinder_source,
        "rangefinder_signal_quality": v.rangefinder_signal_quality,
        "rx_rate_hz": {
            "GLOBAL_POSITION_INT": v.message_rate_hz("GLOBAL_POSITION_INT"),
            "GPS_RAW_INT": v.message_rate_hz("GPS_RAW_INT"),
            "DISTANCE_SENSOR": v.message_rate_hz("DISTANCE_SENSOR"),
            "MISSION_CURRENT": v.message_rate_hz("MISSION_CURRENT"),
            "SYS_STATUS": v.message_rate_hz("SYS_STATUS"),
        },
        "pi_health": pi_health,
    }


CameraStreamWorker._telemetry_snapshot = _telemetry_snapshot_v223


def _system_metadata_snapshot_v22(self):
    out = _original_system_metadata_snapshot(self)
    with self.state.lock:
        out["ai_health"] = dict(self.state.system.get("ai_health") or {})
        out["readiness"] = dict(self.state.system.get("readiness") or {})
        out["ready_for_mission"] = bool(self.state.system.get("ready_for_mission", False))
        out["startup_state"] = str(self.state.system.get("startup_state", "UNKNOWN"))
    return out


CameraStreamWorker._system_metadata_snapshot = _system_metadata_snapshot_v22


def _set_ai_health(session, **fields):
    with session.state.lock:
        health = dict(session.state.system.get("ai_health") or {})
        health.update(fields)
        health["timestamp_ns"] = time.monotonic_ns()
        session.state.system["ai_health"] = health


def _install_camera_open_probe(session, worker):
    original_open = worker.camera.open

    def checked_open():
        try:
            result = original_open()
            session.state.update_vision(worker.camera_name, camera_open=True, vision_error=None)
            print(f"[STARTUP] {worker.camera_name.upper()} CAMERA PASS")
            return result
        except Exception as exc:
            session.state.update_vision(worker.camera_name, camera_open=False, vision_error=repr(exc))
            print(f"[STARTUP] {worker.camera_name.upper()} CAMERA FAIL | {exc}")
            raise

    worker.camera.open = checked_open


def _vision_start_v22(self):
    startup_cfg = self.cfg.get("readiness", {}).get("hailo", {})
    max_attempts = max(1, int(startup_cfg.get("max_attempts", 3)))
    retry_delay_s = max(0.0, float(startup_cfg.get("retry_delay_s", 1.0)))

    _set_ai_health(self, status="INITIALIZING", device="CHECKING", hef="CHECKING",
                   network_group="CHECKING", inference="CHECKING", attempt=0,
                   last_error=None, detail="Hailo startup self-test")

    if self.shared_ai is None:
        _set_ai_health(self, status="FAIL", device="DISABLED", hef="DISABLED",
                       network_group="DISABLED", inference="DISABLED",
                       detail="AI disabled in configuration")
    else:
        last_error = None
        for attempt in range(1, max_attempts + 1):
            _set_ai_health(self, attempt=attempt, status="INITIALIZING")
            print(f"[STARTUP] HAILO attempt {attempt}/{max_attempts}")
            try:
                self.shared_ai.open()
                _set_ai_health(self, device="PASS", hef="PASS", network_group="PASS",
                               detail="device + HEF + network group active; testing inference")
                import numpy as np
                camera_runtime = self.cfg.get("camera", {}).get("runtime", {})
                w = int(camera_runtime.get("canonical_width", 1920))
                h = int(camera_runtime.get("canonical_height", 1200))
                dummy = np.zeros((h, w, 3), dtype=np.uint8)
                detector_class_id = self.ai_router.snapshot().detector_class_id
                self.shared_ai.process(dummy, policy="YOLO26_ONLY", detector_class_id=detector_class_id)
                _set_ai_health(self, status="PASS", device="PASS", hef="PASS", network_group="PASS",
                               inference="PASS", last_error=None,
                               detail=f"Hailo READY after attempt {attempt}; real inference PASS")
                print("[STARTUP] HAILO DEVICE PASS")
                print("[STARTUP] YOLO26 HEF PASS")
                print("[STARTUP] HAILO NETWORK GROUP PASS")
                print("[STARTUP] HAILO REAL INFERENCE PASS")
                break
            except Exception as exc:
                last_error = repr(exc)
                print(f"[STARTUP] HAILO FAIL attempt {attempt}/{max_attempts} | {exc}")
                try: self.shared_ai.close()
                except Exception: pass
                _set_ai_health(self, status="RECOVERING" if attempt < max_attempts else "FAIL",
                               device="FAIL", hef="UNKNOWN", network_group="FAIL", inference="FAIL",
                               last_error=last_error, detail=f"Hailo initialization failed on attempt {attempt}")
                if attempt < max_attempts:
                    print(f"[STARTUP] HAILO RECOVERY retry in {retry_delay_s:.1f}s")
                    time.sleep(retry_delay_s)
        else:
            _set_ai_health(self, status="FAIL", last_error=last_error,
                           detail=f"Hailo OFFLINE after {max_attempts} attempts")

    for worker in self.workers:
        _install_camera_open_probe(self, worker)
        worker.start()
    if self.test_switcher is not None:
        self.test_switcher.start()


VisionStreamSession.start = _vision_start_v22


def _vehicle_monitor_run_v223(self):
    """Keep startup output concise, then hand over to normal measured-rate monitor."""
    last_status = None
    while not self.stop_event.is_set():
        ready = bool(self.state.system.get("ready_for_mission", False))
        v = self.state.vehicle
        if ready:
            print("[STARTUP] READINESS PASS -> enabling normal VehicleMonitor output")
            print("[MAVLINK RX] GPS_RAW_INT={:.1f}Hz GLOBAL_POSITION_INT={:.1f}Hz DISTANCE_SENSOR={:.1f}Hz MISSION_CURRENT={:.1f}Hz".format(
                v.message_rate_hz("GPS_RAW_INT"), v.message_rate_hz("GLOBAL_POSITION_INT"),
                v.message_rate_hz("DISTANCE_SENSOR"), v.message_rate_hz("MISSION_CURRENT")))
            return _original_vehicle_monitor_run(self)
        readiness = self.state.system.get("readiness") or {}
        failed = readiness.get("failed_critical") or []
        signature = tuple(failed)
        if signature != last_status:
            last_status = signature
            print("[STARTUP] FLIGHT RUNTIME BLOCKED | waiting=" + (",".join(failed) if failed else "startup checks"))
            print("[STARTUP] Diagnostic services ACTIVE; mission/control remain LOCKED")
        self.stop_event.wait(1.0)


VehicleMonitor.run = _vehicle_monitor_run_v223


def main():
    print("============================================================")
    print(" KINGPHOENIX V2.2.3 - MAVLINK TELEMETRY / GCS FIX")
    print("============================================================")
    print("[V2.2.3] GPS position/fix/sats + current waypoint + battery current published")
    print("[V2.2.3] LiDAR uses DISTANCE_SENSOR primary with freshness/age diagnostics")
    print("[V2.2.3] Legacy RANGEFINDER remains parser fallback but is not requested")
    print("[V2.2.3] Flight movement/control TX remains DISABLED")

    runtime = KingphoenixRuntime()
    try:
        runtime.start(); runtime.wait()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
