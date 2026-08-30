from __future__ import annotations

import threading
import time


class ReadinessSupervisor(threading.Thread):
    """V2.2 single source of truth for mission readiness.

    Runtime services remain alive for diagnostics when a critical component is
    unavailable, but mission start must consume system['ready_for_mission'].
    """

    daemon = True

    def __init__(self, state, stop_event, cfg=None, hz=2.0):
        super().__init__(name="ReadinessSupervisor")
        self.state = state
        self.stop_event = stop_event
        self.cfg = cfg or {}
        self.hz = max(0.5, float(hz))
        self._last_signature = None

    @staticmethod
    def _check(status, detail, critical=True):
        return {
            "status": str(status).upper(),
            "detail": str(detail),
            "critical": bool(critical),
        }

    def _evaluate(self):
        v = self.state.vehicle
        with self.state.lock:
            system = dict(self.state.system)
            ai = dict(system.get("ai_health") or {})
            pi = dict(system.get("pi_health") or {})
            front = dict(self.state.vision.get("front") or {})
            bottom = dict(self.state.vision.get("bottom") or {})

        checks = {}

        # Pi health: current faults are critical; historical flags stay WARN.
        temp = pi.get("cpu_temp_c")
        if temp is None:
            checks["pi_thermal"] = self._check("WARN", "temperature unavailable", False)
        elif bool(pi.get("soft_temp_limit_now", False)) or float(temp) >= float(self.cfg.get("temp_fail_c", 80.0)):
            checks["pi_thermal"] = self._check("FAIL", f"{float(temp):.1f} C")
        elif float(temp) >= float(self.cfg.get("temp_warn_c", 75.0)):
            checks["pi_thermal"] = self._check("WARN", f"{float(temp):.1f} C", False)
        else:
            checks["pi_thermal"] = self._check("PASS", f"{float(temp):.1f} C")

        if bool(pi.get("undervoltage_now", False)) or bool(pi.get("throttling_now", False)):
            checks["pi_power"] = self._check("FAIL", pi.get("throttled_hex", "current power/throttle fault"))
        elif bool(pi.get("undervoltage_occurred", False)) or bool(pi.get("throttling_occurred", False)):
            checks["pi_power"] = self._check("WARN", f"historical {pi.get('throttled_hex', '--')}", False)
        elif pi.get("throttled_raw") is None:
            checks["pi_power"] = self._check("WARN", "power flags unavailable", False)
        else:
            checks["pi_power"] = self._check("PASS", str(pi.get("throttled_hex", "0x0")))

        hailo_status = str(ai.get("status", "UNKNOWN")).upper()
        checks["hailo_ai"] = self._check(
            "PASS" if hailo_status == "PASS" else "FAIL",
            ai.get("detail") or ai.get("last_error") or hailo_status,
        )

        for name, cam in (("front_camera", front), ("bottom_camera", bottom)):
            opened = cam.get("camera_open") is True
            checks[name] = self._check(
                "PASS" if opened else "FAIL",
                "camera opened" if opened else (cam.get("vision_error") or "camera not opened"),
            )

        hb_age = v.heartbeat_age_s()
        mav_ok = bool(v.connected) and hb_age <= float(self.cfg.get("heartbeat_max_age_s", 2.5))
        checks["mavlink_fc"] = self._check(
            "PASS" if mav_ok else "FAIL",
            f"heartbeat {hb_age:.2f}s" if hb_age != float("inf") else "FC heartbeat missing",
        )

        gps_required = bool(self.cfg.get("require_gps", True))
        gps_ok = v.gps_fix_type is not None and int(v.gps_fix_type) >= int(self.cfg.get("gps_min_fix_type", 3))
        checks["gps"] = self._check(
            "PASS" if gps_ok else ("FAIL" if gps_required else "WARN"),
            f"fix={v.gps_fix_type} sats={v.gps_satellites}",
            gps_required,
        )

        lidar_required = bool(self.cfg.get("require_lidar", True))
        lidar_ok = v.rangefinder_is_fresh(float(self.cfg.get("lidar_max_age_s", 0.30)))
        checks["lidar"] = self._check(
            "PASS" if lidar_ok else ("FAIL" if lidar_required else "WARN"),
            f"{v.rangefinder_distance_m} m / {v.rangefinder_source or '--'}",
            lidar_required,
        )

        wifi_required = bool(self.cfg.get("require_wlan1", True))
        wifi_ok = bool(pi.get("wifi_connected", False))
        checks["wlan1"] = self._check(
            "PASS" if wifi_ok else ("FAIL" if wifi_required else "WARN"),
            f"{pi.get('wifi_rssi_dbm')} dBm" if wifi_ok else "wlan1 offline",
            wifi_required,
        )

        failed = [k for k, x in checks.items() if x["critical"] and x["status"] == "FAIL"]
        ready = len(failed) == 0
        status = "READY" if ready else "NOT_READY"
        return ready, status, checks, failed

    def run(self):
        interval = 1.0 / self.hz
        while not self.stop_event.is_set():
            ready, status, checks, failed = self._evaluate()
            payload = {
                "timestamp_ns": time.monotonic_ns(),
                "status": status,
                "ready_for_mission": ready,
                "failed_critical": failed,
                "checks": checks,
            }
            with self.state.lock:
                self.state.system["readiness"] = payload
                self.state.system["ready_for_mission"] = ready
                self.state.system["startup_state"] = "READY" if ready else "WAITING_FOR_REQUIRED_HARDWARE"

            signature = (status, tuple(failed))
            if signature != self._last_signature:
                self._last_signature = signature
                if ready:
                    print("[READINESS] ALL CRITICAL CHECKS PASS -> KINGPHOENIX READY")
                else:
                    print(f"[READINESS] NOT READY | failed={','.join(failed) if failed else '--'}")
                    print("[READINESS] Runtime remains ACTIVE for diagnostics/recovery; mission start is LOCKED")
            time.sleep(interval)
