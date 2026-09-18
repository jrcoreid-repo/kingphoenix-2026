from __future__ import annotations

import shutil
import subprocess
import threading
import time
from copy import deepcopy

from core.scheduler import Rate


THROTTLE_BITS = {
    "undervoltage_now": 0,
    "frequency_capped_now": 1,
    "throttling_now": 2,
    "soft_temp_limit_now": 3,
    "undervoltage_occurred": 16,
    "frequency_capped_occurred": 17,
    "throttling_occurred": 18,
    "soft_temp_limit_occurred": 19,
}


def _status_rank(status):
    return {"PASS": 0, "WARN": 1, "FAIL": 2}.get(str(status).upper(), 2)


def _worst_status(*statuses):
    if not statuses:
        return "FAIL"
    return max((str(s).upper() for s in statuses), key=_status_rank)


class PiHealthMonitor:
    """Low-overhead Raspberry Pi health sampler.

    Uses Linux /proc + /sys for CPU/RAM/disk/uptime and vcgencmd for
    Raspberry Pi firmware power/throttling state when available.
    """

    def __init__(self, cfg=None):
        cfg = dict(cfg or {})
        self._last_cpu_total = None
        self._last_cpu_idle = None

        self.temp_warn_c = float(cfg.get("temp_warn_c", 75.0))
        self.temp_fail_c = float(cfg.get("temp_fail_c", 80.0))
        self.cpu_warn_pct = float(cfg.get("cpu_warn_pct", 85.0))
        self.cpu_fail_pct = float(cfg.get("cpu_fail_pct", 95.0))
        self.ram_warn_pct = float(cfg.get("ram_warn_pct", 85.0))
        self.ram_fail_pct = float(cfg.get("ram_fail_pct", 95.0))
        self.disk_warn_free_gb = float(cfg.get("disk_warn_free_gb", 2.0))
        self.disk_fail_free_gb = float(cfg.get("disk_fail_free_gb", 0.5))
        self.disk_path = str(cfg.get("disk_path", "/"))
        self.vcgencmd_timeout_s = float(cfg.get("vcgencmd_timeout_s", 0.35))
        self.wifi_interface = str(cfg.get("wifi_interface", "wlan1"))

    def _run_cmd(self, args):
        try:
            result = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=self.vcgencmd_timeout_s,
                check=False,
            )
            if result.returncode != 0:
                return None
            return result.stdout.strip()
        except Exception:
            return None

    def _read_temp(self):
        out = self._run_cmd(["vcgencmd", "measure_temp"])
        if out and "=" in out:
            try:
                return float(out.split("=", 1)[1].split("'", 1)[0])
            except Exception:
                pass

        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return None

    def _read_throttled(self):
        out = self._run_cmd(["vcgencmd", "get_throttled"])
        if not out or "=" not in out:
            return None, None
        try:
            value_hex = out.split("=", 1)[1].strip()
            return int(value_hex, 16), value_hex
        except Exception:
            return None, out

    @staticmethod
    def _decode_throttled(value):
        result = {name: False for name in THROTTLE_BITS}
        if value is None:
            return result
        for name, bit in THROTTLE_BITS.items():
            result[name] = bool(value & (1 << bit))
        return result

    def _read_cpu_pct(self):
        try:
            with open("/proc/stat", "r") as f:
                parts = f.readline().split()
            values = [int(v) for v in parts[1:]]
            if len(values) < 4:
                return None

            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)

            if self._last_cpu_total is None:
                self._last_cpu_total = total
                self._last_cpu_idle = idle
                return None

            total_delta = total - self._last_cpu_total
            idle_delta = idle - self._last_cpu_idle
            self._last_cpu_total = total
            self._last_cpu_idle = idle

            if total_delta <= 0:
                return None

            busy = 100.0 * (1.0 - idle_delta / total_delta)
            return max(0.0, min(100.0, busy))
        except Exception:
            return None

    @staticmethod
    def _read_ram():
        try:
            info = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    key, value = line.split(":", 1)
                    info[key] = float(value.strip().split()[0])  # kB

            total = info.get("MemTotal")
            available = info.get("MemAvailable")
            if not total or available is None:
                return None, None, None

            used = total - available
            return (
                100.0 * used / total,
                used / (1024.0 * 1024.0),
                total / (1024.0 * 1024.0),
            )
        except Exception:
            return None, None, None

    def _read_disk(self):
        try:
            usage = shutil.disk_usage(self.disk_path)
            return (
                100.0 * usage.used / max(1, usage.total),
                usage.free / (1024.0 ** 3),
                usage.total / (1024.0 ** 3),
            )
        except Exception:
            return None, None, None

    @staticmethod
    def _read_uptime_s():
        try:
            with open("/proc/uptime", "r") as f:
                return float(f.read().split()[0])
        except Exception:
            return None

    def _read_wifi(self):
        """Read link state/RSSI without spawning network utilities."""
        iface = self.wifi_interface
        connected = False
        rssi_dbm = None

        try:
            with open(f"/sys/class/net/{iface}/carrier", "r") as f:
                connected = f.read().strip() == "1"
        except Exception:
            try:
                with open(f"/sys/class/net/{iface}/operstate", "r") as f:
                    connected = f.read().strip().lower() == "up"
            except Exception:
                connected = False

        try:
            with open("/proc/net/wireless", "r") as f:
                for line in f:
                    if ":" not in line:
                        continue
                    name, rest = line.split(":", 1)
                    if name.strip() != iface:
                        continue
                    fields = rest.split()
                    if len(fields) >= 3:
                        rssi_dbm = float(fields[2].rstrip("."))
                    break
        except Exception:
            rssi_dbm = None

        return connected, rssi_dbm

    @staticmethod
    def _threshold_high(value, warn, fail):
        if value is None:
            return "WARN"
        if value >= fail:
            return "FAIL"
        if value >= warn:
            return "WARN"
        return "PASS"

    def sample(self):
        temp_c = self._read_temp()
        throttled_raw, throttled_hex = self._read_throttled()
        flags = self._decode_throttled(throttled_raw)

        cpu_pct = self._read_cpu_pct()
        ram_pct, ram_used_gb, ram_total_gb = self._read_ram()
        disk_used_pct, disk_free_gb, disk_total_gb = self._read_disk()
        uptime_s = self._read_uptime_s()
        wifi_connected, wifi_rssi_dbm = self._read_wifi()

        thermal_status = self._threshold_high(
            temp_c, self.temp_warn_c, self.temp_fail_c
        )
        cpu_status = self._threshold_high(
            cpu_pct, self.cpu_warn_pct, self.cpu_fail_pct
        )
        ram_status = self._threshold_high(
            ram_pct, self.ram_warn_pct, self.ram_fail_pct
        )

        if disk_free_gb is None:
            disk_status = "WARN"
        elif disk_free_gb <= self.disk_fail_free_gb:
            disk_status = "FAIL"
        elif disk_free_gb <= self.disk_warn_free_gb:
            disk_status = "WARN"
        else:
            disk_status = "PASS"

        if throttled_raw is None:
            power_status = "WARN"
            throttle_status = "WARN"
        else:
            power_status = (
                "FAIL" if flags["undervoltage_now"]
                else "WARN" if flags["undervoltage_occurred"]
                else "PASS"
            )

            current_throttle = (
                flags["frequency_capped_now"]
                or flags["throttling_now"]
                or flags["soft_temp_limit_now"]
            )
            historical_throttle = (
                flags["frequency_capped_occurred"]
                or flags["throttling_occurred"]
                or flags["soft_temp_limit_occurred"]
            )
            throttle_status = (
                "FAIL" if current_throttle
                else "WARN" if historical_throttle
                else "PASS"
            )

        overall = _worst_status(
            thermal_status,
            power_status,
            throttle_status,
            cpu_status,
            ram_status,
            disk_status,
        )

        checks = {
            "pi_thermal": {
                "status": thermal_status,
                "detail": "N/A" if temp_c is None else f"{temp_c:.1f} C",
            },
            "pi_power": {
                "status": power_status,
                "detail": (
                    "vcgencmd unavailable"
                    if throttled_raw is None
                    else "UNDERVOLTAGE NOW"
                    if flags["undervoltage_now"]
                    else "Historical undervoltage"
                    if flags["undervoltage_occurred"]
                    else "Power OK"
                ),
            },
            "pi_throttling": {
                "status": throttle_status,
                "detail": (
                    "vcgencmd unavailable"
                    if throttled_raw is None
                    else f"{throttled_hex} / CURRENT LIMIT"
                    if throttle_status == "FAIL"
                    else f"{throttled_hex} / HISTORICAL"
                    if throttle_status == "WARN"
                    else f"{throttled_hex} / NORMAL"
                ),
            },
            "pi_cpu": {
                "status": cpu_status,
                "detail": "warming up" if cpu_pct is None else f"{cpu_pct:.1f}%",
            },
            "pi_ram": {
                "status": ram_status,
                "detail": "N/A" if ram_pct is None else f"{ram_pct:.1f}%",
            },
            "pi_disk": {
                "status": disk_status,
                "detail": "N/A" if disk_free_gb is None else f"{disk_free_gb:.2f} GB free",
            },
        }

        return {
            "timestamp_ns": time.monotonic_ns(),
            "status": overall,
            "cpu_temp_c": temp_c,
            "cpu_load_pct": cpu_pct,
            "ram_used_pct": ram_pct,
            "ram_used_gb": ram_used_gb,
            "ram_total_gb": ram_total_gb,
            "disk_used_pct": disk_used_pct,
            "disk_free_gb": disk_free_gb,
            "disk_total_gb": disk_total_gb,
            "uptime_s": uptime_s,
            "wifi_interface": self.wifi_interface,
            "wifi_connected": bool(wifi_connected),
            "wifi_rssi_dbm": wifi_rssi_dbm,
            "throttled_raw": throttled_raw,
            "throttled_hex": throttled_hex,
            **flags,
            "checks": checks,
        }


class HealthWorker(threading.Thread):
    daemon = True

    def __init__(self, state, stop_event, cfg=None, hz=1.0):
        super().__init__(name="PiHealth")
        self.state = state
        self.stop_event = stop_event
        self.monitor = PiHealthMonitor(cfg)
        self.hz = max(0.2, float(hz))

    def run(self):
        rate = Rate(self.hz)

        while not self.stop_event.is_set():
            health = self.monitor.sample()
            with self.state.lock:
                self.state.system["pi_health"] = deepcopy(health)
                self.state.system["cpu_temp_c"] = health.get("cpu_temp_c")
                self.state.system["pi_health_status"] = health.get("status")
            rate.sleep()
