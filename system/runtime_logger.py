from __future__ import annotations

import threading
import time


class RuntimeLogger(threading.Thread):
    """Low-noise periodic operator summary for race runtime."""

    daemon = True

    def __init__(self, state, stop_event, cfg: dict):
        super().__init__(name="RuntimeLogger")
        self.state = state
        self.stop_event = stop_event
        self.cfg = dict(cfg or {})
        self.realtime_s = float(self.cfg.get("realtime_summary_interval_s", 2.0))
        self.system_s = float(self.cfg.get("system_summary_interval_s", 5.0))

    @staticmethod
    def _rate(v, name):
        try:
            return int(round(v.message_rate_hz(name)))
        except Exception:
            return 0

    def run(self):
        next_rt = time.monotonic() + self.realtime_s
        next_sys = time.monotonic() + self.system_s
        while not self.stop_event.is_set():
            now = time.monotonic()
            v = self.state.vehicle

            if now >= next_rt:
                link = "OK" if v.connected and v.heartbeat_age_s() <= 2.5 else "BAD"
                print(
                    f"[MAV] HB {self._rate(v,'HEARTBEAT')}Hz | POS {self._rate(v,'GLOBAL_POSITION_INT')} | "
                    f"ATT {self._rate(v,'ATTITUDE')} | RNG {self._rate(v,'DISTANCE_SENSOR')} | "
                    f"GPS {self._rate(v,'GPS_RAW_INT')} | {link}"
                )
                with self.state.lock:
                    bottom = dict(self.state.vision.get("bottom") or {})
                if bottom.get("ai_state") == "ACTIVE":
                    conf = bottom.get("confidence")
                    conf_t = "--" if conf is None else f"{float(conf):.2f}"
                    print(
                        f"[AI] Bottom {float(bottom.get('fps') or 0.0):.1f}fps | "
                        f"conf={conf_t} | target={'YES' if bottom.get('vision_valid') else 'NO'}"
                    )
                next_rt = now + self.realtime_s

            if now >= next_sys:
                with self.state.lock:
                    health = dict(self.state.system.get("pi_health") or {})
                temp = health.get("cpu_temp_c")
                load = health.get("cpu_load_pct")
                ram = health.get("ram_used_pct")
                t = "--" if temp is None else f"{float(temp):.0f}C"
                l = "--" if load is None else f"{float(load):.0f}%"
                r = "--" if ram is None else f"{float(ram):.0f}%"
                fix = "--" if v.gps_fix_type is None else str(v.gps_fix_type)
                sats = "--" if v.gps_satellites is None else str(v.gps_satellites)
                hdop = "--" if v.gps_hdop is None else f"{float(v.gps_hdop):.2f}"
                print(f"[SYSTEM] CPU {t} | Load {l} | RAM {r} | GPS fix={fix} sats={sats} HDOP={hdop}")
                next_sys = now + self.system_s

            time.sleep(0.10)
