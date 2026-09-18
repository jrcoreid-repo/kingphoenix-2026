from __future__ import annotations

import threading
import time


class ServoController:
    """Dual-output payload servo controller.

    Kingphoenix payload hardware uses MAVLink DO_SET_SERVO on both configured
    outputs. Software state starts OPEN and the runtime actively commands OPEN
    once at startup. CLOSE is operator-only from GCS; mission drop only OPENs.
    """

    def __init__(self, tx, state, cfg: dict):
        self.tx = tx
        self.state = state
        self.cfg = dict(cfg or {})
        self.channels = tuple(int(x) for x in self.cfg.get("channels", (7, 8)))
        self.open_pwm = int(self.cfg.get("open_pwm", 1100))
        self.close_pwm = int(self.cfg.get("close_pwm", 2000))
        self.test_close_s = float(self.cfg.get("test_close_s", 0.5))
        self._lock = threading.RLock()
        self.logical_state = "OPEN"

    def _apply(self, logical_state: str, pwm: int, *, source: str) -> tuple[bool, str]:
        logical_state = str(logical_state).upper()
        with self._lock:
            failed = []
            for channel in self.channels:
                if not self.tx.set_servo(channel, int(pwm)):
                    failed.append(channel)
            ok = not failed
            if ok:
                self.logical_state = logical_state
            with self.state.lock:
                self.state.system["servo"] = {
                    "state": self.logical_state,
                    "requested_state": logical_state,
                    "channels": list(self.channels),
                    "pwm": int(pwm),
                    "source": str(source),
                    "ok": bool(ok),
                    "timestamp_ns": time.monotonic_ns(),
                }
            if ok:
                return True, f"SERVO {logical_state} CH={','.join(map(str, self.channels))} PWM={int(pwm)}"
            return False, f"SERVO {logical_state} TX FAILED channels={failed}"

    def open(self, *, source: str = "MISSION") -> tuple[bool, str]:
        return self._apply("OPEN", self.open_pwm, source=source)

    def close(self, *, source: str = "GCS") -> tuple[bool, str]:
        return self._apply("CLOSE", self.close_pwm, source=source)

    def test(self, *, source: str = "GCS") -> tuple[bool, str]:
        ok, message = self.close(source=f"{source}_TEST")
        if not ok:
            return ok, message
        time.sleep(max(0.0, self.test_close_s))
        ok, message = self.open(source=f"{source}_TEST")
        if not ok:
            return ok, message
        return True, "SERVO TEST COMPLETE -> OPEN"

    def initialize(self) -> tuple[bool, str]:
        return self.open(source="STARTUP_DEFAULT")
