from __future__ import annotations
import threading
import time

MESSAGE_ID_FALLBACK = {
    "DISTANCE_SENSOR": 132,
    "RANGEFINDER": 173,
}

class MavlinkStreamConfigurator(threading.Thread):
    daemon = True

    def __init__(self, tx, state, stop_event, message_rates_hz, retry_interval_s=10.0):
        super().__init__(name="MAVLinkStreamConfig")
        self.tx = tx
        self.state = state
        self.stop_event = stop_event
        self.message_rates_hz = dict(message_rates_hz or {})
        self.retry_interval_s = float(retry_interval_s)

    def _message_id(self, name):
        name = str(name).upper()
        if name in MESSAGE_ID_FALLBACK:
            return MESSAGE_ID_FALLBACK[name]
        try:
            from pymavlink import mavutil
            return getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
        except Exception:
            return None

    def request_all(self):
        print("[MAVLINK:RATE] requesting telemetry intervals")
        for name, rate in self.message_rates_hz.items():
            mid = self._message_id(name)
            if mid is None:
                print(f"[MAVLINK:RATE] unknown message {name}, skipped")
                continue
            try:
                ok = self.tx.set_message_interval(mid, float(rate))
                print(f"[MAVLINK:RATE] {name:<20} target={float(rate):5.1f} Hz  "
                      f"{'REQ' if ok else 'FAILED'}")
            except Exception as e:
                print(f"[MAVLINK:RATE] {name:<20} request failed: {e}")

    def _below_target(self):
        v = self.state.vehicle
        for name, target in self.message_rates_hz.items():
            actual = v.message_rate_hz(name)
            # Missing streams are not enough by themselves to trigger endless
            # retries; this is important for the not-yet-installed rangefinder.
            if actual <= 0:
                continue
            if actual < float(target) * 0.75:
                return True
        return False

    def run(self):
        while not self.stop_event.is_set() and not self.state.vehicle.connected:
            time.sleep(0.1)
        if self.stop_event.is_set():
            return

        self.request_all()
        last = time.monotonic()

        while not self.stop_event.wait(1.0):
            if not self.state.vehicle.connected:
                continue
            now = time.monotonic()
            if now - last >= self.retry_interval_s:
                if self._below_target():
                    print("[MAVLINK:RATE] stream below target -> re-request")
                    self.request_all()
                last = now
