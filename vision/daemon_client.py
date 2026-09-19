from __future__ import annotations

from collections import deque
import json
import socket
import time


class VisionDaemonClient:
    """Race-side client for the standalone drop vision daemon.

    Data channel is receive-only UDP. Control commands are sent to a separate
    localhost UDP port. No MAVLink or flight authority lives here.
    """

    def __init__(
        self,
        *,
        data_bind_ip="127.0.0.1",
        data_port=14600,
        control_host="127.0.0.1",
        control_port=14601,
        stale_s=0.30,
        rate_window_s=1.5,
    ):
        self.data_addr = (str(data_bind_ip), int(data_port))
        self.control_addr = (str(control_host), int(control_port))
        self.stale_s = float(stale_s)
        self.rate_window_s = max(0.5, float(rate_window_s))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.data_addr)
        self.sock.setblocking(False)

        self.ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.latest = None
        self.last_rx_s = None
        self.rx_count = 0
        self.bad_count = 0
        self.rx_times = deque(maxlen=256)

    def close(self):
        for sock in (self.sock, self.ctrl):
            try:
                sock.close()
            except Exception:
                pass

    def poll(self):
        while True:
            try:
                payload, _peer = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                break

            now = time.monotonic()
            try:
                packet = json.loads(payload.decode("utf-8"))
                if packet.get("schema") != "kingphoenix.vision.drop.v1":
                    raise ValueError("unexpected schema")
            except Exception:
                self.bad_count += 1
                continue

            self.latest = packet
            self.last_rx_s = now
            self.rx_count += 1
            self.rx_times.append(now)

        self._prune_rate_window()

    def _prune_rate_window(self):
        cutoff = time.monotonic() - self.rate_window_s
        while self.rx_times and self.rx_times[0] < cutoff:
            self.rx_times.popleft()

    def rx_age_s(self):
        if self.last_rx_s is None:
            return float("inf")
        return max(0.0, time.monotonic() - self.last_rx_s)

    def rx_rate_hz(self):
        self._prune_rate_window()
        if len(self.rx_times) < 2:
            return 0.0
        span = self.rx_times[-1] - self.rx_times[0]
        if span <= 0:
            return 0.0
        return (len(self.rx_times) - 1) / span

    def snapshot(self):
        self.poll()
        packet = dict(self.latest or {})
        age_s = self.rx_age_s()
        packet["link_ok"] = bool(self.latest is not None and age_s <= self.stale_s)
        packet["rx_age_ms"] = None if age_s == float("inf") else age_s * 1000.0
        packet["rx_rate_hz"] = self.rx_rate_hz()
        packet["rx_count"] = int(self.rx_count)
        packet["bad_count"] = int(self.bad_count)
        return packet

    def set_mode(self, mode, timeout_s=0.20):
        mode = str(mode).upper().strip()
        command = f"SET_MODE {mode}".encode("utf-8")
        try:
            self.ctrl.settimeout(float(timeout_s))
            self.ctrl.sendto(command, self.control_addr)
            data, _peer = self.ctrl.recvfrom(4096)
            reply = json.loads(data.decode("utf-8"))
            return bool(reply.get("ok")), reply
        except Exception as exc:
            return False, {
                "ok": False,
                "mode": mode,
                "detail": f"{type(exc).__name__}: {exc}",
            }
        finally:
            self.ctrl.settimeout(None)
