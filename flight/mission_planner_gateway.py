from __future__ import annotations

import queue
import socket
import threading
import time


class MissionPlannerGateway(threading.Thread):
    """Filtered MAVLink UDP gateway for Mission Planner.

    FC -> Mission Planner:
      Mirrored telemetry from the sole FC RX path. The RX worker places raw
      MAVLink frames into ``mirror_queue`` so this worker never calls
      recv_match() on the flight-controller serial connection.

    Mission Planner -> FC:
      RACE mode: GPS_RTCM_DATA only.
      MAINTENANCE mode: permissive pass-through (still serialized through the
      shared MavlinkTx lock).

    The Mission Planner destination IP can be configured explicitly or learned
    from the Kingphoenix GCS endpoint when both applications run on the same
    laptop.
    """

    daemon = True

    def __init__(self, state, tx, mirror_queue, stop_event, cfg=None):
        super().__init__(name="MissionPlannerGateway")
        self.state = state
        self.tx = tx
        self.mirror_queue = mirror_queue
        self.stop_event = stop_event
        self.cfg = cfg or {}

        self.enabled = bool(self.cfg.get("enabled", True))
        self.bind_ip = str(self.cfg.get("bind_ip", "0.0.0.0"))
        self.port = int(self.cfg.get("port", 14550))
        self.destination_port = int(self.cfg.get("destination_port", self.port))
        self.destination_ip = str(self.cfg.get("destination_ip", "AUTO_GCS"))
        self.policy = str(self.cfg.get("policy", "RACE")).upper()
        self.learn_peer = bool(self.cfg.get("learn_peer", True))
        self.peer = None
        self.sock = None
        self._last_status = None

    def _current_destination(self):
        if self.peer is not None:
            return self.peer

        if self.destination_ip and self.destination_ip.upper() != "AUTO_GCS":
            return (self.destination_ip, self.destination_port)

        with self.state.lock:
            endpoint = self.state.system.get("gcs_endpoint")

        if endpoint:
            try:
                ip = endpoint[0]
                return (str(ip), self.destination_port)
            except Exception:
                pass
        return None

    def _publish_status(self, status, detail=""):
        signature = (status, detail, self.peer, self.policy)
        if signature == self._last_status:
            return
        self._last_status = signature
        with self.state.lock:
            self.state.system["mission_planner_gateway"] = {
                "status": status,
                "detail": detail,
                "policy": self.policy,
                "bind_port": self.port,
                "peer": None if self.peer is None else f"{self.peer[0]}:{self.peer[1]}",
            }
        print(f"[MP-GATEWAY] {status} | policy={self.policy} | {detail}")

    def _parse_datagram(self, data):
        try:
            from pymavlink import mavutil
            parser = mavutil.mavlink.MAVLink(None)
            msgs = parser.parse_buffer(data)
            return list(msgs or [])
        except Exception:
            return []

    def _allow_from_mp(self, msg_type):
        msg_type = str(msg_type).upper()
        if self.policy == "MAINTENANCE":
            return msg_type not in {"BAD_DATA"}
        return msg_type == "GPS_RTCM_DATA"

    def _forward_from_mp(self, data, addr):
        if self.learn_peer:
            learned = (str(addr[0]), self.destination_port)
            if self.peer != learned:
                self.peer = learned
                self._publish_status("PEER LEARNED", f"{learned[0]}:{learned[1]}")

        for msg in self._parse_datagram(data):
            msg_type = msg.get_type()
            if not self._allow_from_mp(msg_type):
                continue
            ok = self.tx.send_message(msg)
            if msg_type == "GPS_RTCM_DATA" and ok:
                with self.state.lock:
                    self.state.system["mission_planner_rtcm_last_ns"] = time.monotonic_ns()

    def _send_mirror_packet(self, payload):
        destination = self._current_destination()
        if destination is None or self.sock is None:
            return
        try:
            self.sock.sendto(payload, destination)
        except OSError:
            pass

    def run(self):
        if not self.enabled:
            self._publish_status("DISABLED")
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.bind_ip, self.port))
            self.sock.settimeout(0.02)
        except OSError as exc:
            self._publish_status("FAILED", str(exc))
            return

        self._publish_status(
            "ACTIVE",
            f"UDP {self.bind_ip}:{self.port}; RACE inbound=RTCM only"
            if self.policy == "RACE"
            else f"UDP {self.bind_ip}:{self.port}; maintenance pass-through",
        )

        try:
            while not self.stop_event.is_set():
                # Drain a bounded batch so incoming RTCM is never starved by a
                # high-rate FC telemetry stream.
                for _ in range(64):
                    try:
                        payload = self.mirror_queue.get_nowait()
                    except queue.Empty:
                        break
                    self._send_mirror_packet(payload)

                try:
                    data, addr = self.sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break

                if data:
                    self._forward_from_mp(data, addr)
        finally:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self._publish_status("STOPPED")
