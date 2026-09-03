from __future__ import annotations

import queue
import socket
import threading
import time


class MissionPlannerGateway(threading.Thread):
    """Filtered MAVLink UDP gateway for Mission Planner.

    Network contract:
      - ``bind_ip`` is only the local receive/listen address on the Raspberry Pi.
        ``0.0.0.0`` is valid here because it means "listen on every Pi interface".
      - outbound telemetry is always sent to a *real destination IP*.
        With ``destination_ip=AUTO_GCS`` the destination is learned from the
        Kingphoenix GCS HELLO peer (normally the same laptop running Mission
        Planner), then telemetry is sent to ``<GCS laptop IP>:destination_port``.

    FC -> Mission Planner:
      Mirrored telemetry from the sole FC RX path. The RX worker places raw
      MAVLink frames into ``mirror_queue`` so this worker never calls
      recv_match() on the flight-controller serial connection.

    Mission Planner -> FC:
      RACE mode: GPS_RTCM_DATA only.
      MAINTENANCE mode: permissive pass-through (still serialized through the
      shared MavlinkTx lock).

    Destination precedence:
      1. Explicit configured destination IP (when not AUTO_GCS)
      2. Kingphoenix GCS HELLO peer IP
      3. Learned Mission Planner peer as fallback

    This avoids the old UDP chicken-and-egg problem where the gateway waited
    for an inbound Mission Planner packet before it had any outbound target.
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
        self._last_destination = None
        self._last_destination_source = None
        self._tx_packets = 0
        self._tx_bytes = 0
        self._rtcm_packets = 0
        self._last_diag_s = 0.0

    def _gcs_destination(self):
        with self.state.lock:
            endpoint = self.state.system.get("gcs_endpoint")

        if not endpoint:
            return None

        try:
            ip = endpoint.get("ip") if isinstance(endpoint, dict) else endpoint[0]
            if ip:
                return (str(ip), self.destination_port)
        except Exception:
            pass
        return None

    def _current_destination(self):
        # Explicit destination is deterministic and always wins.
        if self.destination_ip and self.destination_ip.upper() != "AUTO_GCS":
            return (self.destination_ip, self.destination_port), "CONFIG"

        # AUTO_GCS intentionally prefers the already-proven GCS HELLO peer.
        # This makes the behavior equivalent to MAVProxy --out=udp:<LAPTOP_IP>:14555.
        gcs_destination = self._gcs_destination()
        if gcs_destination is not None:
            return gcs_destination, "GCS_HELLO"

        # Peer learning is fallback only. It must never override a valid GCS
        # destination because arbitrary UDP traffic should not hijack telemetry.
        if self.peer is not None:
            return self.peer, "MP_PEER"

        return None, "WAITING"

    def _publish_status(self, status, detail=""):
        destination, destination_source = self._current_destination()
        signature = (
            status,
            detail,
            self.peer,
            self.policy,
            destination,
            destination_source,
        )
        if signature == self._last_status:
            return
        self._last_status = signature

        with self.state.lock:
            self.state.system["mission_planner_gateway"] = {
                "status": status,
                "detail": detail,
                "policy": self.policy,
                "bind": f"{self.bind_ip}:{self.port}",
                "destination": None if destination is None else f"{destination[0]}:{destination[1]}",
                "destination_source": destination_source,
                "peer": None if self.peer is None else f"{self.peer[0]}:{self.peer[1]}",
                "tx_packets": self._tx_packets,
                "tx_bytes": self._tx_bytes,
                "rtcm_packets": self._rtcm_packets,
            }

        destination_text = "--" if destination is None else f"{destination[0]}:{destination[1]}"
        print(
            f"[MP-GATEWAY] {status} | policy={self.policy} | "
            f"bind={self.bind_ip}:{self.port} | dest={destination_text} "
            f"source={destination_source} | {detail}"
        )

    def _report_destination_if_changed(self):
        destination, source = self._current_destination()
        if destination == self._last_destination and source == self._last_destination_source:
            return

        self._last_destination = destination
        self._last_destination_source = source

        if destination is None:
            self._publish_status(
                "WAITING_DESTINATION",
                "connect Kingphoenix GCS or configure an explicit laptop IP",
            )
        else:
            self._publish_status(
                "ACTIVE",
                f"FC telemetry -> Mission Planner {destination[0]}:{destination[1]}",
            )

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
        messages = self._parse_datagram(data)
        allowed = [msg for msg in messages if self._allow_from_mp(msg.get_type())]

        # Learn only from a datagram that contains traffic allowed by the
        # current policy. In RACE mode this means a real RTCM packet.
        if self.learn_peer and allowed:
            learned = (str(addr[0]), self.destination_port)
            if self.peer != learned:
                self.peer = learned
                print(f"[MP-GATEWAY] MP PEER LEARNED {learned[0]}:{learned[1]}")
                self._report_destination_if_changed()

        for msg in allowed:
            msg_type = msg.get_type()
            ok = self.tx.send_message(msg)
            if msg_type == "GPS_RTCM_DATA" and ok:
                self._rtcm_packets += 1
                with self.state.lock:
                    self.state.system["mission_planner_rtcm_last_ns"] = time.monotonic_ns()

    def _send_mirror_packet(self, payload):
        destination, _ = self._current_destination()
        if destination is None or self.sock is None:
            return False

        try:
            self.sock.sendto(payload, destination)
            self._tx_packets += 1
            self._tx_bytes += len(payload)
            return True
        except OSError:
            return False

    def _publish_counters(self):
        now = time.monotonic()
        if now - self._last_diag_s < 5.0:
            return
        self._last_diag_s = now

        destination, source = self._current_destination()
        if destination is None:
            return

        with self.state.lock:
            status = dict(self.state.system.get("mission_planner_gateway") or {})
            status.update({
                "destination": f"{destination[0]}:{destination[1]}",
                "destination_source": source,
                "tx_packets": self._tx_packets,
                "tx_bytes": self._tx_bytes,
                "rtcm_packets": self._rtcm_packets,
            })
            self.state.system["mission_planner_gateway"] = status

        print(
            f"[MP-GATEWAY] FLOW | dest={destination[0]}:{destination[1]} "
            f"source={source} tx_packets={self._tx_packets} rtcm_rx={self._rtcm_packets}"
        )

    def run(self):
        if not self.enabled:
            self._publish_status("DISABLED", "gateway disabled in config")
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.bind_ip, self.port))
            self.sock.settimeout(0.02)
        except OSError as exc:
            self._publish_status("FAILED", str(exc))
            return

        print(
            f"[MP-GATEWAY] UDP LISTEN {self.bind_ip}:{self.port} | "
            f"destination={self.destination_ip}:{self.destination_port}"
        )
        if self.policy == "RACE":
            print("[MP-GATEWAY] RACE inbound policy: GPS_RTCM_DATA only")
        else:
            print("[MP-GATEWAY] MAINTENANCE inbound policy: MAVLink pass-through")

        self._report_destination_if_changed()

        try:
            while not self.stop_event.is_set():
                self._report_destination_if_changed()

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
                    self._publish_counters()
                    continue
                except OSError:
                    break

                if data:
                    self._forward_from_mp(data, addr)

                self._publish_counters()
        finally:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self._publish_status("STOPPED", "gateway thread stopped")
