from __future__ import annotations

import json
import socket
import threading
import time

from streaming.gcs_protocol import ack
from mission.strategy import normalize_strategy
from config import load_json


class GcsServer(threading.Thread):
    daemon = True

    def __init__(self, state, events, waypoints, stop_event):
        super().__init__(name="GCSServer")
        self.state = state
        self.events = events
        self.waypoints = waypoints
        self.stop_event = stop_event

        cfg = load_json("system.json")
        self.stream_cfg = cfg["stream"]
        self.port = int(self.stream_cfg["control_port"])
        self.last_gcs_heartbeat_ns = 0
        self.gcs_endpoint = None

    def _hello_ack(self):
        return {
            "type": "HELLO_ACK",
            "ok": True,
            "video_transport": self.stream_cfg.get(
                "video_transport", "MPEG-TS/H264/UDP"
            ),
            "sync": self.stream_cfg.get(
                "sync", "48-cell barcode exact frame_id"
            ),
            "active_ai_camera": self.state.vision.get(
                "active_ai_camera", "front"
            ),
            "version": "1.9.0.1",
        }

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(0.2)
        print(f"[GCS] control UDP :{self.port}")

        while not self.stop_event.is_set():
            try:
                data, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue

            try:
                msg = json.loads(data.decode("utf-8"))
                response = self.handle(msg, peer)
            except Exception as exc:
                response = ack(None, False, str(exc))

            if response is not None:
                try:
                    sock.sendto(json.dumps(response).encode("utf-8"), peer)
                except Exception:
                    pass

        sock.close()

    def handle(self, msg, peer=None):
        tx = msg.get("tx_id")
        kind = str(msg.get("type", "")).upper()

        if kind == "HELLO":
            # V1.9.0.1: the actual UDP source address is authoritative.
            # The Windows GCS computes pc_ip using its default route, which can
            # be wrong on multi-NIC/VPN/WSL systems. The HELLO packet's peer[0]
            # is the address that demonstrably reached this Raspberry Pi.
            claimed_pc_ip = str(msg.get("pc_ip") or "")
            peer_ip = str(peer[0] if peer else "")
            pc_ip = peer_ip or claimed_pc_ip

            self.gcs_endpoint = {
                "ip": pc_ip,
                "front_video_port": int(msg.get(
                    "front_video_port",
                    self.stream_cfg["front_video_port"],
                )),
                "front_metadata_port": int(msg.get(
                    "front_metadata_port",
                    self.stream_cfg["front_metadata_port"],
                )),
                "bottom_video_port": int(msg.get(
                    "bottom_video_port",
                    self.stream_cfg["bottom_video_port"],
                )),
                "bottom_metadata_port": int(msg.get(
                    "bottom_metadata_port",
                    self.stream_cfg["bottom_metadata_port"],
                )),
            }
            self.state.system["gcs_endpoint"] = dict(self.gcs_endpoint)
            self.state.system["gcs_connected"] = True
            self.last_gcs_heartbeat_ns = time.monotonic_ns()
            print(f"[CONTROL] GCS CONNECTED peer={peer_ip} claimed={claimed_pc_ip or '--'}")
            print(
                "[STREAM] destination "
                f"{pc_ip} | front={self.gcs_endpoint['front_video_port']}/"
                f"{self.gcs_endpoint['front_metadata_port']} | "
                f"bottom={self.gcs_endpoint['bottom_video_port']}/"
                f"{self.gcs_endpoint['bottom_metadata_port']}"
            )
            return self._hello_ack()

        if kind == "HEARTBEAT":
            self.last_gcs_heartbeat_ns = time.monotonic_ns()
            self.state.system["gcs_connected"] = True
            return None

        if kind == "DISCONNECT":
            self.state.system["gcs_connected"] = False
            self.gcs_endpoint = None
            self.state.system["gcs_endpoint"] = None
            print("[CONTROL] GCS DISCONNECTED")
            return {"type": "DISCONNECT_ACK", "ok": True}

        if kind == "UNLOCK_EDIT":
            self.waypoints.unlock()
            return ack(tx, True, "EDIT MODE UNLOCKED")

        if kind == "LOCK_EDIT":
            self.waypoints.lock()
            return ack(tx, True, "EDIT MODE LOCKED")

        if kind == "SAVE_WAYPOINT":
            strategy = normalize_strategy(msg["strategy"])
            ok, message = self.waypoints.save_current(strategy, int(msg["wp"]))
            return ack(tx, ok, message)

        if kind == "SET_STRATEGY":
            strategy = normalize_strategy(msg["strategy"])
            self.state.mission.strategy = strategy
            return ack(tx, True, "STRATEGY " + strategy)

        if kind == "START_MISSION":
            self.events.publish(
                "START_MISSION",
                {"strategy": self.state.mission.strategy},
            )
            return ack(tx, True, "START REQUESTED")

        if kind == "ABORT_MISSION":
            self.events.publish("GLOBAL_ABORT", {"reason": "GCS_ABORT"})
            return ack(tx, True, "ABORT REQUESTED")

        return ack(tx, False, "Unknown command " + kind)
