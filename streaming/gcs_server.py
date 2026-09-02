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

    # Bench-safe mode list exposed to the GCS. Mode changes from this control
    # path are intentionally required to be DISARMED by CommandManager.
    BENCH_MODE_ALLOWLIST = {
        "STABILIZE",
        "ALT_HOLD",
        "LOITER",
        "GUIDED",
        "AUTO",
        "LAND",
    }

    def __init__(self, state, events, waypoints, stop_event, command_manager=None):
        super().__init__(name="GCSServer")
        self.state = state
        self.events = events
        self.waypoints = waypoints
        self.stop_event = stop_event
        self.command = command_manager

        cfg = load_json("system.json")
        self.stream_cfg = cfg["stream"]
        self.port = int(self.stream_cfg["control_port"])
        self.last_gcs_heartbeat_ns = 0
        self.gcs_endpoint = None
        self.gcs_timeout_s = float(self.stream_cfg.get("gcs_timeout_s", 3.0))

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
            "version": "2.3.0",
            "bench_control": self.command is not None,
        }

    def _mark_disconnected(self, reason):
        was_connected = bool(self.state.system.get("gcs_connected", False))
        self.state.system["gcs_connected"] = False
        self.gcs_endpoint = None
        self.state.system["gcs_endpoint"] = None
        self.last_gcs_heartbeat_ns = 0

        if was_connected:
            print(f"[CONTROL] GCS DISCONNECTED | reason={reason}")
            print("[CONTROL] Pi runtime remains ACTIVE and waiting for GCS reconnect")

    def _check_gcs_timeout(self):
        if not self.state.system.get("gcs_connected", False):
            return
        if self.last_gcs_heartbeat_ns <= 0:
            return

        age_s = (time.monotonic_ns() - self.last_gcs_heartbeat_ns) / 1e9
        if age_s > self.gcs_timeout_s:
            self._mark_disconnected(
                f"HEARTBEAT TIMEOUT {age_s:.1f}s > {self.gcs_timeout_s:.1f}s"
            )

    def _touch_gcs_heartbeat(self):
        """Keep the session alive while a verified command is executing."""
        self.last_gcs_heartbeat_ns = time.monotonic_ns()
        self.state.system["gcs_connected"] = True

    @staticmethod
    def _vct_data(result):
        return {
            "outcome": getattr(result, "outcome", "FAILED"),
            "attempts": int(getattr(result, "attempts", 0)),
            "reason": str(getattr(result, "reason", "")),
            "ack_result": getattr(result, "ack_result", None),
            "ack_source": getattr(result, "ack_source", None),
            "elapsed_s": float(getattr(result, "elapsed_s", 0.0)),
        }

    def _run_bench_command(self, tx, label, fn):
        if self.command is None:
            return ack(tx, False, "FC COMMAND BACKEND UNAVAILABLE")

        self._touch_gcs_heartbeat()
        print(f"[GCS-CMD] {label} requested")
        try:
            result = fn()
        except Exception as exc:
            self._touch_gcs_heartbeat()
            print(f"[GCS-CMD] {label} exception: {exc}")
            return ack(tx, False, f"{label} EXCEPTION: {exc}")

        self._touch_gcs_heartbeat()
        data = self._vct_data(result)
        ok = bool(getattr(result, "ok", False))
        message = (
            f"{label} VERIFIED"
            if ok
            else f"{label} FAILED: {data['reason']}"
        )
        print(
            f"[GCS-CMD] {label} ok={ok} outcome={data['outcome']} "
            f"attempts={data['attempts']} reason={data['reason']}"
        )
        return ack(tx, ok, message, data=data)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(0.2)
        print(f"[GCS] control UDP :{self.port}")
        print(f"[GCS] heartbeat watchdog : {self.gcs_timeout_s:.1f}s")
        print("[GCS] V2.3 bench control: ARM/DISARM + DISARMED mode select")

        while not self.stop_event.is_set():
            try:
                data, peer = sock.recvfrom(65535)
            except socket.timeout:
                self._check_gcs_timeout()
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

            self._check_gcs_timeout()

        sock.close()

    def handle(self, msg, peer=None):
        tx = msg.get("tx_id")
        kind = str(msg.get("type", "")).upper()

        if kind == "HELLO":
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
            print("[CONTROL] Heartbeat supervision ACTIVE")
            print(
                "[STREAM] destination "
                f"{pc_ip} | front={self.gcs_endpoint['front_video_port']}/"
                f"{self.gcs_endpoint['front_metadata_port']} | "
                f"bottom={self.gcs_endpoint['bottom_video_port']}/"
                f"{self.gcs_endpoint['bottom_metadata_port']}"
            )
            return self._hello_ack()

        if kind == "HEARTBEAT":
            if self.gcs_endpoint is not None:
                self._touch_gcs_heartbeat()
            return None

        if kind == "DISCONNECT":
            self._mark_disconnected("GCS GRACEFUL CLOSE")
            return {"type": "DISCONNECT_ACK", "ok": True}

        # ------------------------------------------------------------
        # V2.3 guarded FC command path.
        # These commands use the existing CommandManager/VCT path. Movement,
        # takeoff and mission velocity TX remain disabled.
        # ------------------------------------------------------------
        if kind == "ARM":
            if not bool(msg.get("bench_confirmed", False)):
                return ack(tx, False, "ARM BLOCKED: bench confirmation missing")
            return self._run_bench_command(tx, "ARM", self.command.arm if self.command else lambda: None)

        if kind == "DISARM":
            return self._run_bench_command(tx, "DISARM", self.command.disarm if self.command else lambda: None)

        if kind == "SET_MODE":
            mode = str(msg.get("mode") or "").upper().strip()
            if mode not in self.BENCH_MODE_ALLOWLIST:
                return ack(tx, False, f"MODE BLOCKED: {mode or '--'} not in bench allowlist")
            if self.command is None:
                return ack(tx, False, "FC COMMAND BACKEND UNAVAILABLE")
            return self._run_bench_command(
                tx,
                f"MODE {mode}",
                lambda: self.command.set_mode(mode, require_disarmed=True),
            )

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
