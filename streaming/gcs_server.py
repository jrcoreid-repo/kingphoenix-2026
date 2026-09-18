from __future__ import annotations

import ipaddress
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
    SERVO_ACTION_ALLOWLIST = {"OPEN", "CLOSE", "TEST"}

    def __init__(
        self,
        state,
        events,
        waypoints,
        stop_event,
        command_manager=None,
        servo_controller=None,
    ):
        super().__init__(name="GCSServer")
        self.state = state
        self.events = events
        self.waypoints = waypoints
        self.stop_event = stop_event
        self.command = command_manager
        self.servo = servo_controller

        cfg = load_json("system.json")
        self.stream_cfg = cfg["stream"]
        self.mp_cfg = cfg.get("mission_planner_gateway", {})
        self.port = int(self.stream_cfg["control_port"])
        self.mp_destination_port = int(self.mp_cfg.get("destination_port", 14555))
        self.last_gcs_heartbeat_ns = 0
        self.gcs_endpoint = None
        self.gcs_timeout_s = float(self.stream_cfg.get("gcs_timeout_s", 3.0))

    def _hello_ack(self):
        with self.state.lock:
            mp_status = dict(self.state.system.get("mission_planner_gateway") or {})
            mp_override = dict(self.state.system.get("mission_planner_destination_override") or {})
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
            "version": "2.3.8",
            "bench_control": self.command is not None,
            "live_preflight": True,
            "mission_planner_gateway": mp_status,
            "mission_planner_destination_override": mp_override,
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
            print("[CONTROL] Mission Planner destination override remains cached in Pi state")

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

    @staticmethod
    def _copy_check(check, fallback_status="WARN", fallback_detail="unavailable"):
        if not isinstance(check, dict):
            return {"status": fallback_status, "detail": fallback_detail}
        return {
            "status": str(check.get("status", fallback_status)).upper(),
            "detail": str(check.get("detail", fallback_detail)),
        }

    @staticmethod
    def _validated_mp_ip(value):
        text = str(value or "").strip()
        try:
            ip = ipaddress.ip_address(text)
        except ValueError:
            return None, "invalid IPv4 address"
        if ip.version != 4:
            return None, "IPv4 required"
        if ip.is_unspecified or ip.is_multicast or text == "255.255.255.255":
            return None, "unicast IPv4 required"
        return str(ip), None

    def _set_mp_destination(self, tx, msg):
        if not bool(self.mp_cfg.get("enabled", False)):
            return ack(
                tx,
                False,
                "MP GATEWAY DISABLED - FULL_MISSION USES DIRECT MAVPROXY/RTK PATH",
            )
        ip, error = self._validated_mp_ip(msg.get("ip"))
        if error:
            return ack(tx, False, f"MISSION PLANNER IP REJECTED: {error}")

        try:
            port = int(msg.get("port", self.mp_destination_port))
        except Exception:
            return ack(tx, False, "MISSION PLANNER PORT REJECTED")
        if not (1 <= port <= 65535):
            return ack(tx, False, "MISSION PLANNER PORT OUT OF RANGE")

        override = {
            "ip": ip,
            "port": port,
            "set_by": "GCS",
            "timestamp_ns": time.monotonic_ns(),
        }
        with self.state.lock:
            self.state.system["mission_planner_destination_override"] = override

        print(f"[MP-GATEWAY] DESTINATION SET BY GCS -> {ip}:{port}")
        return ack(
            tx,
            True,
            f"MISSION PLANNER DESTINATION {ip}:{port}",
            data={
                "ip": ip,
                "port": port,
                "policy": str(self.mp_cfg.get("policy", "RACE")).upper(),
                "inbound": "GPS_RTCM_DATA_ONLY" if str(self.mp_cfg.get("policy", "RACE")).upper() == "RACE" else "MAINTENANCE",
            },
        )

    def _clear_mp_destination(self, tx):
        if not bool(self.mp_cfg.get("enabled", False)):
            return ack(
                tx,
                False,
                "MP GATEWAY DISABLED - FULL_MISSION USES DIRECT MAVPROXY/RTK PATH",
            )
        with self.state.lock:
            self.state.system.pop("mission_planner_destination_override", None)
        print("[MP-GATEWAY] GCS runtime destination override CLEARED")
        return ack(tx, True, "MISSION PLANNER DESTINATION AUTO")

    def _mp_gateway_status(self):
        with self.state.lock:
            gateway = dict(self.state.system.get("mission_planner_gateway") or {})
            override = dict(self.state.system.get("mission_planner_destination_override") or {})
        return {
            "gateway": gateway,
            "override": override,
            "configured_port": self.mp_destination_port,
            "policy": str(self.mp_cfg.get("policy", "RACE")).upper(),
        }

    def _strategy_preflight_check(self):
        results = {}
        for strategy in ("LEFT", "RIGHT"):
            try:
                ok, detail = self.waypoints.validate_strategy(
                    strategy,
                    int(getattr(self.waypoints, "total_waypoints", 11)),
                )
            except Exception as exc:
                ok, detail = False, f"{type(exc).__name__}: {exc}"
            results[strategy] = (bool(ok), str(detail))

        valid = [name for name, (ok, _) in results.items() if ok]
        if len(valid) == 2:
            status = "PASS"
        elif len(valid) == 1:
            status = "WARN"
        else:
            status = "FAIL"
        detail = " | ".join(
            f"{name}={'OK' if ok else info}" for name, (ok, info) in results.items()
        )
        return {"status": status, "detail": detail}

    def _live_preflight_snapshot(self):
        """Return one point-in-time readiness snapshot directly from Pi state.

        This is intentionally read-only. It does not depend on video metadata
        arriving at the GCS, so PRE-FLIGHT can distinguish an actually missing
        FC/Pi signal from stale/mock GUI values.
        """
        v = self.state.vehicle
        with self.state.lock:
            readiness = dict(self.state.system.get("readiness") or {})
            pi_health = dict(self.state.system.get("pi_health") or {})
            front = dict(self.state.vision.get("front") or {})
            bottom = dict(self.state.vision.get("bottom") or {})
            startup_state = str(self.state.system.get("startup_state", "UNKNOWN"))
            ready_for_mission = bool(self.state.system.get("ready_for_mission", False))

        ready_checks = readiness.get("checks") or {}
        pi_checks = pi_health.get("checks") or {}

        hb_age = v.heartbeat_age_s()
        connected = bool(v.connected)
        hb_fresh = connected and hb_age <= 2.5

        # Vehicle-state freshness uses real FC message ages. HEARTBEAT is the
        # hard minimum; position/power messages are reported in detail when present.
        gpos_age = v.message_age_s("GLOBAL_POSITION_INT")
        sys_age = v.message_age_s("SYS_STATUS")
        vehicle_fresh = hb_fresh and (gpos_age < 1.0 or sys_age < 1.5)

        checks = {
            "MAVLink Link": {
                "status": "PASS" if connected else "FAIL",
                "detail": "FC link connected" if connected else "FC link NOT connected",
            },
            "FC Heartbeat": {
                "status": "PASS" if hb_fresh else "FAIL",
                "detail": f"age {hb_age:.2f}s" if hb_age != float("inf") else "heartbeat missing",
            },
            "Vehicle State": {
                "status": "PASS" if vehicle_fresh else "FAIL",
                "detail": (
                    f"GLOBAL {gpos_age:.2f}s / SYS {sys_age:.2f}s"
                    if connected else "no live FC state"
                ),
            },
            "GPS": self._copy_check(ready_checks.get("gps"), "FAIL", "GPS readiness unavailable"),
            "LiDAR / Rangefinder": self._copy_check(ready_checks.get("lidar"), "FAIL", "LiDAR readiness unavailable"),
            "Front Camera": self._copy_check(ready_checks.get("front_camera"), "FAIL", "front camera readiness unavailable"),
            "Bottom Camera": self._copy_check(ready_checks.get("bottom_camera"), "FAIL", "bottom camera readiness unavailable"),
            "YOLO26 / Hailo": self._copy_check(ready_checks.get("hailo_ai"), "FAIL", "Hailo readiness unavailable"),
            "Mission File": {
                "status": "PASS" if int(getattr(self.waypoints, "total_waypoints", 0)) == 11 else "FAIL",
                "detail": (
                    f"full mission contract WP1-WP{int(getattr(self.waypoints, 'total_waypoints', 0))}"
                ),
            },
            "Strategy Data": self._strategy_preflight_check(),
            "Battery": {
                "status": "PASS" if connected and v.battery_v is not None else "FAIL",
                "detail": (
                    f"{float(v.battery_v):.2f} V / {v.battery_remaining}%"
                    if v.battery_v is not None else "battery telemetry unavailable"
                ),
            },
            "Pi Thermal": self._copy_check(pi_checks.get("pi_thermal"), "FAIL", "Pi health unavailable"),
            "Pi Power": self._copy_check(pi_checks.get("pi_power"), "FAIL", "Pi health unavailable"),
            "Pi Throttling": self._copy_check(pi_checks.get("pi_throttling"), "FAIL", "Pi health unavailable"),
            "Pi CPU Load": self._copy_check(pi_checks.get("pi_cpu"), "FAIL", "Pi health unavailable"),
            "Pi RAM": self._copy_check(pi_checks.get("pi_ram"), "FAIL", "Pi health unavailable"),
            "Pi Disk": self._copy_check(pi_checks.get("pi_disk"), "FAIL", "Pi health unavailable"),
        }

        return {
            "timestamp_ns": time.monotonic_ns(),
            "startup_state": startup_state,
            "ready_for_mission": ready_for_mission,
            "readiness_status": readiness.get("status", "UNKNOWN"),
            "failed_critical": list(readiness.get("failed_critical") or []),
            "checks": checks,
            "raw": {
                "connected": connected,
                "heartbeat_age_s": None if hb_age == float("inf") else hb_age,
                "battery_v": v.battery_v,
                "battery_remaining": v.battery_remaining,
                "front_video_state": front.get("video_state"),
                "bottom_video_state": bottom.get("video_state"),
            },
        }

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(0.2)
        print(f"[GCS] control UDP :{self.port}")
        print(f"[GCS] heartbeat watchdog : {self.gcs_timeout_s:.1f}s")
        print("[GCS] V2.3 bench control: ARM/DISARM + DISARMED mode select")
        print("[GCS] V2.3.4 live PRE-FLIGHT request endpoint enabled")
        print("[GCS] V2.3.5 Mission Planner destination control enabled")
        print("[GCS] V2.3.8 full-mission control endpoint enabled")

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

        if kind == "GET_PREFLIGHT_STATUS":
            self._touch_gcs_heartbeat()
            return ack(
                tx,
                True,
                "LIVE PREFLIGHT SNAPSHOT",
                data=self._live_preflight_snapshot(),
            )

        if kind == "SET_MP_DESTINATION":
            self._touch_gcs_heartbeat()
            return self._set_mp_destination(tx, msg)

        if kind == "CLEAR_MP_DESTINATION":
            self._touch_gcs_heartbeat()
            return self._clear_mp_destination(tx)

        if kind == "GET_MP_GATEWAY_STATUS":
            self._touch_gcs_heartbeat()
            return ack(tx, True, "MISSION PLANNER GATEWAY STATUS", data=self._mp_gateway_status())

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
            if self.state.vehicle.armed:
                return ack(tx, False, "EDIT MODE BLOCKED: vehicle ARMED")
            strategy = normalize_strategy(msg.get("strategy") or self.state.mission.strategy)
            if self.waypoints.edit_unlocked and strategy != self.waypoints.edit_strategy:
                return ack(
                    tx,
                    False,
                    f"EDIT MODE ALREADY LOCKED TO {self.waypoints.edit_strategy}",
                )
            self.state.mission.strategy = strategy
            self.waypoints.unlock(strategy)
            self.events.publish("WAYPOINT_EDIT_UNLOCKED", {"strategy": strategy})
            return ack(tx, True, f"SETUP MODE ON / {strategy} / AI TEST SWITCH 10s")

        if kind == "LOCK_EDIT":
            strategy = normalize_strategy(msg.get("strategy") or self.state.mission.strategy)
            if self.waypoints.edit_unlocked and strategy != self.waypoints.edit_strategy:
                return ack(
                    tx,
                    False,
                    f"LOCK REJECTED: edit session belongs to {self.waypoints.edit_strategy}",
                )
            strategy = self.waypoints.edit_strategy if self.waypoints.edit_unlocked else strategy
            self.state.mission.strategy = strategy
            self.waypoints.lock(strategy)
            self.events.publish("WAYPOINT_EDIT_LOCKED", {"strategy": strategy})
            return ack(tx, True, f"SETUP MODE OFF / {strategy} / AI OFF / FC SYNC QUEUED")

        if kind == "SAVE_WAYPOINT":
            strategy = normalize_strategy(msg["strategy"])
            ok, message = self.waypoints.save_current(strategy, int(msg["wp"]))
            return ack(tx, ok, message)

        if kind == "SET_STRATEGY":
            strategy = normalize_strategy(msg["strategy"])
            self.state.mission.strategy = strategy
            return ack(tx, True, "STRATEGY " + strategy)

        if kind == "START_MISSION":
            if self.waypoints.edit_unlocked:
                return ack(tx, False, "START BLOCKED: EXIT SETUP MODE")
            if self.state.vehicle.armed:
                return ack(tx, False, "START BLOCKED: vehicle already ARMED")
            with self.state.lock:
                ready = bool(self.state.system.get("ready_for_mission", False))
                readiness = dict(self.state.system.get("readiness") or {})
            if not ready:
                failed = ",".join(readiness.get("failed_critical") or []) or "NOT_READY"
                return ack(tx, False, f"START BLOCKED: readiness {failed}")
            if self.servo is not None:
                servo_state = str(getattr(self.servo, "logical_state", "UNKNOWN")).upper()
                if servo_state != "CLOSE":
                    return ack(
                        tx,
                        False,
                        f"START BLOCKED: payload servo is {servo_state}; CLOSE from SETUP first",
                    )
            strategy = self.state.mission.strategy
            ok, detail = self.waypoints.validate_strategy(
                strategy,
                int(getattr(self.waypoints, "total_waypoints", 11)),
            )
            if not ok:
                return ack(tx, False, f"START BLOCKED: {strategy} {detail}")
            self.events.publish(
                "START_MISSION",
                {"strategy": strategy},
            )
            return ack(
                tx,
                True,
                "START REQUESTED",
                data={
                    "strategy": strategy,
                    "accepted": True,
                    "backend": "MISSION_EVENT_BUS",
                },
            )

        if kind == "ABORT_MISSION":
            self.events.publish("GLOBAL_ABORT", {"reason": "GCS_ABORT"})
            return ack(
                tx,
                True,
                "ABORT REQUESTED",
                data={
                    "accepted": True,
                    "backend": "GLOBAL_ABORT_EVENT",
                },
            )

        if kind == "SERVO_CONTROL":
            action = str(msg.get("action") or "").upper().strip()
            if action not in self.SERVO_ACTION_ALLOWLIST:
                return ack(tx, False, f"SERVO ACTION REJECTED: {action or '--'}")
            if self.servo is None:
                return ack(tx, False, "SERVO BACKEND UNAVAILABLE")
            if self.state.vehicle.armed:
                return ack(tx, False, "SERVO GCS CONTROL BLOCKED: vehicle ARMED")
            if not self.waypoints.edit_unlocked:
                return ack(tx, False, "SERVO BLOCKED: ENTER SETUP MODE")

            if action == "OPEN":
                ok, message = self.servo.open(source="GCS")
            elif action == "CLOSE":
                ok, message = self.servo.close(source="GCS")
            else:
                ok, message = self.servo.test(source="GCS")

            print(f"[GCS-SERVO] {message}")
            return ack(
                tx,
                bool(ok),
                message,
                data={
                    "action": action,
                    "accepted": bool(ok),
                    "executed": bool(ok),
                    "backend": "MAV_CMD_DO_SET_SERVO",
                },
            )

        return ack(tx, False, "Unknown command " + kind)
