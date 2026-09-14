#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"


class FlightTestError(RuntimeError):
    pass


def load_json(name: str):
    with (CONFIG_DIR / name).open("r", encoding="utf-8") as f:
        return json.load(f)


class DirectFlightTest:
    """Standalone field-test MAVLink owner for the DEV branch.

    Do not run main.py, MAVProxy, Mission Planner serial ownership, or another
    recv_match consumer on /dev/ttyAMA0 at the same time.
    """

    def __init__(self):
        cfg = load_json("system.json")
        self.cfg = cfg
        self.mav_cfg = cfg["mavlink"]
        self.rf_cfg = cfg.get("rangefinder", {})
        self.master = None
        self.last_hb = 0.0
        self.mode = "UNKNOWN"
        self.armed = False
        self.gps_fix = None
        self.gps_sats = None
        self.local = None
        self.global_pos = None
        self.lidar = None
        self.lidar_ts = 0.0
        self.mission_current = None
        self.reached_seq = None

    def open(self):
        conn = self.mav_cfg["connection"]
        kwargs = {
            "source_system": int(self.mav_cfg.get("source_system", 1)),
            "source_component": int(self.mav_cfg.get("source_component", 191)),
            "autoreconnect": True,
        }
        if str(conn).startswith("/dev/"):
            kwargs["baud"] = int(self.mav_cfg.get("baud", 921600))
        self.master = mavutil.mavlink_connection(conn, **kwargs)
        print(f"[MAVLINK] connecting {conn} ...")
        hb = self.master.wait_heartbeat(timeout=float(self.mav_cfg.get("heartbeat_timeout_s", 2.5)) + 2.0)
        if hb is None:
            raise FlightTestError("heartbeat timeout")
        self._handle(hb)
        print(f"[MAVLINK] heartbeat OK target={self.master.target_system}:{self.master.target_component}")
        self._request_streams()

    def close(self):
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def _request_interval(self, message_id: int, hz: float):
        if self.master is None or hz <= 0:
            return
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id), float(int(round(1_000_000.0 / hz))), 0, 0, 0, 0, 0,
        )

    def _request_streams(self):
        rates = self.mav_cfg.get("message_rates_hz", {})
        ids = {
            "GLOBAL_POSITION_INT": 33,
            "LOCAL_POSITION_NED": 32,
            "GPS_RAW_INT": 24,
            "DISTANCE_SENSOR": 132,
            "MISSION_CURRENT": 42,
        }
        for name, msg_id in ids.items():
            self._request_interval(msg_id, float(rates.get(name, 10)))

    def _handle(self, msg):
        if msg is None or msg.get_type() == "BAD_DATA":
            return
        t = msg.get_type()
        now = time.monotonic()
        if t == "HEARTBEAT":
            self.last_hb = now
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                self.mode = self.master.flightmode or self.mode
            except Exception:
                pass
        elif t == "GPS_RAW_INT":
            self.gps_fix = int(getattr(msg, "fix_type", 0))
            sats = int(getattr(msg, "satellites_visible", 255))
            self.gps_sats = None if sats == 255 else sats
        elif t == "LOCAL_POSITION_NED":
            self.local = (float(msg.x), float(msg.y), float(msg.z))
        elif t == "GLOBAL_POSITION_INT":
            self.global_pos = (float(msg.lat) / 1e7, float(msg.lon) / 1e7, float(msg.relative_alt) / 1000.0)
        elif t == "DISTANCE_SENSOR":
            distance_cm = int(getattr(msg, "current_distance", 0))
            orientation = int(getattr(msg, "orientation", -1))
            expected = int(self.rf_cfg.get("expected_orientation", 25))
            if distance_cm > 0 and orientation == expected:
                self.lidar = distance_cm / 100.0
                self.lidar_ts = now
        elif t == "RANGEFINDER" and bool(self.rf_cfg.get("allow_legacy_fallback", True)):
            if now - self.lidar_ts > float(self.rf_cfg.get("valid_age_s", 0.20)):
                distance = float(getattr(msg, "distance", 0.0))
                if distance > 0:
                    self.lidar = distance
                    self.lidar_ts = now
        elif t == "MISSION_CURRENT":
            self.mission_current = int(msg.seq)
        elif t == "MISSION_ITEM_REACHED":
            self.reached_seq = int(msg.seq)
        elif t == "STATUSTEXT":
            raw = getattr(msg, "text", "")
            if isinstance(raw, (bytes, bytearray)):
                raw = raw.decode("utf-8", errors="replace")
            text = str(raw).rstrip("\x00").strip()
            if text:
                print(f"[FC] {text}")

    def poll(self, timeout=0.2):
        if self.master is None:
            raise FlightTestError("MAVLink is not open")
        msg = self.master.recv_match(blocking=True, timeout=timeout)
        self._handle(msg)
        return msg

    def heartbeat_ok(self):
        return self.last_hb > 0 and (time.monotonic() - self.last_hb) <= float(self.mav_cfg.get("heartbeat_timeout_s", 2.5))

    def lidar_ok(self):
        age = float(self.rf_cfg.get("valid_age_s", 0.20))
        return self.lidar is not None and self.lidar > 0 and (time.monotonic() - self.lidar_ts) <= age

    def collect_preflight(self, require_auto=False, timeout=7.0):
        deadline = time.monotonic() + float(timeout)
        good_lidar_samples = 0
        while time.monotonic() < deadline:
            self.poll(0.2)
            good_lidar_samples = good_lidar_samples + 1 if self.lidar_ok() else 0
            auto_ok = self.gps_fix is not None and self.gps_fix >= 3 and self.local is not None and self.global_pos is not None
            if self.heartbeat_ok() and good_lidar_samples >= 3 and (auto_ok or not require_auto):
                if self.armed:
                    raise FlightTestError("vehicle already ARMED; runner requires DISARMED start")
                return
        problems = []
        if not self.heartbeat_ok(): problems.append("heartbeat missing/stale")
        if not self.lidar_ok(): problems.append("LiDAR missing/stale/wrong orientation")
        if require_auto:
            if self.gps_fix is None or self.gps_fix < 3: problems.append(f"GPS fix < 3 (fix={self.gps_fix})")
            if self.local is None: problems.append("LOCAL_POSITION_NED unavailable")
            if self.global_pos is None: problems.append("GLOBAL_POSITION_INT unavailable")
        raise FlightTestError("preflight failed: " + "; ".join(problems or ["telemetry not stable"]))

    def set_mode(self, mode: str, timeout=4.0):
        mode = str(mode).upper()
        mapping = self.master.mode_mapping()
        if not mapping or mode not in mapping:
            raise FlightTestError(f"mode {mode} unavailable")
        self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode])
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            self.poll(0.2)
            if self.mode == mode:
                print(f"[MODE] {mode}")
                return
        raise FlightTestError(f"failed to enter {mode}")

    def arm(self, timeout=6.0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            self.poll(0.2)
            if self.armed:
                print("[ARM] ARMED")
                return
        raise FlightTestError("arming failed/timed out")

    def takeoff_lidar_confirmed(self, target_m: float, margin=0.15, tolerance=0.05, hold_s=0.50, timeout=12.0):
        target_m = float(target_m)
        if not 0.50 <= target_m <= 3.0:
            raise FlightTestError("takeoff altitude must be 0.50-3.00 m")
        command_m = target_m * (1.0 + float(margin))
        threshold = target_m - float(tolerance)
        print(f"[TAKEOFF] requested={target_m:.2f}m command={command_m:.2f}m (+{margin*100:.0f}%)")
        self.set_mode("GUIDED")
        self.arm()
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, float("nan"), 0, 0, command_m,
        )
        deadline = time.monotonic() + float(timeout)
        reached_since = None
        lidar_loss_since = None
        while time.monotonic() < deadline:
            self.poll(0.1)
            now = time.monotonic()
            if not self.heartbeat_ok():
                raise FlightTestError("heartbeat lost during takeoff")
            if self.lidar_ok():
                lidar_loss_since = None
                print(f"\r[TAKEOFF] LiDAR={self.lidar:.2f}m target>={threshold:.2f}m", end="", flush=True)
                if self.lidar >= threshold:
                    reached_since = reached_since or now
                    if now - reached_since >= float(hold_s):
                        print(f"\n[TAKEOFF] COMPLETE LiDAR={self.lidar:.2f}m")
                        return
                else:
                    reached_since = None
            else:
                lidar_loss_since = lidar_loss_since or now
                if now - lidar_loss_since > 0.60:
                    print()
                    raise FlightTestError("LiDAR invalid/stale >0.60 s during takeoff")
        print()
        raise FlightTestError(f"takeoff timeout; LiDAR={self.lidar}")

    def land_and_wait(self, timeout=20.0):
        print("[LAND] switching to LAND")
        self.set_mode("LAND")
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            self.poll(0.15)
            if not self.heartbeat_ok():
                raise FlightTestError("heartbeat lost during LAND")
            if not self.armed:
                print("[LAND] COMPLETE vehicle DISARMED")
                return
        raise FlightTestError("LAND timeout; vehicle still armed")

    def land_best_effort(self, reason: str):
        print(f"[ABORT] {reason}")
        if self.master is None or not self.armed:
            return
        try:
            self.set_mode("LAND", timeout=2.5)
        except Exception as exc:
            print(f"[ABORT] LAND failed: {exc}")

    @staticmethod
    def validate_waypoint(strategy: str, wp: int):
        data = load_json("waypoints.json")
        defaults = load_json("flight_defaults.json")
        nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))
        strategy = str(strategy).upper()
        key = f"WP{int(wp)}"
        if strategy not in data or key not in data[strategy]:
            raise FlightTestError(f"unknown waypoint {strategy}/{key}")
        item = data[strategy][key]
        x, y = item.get("x"), item.get("y")
        if x is None or y is None:
            raise FlightTestError(f"{strategy}/{key} has no x/y coordinate; AUTO blocked")
        try:
            x, y = float(x), float(y)
            alt_m = float(item.get("alt_m", nominal_alt))
        except Exception as exc:
            raise FlightTestError(f"{strategy}/{key} contains non-numeric data: {exc}")
        if not all(math.isfinite(v) for v in (x, y, alt_m)):
            raise FlightTestError(f"{strategy}/{key} contains invalid numeric data")
        return {"x": x, "y": y, "alt_m": alt_m}

    def local_waypoint_to_global(self, waypoint):
        if self.local is None or self.global_pos is None:
            raise FlightTestError("local/global position unavailable")
        cur_x, cur_y, _ = self.local
        lat, lon, _ = self.global_pos
        north_m = waypoint["x"] - cur_x
        east_m = waypoint["y"] - cur_y
        target_lat = lat + north_m / 111_111.0
        target_lon = lon + east_m / (111_111.0 * max(0.01, abs(math.cos(math.radians(lat)))))
        return target_lat, target_lon, math.hypot(north_m, east_m)

    def clear_mission(self):
        try:
            self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        except TypeError:
            self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self.poll(0.1)
