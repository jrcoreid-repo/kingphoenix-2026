#!/usr/bin/env python3
"""Guarded hardware-flight helpers for Kingphoenix bench/field tests.

IMPORTANT:
- This module owns recv_match() while a test runner is active. Do not run main.py,
  MAVProxy, or another process that consumes the same serial MAVLink stream.
- Low-altitude takeoff completion is confirmed by the downward LiDAR.
- AUTO test converts saved LOCAL_POSITION_NED x/y into a short-range GPS target.
"""
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
        self.lidar_orientation = None
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
        interval_us = int(round(1_000_000.0 / float(hz)))
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id), float(interval_us), 0, 0, 0, 0, 0,
        )

    def _request_streams(self):
        rates = self.mav_cfg.get("message_rates_hz", {})
        ids = {
            "GLOBAL_POSITION_INT": 33,
            "LOCAL_POSITION_NED": 32,
            "GPS_RAW_INT": 24,
            "DISTANCE_SENSOR": 132,
            "RANGEFINDER": 173,
            "MISSION_CURRENT": 42,
        }
        for name, mid in ids.items():
            self._request_interval(mid, float(rates.get(name, 10)))

    def _handle(self, msg):
        if msg is None or msg.get_type() == "BAD_DATA":
            return
        t = msg.get_type()
        now = time.monotonic()
        if t == "HEARTBEAT":
            self.last_hb = now
            try:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            except Exception:
                pass
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
            self.global_pos = (
                float(msg.lat) / 1e7,
                float(msg.lon) / 1e7,
                float(msg.relative_alt) / 1000.0,
            )
        elif t == "DISTANCE_SENSOR":
            d = int(getattr(msg, "current_distance", 0))
            orientation = int(getattr(msg, "orientation", -1))
            expected = int(self.rf_cfg.get("expected_orientation", 25))
            if d > 0 and orientation == expected:
                self.lidar = d / 100.0
                self.lidar_ts = now
                self.lidar_orientation = orientation
        elif t == "RANGEFINDER" and bool(self.rf_cfg.get("allow_legacy_fallback", True)):
            # Only use legacy RANGEFINDER if fresh DISTANCE_SENSOR is unavailable.
            if now - self.lidar_ts > float(self.rf_cfg.get("valid_age_s", 0.15)):
                d = float(getattr(msg, "distance", 0.0))
                if d > 0:
                    self.lidar = d
                    self.lidar_ts = now
                    self.lidar_orientation = None
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
        max_age = float(self.rf_cfg.get("valid_age_s", 0.15))
        return self.lidar is not None and self.lidar > 0 and (time.monotonic() - self.lidar_ts) <= max_age

    def collect_preflight(self, require_auto=False, timeout=6.0):
        deadline = time.monotonic() + timeout
        lidar_good = 0
        while time.monotonic() < deadline:
            self.poll(0.2)
            if self.lidar_ok():
                lidar_good += 1
            else:
                lidar_good = 0
            base_ok = self.heartbeat_ok() and lidar_good >= 3
            auto_ok = (
                self.gps_fix is not None and self.gps_fix >= 3 and
                self.local is not None and self.global_pos is not None
            )
            if base_ok and (auto_ok if require_auto else True):
                if self.armed:
                    raise FlightTestError("vehicle is already ARMED; test runner requires a disarmed start")
                return
        problems = []
        if not self.heartbeat_ok(): problems.append("heartbeat missing/stale")
        if not self.lidar_ok(): problems.append("LiDAR missing/stale/wrong orientation")
        if require_auto:
            if self.gps_fix is None or self.gps_fix < 3: problems.append(f"GPS fix < 3 (fix={self.gps_fix})")
            if self.local is None: problems.append("LOCAL_POSITION_NED unavailable")
            if self.global_pos is None: problems.append("GLOBAL_POSITION_INT unavailable")
        raise FlightTestError("preflight failed: " + "; ".join(problems or ["required telemetry not stable"]))

    def set_mode(self, mode: str, timeout=4.0):
        mode = mode.upper()
        mapping = self.master.mode_mapping()
        if not mapping or mode not in mapping:
            raise FlightTestError(f"flight mode {mode} is not available")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode],
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll(0.2)
            if self.mode == mode:
                print(f"[MODE] {mode}")
                return
        raise FlightTestError(f"failed to enter {mode}")

    def arm(self, timeout=6.0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll(0.2)
            if self.armed:
                print("[ARM] ARMED")
                return
        raise FlightTestError("arming failed or timed out")

    def land_best_effort(self, reason: str):
        print(f"[ABORT] {reason}")
        if self.master is None:
            return
        try:
            self.set_mode("LAND", timeout=2.5)
        except Exception as e:
            print(f"[ABORT] LAND command failed: {e}")

    def takeoff_lidar_confirmed(self, target_m: float, margin=0.15, tolerance=0.05,
                                hold_s=0.50, timeout=12.0):
        target_m = float(target_m)
        if not 0.30 <= target_m <= 3.0:
            raise FlightTestError("test altitude must be between 0.30 m and 3.00 m")
        command_m = target_m * (1.0 + float(margin))
        threshold = max(0.05, target_m - float(tolerance))
        print(f"[TAKEOFF] requested={target_m:.2f}m command={command_m:.2f}m (+{margin*100:.0f}%)")
        self.set_mode("GUIDED")
        self.arm()
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, float("nan"), 0, 0, command_m,
        )
        deadline = time.monotonic() + timeout
        reached_since = None
        lidar_loss_since = None
        last_print = 0.0
        while time.monotonic() < deadline:
            self.poll(0.1)
            now = time.monotonic()
            if not self.heartbeat_ok():
                raise FlightTestError("heartbeat lost during takeoff")
            if self.lidar_ok():
                lidar_loss_since = None
                if now - last_print >= 0.25:
                    print(f"[TAKEOFF] LiDAR={self.lidar:.2f}m target>={threshold:.2f}m")
                    last_print = now
                if self.lidar >= threshold:
                    reached_since = reached_since or now
                    if now - reached_since >= hold_s:
                        print(f"[TAKEOFF] COMPLETE LiDAR={self.lidar:.2f}m stable={hold_s:.2f}s")
                        return
                else:
                    reached_since = None
            else:
                lidar_loss_since = lidar_loss_since or now
                if now - lidar_loss_since > 0.60:
                    raise FlightTestError("LiDAR invalid/stale for >0.60 s during takeoff")
        raise FlightTestError(f"takeoff timeout; LiDAR={self.lidar}")

    @staticmethod
    def validate_waypoint(strategy: str, wp: int):
        data = load_json("waypoints.json")
        strategy = strategy.upper()
        key = f"WP{int(wp)}"
        if strategy not in data:
            raise FlightTestError(f"strategy {strategy} does not exist in waypoints.json")
        if key not in data[strategy]:
            raise FlightTestError(f"{key} does not exist for strategy {strategy}")
        item = data[strategy][key]
        x, y = item.get("x"), item.get("y")
        if x is None or y is None:
            raise FlightTestError(f"{strategy}/{key} has no coordinate yet (x/y is null); AUTO test blocked")
        try:
            x, y = float(x), float(y)
        except Exception as e:
            raise FlightTestError(f"{strategy}/{key} coordinate is not numeric: {e}")
        if not math.isfinite(x) or not math.isfinite(y):
            raise FlightTestError(f"{strategy}/{key} coordinate is not finite")
        return {"x": x, "y": y, "z": item.get("z")}

    def local_waypoint_to_global(self, waypoint):
        if self.local is None or self.global_pos is None:
            raise FlightTestError("cannot convert waypoint: local/global position unavailable")
        cur_x, cur_y, _ = self.local
        lat, lon, _ = self.global_pos
        north_m = waypoint["x"] - cur_x
        east_m = waypoint["y"] - cur_y
        lat_rad = math.radians(lat)
        target_lat = lat + north_m / 111_111.0
        cos_lat = max(0.01, abs(math.cos(lat_rad)))
        target_lon = lon + east_m / (111_111.0 * cos_lat)
        return target_lat, target_lon, math.hypot(north_m, east_m)

    def clear_mission(self):
        try:
            self.master.mav.mission_clear_all_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        # A clear ACK may arrive; do not require it because firmware versions vary.
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            self.poll(0.1)

    def upload_single_auto_waypoint(self, lat: float, lon: float, alt_m: float,
                                    acceptance_radius_m=0.5, timeout=6.0):
        self.clear_mission()
        mtype = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        try:
            self.master.mav.mission_count_send(
                self.master.target_system, self.master.target_component, 1, mtype
            )
        except TypeError:
            self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, 1)

        deadline = time.monotonic() + timeout
        sent = False
        while time.monotonic() < deadline:
            msg = self.poll(0.4)
            if msg is None:
                continue
            t = msg.get_type()
            if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and int(msg.seq) == 0:
                if t == "MISSION_REQUEST_INT":
                    args = [
                        self.master.target_system, self.master.target_component,
                        0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1,
                        0.0, float(acceptance_radius_m), 0.0, float("nan"),
                        int(round(lat * 1e7)), int(round(lon * 1e7)), float(alt_m),
                    ]
                    try:
                        self.master.mav.mission_item_int_send(*args, mtype)
                    except TypeError:
                        self.master.mav.mission_item_int_send(*args)
                else:
                    args = [
                        self.master.target_system, self.master.target_component,
                        0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1,
                        0.0, float(acceptance_radius_m), 0.0, float("nan"),
                        float(lat), float(lon), float(alt_m),
                    ]
                    try:
                        self.master.mav.mission_item_send(*args, mtype)
                    except TypeError:
                        self.master.mav.mission_item_send(*args)
                sent = True
                continue
            if t == "MISSION_ACK":
                ack_type = int(getattr(msg, "type", -1))
                if sent and ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("[MISSION] single AUTO waypoint uploaded and ACKed")
                    return
                raise FlightTestError(f"mission upload rejected, ACK type={ack_type}")
        raise FlightTestError("mission upload timed out")

    def start_auto_and_wait(self, timeout=30.0, lidar_loss_abort_s=1.0):
        self.reached_seq = None
        self.set_mode("AUTO")
        try:
            self.master.mav.mission_set_current_send(self.master.target_system, self.master.target_component, 0)
        except Exception:
            pass
        deadline = time.monotonic() + timeout
        lidar_loss_since = None
        while time.monotonic() < deadline:
            self.poll(0.15)
            now = time.monotonic()
            if not self.heartbeat_ok():
                raise FlightTestError("heartbeat lost during AUTO")
            if self.gps_fix is not None and self.gps_fix < 3:
                raise FlightTestError(f"GPS fix degraded during AUTO (fix={self.gps_fix})")
            if self.lidar_ok():
                lidar_loss_since = None
            else:
                lidar_loss_since = lidar_loss_since or now
                if now - lidar_loss_since > float(lidar_loss_abort_s):
                    raise FlightTestError("LiDAR invalid/stale during low-altitude AUTO")
            if self.reached_seq == 0:
                print("[AUTO] waypoint reached")
                return
        raise FlightTestError("AUTO waypoint timeout")
