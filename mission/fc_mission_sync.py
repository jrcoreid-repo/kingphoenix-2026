from __future__ import annotations

import math
import queue
import threading
import time

from pymavlink import mavutil


class MissionSyncError(RuntimeError):
    pass


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(float(lat1))
    p2 = math.radians(float(lat2))
    dp = math.radians(float(lat2) - float(lat1))
    dl = math.radians(float(lon2) - float(lon1))
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


class FCMissionSync:
    """Serialized FC mission-protocol client fed by the dedicated RX worker."""

    def __init__(self, tx, state, protocol_queue, cfg: dict, log=print):
        self.tx = tx
        self.state = state
        self.q = protocol_queue
        self.cfg = dict(cfg or {})
        self.log = log
        self.lock = threading.RLock()

    def _drain(self):
        while True:
            try:
                self.q.get_nowait()
            except queue.Empty:
                return

    def _wait(self, accepted: set[str], timeout_s: float):
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            try:
                msg = self.q.get(timeout=min(0.20, remaining))
            except queue.Empty:
                continue
            if msg is not None and msg.get_type() in accepted:
                return msg
        raise MissionSyncError(f"mission protocol timeout waiting for {sorted(accepted)}")

    def route_from_waypoints(self, waypoints, strategy: str, full_cfg: dict) -> list[dict]:
        mission_cfg = full_cfg["mission"]
        wp_cfg = full_cfg["waypoint"]
        total = int(mission_cfg["total_waypoints"])
        final_wp = int(mission_cfg["final_wp"])
        default_alt = float(wp_cfg["default_alt_m"])
        final_alt = float(wp_cfg["final_wp_alt_m"])
        strategy = str(strategy).upper()
        route = []
        for n in range(1, total + 1):
            item = waypoints.get(strategy, n)
            lat = item.get("lat")
            lon = item.get("lon")
            if lat is None or lon is None:
                raise MissionSyncError(f"{strategy}/WP{n} is empty")
            lat = float(lat)
            lon = float(lon)
            if not math.isfinite(lat) or not math.isfinite(lon):
                raise MissionSyncError(f"{strategy}/WP{n} invalid coordinates")
            if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
                raise MissionSyncError(f"{strategy}/WP{n} invalid lat/lon")
            alt = final_alt if n == final_wp else default_alt
            route.append({"wp": n, "label": f"{strategy}/WP{n}", "lat": lat, "lon": lon, "alt_m": alt})
        return route

    @staticmethod
    def _decode_item(msg) -> dict:
        t = msg.get_type()
        if t == "MISSION_ITEM_INT":
            lat = float(msg.x) / 1e7
            lon = float(msg.y) / 1e7
        else:
            lat = float(msg.x)
            lon = float(msg.y)
        return {
            "seq": int(msg.seq),
            "command": int(msg.command),
            "frame": int(msg.frame),
            "lat": lat,
            "lon": lon,
            "alt_m": float(msg.z),
        }

    def download(self) -> list[dict]:
        timeout = float(self.cfg.get("read_timeout_s", 5.0))
        with self.lock:
            self._drain()
            if not self.tx.mission_request_list():
                raise MissionSyncError("MISSION_REQUEST_LIST TX failed")
            count_msg = self._wait({"MISSION_COUNT"}, timeout)
            count = int(count_msg.count)
            items = []
            for seq in range(count):
                if not self.tx.mission_request_int(seq):
                    raise MissionSyncError(f"MISSION_REQUEST_INT seq={seq} TX failed")
                deadline = time.monotonic() + timeout
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise MissionSyncError(f"mission item seq={seq} download timeout")
                    msg = self._wait({"MISSION_ITEM_INT", "MISSION_ITEM"}, remaining)
                    if int(getattr(msg, "seq", -1)) == seq:
                        items.append(self._decode_item(msg))
                        break
            return items

    def compare(self, route: list[dict], actual: list[dict]) -> tuple[bool, str]:
        pos_tol = float(self.cfg.get("position_tolerance_m", 0.30))
        alt_tol = float(self.cfg.get("altitude_tolerance_m", 0.05))
        expected_count = len(route) + 1
        if len(actual) != expected_count:
            return False, f"count {len(actual)} != {expected_count}"
        for expected in route:
            seq = int(expected["wp"])
            got = actual[seq]
            if int(got["command"]) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                return False, f"seq{seq} command mismatch"
            pos_err = distance_m(expected["lat"], expected["lon"], got["lat"], got["lon"])
            alt_err = abs(float(expected["alt_m"]) - float(got["alt_m"]))
            if pos_err > pos_tol:
                return False, f"WP{seq} position error {pos_err:.3f}m"
            if alt_err > alt_tol:
                return False, f"WP{seq} altitude error {alt_err:.3f}m"
        return True, "MATCH"

    def upload(self, route: list[dict]):
        timeout = float(self.cfg.get("upload_timeout_s", 12.0))
        v = self.state.vehicle
        if v.armed:
            raise MissionSyncError("mission upload blocked while vehicle ARMED")
        if v.global_lat_deg is None or v.global_lon_deg is None:
            raise MissionSyncError("GLOBAL_POSITION_INT unavailable for HOME slot")
        home_lat = float(v.global_lat_deg)
        home_lon = float(v.global_lon_deg)
        count = len(route) + 1

        with self.lock:
            self._drain()
            if self.tx.mission_clear_all():
                try:
                    self._wait({"MISSION_ACK"}, 1.0)
                except MissionSyncError:
                    pass
            self._drain()
            if not self.tx.mission_count(count):
                raise MissionSyncError("MISSION_COUNT TX failed")
            sent = set()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                msg = self._wait(
                    {"MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"},
                    max(0.05, deadline - time.monotonic()),
                )
                t = msg.get_type()
                if t == "MISSION_ACK":
                    ack_type = int(getattr(msg, "type", -1))
                    if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        raise MissionSyncError(f"FC mission rejected ACK={ack_type}")
                    if len(sent) != count:
                        raise MissionSyncError(f"MISSION_ACK before all items sent ({len(sent)}/{count})")
                    return

                seq = int(msg.seq)
                if not 0 <= seq < count:
                    raise MissionSyncError(f"FC requested invalid seq={seq}")
                if seq == 0:
                    lat, lon, alt_m, acceptance = home_lat, home_lon, 0.0, 0.0
                else:
                    item = route[seq - 1]
                    lat = float(item["lat"])
                    lon = float(item["lon"])
                    alt_m = float(item["alt_m"])
                    acceptance = float(self.cfg.get("acceptance_radius_m", 0.50))
                if not self.tx.mission_item_int(
                    seq=seq,
                    lat=lat,
                    lon=lon,
                    alt_m=alt_m,
                    acceptance_radius_m=acceptance,
                    is_home=(seq == 0),
                ):
                    raise MissionSyncError(f"MISSION_ITEM_INT seq={seq} TX failed")
                sent.add(seq)
            raise MissionSyncError(f"mission upload timeout ({len(sent)}/{count})")

    def ensure(self, waypoints, strategy: str, full_cfg: dict) -> tuple[bool, str]:
        route = self.route_from_waypoints(waypoints, strategy, full_cfg)
        actual = self.download()
        match, reason = self.compare(route, actual)
        if match:
            self.log(f"[MISSION] FC mission {strategy} MATCH | upload skipped")
            return False, "MATCH"
        self.log(f"[MISSION] FC mission {strategy} MISMATCH ({reason}) -> upload")
        self.upload(route)
        if bool(self.cfg.get("verify_after_upload", True)):
            verify = self.download()
            ok, verify_reason = self.compare(route, verify)
            if not ok:
                raise MissionSyncError(f"readback verify failed: {verify_reason}")
        self.log(f"[MISSION] FC mission {strategy} uploaded + VERIFIED")
        return True, reason

    def set_current(self, seq: int):
        timeout = float(self.cfg.get("set_current_timeout_s", 2.0))
        seq = int(seq)
        self.state.vehicle.current_mission_item = None
        if not self.tx.mission_set_current(seq):
            raise MissionSyncError(f"MISSION_SET_CURRENT seq={seq} TX failed")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.state.vehicle.current_mission_item == seq:
                return
            time.sleep(0.02)
        raise MissionSyncError(f"MISSION_SET_CURRENT seq={seq} not confirmed")
