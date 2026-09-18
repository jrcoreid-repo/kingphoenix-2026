from __future__ import annotations

import math

from config import load_json, save_json_atomic


class WaypointManager:
    """Persistent LEFT/RIGHT global waypoint store for the full race mission."""

    def __init__(self, state, full_mission_cfg=None):
        self.state = state
        self.data = load_json("waypoints.json")
        self.edit_unlocked = False
        self.edit_strategy = "LEFT"
        cfg = full_mission_cfg or {}
        wp_cfg = cfg.get("waypoint") or {}
        mission_cfg = cfg.get("mission") or {}
        self.nominal_alt_m = float(wp_cfg.get("default_alt_m", 1.0))
        self.final_alt_m = float(wp_cfg.get("final_wp_alt_m", self.nominal_alt_m))
        self.final_wp = int(mission_cfg.get("final_wp", 11))
        self.total_waypoints = int(mission_cfg.get("total_waypoints", 11))
        self.max_global_age_s = 1.0

    def unlock(self, strategy=None):
        self.edit_unlocked = True
        if strategy is not None:
            self.edit_strategy = str(strategy).upper()

    def lock(self, strategy=None):
        if strategy is not None:
            self.edit_strategy = str(strategy).upper()
        self.edit_unlocked = False

    def get(self, strategy, wp):
        strategy = str(strategy).upper()
        wp = int(wp)
        item = dict(self.data[strategy][f"WP{wp}"])
        item["alt_m"] = self.final_alt_m if wp == self.final_wp else self.nominal_alt_m
        return item

    def validate_strategy(self, strategy, total=None):
        strategy = str(strategy).upper()
        total = self.total_waypoints if total is None else int(total)
        if strategy not in self.data:
            return False, f"strategy {strategy} missing in waypoints.json"
        missing = []
        for n in range(1, total + 1):
            item = self.data[strategy].get(f"WP{n}")
            if not isinstance(item, dict):
                missing.append(f"WP{n}")
                continue
            try:
                lat = float(item.get("lat"))
                lon = float(item.get("lon"))
                valid = (
                    math.isfinite(lat)
                    and math.isfinite(lon)
                    and -90.0 <= lat <= 90.0
                    and -180.0 <= lon <= 180.0
                    and not (lat == 0.0 and lon == 0.0)
                )
            except Exception:
                valid = False
            if not valid:
                missing.append(f"WP{n}")
        if missing:
            return False, f"{len(missing)} waypoint(s) missing/invalid: {', '.join(missing)}"
        return True, f"WP1-WP{total} complete"

    def save_current(self, strategy, wp):
        if not self.edit_unlocked:
            return False, "EDIT MODE LOCKED"
        if self.state.vehicle.armed:
            return False, "WAYPOINT SAVE BLOCKED: vehicle ARMED"

        strategy = str(strategy).upper()
        wp = int(wp)
        if strategy not in ("LEFT", "RIGHT"):
            return False, f"UNKNOWN STRATEGY {strategy}"
        if strategy != self.edit_strategy:
            return False, (
                f"EDIT SESSION LOCKED TO {self.edit_strategy}; "
                f"cannot save {strategy}/WP{wp}"
            )
        if not 1 <= wp <= self.total_waypoints:
            return False, f"WAYPOINT OUT OF RANGE 1..{self.total_waypoints}"

        v = self.state.vehicle
        lat = v.global_lat_deg
        lon = v.global_lon_deg
        if lat is None or lon is None:
            return False, "GLOBAL_POSITION_INT lat/lon unavailable"
        age_s = v.global_position_age_s()
        if age_s > self.max_global_age_s:
            return False, f"GLOBAL_POSITION_INT stale ({age_s:.2f}s)"

        lat = float(lat)
        lon = float(lon)
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return False, "GLOBAL_POSITION_INT lat/lon invalid"
        if lat == 0.0 and lon == 0.0:
            return False, "GLOBAL_POSITION_INT is 0,0"

        wp_key = f"WP{wp}"
        if strategy not in self.data:
            self.data[strategy] = {}
        alt = self.final_alt_m if wp == self.final_wp else self.nominal_alt_m
        self.data[strategy][wp_key] = {"lat": lat, "lon": lon, "alt_m": float(alt)}
        self.edit_strategy = strategy
        save_json_atomic("waypoints.json", self.data)
        return True, f"{wp_key} {strategy} saved lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}m"
