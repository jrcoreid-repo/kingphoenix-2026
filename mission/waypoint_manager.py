import math
from config import load_json, save_json_atomic


class WaypointManager:
    """Capture global horizontal waypoints via existing GCS SAVE WAYPOINT.

    x/y retained for legacy tools; new AUTO tests use persistent WGS84 lat/lon.
    alt_m remains a nominal altitude relative to the FC home, not LiDAR AGL.
    """

    def __init__(self, state):
        self.state = state
        self.data = load_json("waypoints.json")
        self.edit_unlocked = False
        try:
            defaults = load_json("flight_defaults.json")
        except Exception:
            defaults = {}
        self.nominal_alt_m = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    def unlock(self):
        self.edit_unlocked = True

    def lock(self):
        self.edit_unlocked = False

    def get(self, strategy, wp):
        item = dict(self.data[strategy.upper()][f"WP{int(wp)}"])
        # Backward compatibility for old files while freezing the new contract.
        if item.get("alt_m") is None:
            item["alt_m"] = self.nominal_alt_m
        return item

    def save_current(self, strategy, wp):
        if not self.edit_unlocked:
            return False, "EDIT MODE LOCKED"

        if self.state.system.get("wp_test_active"):
            return False, "WAYPOINT TEST ACTIVE; edits locked"
        v = self.state.vehicle
        if (not v.connected or v.heartbeat_age_s() > 2.5 or
                v.global_position_age_s() > 0.5 or
                v.message_age_s("GPS_RAW_INT") > 1.5 or (v.gps_fix_type or 0) < 3):
            return False, "FRESH GLOBAL POSITION + GPS 3D FIX REQUIRED"
        if any(value is None or not math.isfinite(value) for value in (v.global_lat_deg, v.global_lon_deg)):
            return False, "GLOBAL POSITION unavailable"

        key = strategy.upper()
        wp_key = f"WP{int(wp)}"
        if key not in self.data or wp_key not in self.data[key]:
            return False, f"UNKNOWN WAYPOINT {key} {wp_key}"

        self.data[key][wp_key] = {
            "x": v.x,
            "y": v.y,
            "lat_deg": float(v.global_lat_deg),
            "lon_deg": float(v.global_lon_deg),
            "frame": "GLOBAL_RELATIVE_ALT",
            "alt_m": float(self.nominal_alt_m),
        }
        save_json_atomic("waypoints.json", self.data)
        return True, (
            f"{wp_key} {key} saved lat={v.global_lat_deg:.7f} lon={v.global_lon_deg:.7f} "
            f"alt={self.nominal_alt_m:.2f}m"
        )

