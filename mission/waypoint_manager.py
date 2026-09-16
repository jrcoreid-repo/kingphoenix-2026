from config import load_json, save_json_atomic


class WaypointManager:
    """Store field-captured waypoints as global latitude/longitude.

    GCS SAVE WAYPOINT captures the FC/EKF GLOBAL_POSITION_INT latitude and
    longitude directly.  This makes the stored field map independent of the
    current LOCAL_POSITION_NED origin and safe to reuse after FC/Pi reboot.

    Mission altitude is intentionally not captured from instantaneous vehicle
    altitude; every saved waypoint gets the configured nominal altitude
    (1.2 m by default).
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
        self.max_global_age_s = 1.0

    def unlock(self):
        self.edit_unlocked = True

    def lock(self):
        self.edit_unlocked = False

    def get(self, strategy, wp):
        item = dict(self.data[strategy.upper()][f"WP{int(wp)}"])
        if item.get("alt_m") is None:
            item["alt_m"] = self.nominal_alt_m
        return item

    def save_current(self, strategy, wp):
        if not self.edit_unlocked:
            return False, "EDIT MODE LOCKED"

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

        key = strategy.upper()
        wp_key = f"WP{int(wp)}"
        if key not in self.data or wp_key not in self.data[key]:
            return False, f"UNKNOWN WAYPOINT {key} {wp_key}"

        self.data[key][wp_key] = {
            "lat": lat,
            "lon": lon,
            "alt_m": float(self.nominal_alt_m),
        }
        save_json_atomic("waypoints.json", self.data)
        return True, (
            f"{wp_key} {key} saved "
            f"lat={lat:.7f} lon={lon:.7f} alt={self.nominal_alt_m:.2f}m"
        )
