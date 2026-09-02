from config import load_json, save_json_atomic


class WaypointManager:
    """Store field-captured horizontal waypoints with one nominal altitude.

    GCS SAVE WAYPOINT captures the current LOCAL_POSITION_NED x/y only. The
    mission altitude is intentionally not taken from the vehicle's instantaneous
    z value; every saved waypoint gets the configured nominal altitude (1.2 m
    by default). Tools and the later mission runtime therefore consume the same
    stable JSON contract.
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

        v = self.state.vehicle
        if None in (v.x, v.y):
            return False, "LOCAL_POSITION_NED x/y unavailable"

        key = strategy.upper()
        wp_key = f"WP{int(wp)}"
        if key not in self.data or wp_key not in self.data[key]:
            return False, f"UNKNOWN WAYPOINT {key} {wp_key}"

        self.data[key][wp_key] = {
            "x": float(v.x),
            "y": float(v.y),
            "alt_m": float(self.nominal_alt_m),
        }
        save_json_atomic("waypoints.json", self.data)
        return True, (
            f"{wp_key} {key} saved x={float(v.x):.2f} y={float(v.y):.2f} "
            f"alt={self.nominal_alt_m:.2f}m"
        )
