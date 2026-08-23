from copy import deepcopy


def build_telemetry(state):
    v = state.vehicle
    m = state.mission
    with state.lock:
        pi_health = deepcopy(state.system.get("pi_health") or {})

    return {
        "vehicle": {
            "connected": v.connected,
            "armed": v.armed,
            "mode": v.mode,
            "x": v.x,
            "y": v.y,
            "z": v.z,
            "relative_alt_m": v.relative_alt_m,
            "battery_v": v.battery_v,
            "battery_remaining": v.battery_remaining,
            "gps_satellites": v.gps_satellites,
            "rangefinder_m": v.rangefinder_distance_m,
            "current_mission_item": v.current_mission_item,
        },
        "mission": {
            "status": m.status,
            "step_index": m.step_index,
            "strategy": m.strategy,
            "active_ai_camera": m.active_ai_camera,
        },
        "system": {
            "pi_health": pi_health,
            "pi_health_status": pi_health.get("status", "UNKNOWN"),
        },
    }
