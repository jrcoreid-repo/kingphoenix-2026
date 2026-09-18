from __future__ import annotations

import math

from config import load_json


class FullMissionConfigError(ValueError):
    pass


def load_full_mission_config() -> dict:
    cfg = load_json("full_mission.json")
    validate_full_mission_config(cfg)
    return cfg


def _finite(name: str, value) -> float:
    try:
        out = float(value)
    except Exception as exc:
        raise FullMissionConfigError(f"{name} must be numeric") from exc
    if not math.isfinite(out):
        raise FullMissionConfigError(f"{name} must be finite")
    return out


def validate_full_mission_config(cfg: dict) -> None:
    if not isinstance(cfg, dict):
        raise FullMissionConfigError("full_mission.json root must be an object")

    mission = cfg.get("mission") or {}
    total = int(mission.get("total_waypoints", 0))
    ai_wp = int(mission.get("ai_enable_wp", 0))
    drop_wp = int(mission.get("drop_wp", 0))
    final_wp = int(mission.get("final_wp", 0))
    if total != 11:
        raise FullMissionConfigError(f"mission.total_waypoints must be 11, got {total}")
    if not (1 <= ai_wp < drop_wp <= total):
        raise FullMissionConfigError("require 1 <= ai_enable_wp < drop_wp <= total_waypoints")
    if final_wp != total:
        raise FullMissionConfigError("mission.final_wp must equal total_waypoints")

    preflight = cfg.get("preflight") or {}
    heartbeat_max_age = _finite(
        "preflight.heartbeat_max_age_s",
        preflight.get("heartbeat_max_age_s", 2.5),
    )
    lidar_max_age = _finite(
        "preflight.lidar_max_age_s",
        preflight.get("lidar_max_age_s", 0.30),
    )
    gps_min_fix = int(preflight.get("gps_min_fix_type", 3))
    if heartbeat_max_age <= 0.2:
        raise FullMissionConfigError("preflight.heartbeat_max_age_s must be >0.2")
    if lidar_max_age <= 0.05:
        raise FullMissionConfigError("preflight.lidar_max_age_s must be >0.05")
    if not 3 <= gps_min_fix <= 6:
        raise FullMissionConfigError("preflight.gps_min_fix_type must be 3..6")
    for key in ("require_gps", "require_lidar", "require_wlan1"):
        if not isinstance(preflight.get(key, True), bool):
            raise FullMissionConfigError(f"preflight.{key} must be boolean")

    takeoff = cfg.get("takeoff") or {}
    target = _finite("takeoff.target_alt_m", takeoff.get("target_alt_m"))
    fast = _finite("takeoff.fast_vz_mps", takeoff.get("fast_vz_mps"))
    near = _finite("takeoff.near_vz_mps", takeoff.get("near_vz_mps"))
    handoff = _finite("takeoff.handoff_m", takeoff.get("handoff_m"))
    tol = _finite("takeoff.tolerance_m", takeoff.get("tolerance_m"))
    ceiling = _finite("takeoff.ceiling_m", takeoff.get("ceiling_m"))
    if not 0.50 <= target <= 3.0:
        raise FullMissionConfigError("takeoff.target_alt_m must be 0.50..3.00")
    if not 0.10 <= fast <= 0.80:
        raise FullMissionConfigError("takeoff.fast_vz_mps must be 0.10..0.80")
    if not 0.08 <= near <= fast:
        raise FullMissionConfigError("takeoff.near_vz_mps must be 0.08..fast_vz_mps")
    if not 0.12 <= handoff < target - tol:
        raise FullMissionConfigError("takeoff.handoff_m must be >=0.12 and below target-tolerance")
    if ceiling <= target:
        raise FullMissionConfigError("takeoff.ceiling_m must be above target_alt_m")

    waypoint = cfg.get("waypoint") or {}
    default_alt = _finite("waypoint.default_alt_m", waypoint.get("default_alt_m"))
    final_alt = _finite("waypoint.final_wp_alt_m", waypoint.get("final_wp_alt_m"))
    if not 0.30 <= default_alt <= 5.0 or not 0.30 <= final_alt <= 5.0:
        raise FullMissionConfigError("waypoint altitudes must be 0.30..5.00m")
    if bool(waypoint.get("override_auto_speed", False)):
        raise FullMissionConfigError("waypoint.override_auto_speed must remain false; AUTO speed belongs to FC parameters")

    vision = cfg.get("vision") or {}
    conf = _finite("vision.confidence_threshold", vision.get("confidence_threshold"))
    frames = int(vision.get("confirm_frames", 0))
    coarse = _finite("vision.coarse_center_error_max", vision.get("coarse_center_error_max"))
    if not 0.01 <= conf <= 0.95:
        raise FullMissionConfigError("vision.confidence_threshold must be 0.01..0.95")
    if not 1 <= frames <= 10:
        raise FullMissionConfigError("vision.confirm_frames must be 1..10")
    if not 0.05 <= coarse <= 1.0:
        raise FullMissionConfigError("vision.coarse_center_error_max must be 0.05..1.0")

    fuzzy = cfg.get("guided_fuzzy") or {}
    center = _finite("guided_fuzzy.center_error", fuzzy.get("center_error"))
    mid = _finite("guided_fuzzy.mid_error", fuzzy.get("mid_error"))
    far = _finite("guided_fuzzy.far_error", fuzzy.get("far_error"))
    vmin = _finite("guided_fuzzy.minimum_speed_mps", fuzzy.get("minimum_speed_mps"))
    vmid = _finite("guided_fuzzy.mid_speed_mps", fuzzy.get("mid_speed_mps"))
    vfar = _finite("guided_fuzzy.far_speed_mps", fuzzy.get("far_speed_mps"))
    if not 0.02 <= center < mid < far <= 1.0:
        raise FullMissionConfigError("guided_fuzzy requires center_error < mid_error < far_error <= 1")
    if not 0.03 <= vmin <= vmid <= vfar <= 2.0:
        raise FullMissionConfigError("guided_fuzzy speeds must satisfy minimum <= mid <= far <= 2m/s")
    floor = _finite("guided_fuzzy.guard_floor_m", fuzzy.get("guard_floor_m"))
    guard_ceiling = _finite("guided_fuzzy.guard_ceiling_m", fuzzy.get("guard_ceiling_m"))
    if not 0.30 <= floor < guard_ceiling <= 3.0:
        raise FullMissionConfigError("invalid guided_fuzzy LiDAR guard")

    servo = cfg.get("servo") or {}
    channels = servo.get("channels") or []
    if not isinstance(channels, list) or not channels:
        raise FullMissionConfigError("servo.channels must contain at least one output")
    if any(int(ch) <= 0 for ch in channels):
        raise FullMissionConfigError("servo.channels must be positive integers")
    open_pwm = int(servo.get("open_pwm", 0))
    close_pwm = int(servo.get("close_pwm", 0))
    if not 800 <= open_pwm <= 2200 or not 800 <= close_pwm <= 2200:
        raise FullMissionConfigError("servo OPEN/CLOSE PWM must be 800..2200us")
    if open_pwm == close_pwm:
        raise FullMissionConfigError("servo OPEN and CLOSE PWM must differ")
    require_closed = servo.get("require_closed_before_start", True)
    if not isinstance(require_closed, bool):
        raise FullMissionConfigError("servo.require_closed_before_start must be boolean")

    landing = cfg.get("landing") or {}
    land_timeout = _finite("landing.timeout_s", landing.get("timeout_s", 45.0))
    if land_timeout < 5.0:
        raise FullMissionConfigError("landing.timeout_s must be >=5.0")

    drop = cfg.get("drop") or {}
    if int(drop.get("armed_from_wp", ai_wp)) != ai_wp:
        raise FullMissionConfigError("drop.armed_from_wp must match mission.ai_enable_wp")
    if int(drop.get("fallback_wp", drop_wp)) != drop_wp:
        raise FullMissionConfigError("drop.fallback_wp must match mission.drop_wp")
    if str(drop.get("servo_default_state", "OPEN")).upper() != "OPEN":
        raise FullMissionConfigError("drop.servo_default_state must remain OPEN")
    if str(drop.get("servo_drop_state", "OPEN")).upper() != "OPEN":
        raise FullMissionConfigError("drop.servo_drop_state must remain OPEN")
    if bool(drop.get("auto_close_after_drop", False)):
        raise FullMissionConfigError("drop.auto_close_after_drop must remain false")

    camera = cfg.get("camera") or {}
    required_camera_flags = (
        "front_on_at_start",
        "bottom_on_at_start",
        "front_stream",
        "bottom_stream",
        "preload_ai_model",
    )
    for key in required_camera_flags:
        if camera.get(key) is not True:
            raise FullMissionConfigError(f"camera.{key} must be true")
    if bool(camera.get("front_ai", False)):
        raise FullMissionConfigError("camera.front_ai must remain false")
    if bool(camera.get("bottom_ai_on_start", False)):
        raise FullMissionConfigError("camera.bottom_ai_on_start must remain false")

    if str(vision.get("camera", "bottom")).lower() != "bottom":
        raise FullMissionConfigError("vision.camera must be bottom")
    if str(vision.get("ai_initial_state", "OFF")).upper() != "OFF":
        raise FullMissionConfigError("vision.ai_initial_state must be OFF")

    if not bool(fuzzy.get("enabled", True)):
        raise FullMissionConfigError("guided_fuzzy.enabled must remain true")

    landing = cfg.get("landing") or {}
    if not bool(landing.get("land_after_final_wp", True)):
        raise FullMissionConfigError("landing.land_after_final_wp must remain true")
    if int(landing.get("final_wp_number", final_wp)) != final_wp:
        raise FullMissionConfigError("landing.final_wp_number must match mission.final_wp")

    sync = cfg.get("mission_sync") or {}
    if not bool(sync.get("compare_before_upload", True)):
        raise FullMissionConfigError("mission_sync.compare_before_upload must remain true")
    if not bool(sync.get("verify_after_upload", True)):
        raise FullMissionConfigError("mission_sync.verify_after_upload must remain true")
    if _finite(
        "mission_sync.acceptance_radius_tolerance_m",
        sync.get("acceptance_radius_tolerance_m", 0.05),
    ) < 0.0:
        raise FullMissionConfigError(
            "mission_sync.acceptance_radius_tolerance_m must be >=0"
        )

    logging = cfg.get("logging") or {}
    if _finite("logging.realtime_summary_interval_s", logging.get("realtime_summary_interval_s")) < 0.5:
        raise FullMissionConfigError("realtime summary interval must be >=0.5s")
    if _finite("logging.system_summary_interval_s", logging.get("system_summary_interval_s")) < 1.0:
        raise FullMissionConfigError("system summary interval must be >=1.0s")
