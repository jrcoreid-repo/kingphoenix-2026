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
    if _finite("mission.auto_wp_timeout_s", mission.get("auto_wp_timeout_s", 60.0)) < 5.0:
        raise FullMissionConfigError("mission.auto_wp_timeout_s must be >=5.0")
    if str(mission.get("abort_mode", "RTL")).upper() not in {
        "RTL", "LOITER", "ALT_HOLD", "BRAKE"
    }:
        raise FullMissionConfigError(
            "mission.abort_mode must be RTL, LOITER, ALT_HOLD, or BRAKE"
        )

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
    taper = _finite("takeoff.taper_zone_m", takeoff.get("taper_zone_m"))
    kick_alt = _finite("takeoff.kick_alt_m", takeoff.get("kick_alt_m"))
    zero_flush = _finite("takeoff.zero_flush_s", takeoff.get("zero_flush_s", 0.15))
    timeout = _finite("takeoff.timeout_s", takeoff.get("timeout_s", 15.0))
    settle = _finite("takeoff.arm_settle_s", takeoff.get("arm_settle_s", 0.35))
    stale = _finite("takeoff.lidar_stale_s", takeoff.get("lidar_stale_s", 0.60))
    command_hz = _finite("takeoff.command_hz", takeoff.get("command_hz", 10.0))
    if tol <= 0.0 or tol >= target:
        raise FullMissionConfigError("takeoff.tolerance_m must be >0 and < target")
    if taper <= 0.0 or taper >= target:
        raise FullMissionConfigError("takeoff.taper_zone_m must be >0 and < target")
    if kick_alt < target or kick_alt > 10.0:
        raise FullMissionConfigError("takeoff.kick_alt_m must be target..10.0m")
    if not 0.0 <= zero_flush <= 2.0:
        raise FullMissionConfigError("takeoff.zero_flush_s must be 0..2.0")
    if timeout < 3.0:
        raise FullMissionConfigError("takeoff.timeout_s must be >=3.0")
    if not 0.0 <= settle <= 3.0:
        raise FullMissionConfigError("takeoff.arm_settle_s must be 0..3.0")
    if stale <= 0.05:
        raise FullMissionConfigError("takeoff.lidar_stale_s must be >0.05")
    if not 2.0 <= command_hz <= 30.0:
        raise FullMissionConfigError("takeoff.command_hz must be 2..30Hz")

    waypoint = cfg.get("waypoint") or {}
    default_alt = _finite("waypoint.default_alt_m", waypoint.get("default_alt_m"))
    final_alt = _finite("waypoint.final_wp_alt_m", waypoint.get("final_wp_alt_m"))
    if not 0.30 <= default_alt <= 5.0 or not 0.30 <= final_alt <= 5.0:
        raise FullMissionConfigError("waypoint altitudes must be 0.30..5.00m")
    if bool(waypoint.get("override_auto_speed", False)):
        raise FullMissionConfigError("waypoint.override_auto_speed must remain false; AUTO speed belongs to FC parameters")
    acceptance = _finite(
        "waypoint.acceptance_radius_m",
        waypoint.get("acceptance_radius_m", 0.50),
    )
    if not 0.05 <= acceptance <= 10.0:
        raise FullMissionConfigError("waypoint.acceptance_radius_m must be 0.05..10.0")

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
    if _finite("vision.freshness_s", vision.get("freshness_s", 0.20)) <= 0.02:
        raise FullMissionConfigError("vision.freshness_s must be >0.02")
    if int(vision.get("detector_class_id", 0)) < 0:
        raise FullMissionConfigError("vision.detector_class_id must be >=0")
    if str(vision.get("model_mode", "YOLO26_ONLY")).upper() != "YOLO26_ONLY":
        raise FullMissionConfigError("vision.model_mode must remain YOLO26_ONLY")

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
    fuzzy_hz = _finite("guided_fuzzy.control_hz", fuzzy.get("control_hz", 20.0))
    fuzzy_fresh = _finite(
        "guided_fuzzy.vision_freshness_s",
        fuzzy.get("vision_freshness_s", 0.20),
    )
    center_frames = int(fuzzy.get("center_confirm_frames", 2))
    alpha = _finite("guided_fuzzy.filter_alpha", fuzzy.get("filter_alpha", 0.40))
    slew = _finite("guided_fuzzy.xy_slew_mps2", fuzzy.get("xy_slew_mps2", 1.50))
    lost = _finite("guided_fuzzy.lost_timeout_s", fuzzy.get("lost_timeout_s", 0.15))
    outside = _finite("guided_fuzzy.guard_outside_s", fuzzy.get("guard_outside_s", 0.35))
    fuzzy_lidar_stale = _finite(
        "guided_fuzzy.lidar_stale_s",
        fuzzy.get("lidar_stale_s", 0.60),
    )
    if not 10.0 <= fuzzy_hz <= 30.0:
        raise FullMissionConfigError("guided_fuzzy.control_hz must be 10..30Hz")
    if fuzzy_fresh <= 0.02:
        raise FullMissionConfigError("guided_fuzzy.vision_freshness_s must be >0.02")
    if not 1 <= center_frames <= 10:
        raise FullMissionConfigError("guided_fuzzy.center_confirm_frames must be 1..10")
    if not 0.0 < alpha <= 1.0:
        raise FullMissionConfigError("guided_fuzzy.filter_alpha must be (0,1]")
    if slew <= 0.0:
        raise FullMissionConfigError("guided_fuzzy.xy_slew_mps2 must be >0")
    if lost <= 0.0 or outside <= 0.0 or fuzzy_lidar_stale <= 0.05:
        raise FullMissionConfigError("guided_fuzzy time guards must be positive")
    for axis in ("x_sign", "y_sign"):
        if float(fuzzy.get(axis, 0.0)) not in (-1.0, 1.0):
            raise FullMissionConfigError(f"guided_fuzzy.{axis} must be -1 or +1")
    for axis in ("center_offset_x", "center_offset_y"):
        offset = _finite(f"guided_fuzzy.{axis}", fuzzy.get(axis, 0.0))
        if not -0.25 <= offset <= 0.25:
            raise FullMissionConfigError(f"guided_fuzzy.{axis} must be -0.25..+0.25")

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
    force_drop_s = _finite(
        "drop.force_drop_after_fuzzy_s",
        drop.get("force_drop_after_fuzzy_s", 8.0),
    )
    if not 1.0 <= force_drop_s <= 30.0:
        raise FullMissionConfigError("drop.force_drop_after_fuzzy_s must be 1..30s")

    setup_ai = cfg.get("setup_ai") or {}
    if setup_ai.get("enabled", True) is not True:
        raise FullMissionConfigError("setup_ai.enabled must remain true")
    setup_interval = _finite(
        "setup_ai.switch_interval_s",
        setup_ai.get("switch_interval_s", 10.0),
    )
    if not 1.0 <= setup_interval <= 60.0:
        raise FullMissionConfigError("setup_ai.switch_interval_s must be 1..60s")
    if str(setup_ai.get("model_mode", "YOLO26_ONLY")).upper() != "YOLO26_ONLY":
        raise FullMissionConfigError("setup_ai.model_mode must remain YOLO26_ONLY")
    for camera_name, expected_task, expected_class in (
        ("front", "GATE_NAV", 1),
        ("bottom", "DROP_ALIGNMENT", 0),
    ):
        route = setup_ai.get(camera_name) or {}
        if str(route.get("task", "")).upper() != expected_task:
            raise FullMissionConfigError(
                f"setup_ai.{camera_name}.task must be {expected_task}"
            )
        if int(route.get("detector_class_id", -1)) != expected_class:
            raise FullMissionConfigError(
                f"setup_ai.{camera_name}.detector_class_id must be {expected_class}"
            )

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
    for key in ("position_tolerance_m", "altitude_tolerance_m"):
        if _finite(f"mission_sync.{key}", sync.get(key, 0.0)) < 0.0:
            raise FullMissionConfigError(f"mission_sync.{key} must be >=0")
    for key in ("read_timeout_s", "upload_timeout_s", "set_current_timeout_s"):
        if _finite(f"mission_sync.{key}", sync.get(key, 0.0)) <= 0.0:
            raise FullMissionConfigError(f"mission_sync.{key} must be >0")

    logging = cfg.get("logging") or {}
    if _finite("logging.realtime_summary_interval_s", logging.get("realtime_summary_interval_s")) < 0.5:
        raise FullMissionConfigError("realtime summary interval must be >=0.5s")
    if _finite("logging.system_summary_interval_s", logging.get("system_summary_interval_s")) < 1.0:
        raise FullMissionConfigError("system summary interval must be >=1.0s")
