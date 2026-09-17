#!/usr/bin/env python3
"""Kingphoenix hybrid drop mission V1.

Real-condition field flow:
  stream FRONT+BOTTOM ON continuously
  shared Hailo/model initialized once
  AI route OFF during takeoff/AUTO
  GUIDED LiDAR takeoff -> AUTO WP1
  WP1 reached -> BOTTOM YOLO ON -> QUICK DROP budget
  drop success/timeout -> AI OFF -> AUTO WP2 -> LAND

The drop stage is intentionally tunable from CLI. It does not hunt for
pixel-perfect center: after a short detection lock, raw image ex/ey entering
an acceptable box can release immediately. Default field signs are the
validated X=-1, Y=-1 corrections for the current aircraft/camera mounting.

Normal mission completion lands at WP2. Abort/exception paths ZERO XY and do
not force a software LAND because the vehicle may be above the drop container;
the safety pilot takes over.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.event_bus import EventBus
from core.shared_state import SharedState
from mission.waypoint_manager import WaypointManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_bottom_drop_aggressive_headless import (
    require_word,
    set_servo_pair,
    verify_servo_functions,
)
from tools.run_bottom_target_pd_headless import HeadlessLoopbackGcs
from tools.run_bottom_target_pd_tuning import (
    bottom_config,
    clamp,
    deadband_remap,
    response_curve,
    send_body_xy,
    slew,
    vector_cap,
    zero_xy,
)
from tools.run_hybrid_mission_test import (
    FC_WP1_SEQ,
    FC_WP2_SEQ,
    check_flight_health,
    check_lidar_guard,
    distance_m,
    fly_auto_to_wp,
    load_waypoint,
    upload_two_wp_route,
    verify_two_wp_route,
)


class PilotTakeover(RuntimeError):
    pass


def parse_args():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description=(
            "Hybrid drop V1: LiDAR takeoff -> AUTO WP1 -> GUIDED bottom quick-drop "
            "-> AUTO WP2 -> LAND"
        )
    )

    # Mission / AUTO.
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp1", type=int, default=1)
    ap.add_argument("--wp2", type=int, default=2)
    ap.add_argument("--accept-radius", type=float, default=0.50)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--wp-timeout", type=float, default=60.0)
    ap.add_argument("--preload-only", action="store_true")

    # LiDAR takeoff: field-validated profiled baseline.
    ap.add_argument("--alt", type=float, default=nominal_alt)
    ap.add_argument("--climb-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.25)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)

    # QUICK DROP budget / trigger. These are the most important tuning knobs.
    ap.add_argument(
        "--drop-timeout",
        type=float,
        default=2.0,
        help="total WP1-reached -> drop-stage budget in seconds",
    )
    ap.add_argument(
        "--drop-error",
        type=float,
        default=0.15,
        help="release when abs(raw ex) AND abs(raw ey) are both <= this value",
    )
    ap.add_argument(
        "--drop-frames",
        type=int,
        default=1,
        help="consecutive in-box frames required after target lock",
    )
    ap.add_argument(
        "--lock-frames",
        type=int,
        default=2,
        help="valid detection frames required before quick-drop is armed",
    )
    ap.add_argument("--confidence", type=float, default=0.50)

    # Bottom XY PD / movement-zone tuning.
    ap.add_argument("--control-hz", type=float, default=20.0)
    ap.add_argument("--kp-x", type=float, default=1.2)
    ap.add_argument("--kd-x", type=float, default=0.12)
    ap.add_argument("--kp-y", type=float, default=1.2)
    ap.add_argument("--kd-y", type=float, default=0.12)
    ap.add_argument("--xy-slew", type=float, default=1.5)
    ap.add_argument("--far-error", type=float, default=0.30)
    ap.add_argument("--brake-error", type=float, default=0.12)
    ap.add_argument("--far-cap", type=float, default=0.40)
    ap.add_argument("--brake-cap", type=float, default=0.25)
    ap.add_argument("--final-cap", type=float, default=0.12)
    ap.add_argument("--deadband-x", type=float, default=0.04)
    ap.add_argument("--deadband-y", type=float, default=0.04)
    ap.add_argument("--filter-alpha", type=float, default=0.40)
    ap.add_argument("--d-alpha", type=float, default=0.25)
    ap.add_argument("--lost-timeout", type=float, default=0.15)
    ap.add_argument("--x-sign", type=int, choices=[-1, 1], default=-1)
    ap.add_argument("--y-sign", type=int, choices=[-1, 1], default=-1)

    # LiDAR guard during AUTO/GUIDED drop.
    ap.add_argument("--guard-floor", type=float, default=0.80)
    ap.add_argument("--guard-ceiling", type=float, default=1.45)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)

    # Drop servos.
    ap.add_argument("--servo-a", type=int, default=7)
    ap.add_argument("--servo-b", type=int, default=8)
    ap.add_argument("--open-pwm", type=int, default=1100)
    ap.add_argument("--close-pwm", type=int, default=2000)
    ap.add_argument("--servo-settle", type=float, default=0.40)
    ap.add_argument("--skip-servo-function-check", action="store_true")

    # Streaming. Default = real Windows GCS; --headless loops streams locally.
    ap.add_argument(
        "--headless",
        action="store_true",
        help="do not require Windows GCS; loop FRONT/BOTTOM streams to localhost",
    )
    ap.add_argument("--stream-wait", type=float, default=60.0)
    return ap.parse_args()


def validate(a):
    if a.wp1 < 1 or a.wp2 < 1 or a.wp1 == a.wp2:
        raise SystemExit("--wp1 and --wp2 must be different positive waypoint numbers")
    if not 0.10 <= float(a.accept_radius) <= 5.0:
        raise SystemExit("--accept-radius must be 0.10..5.00m")
    if not 0.50 <= float(a.alt) <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if not 0.10 <= float(a.climb_rate) <= 0.80:
        raise SystemExit("--climb-rate must be 0.10..0.80m/s")
    if not 0.08 <= float(a.near_rate) <= float(a.climb_rate):
        raise SystemExit("--near-rate must be 0.08m/s..--climb-rate")
    if not 0.05 <= float(a.taper_zone) <= 0.60:
        raise SystemExit("--taper-zone must be 0.05..0.60m")
    if a.takeoff_ceiling <= a.alt:
        raise SystemExit("--takeoff-ceiling must be above --alt")
    if a.auto_floor >= a.auto_ceiling:
        raise SystemExit("--auto-floor must be below --auto-ceiling")

    if not 0.50 <= float(a.drop_timeout) <= 5.0:
        raise SystemExit("--drop-timeout must be 0.50..5.00s")
    if not 0.01 <= float(a.drop_error) <= 0.30:
        raise SystemExit("--drop-error must be 0.01..0.30")
    if not 1 <= int(a.drop_frames) <= 20:
        raise SystemExit("--drop-frames must be 1..20")
    if not 1 <= int(a.lock_frames) <= 20:
        raise SystemExit("--lock-frames must be 1..20")
    if not 0.25 <= float(a.confidence) <= 0.95:
        raise SystemExit("--confidence must be 0.25..0.95")

    if not 10 <= float(a.control_hz) <= 30:
        raise SystemExit("--control-hz must be 10..30Hz")
    if not 0.05 <= a.kp_x <= 5 or not 0.05 <= a.kp_y <= 5:
        raise SystemExit("Kp must be 0.05..5")
    if not 0 <= a.kd_x <= 2 or not 0 <= a.kd_y <= 2:
        raise SystemExit("Kd must be 0..2")
    if not 0.2 <= a.xy_slew <= 12:
        raise SystemExit("--xy-slew must be 0.2..12m/s^2")
    if not 0.05 < a.brake_error < a.far_error <= 1.0:
        raise SystemExit("require 0.05<brake-error<far-error<=1.0")
    if not 0.05 <= a.final_cap <= a.brake_cap <= a.far_cap <= 2.5:
        raise SystemExit("require final-cap<=brake-cap<=far-cap<=2.5")
    if not 0.01 <= a.deadband_x <= 0.20 or not 0.01 <= a.deadband_y <= 0.20:
        raise SystemExit("deadband must be 0.01..0.20")
    if not 0.05 <= a.filter_alpha <= 1.0 or not 0.05 <= a.d_alpha <= 1.0:
        raise SystemExit("filter alpha must be 0.05..1.0")
    if not 0.0 <= a.lost_timeout <= 1.0:
        raise SystemExit("--lost-timeout must be 0..1s")

    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 2.0:
        raise SystemExit("invalid LiDAR guard")
    if not 1 <= a.servo_a <= 32 or not 1 <= a.servo_b <= 32:
        raise SystemExit("servo channels must be 1..32")
    if a.servo_a == a.servo_b:
        raise SystemExit("--servo-a and --servo-b must differ")
    if not 900 <= a.open_pwm <= 2100 or not 900 <= a.close_pwm <= 2100:
        raise SystemExit("servo PWM must be 900..2100")
    if not 0.05 <= a.servo_settle <= 3.0:
        raise SystemExit("--servo-settle must be 0.05..3.0s")


def wait_streams(state: SharedState, timeout_s: float):
    print("[STREAM] waiting for FRONT + BOTTOM camera streams ...")
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        with state.lock:
            connected = bool(state.system.get("gcs_connected", False))
            front = dict(state.vision.get("front") or {})
            bottom = dict(state.vision.get("bottom") or {})
        front_live = str(front.get("video_state", "")).upper() == "LIVE"
        bottom_live = str(bottom.get("video_state", "")).upper() == "LIVE"
        if connected and front_live and bottom_live:
            print(
                f"[STREAM] FRONT LIVE AI={front.get('ai_state')} | "
                f"BOTTOM LIVE AI={bottom.get('ai_state')}"
            )
            return
        time.sleep(0.10)
    raise FlightTestError("FRONT/BOTTOM stream not ready before timeout")


def activate_bottom_ai(vision: VisionStreamSession, detector_class_id: int):
    route = vision.set_ai_route(
        "bottom",
        model_mode="YOLO26_ONLY",
        task="DROP_ALIGNMENT",
        detector_class_id=int(detector_class_id),
        source="HYBRID_DROP_V1",
    )
    print(
        f"[AI] BOTTOM ON | mode={route.model_mode} task={route.task} "
        f"class={route.detector_class_id} gen={route.generation}"
    )


def disable_ai(vision: VisionStreamSession, reason: str):
    route = vision.disable_ai(source=reason)
    print(f"[AI] OFF | source={route.source} gen={route.generation}")


def run_quick_drop(test, state, vision, a, detector_class_id: int, drop_start: float):
    """Run the <=drop-timeout GUIDED vision stage and optionally release payload."""
    activate_bottom_ai(vision, detector_class_id)

    # AI starts processing immediately while FC transitions AUTO -> GUIDED.
    test.set_mode("GUIDED")
    zero_xy(test)

    deadline = drop_start + float(a.drop_timeout)
    period = 1.0 / float(a.control_hz)
    next_tx = 0.0
    last_print = 0.0
    last_frame = None
    valid_streak = 0
    drop_streak = 0
    lost_since = None

    filt_bx = filt_by = None
    prev_ebx = prev_eby = None
    prev_t = None
    dfx = dfy = 0.0
    vx_cmd = vy_cmd = 0.0
    outside_since = lidar_loss_since = None
    phase = "SEARCH"

    first_valid_elapsed = None

    while time.monotonic() < deadline:
        test.poll(0.02)
        check_flight_health(test)
        if test.mode != "GUIDED":
            try:
                zero_xy(test)
            except Exception:
                pass
            raise PilotTakeover(f"mode changed during drop phase to {test.mode}")

        outside_since, lidar_loss_since = check_lidar_guard(
            test,
            floor_m=float(a.guard_floor),
            ceiling_m=float(a.guard_ceiling),
            outside_since=outside_since,
            lidar_loss_since=lidar_loss_since,
            max_outside_s=float(a.guard_outside_s),
            max_stale_s=float(a.lidar_stale_s),
        )

        now = time.monotonic()
        with state.lock:
            b = dict(state.vision.get("bottom") or {})

        conf = b.get("confidence")
        ex = b.get("ex")
        ey = b.get("ey")
        frame_id = b.get("frame_id")
        ts_ns = int(b.get("frame_timestamp_ns") or 0)
        age = float("inf") if ts_ns <= 0 else max(
            0.0, (time.monotonic_ns() - ts_ns) / 1e9
        )
        valid = bool(
            b.get("vision_valid")
            and conf is not None
            and float(conf) >= float(a.confidence)
            and ex is not None
            and ey is not None
            and age <= 0.20
        )

        if valid and frame_id != last_frame:
            last_frame = frame_id
            valid_streak += 1
            lost_since = None
            if first_valid_elapsed is None:
                first_valid_elapsed = now - drop_start
                print(f"[DROP TIMER] first valid detection = {first_valid_elapsed:.3f}s")

            raw_ex = float(ex)
            raw_ey = float(ey)

            # Quick-drop gate uses raw image errors, intentionally before the
            # filtered PD path. No command-speed gate is used.
            in_drop_box = (
                abs(raw_ex) <= float(a.drop_error)
                and abs(raw_ey) <= float(a.drop_error)
            )
            if valid_streak >= int(a.lock_frames) and in_drop_box:
                drop_streak += 1
            else:
                drop_streak = 0

            if (
                valid_streak >= int(a.lock_frames)
                and drop_streak >= int(a.drop_frames)
            ):
                gate_elapsed = now - drop_start
                print(
                    f"[DROP GATE] t={gate_elapsed:.3f}s rawEX={raw_ex:+.3f} "
                    f"rawEY={raw_ey:+.3f} lock={valid_streak}"
                )
                zero_xy(test)
                vx_cmd = vy_cmd = 0.0
                servo_cmd_elapsed = time.monotonic() - drop_start
                print(f"[DROP TIMER] SERVO OPEN command @ {servo_cmd_elapsed:.3f}s")
                set_servo_pair(
                    test,
                    a.servo_a,
                    a.servo_b,
                    a.open_pwm,
                    "DROP OPEN",
                    a.servo_settle,
                )
                complete_elapsed = time.monotonic() - drop_start
                print(f"[DROP TIMER] DROP COMPLETE @ {complete_elapsed:.3f}s")
                return {
                    "dropped": True,
                    "servo_cmd_elapsed": servo_cmd_elapsed,
                    "complete_elapsed": complete_elapsed,
                    "first_valid_elapsed": first_valid_elapsed,
                    "ex": raw_ex,
                    "ey": raw_ey,
                }

            # Field-validated sign correction. With defaults X=-1/Y=-1,
            # effective BODY response matches the latest physical flight tests.
            bx = clamp(-raw_ex * int(a.x_sign), -1.0, 1.0)
            by = clamp(-raw_ey * int(a.y_sign), -1.0, 1.0)

            if filt_bx is None:
                filt_bx, filt_by = bx, by
            else:
                alpha = float(a.filter_alpha)
                filt_bx = alpha * bx + (1.0 - alpha) * filt_bx
                filt_by = alpha * by + (1.0 - alpha) * filt_by

            ebx = response_curve(
                deadband_remap(filt_bx, float(a.deadband_x)), "linear", 0.0
            )
            eby = response_curve(
                deadband_remap(filt_by, float(a.deadband_y)), "linear", 0.0
            )

            if prev_t is not None and prev_ebx is not None:
                dtv = max(1e-3, now - prev_t)
                raw_dx = (ebx - prev_ebx) / dtv
                raw_dy = (eby - prev_eby) / dtv
                da = float(a.d_alpha)
                dfx = da * raw_dx + (1.0 - da) * dfx
                dfy = da * raw_dy + (1.0 - da) * dfy
            else:
                dfx = dfy = 0.0
            prev_ebx, prev_eby, prev_t = ebx, eby, now

            if valid_streak >= int(a.lock_frames):
                radial = math.hypot(float(filt_bx), float(filt_by))
                if radial > float(a.far_error):
                    cap = float(a.far_cap)
                    phase = "CHASE"
                elif radial > float(a.brake_error):
                    cap = float(a.brake_cap)
                    phase = "BRAKE"
                else:
                    cap = float(a.final_cap)
                    phase = "FINAL"

                target_vx = float(a.kp_x) * ebx + float(a.kd_x) * dfx
                target_vy = float(a.kp_y) * eby + float(a.kd_y) * dfy
                target_vx, target_vy = vector_cap(target_vx, target_vy, cap)
                vx_cmd = slew(vx_cmd, target_vx, float(a.xy_slew), period)
                vy_cmd = slew(vy_cmd, target_vy, float(a.xy_slew), period)
            else:
                phase = "LOCK"
                vx_cmd = vy_cmd = 0.0

        elif not valid:
            valid_streak = 0
            drop_streak = 0
            lost_since = lost_since or now
            vx_cmd = vy_cmd = 0.0
            phase = "TARGET LOST"
            zero_xy(test)
            if now - lost_since >= float(a.lost_timeout):
                filt_bx = filt_by = None
                prev_ebx = prev_eby = prev_t = None
                dfx = dfy = 0.0

        if now >= next_tx:
            send_body_xy(test, vx_cmd, vy_cmd)
            next_tx = now + period

        if now - last_print >= 0.20:
            ex_t = "--" if ex is None else f"{float(ex):+.3f}"
            ey_t = "--" if ey is None else f"{float(ey):+.3f}"
            conf_t = "--" if conf is None else f"{float(conf):.2f}"
            remaining = max(0.0, deadline - now)
            print(
                f"[DROPXY] {phase:11s} t={now-drop_start:.2f}s rem={remaining:.2f}s "
                f"conf={conf_t} ex={ex_t} ey={ey_t} | "
                f"VX={vx_cmd:+.2f} VY={vy_cmd:+.2f} "
                f"lock={valid_streak} drop={drop_streak}/{a.drop_frames}"
            )
            last_print = now

        time.sleep(0.003)

    zero_xy(test)
    elapsed = time.monotonic() - drop_start
    print(f"[DROP TIMEOUT] no release after {elapsed:.3f}s -> servo remains CLOSED")
    return {
        "dropped": False,
        "servo_cmd_elapsed": None,
        "complete_elapsed": elapsed,
        "first_valid_elapsed": first_valid_elapsed,
        "ex": None,
        "ey": None,
    }


def main() -> int:
    a = parse_args()
    validate(a)

    try:
        wp1 = load_waypoint(a.strategy, a.wp1)
        wp2 = load_waypoint(a.strategy, a.wp2)
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    route_distance = distance_m(wp1["lat"], wp1["lon"], wp2["lat"], wp2["lon"])
    if route_distance < 0.50:
        print(f"[BLOCKED] WP1 -> WP2 only {route_distance:.2f}m; need >=0.50m")
        return 2

    cfg = bottom_config(float(a.confidence))
    detector_class_id = int(
        cfg.get("ai", {}).get("bottom", {}).get("detector_class_id", 0)
    )

    stop_event = threading.Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)
    gcs = (
        HeadlessLoopbackGcs(state, events, waypoints, stop_event, command_manager=None)
        if a.headless
        else GcsServer(state, events, waypoints, stop_event, command_manager=None)
    )
    vision = VisionStreamSession(cfg, state, stop_event)

    # Engine/model is still initialized by vision.start(), but inference is OFF
    # until the mission explicitly activates the bottom route at WP1.
    disable_ai(vision, "HYBRID_INIT")

    test = DirectFlightTest()
    flight_open = False
    drop_phase_active = False
    servo_prepared = False
    result = None

    print("============================================================")
    print(" KINGPHOENIX HYBRID DROP MISSION V1")
    print("============================================================")
    print(
        f"[PLAN] LiDAR TAKEOFF {a.alt:.2f}m -> AUTO {wp1['label']} -> "
        f"GUIDED QUICK DROP <= {a.drop_timeout:.2f}s -> AUTO {wp2['label']} -> LAND"
    )
    print("[AI SCHEDULER] startup/model preload ON | inference OFF during TAKEOFF/AUTO")
    print("[AI SCHEDULER] BOTTOM YOLO ON only during QUICK DROP; OFF again before WP2")
    print("[STREAM] FRONT + BOTTOM remain ON for the whole mission")
    print(
        f"[DROP] raw |ex|,|ey| <= {a.drop_error:.2f} | lock={a.lock_frames} "
        f"frame(s) | in-box={a.drop_frames} frame(s)"
    )
    print(
        f"[PD] X={a.kp_x:.2f}/{a.kd_x:.2f} Y={a.kp_y:.2f}/{a.kd_y:.2f} "
        f"slew={a.xy_slew:.2f} | caps {a.far_cap:.2f}/{a.brake_cap:.2f}/{a.final_cap:.2f}"
    )
    print(f"[FIELD SIGN] X={a.x_sign:+d} Y={a.y_sign:+d}")
    print(
        f"[SERVO] CH{a.servo_a}+CH{a.servo_b} OPEN={a.open_pwm}us "
        f"CLOSE={a.close_pwm}us"
    )
    print("[ABORT POLICY] XY ZERO + RC TAKEOVER; no forced software LAND above drop zone")
    print("------------------------------------------------------------")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("shared Hailo YOLO26 did not become READY")
        wait_streams(state, float(a.stream_wait))

        test.open()
        flight_open = True
        test.collect_preflight(require_auto=True)
        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")

        cur_lat, cur_lon, prearm_rel = test.global_pos
        for wp in (wp1, wp2):
            d = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
            if d > 100.0:
                raise FlightTestError(f"{wp['label']} is {d:.1f}m away; test limit 100m")
            print(
                f"[WP] {wp['label']} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={d:.2f}m relAlt={wp['alt_m']:.2f}m"
            )
        print(f"[PREARM] FCrelAlt={prearm_rel:.2f}m DIAGNOSTIC ONLY")

        upload_two_wp_route(
            test,
            wp1,
            wp2,
            acceptance_radius_m=float(a.accept_radius),
        )
        verify_two_wp_route(test, wp1, wp2)
        print("[SAFE] HOME -> WP1 -> WP2 verified while DISARMED")

        if a.preload_only:
            print("[PRELOAD-ONLY] route/stream/AI preload verified; vehicle remains DISARMED")
            return 0

        if not a.skip_servo_function_check:
            verify_servo_functions(test, (a.servo_a, a.servo_b))
        else:
            print("[SERVO FUNCTION] CHECK SKIPPED by operator")

        print()
        print("[GROUND SERVO PRETEST]")
        require_word(
            f"Type OPEN to command CH{a.servo_a}/CH{a.servo_b} -> {a.open_pwm}us: ",
            "OPEN",
        )
        set_servo_pair(
            test, a.servo_a, a.servo_b, a.open_pwm, "OPEN", a.servo_settle
        )
        require_word("Confirm mechanism OPEN, then type READY to CLOSE: ", "READY")
        set_servo_pair(
            test, a.servo_a, a.servo_b, a.close_pwm, "CLOSE", a.servo_settle
        )
        servo_prepared = True
        require_word("Servos CLOSED. Safety pilot ready. Type GO: ", "GO")

        # TAKEOFF + AUTO WP1 with AI inference OFF. Streams remain live.
        disable_ai(vision, "TAKEOFF_AUTO_WP1")
        profiled_lidar_takeoff(
            test,
            float(a.alt),
            tolerance_m=float(a.tolerance),
            handoff_m=float(a.handoff),
            fast_rate_mps=float(a.climb_rate),
            near_rate_mps=float(a.near_rate),
            taper_zone_m=float(a.taper_zone),
            kick_alt_m=float(a.kick_alt),
            timeout_s=float(a.takeoff_timeout),
            ceiling_m=float(a.takeoff_ceiling),
            zero_flush_s=float(a.zero_flush),
        )

        rel = None if test.global_pos is None else test.global_pos[2]
        print(
            f"[POST-TAKEOFF] LiDAR={test.lidar:.2f}m "
            f"FCrelAlt={rel if rel is not None else float('nan'):.2f}m -> AUTO WP1"
        )
        fly_auto_to_wp(
            test,
            seq=FC_WP1_SEQ,
            label=wp1["label"],
            floor_m=float(a.auto_floor),
            ceiling_m=float(a.auto_ceiling),
            timeout=float(a.wp_timeout),
        )

        # Honest stopwatch starts immediately at WP1 completion and includes
        # AI routing + AUTO->GUIDED transition latency.
        drop_phase_active = True
        drop_start = time.monotonic()
        print("[DROP TIMER] START @ WP1 REACHED")
        result = run_quick_drop(
            test,
            state,
            vision,
            a,
            detector_class_id,
            drop_start,
        )

        zero_xy(test)
        disable_ai(vision, "DROP_DONE" if result["dropped"] else "DROP_TIMEOUT")
        drop_phase_active = False

        if result["dropped"]:
            print(
                f"[DROP RESULT] SUCCESS | servo command={result['servo_cmd_elapsed']:.3f}s "
                f"complete={result['complete_elapsed']:.3f}s"
            )
        else:
            print(
                f"[DROP RESULT] TIMEOUT | budget={a.drop_timeout:.2f}s | "
                "servo stayed CLOSED"
            )

        # Resume exact mission seq2, not an ambiguous AUTO resume.
        print("[MISSION] QUICK DROP phase finished -> AI OFF -> AUTO WP2")
        fly_auto_to_wp(
            test,
            seq=FC_WP2_SEQ,
            label=wp2["label"],
            floor_m=float(a.auto_floor),
            ceiling_m=float(a.auto_ceiling),
            timeout=float(a.wp_timeout),
        )

        test.land_and_wait()
        if result["dropped"]:
            print("[RESULT] HYBRID DROP V1 PASS: TAKEOFF -> WP1 -> DROP -> WP2 -> LAND")
            return 0

        print("[RESULT] HYBRID ROUTE COMPLETE, BUT DROP TIMED OUT")
        return 4

    except PilotTakeover as exc:
        print(f"[PILOT TAKEOVER] {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "PILOT_TAKEOVER")
        except Exception:
            pass
        print("[RC TAKEOVER] no software LAND commanded")
        return 130

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "OPERATOR_INTERRUPT")
        except Exception:
            pass
        print("[RC TAKEOVER] no software LAND commanded")
        return 130

    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "FLIGHT_ABORT")
        except Exception:
            pass
        if flight_open and bool(getattr(test, "armed", False)):
            where = " during DROP" if drop_phase_active else ""
            print(f"[RC TAKEOVER]{where}; software LAND suppressed")
        return 2

    except Exception as exc:
        print(f"[UNEXPECTED] {type(exc).__name__}: {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "UNEXPECTED_ABORT")
        except Exception:
            pass
        if flight_open and bool(getattr(test, "armed", False)):
            print("[RC TAKEOVER] software LAND suppressed")
        return 3

    finally:
        stop_event.set()
        try:
            if flight_open:
                test.close()
        except Exception:
            pass
        try:
            vision.join(timeout=1.0)
        except Exception:
            pass
        try:
            gcs.join(timeout=1.0)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
