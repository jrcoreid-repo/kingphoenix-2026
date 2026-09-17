#!/usr/bin/env python3
"""Kingphoenix hybrid drop mission V2 with racing launch.

Flow:
  FRONT+BOTTOM streams stay ON; shared Hailo/model is preloaded once.
  AI inference stays OFF during launch/AUTO.
  GUIDED LiDAR launch only to --race-launch-alt (default 0.50m AGL).
  At the launch threshold, hand off immediately to AUTO WP1 while WP1 keeps
  its normal mission altitude (normally 1.20m REL_HOME), so horizontal sprint
  and climb happen together.
  WP1 reached -> BOTTOM YOLO ON -> GUIDED QUICK DROP <= budget.
  Drop success/timeout -> AI OFF -> AUTO WP2 -> LAND.

V1 is intentionally left untouched. This V2 is for the racing-style launch
experiment and keeps all quick-drop tuning parameters available from CLI.
"""

from __future__ import annotations

import argparse
import time

from tools.flight_test_common import DirectFlightTest, FlightTestError, load_json
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_bottom_drop_aggressive_headless import (
    require_word,
    set_servo_pair,
    verify_servo_functions,
)
from tools.run_bottom_target_pd_headless import HeadlessLoopbackGcs
from tools.run_bottom_target_pd_tuning import bottom_config, zero_xy
from tools.run_hybrid_drop_mission_v1 import (
    EventBus,
    GcsServer,
    SharedState,
    VisionStreamSession,
    WaypointManager,
    PilotTakeover,
    disable_ai,
    run_quick_drop,
    wait_streams,
)
from tools.run_hybrid_mission_test import (
    FC_WP1_SEQ,
    FC_WP2_SEQ,
    check_flight_health,
    check_lidar_guard,
    distance_m,
    fly_auto_to_wp,
    load_waypoint,
    set_mission_current,
    upload_two_wp_route,
    verify_two_wp_route,
)


def parse_args():
    defaults = load_json("flight_defaults.json")
    nominal_alt = float(defaults.get("nominal_waypoint_alt_m", 1.2))

    ap = argparse.ArgumentParser(
        description=(
            "Hybrid drop V2 RACE LAUNCH: LiDAR 0.5m handoff -> AUTO WP1 climbing "
            "to mission altitude -> quick drop -> AUTO WP2 -> LAND"
        )
    )

    # Mission/AUTO.
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], default="LEFT")
    ap.add_argument("--wp1", type=int, default=1)
    ap.add_argument("--wp2", type=int, default=2)
    ap.add_argument("--accept-radius", type=float, default=0.50)
    ap.add_argument("--auto-floor", type=float, default=0.65)
    ap.add_argument("--auto-ceiling", type=float, default=1.55)
    ap.add_argument("--wp-timeout", type=float, default=60.0)
    ap.add_argument("--preload-only", action="store_true")

    # Normal waypoint/cruise altitude remains in waypoints.json; --alt is kept
    # as a consistency check/documentation value for the expected race level.
    ap.add_argument("--alt", type=float, default=nominal_alt)

    # Racing launch. The helper target is internally launch_alt+tolerance so
    # its raw-LiDAR completion trigger is exactly launch_alt.
    ap.add_argument(
        "--race-launch-alt",
        type=float,
        default=0.50,
        help="raw LiDAR AGL that triggers immediate AUTO WP1 handoff",
    )
    ap.add_argument("--race-launch-tolerance", type=float, default=0.05)
    ap.add_argument("--race-launch-rate", type=float, default=0.45)
    ap.add_argument("--race-launch-ceiling", type=float, default=0.85)
    ap.add_argument("--race-zero-flush", type=float, default=0.05)
    ap.add_argument(
        "--race-auto-floor",
        type=float,
        default=0.35,
        help="temporary LiDAR floor immediately after 0.5m AUTO handoff",
    )
    ap.add_argument(
        "--race-guard-arm-agl",
        type=float,
        default=0.85,
        help="once LiDAR reaches this AGL, switch to normal --auto-floor guard",
    )
    ap.add_argument(
        "--race-climb-confirm-timeout",
        type=float,
        default=3.0,
        help="AUTO must climb to race-guard-arm-agl within this time",
    )
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-timeout", type=float, default=15.0)

    # QUICK DROP.
    ap.add_argument("--drop-timeout", type=float, default=2.0)
    ap.add_argument("--drop-error", type=float, default=0.15)
    ap.add_argument("--drop-frames", type=int, default=1)
    ap.add_argument("--lock-frames", type=int, default=2)
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

    # LiDAR guard during normal race/drop phases.
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

    # Streaming.
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--stream-wait", type=float, default=60.0)
    return ap.parse_args()


def validate(a):
    if a.wp1 < 1 or a.wp2 < 1 or a.wp1 == a.wp2:
        raise SystemExit("--wp1 and --wp2 must be different positive waypoint numbers")
    if not 0.10 <= a.accept_radius <= 5.0:
        raise SystemExit("--accept-radius must be 0.10..5.00m")
    if not 0.50 <= a.alt <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if a.auto_floor >= a.auto_ceiling:
        raise SystemExit("--auto-floor must be below --auto-ceiling")

    if not 0.45 <= a.race_launch_alt <= 0.80:
        raise SystemExit("--race-launch-alt must be 0.45..0.80m for this experiment")
    if not 0.01 <= a.race_launch_tolerance <= 0.10:
        raise SystemExit("--race-launch-tolerance must be 0.01..0.10m")
    if not 0.10 <= a.race_launch_rate <= 0.80:
        raise SystemExit("--race-launch-rate must be 0.10..0.80m/s")
    synthetic_target = a.race_launch_alt + a.race_launch_tolerance
    if a.race_launch_ceiling <= synthetic_target:
        raise SystemExit("--race-launch-ceiling must exceed launch_alt+tolerance")
    if not 0.0 <= a.race_zero_flush <= 0.20:
        raise SystemExit("--race-zero-flush must be 0..0.20s")
    if not 0.20 <= a.race_auto_floor < a.race_launch_alt:
        raise SystemExit("--race-auto-floor must be below race-launch-alt")
    if not a.race_launch_alt < a.race_guard_arm_agl <= a.auto_ceiling:
        raise SystemExit("race-guard-arm-agl must be above launch alt and <= auto ceiling")
    if not 0.5 <= a.race_climb_confirm_timeout <= 8.0:
        raise SystemExit("--race-climb-confirm-timeout must be 0.5..8.0s")
    if not 0.12 <= a.handoff < a.race_launch_alt:
        raise SystemExit("--handoff must be >=0.12m and below race-launch-alt")

    if not 0.50 <= a.drop_timeout <= 5.0:
        raise SystemExit("--drop-timeout must be 0.50..5.00s")
    if not 0.01 <= a.drop_error <= 0.30:
        raise SystemExit("--drop-error must be 0.01..0.30")
    if not 1 <= a.drop_frames <= 20 or not 1 <= a.lock_frames <= 20:
        raise SystemExit("drop/lock frames must be 1..20")
    if not 0.25 <= a.confidence <= 0.95:
        raise SystemExit("--confidence must be 0.25..0.95")

    if not 10 <= a.control_hz <= 30:
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
        raise SystemExit("invalid normal LiDAR guard")
    if not 1 <= a.servo_a <= 32 or not 1 <= a.servo_b <= 32 or a.servo_a == a.servo_b:
        raise SystemExit("invalid servo channels")
    if not 900 <= a.open_pwm <= 2100 or not 900 <= a.close_pwm <= 2100:
        raise SystemExit("servo PWM must be 900..2100")


def race_lidar_launch(test: DirectFlightTest, a):
    """Launch only to the raw-LiDAR race threshold, then return for AUTO handoff."""
    synthetic_target = float(a.race_launch_alt) + float(a.race_launch_tolerance)
    print(
        f"[RACE-LAUNCH] raw LiDAR trigger={a.race_launch_alt:.2f}m; "
        f"AUTO WP1 immediately after trigger; WP altitude remains mission-defined"
    )
    profiled_lidar_takeoff(
        test,
        synthetic_target,
        tolerance_m=float(a.race_launch_tolerance),
        handoff_m=float(a.handoff),
        fast_rate_mps=float(a.race_launch_rate),
        near_rate_mps=float(a.race_launch_rate),
        taper_zone_m=0.05,
        kick_alt_m=float(a.kick_alt),
        timeout_s=float(a.takeoff_timeout),
        ceiling_m=float(a.race_launch_ceiling),
        zero_flush_s=float(a.race_zero_flush),
    )


def fly_auto_to_wp_race_launch(test: DirectFlightTest, *, seq: int, label: str, a):
    """AUTO WP1 with a temporary low launch floor until normal race AGL is reached."""
    set_mission_current(test, seq, label)
    test.reached_seq = None
    test.set_mode("AUTO")
    print(
        f"[RACE-AUTO] sprint to {label} from LiDAR~{a.race_launch_alt:.2f}m; "
        f"temporary floor={a.race_auto_floor:.2f}m until AGL>={a.race_guard_arm_agl:.2f}m"
    )

    outside_since = None
    lidar_loss_since = None
    normal_guard_armed = False
    last_print = 0.0
    start = time.monotonic()
    climb_deadline = start + float(a.race_climb_confirm_timeout)
    deadline = start + float(a.wp_timeout)

    while time.monotonic() < deadline:
        test.poll(0.05)
        check_flight_health(test)
        if test.mode != "AUTO":
            raise FlightTestError(f"mode changed during race AUTO launch: {test.mode}")

        if test.lidar_ok() and float(test.lidar) >= float(a.race_guard_arm_agl):
            if not normal_guard_armed:
                normal_guard_armed = True
                outside_since = None
                print(
                    f"[RACE-GUARD] LiDAR={test.lidar:.2f}m -> normal AUTO floor "
                    f"{a.auto_floor:.2f}m ARMED"
                )

        if not normal_guard_armed and time.monotonic() > climb_deadline:
            lidar_text = "--" if test.lidar is None else f"{test.lidar:.2f}"
            raise FlightTestError(
                f"race AUTO did not climb to {a.race_guard_arm_agl:.2f}m within "
                f"{a.race_climb_confirm_timeout:.1f}s (LiDAR={lidar_text}m)"
            )

        floor = float(a.auto_floor) if normal_guard_armed else float(a.race_auto_floor)
        outside_since, lidar_loss_since = check_lidar_guard(
            test,
            floor_m=floor,
            ceiling_m=float(a.auto_ceiling),
            outside_since=outside_since,
            lidar_loss_since=lidar_loss_since,
            max_outside_s=float(a.guard_outside_s),
            max_stale_s=float(a.lidar_stale_s),
        )

        now = time.monotonic()
        if now - last_print >= 0.25:
            rel = None if test.global_pos is None else test.global_pos[2]
            lidar_text = "--" if test.lidar is None else f"{test.lidar:.2f}"
            guard_name = "NORMAL" if normal_guard_armed else "LAUNCH"
            print(
                f"[AUTO:{label}] RACE-{guard_name} LiDAR={lidar_text}m "
                f"relAlt={rel if rel is not None else float('nan'):.2f}m "
                f"current={test.mission_current}"
            )
            last_print = now

        if test.reached_seq == int(seq):
            if not normal_guard_armed:
                raise FlightTestError(
                    f"{label} reached before normal race AGL {a.race_guard_arm_agl:.2f}m was confirmed"
                )
            print(f"[RACE-AUTO] REACHED {label} -> normal race altitude regime confirmed")
            return

    raise FlightTestError(f"AUTO timeout before reaching {label}")


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
    detector_class_id = int(cfg.get("ai", {}).get("bottom", {}).get("detector_class_id", 0))

    stop_event = __import__("threading").Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)
    gcs = (
        HeadlessLoopbackGcs(state, events, waypoints, stop_event, command_manager=None)
        if a.headless
        else GcsServer(state, events, waypoints, stop_event, command_manager=None)
    )
    vision = VisionStreamSession(cfg, state, stop_event)
    disable_ai(vision, "HYBRID_RACE_INIT")

    test = DirectFlightTest()
    flight_open = False
    drop_phase_active = False
    result = None

    print("============================================================")
    print(" KINGPHOENIX HYBRID DROP V2 - RACE LAUNCH")
    print("============================================================")
    print(
        f"[PLAN] LiDAR {a.race_launch_alt:.2f}m -> immediate AUTO {wp1['label']} "
        f"(WP alt={wp1['alt_m']:.2f}m) -> DROP <= {a.drop_timeout:.2f}s -> "
        f"AUTO {wp2['label']} -> LAND"
    )
    print(
        f"[RACE] horizontal sprint begins at {a.race_launch_alt:.2f}m AGL while AUTO "
        f"continues climb toward WP1 mission altitude"
    )
    print("[AI] model preloaded; inference OFF during launch/AUTO; BOTTOM ON only for DROP")
    print("[STREAM] FRONT + BOTTOM remain ON throughout")
    print(
        f"[DROP] |raw ex|,|raw ey|<={a.drop_error:.2f} | lock={a.lock_frames} | "
        f"drop-frames={a.drop_frames}"
    )
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

        upload_two_wp_route(test, wp1, wp2, acceptance_radius_m=float(a.accept_radius))
        verify_two_wp_route(test, wp1, wp2)
        print("[SAFE] HOME -> WP1 -> WP2 verified while DISARMED")

        if a.preload_only:
            print("[PRELOAD-ONLY] route/stream/AI preload verified; vehicle remains DISARMED")
            return 0

        if not a.skip_servo_function_check:
            verify_servo_functions(test, (a.servo_a, a.servo_b))
        else:
            print("[SERVO FUNCTION] CHECK SKIPPED by operator")

        print("[GROUND SERVO PRETEST]")
        require_word(
            f"Type OPEN to command CH{a.servo_a}/CH{a.servo_b} -> {a.open_pwm}us: ",
            "OPEN",
        )
        set_servo_pair(test, a.servo_a, a.servo_b, a.open_pwm, "OPEN", a.servo_settle)
        require_word("Confirm mechanism OPEN, then type READY to CLOSE: ", "READY")
        set_servo_pair(test, a.servo_a, a.servo_b, a.close_pwm, "CLOSE", a.servo_settle)
        require_word("Servos CLOSED. Safety pilot ready. Type GO: ", "GO")

        disable_ai(vision, "RACE_LAUNCH_AUTO_WP1")
        race_lidar_launch(test, a)
        rel = None if test.global_pos is None else test.global_pos[2]
        print(
            f"[RACE HANDOFF] LiDAR={test.lidar:.2f}m "
            f"FCrelAlt={rel if rel is not None else float('nan'):.2f}m -> AUTO WP1 NOW"
        )

        fly_auto_to_wp_race_launch(test, seq=FC_WP1_SEQ, label=wp1["label"], a=a)

        drop_phase_active = True
        drop_start = time.monotonic()
        print("[DROP TIMER] START @ WP1 REACHED")
        result = run_quick_drop(test, state, vision, a, detector_class_id, drop_start)

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
                f"[DROP RESULT] TIMEOUT | budget={a.drop_timeout:.2f}s | servo stayed CLOSED"
            )

        print("[MISSION] DROP phase finished -> AI OFF -> AUTO WP2")
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
            print("[RESULT] HYBRID DROP V2 RACE-LAUNCH PASS")
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
