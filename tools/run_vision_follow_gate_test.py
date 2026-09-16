#!/usr/bin/env python3
"""Kingphoenix FRONT gate vision-follow field test.

Validated flight foundation:
    profiled LiDAR GUIDED takeoff -> GUIDED vision follow -> LAND

This is intentionally a constrained tuning tool, not the final race controller.
The first/default mode is LATERAL ONLY. Forward/backward and vertical velocity
are hard-coded to zero for the entire follow phase.

The normal FRONT camera + YOLO26/Hailo stream is also exposed to the Windows
GCS so the operator can see the exact bbox/EX/EY used by the controller.

IMPORTANT
---------
- Do NOT run main.py, MAVProxy, another flight tool, or another camera/Hailo
  owner at the same time.
- Default gains/speed limits are deliberately conservative for first field
  tuning.
- Positive EX means the gate is right of image center. In BODY_OFFSET_NED,
  positive VY commands aircraft motion to the right.
- Yaw sign is available for a later explicit test, but default control is
  lateral-only so lateral and yaw signs are validated separately.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from core.event_bus import EventBus
from core.shared_state import SharedState
from mission.waypoint_manager import WaypointManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from tools.flight_test_common import DirectFlightTest, FlightTestError
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_front_gate_shadow_gcs import shadow_config
from tools.run_hybrid_mission_test import check_flight_health, check_lidar_guard


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def deadband_remap(value: float, deadband: float) -> float:
    """Continuous deadband: zero inside DB, smoothly remap outside to [-1,1]."""
    value = clamp(float(value), -1.0, 1.0)
    deadband = clamp(abs(float(deadband)), 0.0, 0.95)
    if abs(value) <= deadband:
        return 0.0
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * (abs(value) - deadband) / max(1.0 - deadband, 1e-6)


def send_body_follow_command(
    test: DirectFlightTest,
    *,
    lateral_mps: float,
    yaw_rate_dps: float,
):
    """Send the constrained vision-follow command.

    BODY_OFFSET_NED:
      VX + = forward       -> HARD-CODED 0.0 in this test
      VY + = right
      VZ + = down          -> HARD-CODED 0.0 in this test
      yaw_rate + = positive NED yaw-rate

    type_mask 1479 enables velocity XYZ + yaw-rate while ignoring
    position, acceleration and absolute yaw.
    """
    if test.master is None:
        raise FlightTestError("MAVLink is not open")

    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        1479,
        0.0, 0.0, 0.0,
        0.0,                       # VX: FORWARD/BACKWARD HARD OFF
        float(lateral_mps),        # VY: RIGHT + / LEFT -
        0.0,                       # VZ: VERTICAL VELOCITY HARD OFF
        0.0, 0.0, 0.0,
        0.0,                       # absolute yaw ignored
        math.radians(float(yaw_rate_dps)),
    )


def zero_follow_command(test: DirectFlightTest):
    send_body_follow_command(test, lateral_mps=0.0, yaw_rate_dps=0.0)


def wait_for_gcs_and_front_stream(
    state: SharedState,
    *,
    timeout_s: float,
):
    print("[GCS] waiting for Windows GCS CONNECT ...")
    deadline = time.monotonic() + float(timeout_s)
    connected_seen = False

    while time.monotonic() < deadline:
        with state.lock:
            connected = bool(state.system.get("gcs_connected", False))
            front = dict(state.vision.get("front") or {})

        if connected and not connected_seen:
            print("[GCS] CONNECTED - waiting for FRONT stream metadata")
            connected_seen = True

        if connected and str(front.get("video_state", "")).upper() == "LIVE":
            ai_state = str(front.get("ai_state", "UNKNOWN")).upper()
            if ai_state not in {"DISABLED", "ERROR"}:
                print(
                    f"[GCS] FRONT LIVE | AI={ai_state} "
                    f"mode={front.get('vision_mode', 'SEARCH')}"
                )
                return
        time.sleep(0.10)

    raise FlightTestError("GCS/FRONT stream not ready before timeout")


def parse_args():
    ap = argparse.ArgumentParser(
        description=(
            "Kingphoenix: LiDAR takeoff -> GUIDED front-gate follow -> LAND. "
            "Default is conservative LATERAL ONLY; forward/backward hard OFF."
        )
    )

    # Test/control mode.
    ap.add_argument(
        "--control",
        choices=["lateral", "yaw", "lateral-yaw"],
        default="lateral",
        help="default lateral-only; validate yaw separately before combining",
    )
    ap.add_argument("--duration", type=float, default=20.0, help="follow-test seconds before automatic LAND")
    ap.add_argument("--control-hz", type=float, default=10.0, help="MAVLink control update rate")

    # Tunable P controller. Safe defaults for first test.
    ap.add_argument("--kp-lat", type=float, default=0.40, help="lateral P gain: m/s per normalized EX")
    ap.add_argument("--kp-yaw", type=float, default=10.0, help="yaw P gain: deg/s per normalized EX")
    ap.add_argument("--lat-cap", type=float, default=0.15, help="absolute lateral speed cap m/s")
    ap.add_argument("--yaw-cap", type=float, default=5.0, help="absolute yaw-rate cap deg/s")
    ap.add_argument("--deadband", type=float, default=0.05, help="normalized EX deadband around image center")
    ap.add_argument(
        "--filter-alpha",
        type=float,
        default=0.25,
        help="EX low-pass new-sample weight; lower=smoother/slower",
    )
    ap.add_argument("--confidence", type=float, default=0.50, help="minimum YOLO gate confidence for control")
    ap.add_argument("--lock-frames", type=int, default=3, help="consecutive valid detections before movement is allowed")
    ap.add_argument(
        "--lost-timeout",
        type=float,
        default=0.15,
        help="seconds target may remain lost before filter/lock reset; command is zero immediately on loss",
    )

    # Proven LiDAR takeoff profile; defaults match validated low-alt baseline.
    ap.add_argument("--alt", type=float, default=1.20)
    ap.add_argument("--climb-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.25)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15)

    # In-flight altitude safety guard. This tool does NOT command vertical speed.
    ap.add_argument("--guard-floor", type=float, default=0.80)
    ap.add_argument("--guard-ceiling", type=float, default=1.45)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)

    # GCS is required by default so the operator sees what vision sees.
    ap.add_argument("--gcs-wait", type=float, default=60.0, help="seconds to wait for GCS/front stream before arming")
    ap.add_argument("--headless", action="store_true", help="do not require GCS connection; still starts front vision")
    ap.add_argument("--yes", action="store_true", help="skip final interactive GO confirmation")

    return ap.parse_args()


def validate_args(args):
    if not 1.0 <= args.duration <= 120.0:
        raise SystemExit("--duration must be 1..120s")
    if not 5.0 <= args.control_hz <= 20.0:
        raise SystemExit("--control-hz must be 5..20Hz")
    if not 0.05 <= args.kp_lat <= 1.0:
        raise SystemExit("--kp-lat must be 0.05..1.0")
    if not 1.0 <= args.kp_yaw <= 30.0:
        raise SystemExit("--kp-yaw must be 1..30 deg/s per EX")
    if not 0.05 <= args.lat_cap <= 0.30:
        raise SystemExit("--lat-cap must be 0.05..0.30m/s")
    if not 1.0 <= args.yaw_cap <= 10.0:
        raise SystemExit("--yaw-cap must be 1..10deg/s")
    if not 0.01 <= args.deadband <= 0.15:
        raise SystemExit("--deadband must be 0.01..0.15")
    if not 0.05 <= args.filter_alpha <= 1.0:
        raise SystemExit("--filter-alpha must be 0.05..1.0")
    if not 0.25 <= args.confidence <= 0.95:
        raise SystemExit("--confidence must be 0.25..0.95")
    if not 1 <= args.lock_frames <= 20:
        raise SystemExit("--lock-frames must be 1..20")
    if not 0.0 <= args.lost_timeout <= 1.0:
        raise SystemExit("--lost-timeout must be 0..1.0s")
    if not 0.50 <= args.alt <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if not 0.10 <= args.climb_rate <= 0.80:
        raise SystemExit("--climb-rate must be 0.10..0.80m/s")
    if not 0.08 <= args.near_rate <= args.climb_rate:
        raise SystemExit("--near-rate must be 0.08..climb-rate")
    if args.takeoff_ceiling <= args.alt:
        raise SystemExit("--takeoff-ceiling must be above --alt")
    if not 0.30 <= args.guard_floor < args.guard_ceiling:
        raise SystemExit("invalid --guard-floor/--guard-ceiling")
    if args.guard_ceiling > 2.0:
        raise SystemExit("--guard-ceiling max for this field tool is 2.0m")
    if not 0.10 <= args.guard_outside_s <= 1.0:
        raise SystemExit("--guard-outside-s must be 0.10..1.0")
    if not 0.20 <= args.lidar_stale_s <= 1.0:
        raise SystemExit("--lidar-stale-s must be 0.20..1.0")


def main() -> int:
    args = parse_args()
    validate_args(args)

    cfg = shadow_config()
    # Make the GCS overlay/vision validity use the same confidence threshold as
    # this flight controller. Streaming remains estimator-only; movement TX in
    # the normal vision stack stays disabled.
    cfg.setdefault("ai", {}).setdefault("detector", {})["confidence_threshold"] = float(args.confidence)
    cfg.setdefault("vision_control", {})["movement_tx_enabled"] = False
    cfg["vision_control"]["confidence_min"] = float(args.confidence)

    stop_event = threading.Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)

    gcs = GcsServer(
        state,
        events,
        waypoints,
        stop_event,
        command_manager=None,
    )
    vision = VisionStreamSession(cfg, state, stop_event)

    # FRONT only for this phase: no bottom capture/encode load.
    vision.workers = [vision.front]

    test = DirectFlightTest()
    flight_open = False
    flight_started = False

    control_lateral = args.control in {"lateral", "lateral-yaw"}
    control_yaw = args.control in {"yaw", "lateral-yaw"}

    print("============================================================")
    print(" KINGPHOENIX VISION FOLLOW GATE - FIELD TUNING")
    print("============================================================")
    print(f"[CONTROL] mode          : {args.control.upper()}")
    print(f"[CONTROL] Kp lateral    : {args.kp_lat:.3f}")
    print(f"[CONTROL] Kp yaw        : {args.kp_yaw:.2f} deg/s per EX")
    print(f"[LIMIT]   lateral       : +/-{args.lat_cap:.2f} m/s")
    print(f"[LIMIT]   yaw rate      : +/-{args.yaw_cap:.1f} deg/s")
    print(f"[FILTER]  deadband      : {args.deadband:.3f}")
    print(f"[FILTER]  alpha         : {args.filter_alpha:.3f}")
    print(f"[VISION]  confidence    : >= {args.confidence:.2f}")
    print(f"[VISION]  lock          : {args.lock_frames} consecutive frames")
    print(f"[TEST]    duration      : {args.duration:.1f}s")
    print(f"[TAKEOFF] LiDAR target  : {args.alt:.2f}m")
    print(f"[GUARD]   LiDAR         : {args.guard_floor:.2f}..{args.guard_ceiling:.2f}m")
    print("[HARD OFF] FORWARD/BACKWARD VX = 0.00 m/s")
    print("[HARD OFF] VERTICAL VELOCITY VZ = 0.00 m/s")
    if not control_yaw:
        print("[HARD OFF] YAW RATE = 0.00 deg/s (lateral validation phase)")
    if not control_lateral:
        print("[HARD OFF] LATERAL VY = 0.00 m/s (yaw validation phase)")
    print("------------------------------------------------------------")

    try:
        # Start monitor/perception first. It has no FC command backend.
        gcs.start()
        vision.start()

        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("Hailo YOLO26 did not become READY")

        if not args.headless:
            wait_for_gcs_and_front_stream(state, timeout_s=float(args.gcs_wait))
        else:
            print("[GCS] HEADLESS override selected; operator stream is not required")
            # CameraStreamWorker processes only while a GCS session exists, so
            # headless control is intentionally blocked for now rather than
            # pretending vision is active without frames flowing.
            raise FlightTestError("--headless is reserved; current vision worker requires GCS session")

        test.open()
        flight_open = True
        # Horizontal GUIDED velocity needs the navigation estimate healthy.
        test.collect_preflight(require_auto=True)

        if not args.yes:
            print("\n[READY] Safety pilot ready. Gate follow starts after LiDAR takeoff.")
            print("[READY] Default/first test should remain --control lateral.")
            answer = input("Type GO to arm and begin, anything else cancels: ").strip().upper()
            if answer != "GO":
                print("[CANCELLED] vehicle remains DISARMED")
                return 0

        profiled_lidar_takeoff(
            test,
            float(args.alt),
            tolerance_m=float(args.tolerance),
            handoff_m=float(args.handoff),
            fast_rate_mps=float(args.climb_rate),
            near_rate_mps=float(args.near_rate),
            taper_zone_m=float(args.taper_zone),
            kick_alt_m=float(args.kick_alt),
            timeout_s=15.0,
            ceiling_m=float(args.takeoff_ceiling),
            zero_flush_s=float(args.zero_flush),
        )
        flight_started = True

        # profiled_lidar_takeoff ends in GUIDED; make that explicit and verify.
        if test.mode != "GUIDED":
            test.set_mode("GUIDED")
        zero_follow_command(test)

        print("------------------------------------------------------------")
        print("[FOLLOW] ACTIVE - move the GATE manually left/right")
        print("[FOLLOW] Target lost => command ZERO immediately")
        print("[FOLLOW] Ctrl+C or duration end => ZERO then LAND")
        print("------------------------------------------------------------")

        period = 1.0 / float(args.control_hz)
        end_time = time.monotonic() + float(args.duration)
        next_tx = 0.0
        last_print = 0.0
        filtered_ex = None
        valid_streak = 0
        lost_since = None
        gcs_lost_since = None
        outside_since = None
        lidar_loss_since = None

        while time.monotonic() < end_time:
            # Maintain FC telemetry/failsafe awareness.
            test.poll(0.03)
            check_flight_health(test)
            if test.mode != "GUIDED":
                raise FlightTestError(f"mode changed during follow test: {test.mode}")

            outside_since, lidar_loss_since = check_lidar_guard(
                test,
                floor_m=float(args.guard_floor),
                ceiling_m=float(args.guard_ceiling),
                outside_since=outside_since,
                lidar_loss_since=lidar_loss_since,
                max_outside_s=float(args.guard_outside_s),
                max_stale_s=float(args.lidar_stale_s),
            )

            now = time.monotonic()
            with state.lock:
                front = dict(state.vision.get("front") or {})
                gcs_connected = bool(state.system.get("gcs_connected", False))

            if not gcs_connected:
                gcs_lost_since = gcs_lost_since or now
                zero_follow_command(test)
                if now - gcs_lost_since > 1.0:
                    raise FlightTestError("GCS connection lost >1.0s during vision field test")
            else:
                gcs_lost_since = None

            valid = bool(front.get("vision_valid", False))
            confidence = front.get("confidence")
            raw_ex = front.get("ex")
            frame_ts_ns = int(front.get("frame_timestamp_ns") or 0)
            frame_age_s = (
                float("inf")
                if frame_ts_ns <= 0
                else max(0.0, (time.monotonic_ns() - frame_ts_ns) / 1e9)
            )

            valid = bool(
                valid
                and confidence is not None
                and float(confidence) >= float(args.confidence)
                and raw_ex is not None
                and frame_age_s <= 0.20
            )

            lat_cmd = 0.0
            yaw_cmd = 0.0
            eff_ex = 0.0
            status = "SEARCH"

            if valid:
                lost_since = None
                valid_streak += 1
                ex = clamp(float(raw_ex), -1.0, 1.0)
                if filtered_ex is None:
                    filtered_ex = ex
                else:
                    a = float(args.filter_alpha)
                    filtered_ex = a * ex + (1.0 - a) * float(filtered_ex)

                eff_ex = deadband_remap(float(filtered_ex), float(args.deadband))
                locked = valid_streak >= int(args.lock_frames)

                if locked:
                    if control_lateral:
                        lat_cmd = clamp(
                            float(args.kp_lat) * eff_ex,
                            -float(args.lat_cap),
                            float(args.lat_cap),
                        )
                    if control_yaw:
                        yaw_cmd = clamp(
                            float(args.kp_yaw) * eff_ex,
                            -float(args.yaw_cap),
                            float(args.yaw_cap),
                        )
                    status = "CENTER" if eff_ex == 0.0 else "TRACK"
                else:
                    status = f"LOCK {valid_streak}/{args.lock_frames}"
            else:
                valid_streak = 0
                lost_since = lost_since or now
                # Safety rule: NEVER coast on a stale target.
                lat_cmd = 0.0
                yaw_cmd = 0.0
                status = "TARGET LOST"
                if now - lost_since >= float(args.lost_timeout):
                    filtered_ex = None

            if now >= next_tx:
                send_body_follow_command(
                    test,
                    lateral_mps=lat_cmd,
                    yaw_rate_dps=yaw_cmd,
                )
                next_tx = now + period

            if now - last_print >= 0.25:
                lidar_text = "--" if test.lidar is None else f"{float(test.lidar):.2f}"
                conf_text = "--" if confidence is None else f"{float(confidence):.2f}"
                raw_text = "--" if raw_ex is None else f"{float(raw_ex):+.3f}"
                filt_text = "--" if filtered_ex is None else f"{float(filtered_ex):+.3f}"
                print(
                    f"[FOLLOW] {status:12s} conf={conf_text} "
                    f"EX={raw_text} filt={filt_text} eff={eff_ex:+.3f} | "
                    f"VY={lat_cmd:+.3f}m/s YAW={yaw_cmd:+.2f}deg/s "
                    f"VX=0.000 VZ=0.000 | LiDAR={lidar_text}m"
                )
                last_print = now

        zero_follow_command(test)
        print("[FOLLOW] duration complete -> ZERO -> LAND")
        test.land_and_wait()
        flight_started = False
        print("[RESULT] VISION FOLLOW TEST COMPLETE")
        return 0

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        if flight_open:
            try:
                zero_follow_command(test)
            except Exception:
                pass
            test.land_best_effort("operator interrupt")
        return 130
    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        if flight_open:
            try:
                zero_follow_command(test)
            except Exception:
                pass
            test.land_best_effort(str(exc))
        return 2
    except Exception as exc:
        print(f"[UNEXPECTED] {type(exc).__name__}: {exc}")
        if flight_open:
            try:
                zero_follow_command(test)
            except Exception:
                pass
            test.land_best_effort(f"unexpected {type(exc).__name__}: {exc}")
        return 3
    finally:
        stop_event.set()
        try:
            if flight_open:
                test.close()
        except Exception:
            pass
        try:
            if gcs.is_alive():
                gcs.join(timeout=1.0)
        except Exception:
            pass
        try:
            vision.join(timeout=1.0)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
