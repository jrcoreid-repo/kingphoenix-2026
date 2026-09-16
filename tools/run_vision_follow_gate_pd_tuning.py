#!/usr/bin/env python3
"""Kingphoenix FRONT gate PD field-tuning tool.

Purpose
-------
LiDAR-guided low-altitude takeoff -> GUIDED front-gate tracking -> LAND,
with independently tunable lateral and yaw PD control.

This file is intentionally separate from the validated P-only tools so their
known-good behavior is preserved.

Safety contract
---------------
- FRONT camera + YOLO26/Hailo only.
- FORWARD/BACKWARD velocity (VX) is hard-coded 0.0 m/s.
- VERTICAL velocity (VZ) is hard-coded 0.0 m/s.
- Target loss/stale vision sends ZERO immediately (no slew/coast).
- GCS loss sends ZERO immediately and aborts after 1 s.
- LiDAR floor/ceiling guard remains active.
- Default control mode is lateral-only. Validate yaw-only before combining.
- Do not run main.py, MAVProxy, another flight tool, or another camera/Hailo
  owner at the same time.
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


def clamp(v: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(v)))


def deadband_remap(value: float, deadband: float) -> float:
    value = clamp(float(value), -1.0, 1.0)
    db = clamp(abs(float(deadband)), 0.0, 0.95)
    if abs(value) <= db:
        return 0.0
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * (abs(value) - db) / max(1.0 - db, 1e-6)


def response_curve(value: float, mode: str, expo: float) -> float:
    """RC-style expo: softer near center while preserving full-scale output."""
    value = clamp(float(value), -1.0, 1.0)
    if str(mode).lower() == "linear":
        return value
    a = abs(value)
    e = clamp(float(expo), 0.0, 1.0)
    shaped = (1.0 - e) * a + e * (a ** 3)
    return math.copysign(shaped, value)


def slew(current: float, target: float, rate_per_s: float, dt: float) -> float:
    max_delta = max(0.0, float(rate_per_s)) * max(0.0, float(dt))
    return float(current) + clamp(float(target) - float(current), -max_delta, max_delta)


def send_body_command(
    test: DirectFlightTest,
    *,
    lateral_mps: float,
    yaw_rate_dps: float,
):
    """BODY_OFFSET_NED command; forward/back and vertical are hard OFF."""
    if test.master is None:
        raise FlightTestError("MAVLink is not open")

    # 1479: velocity XYZ + yaw-rate enabled; position/accel/absolute yaw ignored.
    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        1479,
        0.0, 0.0, 0.0,
        0.0,                       # VX: forward/backward HARD OFF
        float(lateral_mps),        # VY: + right / - left
        0.0,                       # VZ: vertical HARD OFF
        0.0, 0.0, 0.0,
        0.0,
        math.radians(float(yaw_rate_dps)),
    )


def zero_command(test: DirectFlightTest):
    send_body_command(test, lateral_mps=0.0, yaw_rate_dps=0.0)


def wait_for_gcs(state: SharedState, timeout_s: float):
    print("[GCS] waiting for Windows GCS CONNECT ...")
    deadline = time.monotonic() + float(timeout_s)
    seen = False
    while time.monotonic() < deadline:
        with state.lock:
            connected = bool(state.system.get("gcs_connected", False))
            front = dict(state.vision.get("front") or {})
        if connected and not seen:
            print("[GCS] CONNECTED - waiting for FRONT stream metadata")
            seen = True
        if connected and str(front.get("video_state", "")).upper() == "LIVE":
            ai_state = str(front.get("ai_state", "UNKNOWN")).upper()
            if ai_state not in {"DISABLED", "ERROR"}:
                print(f"[GCS] FRONT LIVE | AI={ai_state} mode={front.get('vision_mode', 'SEARCH')}")
                return
        time.sleep(0.10)
    raise FlightTestError("GCS/FRONT stream not ready before timeout")


def parse_args():
    ap = argparse.ArgumentParser(
        description="Kingphoenix gate-follow PD tuner: LiDAR takeoff -> GUIDED follow -> LAND"
    )

    ap.add_argument("--control", choices=["lateral", "yaw", "lateral-yaw"], default="lateral")
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--control-hz", type=float, default=20.0)

    # Lateral PD. 2 m/s cap is available because it has now been field-tested
    # by the operator, but gain/slew remain independently tunable.
    ap.add_argument("--kp-lat", type=float, default=2.0, help="m/s per normalized error")
    ap.add_argument("--kd-lat", type=float, default=0.20, help="m/s per (normalized error/s)")
    ap.add_argument("--lat-cap", type=float, default=2.0, help="absolute lateral velocity cap m/s")
    ap.add_argument("--lat-slew", type=float, default=4.0, help="max lateral command change, m/s^2")

    # Yaw PD. Start yaw-only before combining with lateral.
    ap.add_argument("--kp-yaw", type=float, default=20.0, help="deg/s per normalized error")
    ap.add_argument("--kd-yaw", type=float, default=1.0, help="deg/s per (normalized error/s)")
    ap.add_argument("--yaw-cap", type=float, default=12.0, help="absolute yaw rate cap deg/s")
    ap.add_argument("--yaw-slew", type=float, default=60.0, help="max yaw-rate command change, deg/s^2")
    ap.add_argument("--yaw-sign", type=int, choices=[-1, 1], default=1, help="use -1 only if yaw-only sign test proves reversed")

    # Vision/filter/shaping.
    ap.add_argument("--deadband", type=float, default=0.03)
    ap.add_argument("--filter-alpha", type=float, default=0.45, help="LPF new-sample weight for EX")
    ap.add_argument("--d-alpha", type=float, default=0.25, help="LPF new-sample weight for derivative")
    ap.add_argument("--response", choices=["linear", "expo"], default="linear")
    ap.add_argument("--expo", type=float, default=0.35, help="0..1; used only with --response expo")
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--lock-frames", type=int, default=3, help="unique consecutive vision frames before movement")
    ap.add_argument("--lost-timeout", type=float, default=0.15, help="reset filters after target remains lost this long")

    # Validated takeoff defaults.
    ap.add_argument("--alt", type=float, default=1.20)
    ap.add_argument("--climb-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.25)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15)

    # Flight guards.
    ap.add_argument("--guard-floor", type=float, default=0.80)
    ap.add_argument("--guard-ceiling", type=float, default=1.45)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)
    ap.add_argument("--gcs-wait", type=float, default=60.0)
    ap.add_argument("--yes", action="store_true", help="skip final GO prompt")

    return ap.parse_args()


def validate_args(a):
    if not 1.0 <= a.duration <= 120.0:
        raise SystemExit("--duration must be 1..120s")
    if not 10.0 <= a.control_hz <= 30.0:
        raise SystemExit("--control-hz must be 10..30Hz")
    if not 0.05 <= a.kp_lat <= 5.0:
        raise SystemExit("--kp-lat must be 0.05..5.0")
    if not 0.0 <= a.kd_lat <= 2.0:
        raise SystemExit("--kd-lat must be 0..2.0")
    if not 0.05 <= a.lat_cap <= 2.5:
        raise SystemExit("--lat-cap must be 0.05..2.5m/s")
    if not 0.2 <= a.lat_slew <= 12.0:
        raise SystemExit("--lat-slew must be 0.2..12m/s^2")
    if not 1.0 <= a.kp_yaw <= 80.0:
        raise SystemExit("--kp-yaw must be 1..80")
    if not 0.0 <= a.kd_yaw <= 20.0:
        raise SystemExit("--kd-yaw must be 0..20")
    if not 1.0 <= a.yaw_cap <= 40.0:
        raise SystemExit("--yaw-cap must be 1..40deg/s")
    if not 5.0 <= a.yaw_slew <= 240.0:
        raise SystemExit("--yaw-slew must be 5..240deg/s^2")
    if not 0.01 <= a.deadband <= 0.15:
        raise SystemExit("--deadband must be 0.01..0.15")
    if not 0.05 <= a.filter_alpha <= 1.0:
        raise SystemExit("--filter-alpha must be 0.05..1.0")
    if not 0.05 <= a.d_alpha <= 1.0:
        raise SystemExit("--d-alpha must be 0.05..1.0")
    if not 0.0 <= a.expo <= 1.0:
        raise SystemExit("--expo must be 0..1")
    if not 0.25 <= a.confidence <= 0.95:
        raise SystemExit("--confidence must be 0.25..0.95")
    if not 1 <= a.lock_frames <= 20:
        raise SystemExit("--lock-frames must be 1..20")
    if not 0.0 <= a.lost_timeout <= 1.0:
        raise SystemExit("--lost-timeout must be 0..1.0s")
    if not 0.50 <= a.alt <= 3.0:
        raise SystemExit("--alt must be 0.50..3.00m")
    if not 0.10 <= a.climb_rate <= 0.80:
        raise SystemExit("--climb-rate must be 0.10..0.80m/s")
    if not 0.08 <= a.near_rate <= a.climb_rate:
        raise SystemExit("--near-rate must be 0.08..climb-rate")
    if a.takeoff_ceiling <= a.alt:
        raise SystemExit("--takeoff-ceiling must be above --alt")
    if not 0.30 <= a.guard_floor < a.guard_ceiling:
        raise SystemExit("invalid altitude guard")
    if a.guard_ceiling > 2.0:
        raise SystemExit("--guard-ceiling max is 2.0m")


def main() -> int:
    args = parse_args()
    validate_args(args)

    control_lat = args.control in {"lateral", "lateral-yaw"}
    control_yaw = args.control in {"yaw", "lateral-yaw"}

    cfg = shadow_config()
    cfg.setdefault("ai", {}).setdefault("detector", {})["confidence_threshold"] = float(args.confidence)
    cfg.setdefault("vision_control", {})["movement_tx_enabled"] = False
    cfg["vision_control"]["confidence_min"] = float(args.confidence)

    stop_event = threading.Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)
    gcs = GcsServer(state, events, waypoints, stop_event, command_manager=None)
    vision = VisionStreamSession(cfg, state, stop_event)
    vision.workers = [vision.front]

    test = DirectFlightTest()
    flight_open = False

    print("============================================================")
    print(" KINGPHOENIX VISION FOLLOW GATE - PD TUNING")
    print("============================================================")
    print(f"[CONTROL] {args.control.upper()} @ {args.control_hz:.0f}Hz | response={args.response} expo={args.expo:.2f}")
    print(f"[LAT] Kp={args.kp_lat:.2f} Kd={args.kd_lat:.2f} cap=+/-{args.lat_cap:.2f}m/s slew={args.lat_slew:.1f}m/s^2")
    print(f"[YAW] Kp={args.kp_yaw:.1f} Kd={args.kd_yaw:.1f} cap=+/-{args.yaw_cap:.1f}deg/s slew={args.yaw_slew:.1f}deg/s^2 sign={args.yaw_sign:+d}")
    print(f"[VISION] conf>={args.confidence:.2f} deadband={args.deadband:.3f} EX alpha={args.filter_alpha:.2f} D alpha={args.d_alpha:.2f}")
    print(f"[TAKEOFF] LiDAR {args.alt:.2f}m | guard={args.guard_floor:.2f}..{args.guard_ceiling:.2f}m")
    print("[HARD OFF] VX FORWARD/BACKWARD = 0.00m/s")
    print("[HARD OFF] VZ VERTICAL = 0.00m/s")
    if not control_lat:
        print("[HARD OFF] LATERAL VY = 0.00m/s")
    if not control_yaw:
        print("[HARD OFF] YAW RATE = 0.00deg/s")
    print("------------------------------------------------------------")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("Hailo YOLO26 did not become READY")
        wait_for_gcs(state, float(args.gcs_wait))

        test.open()
        flight_open = True
        test.collect_preflight(require_auto=True)

        if not args.yes:
            print("\n[READY] Safety pilot ready.")
            if args.control == "yaw":
                print("[READY] YAW-ONLY sign validation: gate right should command a right yaw.")
            elif args.control == "lateral-yaw":
                print("[READY] Use combined mode only after lateral and yaw signs were checked separately.")
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
        if test.mode != "GUIDED":
            test.set_mode("GUIDED")
        zero_command(test)

        print("------------------------------------------------------------")
        print("[FOLLOW] ACTIVE - move gate manually left/right")
        print("[FOLLOW] LOST/STALE target => ZERO immediately")
        print("[FOLLOW] duration/Ctrl+C => ZERO then LAND")
        print("------------------------------------------------------------")

        period = 1.0 / float(args.control_hz)
        end_time = time.monotonic() + float(args.duration)
        next_tx = time.monotonic()
        last_tx = next_tx
        last_print = 0.0

        filtered_ex = None
        prev_ctrl_error = None
        d_filtered = 0.0
        last_vision_update = None
        last_frame_id = None
        valid_streak = 0
        lost_since = None
        gcs_lost_since = None
        outside_since = None
        lidar_loss_since = None

        lat_cmd = 0.0
        yaw_cmd = 0.0
        target_lat = 0.0
        target_yaw = 0.0
        ctrl_error = 0.0
        p_lat = d_lat = p_yaw = d_yaw = 0.0
        latest_valid = False
        latest_conf = None
        latest_raw_ex = None
        status = "SEARCH"

        while time.monotonic() < end_time:
            test.poll(0.02)
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
                lat_cmd = yaw_cmd = 0.0
                zero_command(test)
                if now - gcs_lost_since > 1.0:
                    raise FlightTestError("GCS connection lost >1.0s")
            else:
                gcs_lost_since = None

            frame_id = front.get("frame_id")
            frame_ts_ns = int(front.get("frame_timestamp_ns") or 0)
            frame_age_s = float("inf") if frame_ts_ns <= 0 else max(0.0, (time.monotonic_ns() - frame_ts_ns) / 1e9)
            conf = front.get("confidence")
            raw_ex = front.get("ex")
            valid = bool(
                front.get("vision_valid", False)
                and conf is not None
                and float(conf) >= float(args.confidence)
                and raw_ex is not None
                and frame_age_s <= 0.20
            )
            latest_valid = valid
            latest_conf = conf
            latest_raw_ex = raw_ex

            # Update controller state only once per unique camera frame.
            if frame_id is not None and frame_id != last_frame_id:
                last_frame_id = frame_id
                if valid:
                    lost_since = None
                    valid_streak += 1
                    ex = clamp(float(raw_ex), -1.0, 1.0)
                    if filtered_ex is None:
                        filtered_ex = ex
                    else:
                        a = float(args.filter_alpha)
                        filtered_ex = a * ex + (1.0 - a) * float(filtered_ex)

                    db_error = deadband_remap(float(filtered_ex), float(args.deadband))
                    ctrl_error = response_curve(db_error, args.response, float(args.expo))

                    if last_vision_update is not None and prev_ctrl_error is not None:
                        dtv = max(1e-3, now - last_vision_update)
                        d_raw = (ctrl_error - float(prev_ctrl_error)) / dtv
                        da = float(args.d_alpha)
                        d_filtered = da * d_raw + (1.0 - da) * d_filtered
                    else:
                        d_filtered = 0.0

                    prev_ctrl_error = ctrl_error
                    last_vision_update = now

                    if valid_streak >= int(args.lock_frames):
                        p_lat = float(args.kp_lat) * ctrl_error
                        d_lat = float(args.kd_lat) * d_filtered
                        p_yaw = float(args.kp_yaw) * ctrl_error
                        d_yaw = float(args.kd_yaw) * d_filtered

                        target_lat = clamp(p_lat + d_lat, -float(args.lat_cap), float(args.lat_cap)) if control_lat else 0.0
                        target_yaw = clamp((p_yaw + d_yaw) * int(args.yaw_sign), -float(args.yaw_cap), float(args.yaw_cap)) if control_yaw else 0.0
                        status = "CENTER" if ctrl_error == 0.0 else "TRACK"
                    else:
                        target_lat = target_yaw = 0.0
                        status = f"LOCK {valid_streak}/{args.lock_frames}"
                else:
                    valid_streak = 0
                    lost_since = lost_since or now
                    target_lat = target_yaw = 0.0
                    status = "TARGET LOST"
                    # Safety: no coasting on invalid/stale vision.
                    lat_cmd = yaw_cmd = 0.0
                    zero_command(test)
                    if now - lost_since >= float(args.lost_timeout):
                        filtered_ex = None
                        prev_ctrl_error = None
                        d_filtered = 0.0
                        last_vision_update = None

            if now >= next_tx:
                dt = max(1e-3, now - last_tx)
                last_tx = now
                next_tx = now + period

                if latest_valid and valid_streak >= int(args.lock_frames) and gcs_connected:
                    lat_cmd = slew(lat_cmd, target_lat, float(args.lat_slew), dt) if control_lat else 0.0
                    yaw_cmd = slew(yaw_cmd, target_yaw, float(args.yaw_slew), dt) if control_yaw else 0.0
                else:
                    # Invalid/stale/lost target: ZERO immediately, bypass slew.
                    lat_cmd = 0.0
                    yaw_cmd = 0.0

                send_body_command(test, lateral_mps=lat_cmd, yaw_rate_dps=yaw_cmd)

            if now - last_print >= 0.25:
                lidar_text = "--" if test.lidar is None else f"{float(test.lidar):.2f}"
                conf_text = "--" if latest_conf is None else f"{float(latest_conf):.2f}"
                ex_text = "--" if latest_raw_ex is None else f"{float(latest_raw_ex):+.3f}"
                filt_text = "--" if filtered_ex is None else f"{float(filtered_ex):+.3f}"
                print(
                    f"[PD] {status:12s} conf={conf_text} EX={ex_text} filt={filt_text} "
                    f"e={ctrl_error:+.3f} de={d_filtered:+.3f}/s | "
                    f"LAT P={p_lat:+.2f} D={d_lat:+.2f} -> {lat_cmd:+.2f}m/s | "
                    f"YAW P={p_yaw:+.1f} D={d_yaw:+.1f} -> {yaw_cmd:+.1f}deg/s | "
                    f"VX=0 VZ=0 LiDAR={lidar_text}m"
                )
                last_print = now

        zero_command(test)
        print("[FOLLOW] duration complete -> ZERO -> LAND")
        test.land_and_wait()
        print("[RESULT] PD VISION FOLLOW TEST COMPLETE")
        return 0

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        if flight_open:
            try:
                zero_command(test)
            except Exception:
                pass
            test.land_best_effort("operator interrupt")
        return 130
    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        if flight_open:
            try:
                zero_command(test)
            except Exception:
                pass
            test.land_best_effort(str(exc))
        return 2
    except Exception as exc:
        print(f"[UNEXPECTED] {type(exc).__name__}: {exc}")
        if flight_open:
            try:
                zero_command(test)
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
