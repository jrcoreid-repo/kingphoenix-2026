#!/usr/bin/env python3
"""Kingphoenix FRONT gate Batch-2 commit/pass-through field tuner.

Builds on the field-smooth Batch-1 defaults without changing them:
  lateral PD: Kp=2.0, Kd=0.20, cap=2.0 m/s, slew=4.0 m/s^2
  yaw PD:     Kp=20,  Kd=1.0,  cap=12 deg/s, slew=60 deg/s^2
  forward:    adaptive 0.6 -> 2.0 m/s

New Batch-2 behavior:
  TRACK -> NEAR_ARMED -> COMMIT/PASS

The key idea is that detector loss is normally unsafe and commands ZERO, but
once the gate is already large, centered, and NEAR_ARMED, loss is interpreted
as the expected close-range/FOV condition and the vehicle commits through the
gate instead of stopping in front of it.

This runner is headless. Windows GCS is not required. Vertical velocity remains
hard OFF (VZ=0); altitude stays under the validated low-altitude LiDAR guards.
At the end of the pass-through test the vehicle commands ZERO then LAND.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import tools.run_vision_follow_gate_pd_cruise_tuning as base
from tools.run_vision_follow_gate_pd_cruise_headless import (
    HeadlessLoopbackGcs,
    wait_for_headless_front,
)


def parse_args():
    ap = argparse.ArgumentParser(
        description="Headless gate Batch-2: PD tracking -> close-range commit -> pass-through"
    )
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--control-hz", type=float, default=20.0)

    # Batch-1 field-smooth defaults. Keep these unchanged unless deliberately tuning.
    ap.add_argument("--kp-lat", type=float, default=2.0)
    ap.add_argument("--kd-lat", type=float, default=0.20)
    ap.add_argument("--lat-cap", type=float, default=2.0)
    ap.add_argument("--lat-slew", type=float, default=4.0)
    ap.add_argument("--kp-yaw", type=float, default=20.0)
    ap.add_argument("--kd-yaw", type=float, default=1.0)
    ap.add_argument("--yaw-cap", type=float, default=12.0)
    ap.add_argument("--yaw-slew", type=float, default=60.0)
    ap.add_argument("--yaw-sign", type=int, choices=[-1, 1], default=1)

    # Same adaptive-forward defaults as Batch-1 cruise.
    ap.add_argument("--forward-min", type=float, default=0.60)
    ap.add_argument("--forward-max", type=float, default=2.00)
    ap.add_argument("--forward-slow-error", type=float, default=0.35)
    ap.add_argument("--forward-gamma", type=float, default=1.7)
    ap.add_argument("--forward-slew", type=float, default=2.5)

    # Vision/filter defaults from Batch-1.
    ap.add_argument("--deadband", type=float, default=0.03)
    ap.add_argument("--filter-alpha", type=float, default=0.45)
    ap.add_argument("--d-alpha", type=float, default=0.25)
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--lock-frames", type=int, default=3)

    # Batch-2 near/commit logic.
    ap.add_argument("--near-area", type=float, default=0.12,
                    help="bbox area ratio that arms close-range pass logic")
    ap.add_argument("--commit-area", type=float, default=0.18,
                    help="bbox area ratio for direct COMMIT")
    ap.add_argument("--commit-error", type=float, default=0.12,
                    help="max |filtered EX| while arming/committing")
    ap.add_argument("--near-frames", type=int, default=2,
                    help="consecutive near+center frames before NEAR_ARMED")
    ap.add_argument("--commit-frames", type=int, default=3,
                    help="consecutive commit-area+center frames for hard COMMIT")
    ap.add_argument("--commit-loss-grace", type=float, default=0.30,
                    help="after NEAR_ARMED, detector loss within this time commits pass")
    ap.add_argument("--pass-speed", type=float, default=2.0,
                    help="forward speed during latched pass-through")
    ap.add_argument("--pass-duration", type=float, default=0.80,
                    help="seconds of committed forward flight before test completion")
    ap.add_argument("--pass-lat-cap", type=float, default=0.50,
                    help="small lateral authority while gate is still visible in PASS")
    ap.add_argument("--pass-yaw-cap", type=float, default=5.0,
                    help="small yaw authority while gate is still visible in PASS")

    # Validated takeoff/guards.
    ap.add_argument("--alt", type=float, default=1.20)
    ap.add_argument("--climb-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.25)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15)
    ap.add_argument("--guard-floor", type=float, default=0.80)
    ap.add_argument("--guard-ceiling", type=float, default=1.45)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)
    ap.add_argument("--vision-wait", type=float, default=60.0)
    ap.add_argument("--yes", action="store_true")
    return ap.parse_args()


def validate(a):
    if not 1 <= a.duration <= 120: raise SystemExit("--duration must be 1..120s")
    if not 10 <= a.control_hz <= 30: raise SystemExit("--control-hz must be 10..30Hz")
    if not 0.05 <= a.kp_lat <= 5: raise SystemExit("--kp-lat must be 0.05..5")
    if not 0 <= a.kd_lat <= 2: raise SystemExit("--kd-lat must be 0..2")
    if not 0.05 <= a.lat_cap <= 2.5: raise SystemExit("--lat-cap must be 0.05..2.5m/s")
    if not 1 <= a.kp_yaw <= 80: raise SystemExit("--kp-yaw must be 1..80")
    if not 0 <= a.kd_yaw <= 20: raise SystemExit("--kd-yaw must be 0..20")
    if not 1 <= a.yaw_cap <= 40: raise SystemExit("--yaw-cap must be 1..40deg/s")
    if not 0 <= a.forward_min <= a.forward_max <= 5: raise SystemExit("invalid forward range")
    if not 0.02 <= a.near_area < a.commit_area <= 0.60: raise SystemExit("require 0.02<=near-area<commit-area<=0.60")
    if not 0.03 <= a.commit_error <= 0.30: raise SystemExit("--commit-error must be 0.03..0.30")
    if not 1 <= a.near_frames <= 10 or not 1 <= a.commit_frames <= 10: raise SystemExit("near/commit frames must be 1..10")
    if not 0.05 <= a.commit_loss_grace <= 1.0: raise SystemExit("--commit-loss-grace must be 0.05..1.0s")
    if not 0.3 <= a.pass_speed <= 5.0: raise SystemExit("--pass-speed must be 0.3..5m/s")
    if not 0.2 <= a.pass_duration <= 2.0: raise SystemExit("--pass-duration must be 0.2..2.0s")
    if not 0 <= a.pass_lat_cap <= a.lat_cap: raise SystemExit("invalid --pass-lat-cap")
    if not 0 <= a.pass_yaw_cap <= a.yaw_cap: raise SystemExit("invalid --pass-yaw-cap")
    if not 0.50 <= a.alt <= 3.0: raise SystemExit("--alt must be 0.50..3.0m")
    if a.takeoff_ceiling <= a.alt: raise SystemExit("takeoff ceiling must be above target")
    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 2.0: raise SystemExit("invalid altitude guard")


def main() -> int:
    a = parse_args()
    validate(a)

    cfg = base.shadow_config()
    cfg.setdefault("ai", {}).setdefault("detector", {})["confidence_threshold"] = float(a.confidence)
    cfg.setdefault("vision_control", {})["movement_tx_enabled"] = False
    cfg["vision_control"]["confidence_min"] = float(a.confidence)

    stop_event = threading.Event()
    state = base.SharedState()
    events = base.EventBus()
    waypoints = base.WaypointManager(state)
    gcs = HeadlessLoopbackGcs(state, events, waypoints, stop_event, command_manager=None)
    vision = base.VisionStreamSession(cfg, state, stop_event)
    vision.workers = [vision.front]

    test = base.DirectFlightTest()
    flight_open = False
    pass_complete = False

    print("============================================================")
    print(" KINGPHOENIX GATE BATCH-2 - HEADLESS COMMIT/PASS")
    print("============================================================")
    print(f"[BATCH1 LAT] Kp={a.kp_lat:.2f} Kd={a.kd_lat:.2f} cap={a.lat_cap:.2f}m/s")
    print(f"[BATCH1 YAW] Kp={a.kp_yaw:.1f} Kd={a.kd_yaw:.1f} cap={a.yaw_cap:.1f}deg/s")
    print(f"[FORWARD] adaptive {a.forward_min:.2f}->{a.forward_max:.2f}m/s")
    print(f"[NEAR] area>={a.near_area:.3f}, |EX|<={a.commit_error:.3f}, {a.near_frames} frames")
    print(f"[COMMIT] area>={a.commit_area:.3f}, {a.commit_frames} frames OR near-armed detector loss")
    print(f"[PASS] VX={a.pass_speed:.2f}m/s for {a.pass_duration:.2f}s | lat cap={a.pass_lat_cap:.2f} | yaw cap={a.pass_yaw_cap:.1f}")
    print("[HARD OFF] VZ = 0.00m/s")
    print("------------------------------------------------------------")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise base.FlightTestError("Hailo YOLO26 did not become READY")
        wait_for_headless_front(state, float(a.vision_wait))

        test.open(); flight_open = True
        test.collect_preflight(require_auto=True)
        if not a.yes:
            print("\n[READY] Safety pilot ready. Batch-2 will continue through the gate after COMMIT.")
            ans = input("Type GO to arm and begin, anything else cancels: ").strip().upper()
            if ans != "GO":
                print("[CANCELLED] vehicle remains DISARMED")
                return 0

        base.profiled_lidar_takeoff(
            test, float(a.alt),
            tolerance_m=float(a.tolerance), handoff_m=float(a.handoff),
            fast_rate_mps=float(a.climb_rate), near_rate_mps=float(a.near_rate),
            taper_zone_m=float(a.taper_zone), kick_alt_m=float(a.kick_alt),
            timeout_s=15.0, ceiling_m=float(a.takeoff_ceiling),
            zero_flush_s=float(a.zero_flush),
        )
        if test.mode != "GUIDED": test.set_mode("GUIDED")
        base.zero_command(test)

        period = 1.0 / float(a.control_hz)
        end_time = time.monotonic() + float(a.duration)
        next_tx = last_tx = time.monotonic()
        last_print = 0.0
        filtered_ex = None
        prev_ctrl_error = None
        d_filtered = 0.0
        last_vision_update = None
        last_frame_id = None
        valid_streak = 0
        near_streak = 0
        commit_streak = 0
        near_armed = False
        near_armed_at = None
        last_near_valid_at = None
        commit_latched = False
        commit_at = None
        commit_reason = None
        outside_since = lidar_loss_since = None
        vx_cmd = lat_cmd = yaw_cmd = 0.0
        target_vx = target_lat = target_yaw = 0.0
        ctrl_error = 0.0
        p_lat = d_lat = p_yaw = d_yaw = 0.0
        latest_valid = False
        latest_conf = None
        latest_raw_ex = None
        latest_area = None
        status = "SEARCH"

        while time.monotonic() < end_time:
            test.poll(0.02)
            base.check_flight_health(test)
            if test.mode != "GUIDED":
                try: base.zero_command(test)
                except Exception: pass
                raise base.PilotTakeover(f"mode changed to {test.mode}")

            outside_since, lidar_loss_since = base.check_lidar_guard(
                test,
                floor_m=float(a.guard_floor), ceiling_m=float(a.guard_ceiling),
                outside_since=outside_since, lidar_loss_since=lidar_loss_since,
                max_outside_s=float(a.guard_outside_s), max_stale_s=float(a.lidar_stale_s),
            )

            now = time.monotonic()
            with state.lock:
                front = dict(state.vision.get("front") or {})

            frame_id = front.get("frame_id")
            frame_ts_ns = int(front.get("frame_timestamp_ns") or 0)
            age = float("inf") if frame_ts_ns <= 0 else max(0.0, (time.monotonic_ns()-frame_ts_ns)/1e9)
            conf = front.get("confidence")
            raw_ex = front.get("ex")
            area = front.get("bbox_area_ratio")
            valid = bool(
                front.get("vision_valid", False)
                and conf is not None and float(conf) >= float(a.confidence)
                and raw_ex is not None and age <= 0.20
            )
            latest_valid = valid
            latest_conf = conf
            latest_raw_ex = raw_ex
            latest_area = area

            if not commit_latched and frame_id is not None and frame_id != last_frame_id:
                last_frame_id = frame_id
                if valid:
                    valid_streak += 1
                    ex = base.clamp(float(raw_ex), -1.0, 1.0)
                    if filtered_ex is None:
                        filtered_ex = ex
                    else:
                        fa = float(a.filter_alpha)
                        filtered_ex = fa*ex + (1.0-fa)*float(filtered_ex)

                    db_error = base.deadband_remap(float(filtered_ex), float(a.deadband))
                    ctrl_error = base.response_curve(db_error, "linear", 0.0)
                    if last_vision_update is not None and prev_ctrl_error is not None:
                        dtv = max(1e-3, now-last_vision_update)
                        d_raw = (ctrl_error-float(prev_ctrl_error))/dtv
                        da = float(a.d_alpha)
                        d_filtered = da*d_raw + (1.0-da)*d_filtered
                    else:
                        d_filtered = 0.0
                    prev_ctrl_error = ctrl_error
                    last_vision_update = now

                    p_lat = float(a.kp_lat)*ctrl_error
                    d_lat = float(a.kd_lat)*d_filtered
                    p_yaw = float(a.kp_yaw)*ctrl_error
                    d_yaw = float(a.kd_yaw)*d_filtered

                    if valid_streak >= int(a.lock_frames):
                        target_lat = base.clamp(p_lat+d_lat, -float(a.lat_cap), float(a.lat_cap))
                        target_yaw = base.clamp((p_yaw+d_yaw)*int(a.yaw_sign), -float(a.yaw_cap), float(a.yaw_cap))
                        target_vx = base.adaptive_forward_speed(
                            abs(float(filtered_ex)),
                            vmin=float(a.forward_min), vmax=float(a.forward_max),
                            slow_error=float(a.forward_slow_error), gamma=float(a.forward_gamma),
                        )
                        status = "TRACK"

                    centered = abs(float(filtered_ex)) <= float(a.commit_error)
                    ar = 0.0 if area is None else float(area)
                    if centered and ar >= float(a.near_area):
                        near_streak += 1
                        last_near_valid_at = now
                    else:
                        near_streak = 0
                    if near_streak >= int(a.near_frames):
                        if not near_armed:
                            print(f"[NEAR] ARMED area={ar:.3f} EX={float(filtered_ex):+.3f}")
                        near_armed = True
                        near_armed_at = near_armed_at or now
                        status = "NEAR_ARMED"

                    if centered and ar >= float(a.commit_area):
                        commit_streak += 1
                    else:
                        commit_streak = 0
                    if commit_streak >= int(a.commit_frames):
                        commit_latched = True
                        commit_at = now
                        commit_reason = "AREA+CENTER"
                else:
                    valid_streak = 0
                    target_vx = target_lat = target_yaw = 0.0
                    # Before NEAR_ARMED, loss is still a normal safety stop.
                    if near_armed and last_near_valid_at is not None and now-last_near_valid_at <= float(a.commit_loss_grace):
                        commit_latched = True
                        commit_at = now
                        commit_reason = "EXPECTED_CLOSE_LOSS"
                    else:
                        status = "TARGET LOST"
                        vx_cmd = lat_cmd = yaw_cmd = 0.0
                        base.zero_command(test)

                if commit_latched:
                    print(f"[COMMIT] LATCHED reason={commit_reason}; detector loss is now EXPECTED")
                    print(f"[PASS] forward {a.pass_speed:.2f}m/s for {a.pass_duration:.2f}s")
                    status = "PASS"

            if commit_latched:
                status = "PASS"
                target_vx = float(a.pass_speed)
                # If YOLO still gives a valid frame, permit only small final corrections.
                if latest_valid and filtered_ex is not None:
                    target_lat = base.clamp(p_lat+d_lat, -float(a.pass_lat_cap), float(a.pass_lat_cap))
                    target_yaw = base.clamp((p_yaw+d_yaw)*int(a.yaw_sign), -float(a.pass_yaw_cap), float(a.pass_yaw_cap))
                else:
                    # Detector loss close to/in the gate: hold committed heading.
                    target_lat = 0.0
                    target_yaw = 0.0
                if commit_at is not None and now-commit_at >= float(a.pass_duration):
                    pass_complete = True
                    break

            if now >= next_tx:
                dt = max(1e-3, now-last_tx)
                last_tx = now
                next_tx = now + period
                if commit_latched or (latest_valid and valid_streak >= int(a.lock_frames)):
                    vx_cmd = base.slew(vx_cmd, target_vx, float(a.forward_slew), dt)
                    lat_cmd = base.slew(lat_cmd, target_lat, float(a.lat_slew), dt)
                    yaw_cmd = base.slew(yaw_cmd, target_yaw, float(a.yaw_slew), dt)
                else:
                    vx_cmd = lat_cmd = yaw_cmd = 0.0
                base.send_body_command(test, forward_mps=vx_cmd, lateral_mps=lat_cmd, yaw_rate_dps=yaw_cmd)

            if now-last_print >= 0.20:
                conf_t = "--" if latest_conf is None else f"{float(latest_conf):.2f}"
                ex_t = "--" if latest_raw_ex is None else f"{float(latest_raw_ex):+.3f}"
                area_t = "--" if latest_area is None else f"{float(latest_area):.3f}"
                lidar_t = "--" if test.lidar is None else f"{float(test.lidar):.2f}"
                print(
                    f"[GATE2] {status:12s} conf={conf_t} EX={ex_t} area={area_t} "
                    f"near={int(near_armed)} commit={int(commit_latched)} | "
                    f"VX={vx_cmd:+.2f} LAT={lat_cmd:+.2f} YAW={yaw_cmd:+.1f} | LiDAR={lidar_t}m"
                )
                last_print = now

        base.zero_command(test)
        if pass_complete:
            print("[PASS] COMPLETE -> ZERO -> LAND")
        else:
            print("[FOLLOW] duration ended without completed pass -> ZERO -> LAND")
        test.land_and_wait()
        print("[RESULT] GATE BATCH-2 TEST COMPLETE")
        return 0

    except base.PilotTakeover as exc:
        print(f"[PILOT TAKEOVER] {exc}; TX stopped; runner will not force LAND")
        return 0
    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        if flight_open:
            try: base.zero_command(test)
            except Exception: pass
            test.land_best_effort("operator interrupt")
        return 130
    except base.FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        if flight_open:
            try: base.zero_command(test)
            except Exception: pass
            test.land_best_effort(str(exc))
        return 2
    except Exception as exc:
        print(f"[UNEXPECTED] {type(exc).__name__}: {exc}")
        if flight_open:
            try: base.zero_command(test)
            except Exception: pass
            test.land_best_effort(f"unexpected {type(exc).__name__}: {exc}")
        return 3
    finally:
        stop_event.set()
        try:
            if flight_open: test.close()
        except Exception: pass
        try: vision.join(timeout=1.0)
        except Exception: pass


if __name__ == "__main__":
    raise SystemExit(main())
