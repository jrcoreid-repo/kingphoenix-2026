#!/usr/bin/env python3
"""Standalone PURE-FUZZY bottom-vision GUIDED flight test.

Purpose
-------
Isolate only the difficult part of the drop mission: GUIDED + bottom YOLO XY
alignment.  There is no AUTO mission, no waypoint navigation, no payload drop,
and no software LAND in this runner.

Flow
----
  real GCS/streams + shared YOLO preload
  -> operator GO
  -> profiled LiDAR takeoff to 2.0 m AGL (default)
  -> bottom YOLO ON
  -> PURE Sugeno-style fuzzy BODY XY velocity controller
  -> target centred for N consecutive frames
  -> XY ZERO + AI OFF
  -> hover in GUIDED until operator changes mode / lands manually from laptop

The FC still owns attitude, altitude and motor stabilization.  Raspberry Pi
only sends BODY XY velocity setpoints.  No Kp/Kd terms exist in this script.
The only non-fuzzy shaping after inference is an XY slew-rate limiter to avoid
an instantaneous stick reversal.
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
from tools.flight_test_common import DirectFlightTest, FlightTestError
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_bottom_drop_aggressive_headless import require_word
from tools.run_bottom_target_pd_headless import HeadlessLoopbackGcs
from tools.run_bottom_target_pd_tuning import (
    bottom_config,
    clamp,
    send_body_xy,
    slew,
    zero_xy,
)
from tools.run_hybrid_drop_mission_v1 import (
    PilotTakeover,
    activate_bottom_ai,
    check_flight_health,
    check_lidar_guard,
    disable_ai,
    wait_streams,
)


def parse_args():
    ap = argparse.ArgumentParser(
        description=(
            "PURE FUZZY bottom-vision test: LiDAR takeoff -> GUIDED YOLO fuzzy XY "
            "-> centre -> manual laptop landing"
        )
    )

    # Takeoff only. No AUTO mission exists in this test.
    ap.add_argument("--takeoff-alt", type=float, default=2.00)
    ap.add_argument("--takeoff-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.30)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=2.35)
    ap.add_argument("--zero-flush", type=float, default=0.15)
    ap.add_argument("--takeoff-timeout", type=float, default=20.0)

    # Vision / centre criterion.
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--control-hz", type=float, default=20.0)
    ap.add_argument("--fuzzy-timeout", type=float, default=20.0)
    ap.add_argument("--center-error", type=float, default=0.10)
    ap.add_argument("--center-frames", type=int, default=5)

    # PURE fuzzy error memberships and singleton output speeds.
    # Error input = radial normalized image error sqrt(ex^2 + ey^2).
    ap.add_argument("--mid-error", type=float, default=0.25)
    ap.add_argument("--far-error", type=float, default=0.45)
    ap.add_argument("--near-speed", type=float, default=0.10)
    ap.add_argument("--mid-speed", type=float, default=0.30)
    ap.add_argument("--far-speed", type=float, default=0.60)

    # Sensor/output smoothing only; these are NOT proportional/derivative gains.
    ap.add_argument("--filter-alpha", type=float, default=0.40)
    ap.add_argument("--rate-alpha", type=float, default=0.25)
    ap.add_argument("--xy-slew", type=float, default=1.50)
    ap.add_argument("--x-sign", type=int, choices=[-1, 1], default=-1)
    ap.add_argument("--y-sign", type=int, choices=[-1, 1], default=-1)
    ap.add_argument("--lost-timeout", type=float, default=0.15)

    # 2 m hover guard for this standalone test.
    ap.add_argument("--guard-floor", type=float, default=1.50)
    ap.add_argument("--guard-ceiling", type=float, default=2.35)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)

    # Streaming/GCS.
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--stream-wait", type=float, default=60.0)
    return ap.parse_args()


def validate(a):
    if not 0.50 <= a.takeoff_alt <= 3.0:
        raise SystemExit("--takeoff-alt must be 0.50..3.00m")
    if a.takeoff_ceiling <= a.takeoff_alt:
        raise SystemExit("--takeoff-ceiling must be above --takeoff-alt")
    if not 0.10 <= a.takeoff_rate <= 0.80:
        raise SystemExit("--takeoff-rate must be 0.10..0.80m/s")
    if not 0.08 <= a.near_rate <= a.takeoff_rate:
        raise SystemExit("--near-rate must be 0.08..takeoff-rate")
    if not 10 <= a.control_hz <= 30:
        raise SystemExit("--control-hz must be 10..30Hz")
    if not 2.0 <= a.fuzzy_timeout <= 120.0:
        raise SystemExit("--fuzzy-timeout must be 2..120s")
    if not 0.02 <= a.center_error < a.mid_error < a.far_error <= 1.0:
        raise SystemExit("require center-error < mid-error < far-error <= 1.0")
    if not 1 <= a.center_frames <= 30:
        raise SystemExit("--center-frames must be 1..30")
    if not 0.03 <= a.near_speed <= a.mid_speed <= a.far_speed <= 2.0:
        raise SystemExit("require near-speed <= mid-speed <= far-speed <= 2.0m/s")
    if not 0.05 <= a.filter_alpha <= 1.0 or not 0.05 <= a.rate_alpha <= 1.0:
        raise SystemExit("filter/rate alpha must be 0.05..1.0")
    if not 0.2 <= a.xy_slew <= 12.0:
        raise SystemExit("--xy-slew must be 0.2..12m/s^2")
    if not 0.0 <= a.lost_timeout <= 1.0:
        raise SystemExit("--lost-timeout must be 0..1s")
    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 3.0:
        raise SystemExit("invalid LiDAR guard")


def _smoothstep(lo: float, hi: float, x: float) -> float:
    if hi <= lo:
        return 1.0 if x >= hi else 0.0
    t = clamp((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def error_memberships(radial: float, a):
    """Fuzzy memberships for NEAR / MID / FAR image error."""
    # NEAR fades from 1 at centre_error to 0 at mid_error.
    mu_near = 1.0 - _smoothstep(float(a.center_error), float(a.mid_error), radial)
    # FAR rises from 0 at mid_error to 1 at far_error.
    mu_far = _smoothstep(float(a.mid_error), float(a.far_error), radial)
    # MID owns the remaining region; normalization is done in inference.
    mu_mid = max(0.0, 1.0 - mu_near - mu_far)
    return mu_near, mu_mid, mu_far


def trend_memberships(radial_rate: float):
    """Fuzzy memberships for CLOSING / STABLE / DIVERGING.

    radial_rate < 0 means target is moving toward image centre.
    radial_rate > 0 means error is growing.
    """
    mu_closing = _smoothstep(0.05, 0.30, -float(radial_rate))
    mu_diverging = _smoothstep(0.05, 0.18, float(radial_rate))
    mu_stable = max(0.0, 1.0 - max(mu_closing, mu_diverging))
    total = max(1e-6, mu_closing + mu_stable + mu_diverging)
    return mu_closing / total, mu_stable / total, mu_diverging / total


def fuzzy_infer_speed(radial: float, radial_rate: float, a):
    """Zero-order Sugeno fuzzy inference; no Kp/Kd anywhere.

    Rule intent:
      - FAR: strong stick while stable/diverging, ease if already closing fast.
      - MID: medium stick, further eased while closing.
      - NEAR: tiny stick; if error grows, allow a slightly stronger correction.
    """
    mn, mm, mf = error_memberships(radial, a)
    mc, ms, md = trend_memberships(radial_rate)

    near = float(a.near_speed)
    mid = float(a.mid_speed)
    far = float(a.far_speed)

    # 3x3 Sugeno singleton rule table: rows NEAR/MID/FAR,
    # columns CLOSING/STABLE/DIVERGING.
    outputs = (
        (near * 0.60, near, min(mid, near * 1.50)),
        (mid * 0.65, mid, min(far, mid * 1.25)),
        (far * 0.75, far, far),
    )
    em = (mn, mm, mf)
    tm = (mc, ms, md)

    num = 0.0
    den = 0.0
    for i in range(3):
        for j in range(3):
            w = em[i] * tm[j]
            num += w * outputs[i][j]
            den += w

    speed = 0.0 if den <= 1e-9 else num / den
    speed = clamp(speed, 0.0, far)
    return speed, (mn, mm, mf), (mc, ms, md)


def run_fuzzy_alignment(test, state, vision, a, detector_class_id: int):
    activate_bottom_ai(vision, detector_class_id)
    test.set_mode("GUIDED")
    zero_xy(test)

    print("[FUZZY] PURE FUZZY controller ACTIVE - no Kp/Kd")
    print(
        f"[FUZZY] error sets center/mid/far={a.center_error:.2f}/{a.mid_error:.2f}/{a.far_error:.2f} | "
        f"speed sets near/mid/far={a.near_speed:.2f}/{a.mid_speed:.2f}/{a.far_speed:.2f}m/s"
    )

    deadline = time.monotonic() + float(a.fuzzy_timeout)
    period = 1.0 / float(a.control_hz)
    next_tx = 0.0
    last_print = 0.0
    last_frame = None
    center_streak = 0
    lost_since = None
    filt_bx = filt_by = None
    prev_radial = None
    prev_t = None
    radial_rate_f = 0.0
    vx_cmd = vy_cmd = 0.0
    outside_since = lidar_loss_since = None

    while time.monotonic() < deadline:
        test.poll(0.02)
        check_flight_health(test)
        if test.mode != "GUIDED":
            zero_xy(test)
            raise PilotTakeover(f"operator changed mode during fuzzy alignment to {test.mode}")

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
        age = float("inf") if ts_ns <= 0 else max(0.0, (time.monotonic_ns() - ts_ns) / 1e9)
        valid = bool(
            b.get("vision_valid")
            and conf is not None
            and float(conf) >= float(a.confidence)
            and ex is not None
            and ey is not None
            and age <= 0.20
        )

        phase = "SEARCH"
        speed = 0.0
        radial = 0.0
        em = (0.0, 0.0, 0.0)
        tm = (0.0, 0.0, 0.0)

        if valid and frame_id != last_frame:
            last_frame = frame_id
            lost_since = None
            raw_ex = float(ex)
            raw_ey = float(ey)

            # Centre criterion uses RAW detector error, not filtered control error.
            if abs(raw_ex) <= float(a.center_error) and abs(raw_ey) <= float(a.center_error):
                center_streak += 1
            else:
                center_streak = 0

            if center_streak >= int(a.center_frames):
                zero_xy(test)
                print(
                    f"[CENTERED] rawEX={raw_ex:+.3f} rawEY={raw_ey:+.3f} "
                    f"for {center_streak} frames -> XY ZERO"
                )
                return True

            # Field-validated camera/body sign mapping only. No P or D gain.
            bx = clamp(-raw_ex * int(a.x_sign), -1.0, 1.0)
            by = clamp(-raw_ey * int(a.y_sign), -1.0, 1.0)

            if filt_bx is None:
                filt_bx, filt_by = bx, by
            else:
                alpha = float(a.filter_alpha)
                filt_bx = alpha * bx + (1.0 - alpha) * filt_bx
                filt_by = alpha * by + (1.0 - alpha) * filt_by

            radial = math.hypot(float(filt_bx), float(filt_by))
            if prev_t is not None and prev_radial is not None:
                dt = max(1e-3, now - prev_t)
                raw_rate = (radial - prev_radial) / dt
                ra = float(a.rate_alpha)
                radial_rate_f = ra * raw_rate + (1.0 - ra) * radial_rate_f
            else:
                radial_rate_f = 0.0
            prev_radial, prev_t = radial, now

            speed, em, tm = fuzzy_infer_speed(radial, radial_rate_f, a)

            # Direction comes only from signed camera error. Fuzzy inference
            # decides the magnitude of the virtual stick/velocity command.
            norm = math.hypot(float(filt_bx), float(filt_by))
            if norm > 1e-6:
                target_vx = speed * float(filt_bx) / norm
                target_vy = speed * float(filt_by) / norm
            else:
                target_vx = target_vy = 0.0

            # Final actuator-command rate limiter only; not part of fuzzy law.
            vx_cmd = slew(vx_cmd, target_vx, float(a.xy_slew), period)
            vy_cmd = slew(vy_cmd, target_vy, float(a.xy_slew), period)

            if em[2] >= max(em[0], em[1]):
                phase = "FAR"
            elif em[1] >= em[0]:
                phase = "MID"
            else:
                phase = "NEAR"

        elif not valid:
            center_streak = 0
            lost_since = lost_since or now
            vx_cmd = vy_cmd = 0.0
            zero_xy(test)
            phase = "TARGET LOST"
            if now - lost_since >= float(a.lost_timeout):
                filt_bx = filt_by = None
                prev_radial = prev_t = None
                radial_rate_f = 0.0

        if now >= next_tx:
            send_body_xy(test, vx_cmd, vy_cmd)
            next_tx = now + period

        if now - last_print >= 0.20:
            ex_t = "--" if ex is None else f"{float(ex):+.3f}"
            ey_t = "--" if ey is None else f"{float(ey):+.3f}"
            conf_t = "--" if conf is None else f"{float(conf):.2f}"
            print(
                f"[FUZZYXY] {phase:11s} conf={conf_t} ex={ex_t} ey={ey_t} "
                f"r={radial:.3f} dr={radial_rate_f:+.2f} "
                f"E(N/M/F)={em[0]:.2f}/{em[1]:.2f}/{em[2]:.2f} "
                f"T(C/S/D)={tm[0]:.2f}/{tm[1]:.2f}/{tm[2]:.2f} "
                f"spd={speed:.2f} VX={vx_cmd:+.2f} VY={vy_cmd:+.2f} "
                f"center={center_streak}/{a.center_frames}"
            )
            last_print = now

        time.sleep(0.003)

    zero_xy(test)
    print(f"[FUZZY TIMEOUT] {a.fuzzy_timeout:.1f}s -> XY ZERO, NO AUTO LAND")
    return False


def wait_for_manual_landing(test: DirectFlightTest):
    """Hold zero only while still in GUIDED; never command LAND."""
    print("============================================================")
    print(" FUZZY TEST FINISHED - MANUAL LANDING FROM LAPTOP")
    print("============================================================")
    print("[MANUAL] While mode remains GUIDED, runner sends XY=0 to hold position.")
    print("[MANUAL] Change mode / LAND from laptop. No software LAND will be sent here.")

    next_zero = 0.0
    last_mode = test.mode
    while test.armed:
        test.poll(0.10)
        now = time.monotonic()
        if test.mode != last_mode:
            print(f"[MANUAL] mode {last_mode} -> {test.mode}; fuzzy XY control released")
            last_mode = test.mode

        if test.mode == "GUIDED" and now >= next_zero:
            send_body_xy(test, 0.0, 0.0)
            next_zero = now + 0.20

        if not test.heartbeat_ok():
            print("[MANUAL] heartbeat stale; stop sending commands and rely on operator")
            return

    print("[MANUAL] vehicle DISARMED - test complete")


def main() -> int:
    a = parse_args()
    validate(a)

    cfg = bottom_config(float(a.confidence))
    detector_class_id = int(cfg.get("ai", {}).get("bottom", {}).get("detector_class_id", 0))

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
    disable_ai(vision, "PURE_FUZZY_INIT")

    test = DirectFlightTest()
    flight_open = False

    print("============================================================")
    print(" KINGPHOENIX PURE FUZZY GUIDED + BOTTOM YOLO TEST")
    print("============================================================")
    print(f"[PLAN] LiDAR takeoff {a.takeoff_alt:.2f}m -> PURE FUZZY vision -> manual laptop LAND")
    print("[CONTROL] NO AUTO mission | NO Kp/Kd | NO payload drop | NO software LAND")
    print("[FC] attitude + altitude stabilization remains inside ArduPilot")
    print("------------------------------------------------------------")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("shared Hailo YOLO26 did not become READY")
        wait_streams(state, float(a.stream_wait))

        test.open()
        flight_open = True
        test.collect_preflight(require_auto=False)
        print(
            f"[PREARM] LiDAR={test.lidar if test.lidar is not None else float('nan'):.2f}m "
            f"mode={test.mode} armed={test.armed}"
        )

        require_word(
            f"Safety pilot ready. Takeoff to {a.takeoff_alt:.2f}m and start PURE FUZZY test. Type GO: ",
            "GO",
        )

        profiled_lidar_takeoff(
            test,
            float(a.takeoff_alt),
            tolerance_m=float(a.tolerance),
            handoff_m=float(a.handoff),
            fast_rate_mps=float(a.takeoff_rate),
            near_rate_mps=float(a.near_rate),
            taper_zone_m=float(a.taper_zone),
            kick_alt_m=float(a.kick_alt),
            timeout_s=float(a.takeoff_timeout),
            ceiling_m=float(a.takeoff_ceiling),
            zero_flush_s=float(a.zero_flush),
        )

        print(
            f"[TAKEOFF DONE] LiDAR={test.lidar if test.lidar is not None else float('nan'):.2f}m "
            "-> BOTTOM YOLO + PURE FUZZY"
        )

        centered = run_fuzzy_alignment(test, state, vision, a, detector_class_id)
        zero_xy(test)
        disable_ai(vision, "PURE_FUZZY_CENTERED" if centered else "PURE_FUZZY_TIMEOUT")
        wait_for_manual_landing(test)
        return 0

    except PilotTakeover as exc:
        print(f"[OPERATOR TAKEOVER] {exc}")
        try:
            if flight_open and test.mode == "GUIDED":
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "OPERATOR_TAKEOVER")
        except Exception:
            pass
        print("[MANUAL] no software LAND commanded")
        return 130

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        try:
            if flight_open and test.mode == "GUIDED":
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "OPERATOR_INTERRUPT")
        except Exception:
            pass
        print("[MANUAL] no software LAND commanded")
        return 130

    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        try:
            if flight_open and test.mode == "GUIDED":
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "FUZZY_ABORT")
        except Exception:
            pass
        if flight_open and test.armed:
            print("[RC/LAPTOP TAKEOVER] vehicle still armed; NO software LAND commanded")
        return 2

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
