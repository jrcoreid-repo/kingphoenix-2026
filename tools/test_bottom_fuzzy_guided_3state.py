#!/usr/bin/env python3
"""Standalone 3-state PURE-FUZZY bottom-vision GUIDED flight test.

Goal: field-friendly controller with only three fuzzy distance states:
  MINIMUM -> MID -> FAR

No AUTO mission, no waypoint navigation, no payload drop, no Kp/Kd, and no
software LAND. The FC keeps attitude/altitude stabilization; Raspberry Pi sends
BODY XY velocity setpoints only.

Flow:
  GCS + FRONT/BOTTOM streams + shared YOLO preload
  -> operator GO
  -> profiled LiDAR takeoff to 2.0 m AGL
  -> bottom YOLO ON
  -> 3-state fuzzy BODY XY cruise
  -> target inside centre box for a few frames
  -> XY ZERO + AI OFF
  -> remain GUIDED/hover until operator changes mode or lands manually.

The controller uses only radial image error r=sqrt(ex^2+ey^2). Memberships are
linearly blended so FAR->MID->MINIMUM transitions are smooth. A final XY slew
limiter prevents an instantaneous velocity reversal. Target loss always sends
immediate XY ZERO.
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
from tools.run_bottom_target_pd_tuning import bottom_config, clamp, send_body_xy, slew, zero_xy
from tools.run_hybrid_drop_mission_v1 import (
    PilotTakeover,
    activate_bottom_ai,
    check_flight_health,
    check_lidar_guard,
    disable_ai,
    wait_streams,
)

# Field-friendly fixed smoothing. These are deliberately not exposed as fuzzy
# states: only three fuzzy states exist below.
FILTER_ALPHA = 0.40
XY_SLEW_MPS2 = 1.50
X_SIGN = -1
Y_SIGN = -1
LOST_TIMEOUT_S = 0.15


def parse_args():
    ap = argparse.ArgumentParser(
        description="3-state PURE FUZZY test: LiDAR 2m -> GUIDED YOLO -> manual landing"
    )

    # Takeoff only.
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

    # Vision / centre.
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--control-hz", type=float, default=20.0)
    ap.add_argument("--fuzzy-timeout", type=float, default=30.0)
    ap.add_argument("--center-error", type=float, default=0.12)
    ap.add_argument("--center-frames", type=int, default=2)
    ap.add_argument(
        "--center-offset-x",
        type=float,
        default=0.0,
        help=(
            "horizontal virtual-center offset as fraction of FULL frame width; "
            "+ shifts desired center right, - shifts it left (e.g. +0.05 = 5% width)"
        ),
    )

    # ONLY THREE fuzzy states.
    # r <= mid-error starts in MINIMUM territory.
    # r == mid-error is MID peak.
    # r >= far-error becomes FAR.
    ap.add_argument("--mid-error", type=float, default=0.25)
    ap.add_argument("--far-error", type=float, default=0.45)
    ap.add_argument("--minimum-speed", type=float, default=0.12)
    ap.add_argument("--mid-speed", type=float, default=0.30)
    ap.add_argument("--far-speed", type=float, default=0.60)

    # 2m flight guard.
    ap.add_argument("--guard-floor", type=float, default=1.50)
    ap.add_argument("--guard-ceiling", type=float, default=2.35)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)

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
    if not 1 <= a.center_frames <= 10:
        raise SystemExit("--center-frames must be 1..10")
    if not -0.25 <= float(a.center_offset_x) <= 0.25:
        raise SystemExit("--center-offset-x must be -0.25..+0.25 of full frame width")
    if not 0.03 <= a.minimum_speed <= a.mid_speed <= a.far_speed <= 2.0:
        raise SystemExit("require minimum-speed <= mid-speed <= far-speed <= 2.0m/s")
    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 3.0:
        raise SystemExit("invalid LiDAR guard")


def memberships_3state(radial: float, a):
    """Piecewise triangular/shoulder membership for MINIMUM/MID/FAR.

    Exactly one blend region is active at a time, keeping field tuning simple.
    Returns (mu_min, mu_mid, mu_far).
    """
    r = max(0.0, float(radial))
    c = float(a.center_error)
    m = float(a.mid_error)
    f = float(a.far_error)

    if r <= c:
        return 1.0, 0.0, 0.0
    if r < m:
        t = clamp((r - c) / (m - c), 0.0, 1.0)
        return 1.0 - t, t, 0.0
    if r < f:
        t = clamp((r - m) / (f - m), 0.0, 1.0)
        return 0.0, 1.0 - t, t
    return 0.0, 0.0, 1.0


def fuzzy_speed_3state(radial: float, a):
    mu_min, mu_mid, mu_far = memberships_3state(radial, a)
    speed = (
        mu_min * float(a.minimum_speed)
        + mu_mid * float(a.mid_speed)
        + mu_far * float(a.far_speed)
    )
    return speed, (mu_min, mu_mid, mu_far)


def run_alignment(test, state, vision, a, detector_class_id: int):
    activate_bottom_ai(vision, detector_class_id)
    test.set_mode("GUIDED")
    zero_xy(test)

    print("[FUZZY3] ACTIVE: PURE 3-state fuzzy, NO Kp/Kd, NO trend states")
    print(
        f"[FUZZY3] error centre/mid/far={a.center_error:.2f}/{a.mid_error:.2f}/{a.far_error:.2f} | "
        f"speed MIN/MID/FAR={a.minimum_speed:.2f}/{a.mid_speed:.2f}/{a.far_speed:.2f}m/s"
    )
    print(
        f"[FUZZY3] virtual center X offset={a.center_offset_x:+.3f} of full frame width "
        f"(normalized ex shift={2.0 * float(a.center_offset_x):+.3f})"
    )
    print(f"[FUZZY3] fixed smoothing: alpha={FILTER_ALPHA:.2f}, XY slew={XY_SLEW_MPS2:.2f}m/s^2")

    deadline = time.monotonic() + float(a.fuzzy_timeout)
    period = 1.0 / float(a.control_hz)
    next_tx = 0.0
    last_print = 0.0
    last_frame = None
    center_streak = 0
    lost_since = None
    filt_bx = filt_by = None
    vx_cmd = vy_cmd = 0.0
    outside_since = lidar_loss_since = None

    while time.monotonic() < deadline:
        test.poll(0.02)
        check_flight_health(test)
        if test.mode != "GUIDED":
            zero_xy(test)
            raise PilotTakeover(f"operator changed mode during fuzzy cruise to {test.mode}")

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
        radial = 0.0
        speed = 0.0
        mus = (0.0, 0.0, 0.0)

        if valid and frame_id != last_frame:
            last_frame = frame_id
            lost_since = None
            raw_ex = float(ex)
            raw_ey = float(ey)

            # bbox_control normalizes ex by half-frame width: ex=(cx-W/2)/(W/2).
            # CLI offset is a fraction of FULL frame width, so +0.05 means
            # desired center 5% of image width to the RIGHT.
            adj_ex = clamp(raw_ex - 2.0 * float(a.center_offset_x), -1.0, 1.0)
            adj_ey = raw_ey

            if abs(adj_ex) <= float(a.center_error) and abs(adj_ey) <= float(a.center_error):
                center_streak += 1
            else:
                center_streak = 0

            if center_streak >= int(a.center_frames):
                zero_xy(test)
                disable_ai(vision, "FUZZY3_CENTERED")
                print(
                    f"[FUZZY3 CENTER] rawEX={raw_ex:+.3f} adjEX={adj_ex:+.3f} ey={adj_ey:+.3f} "
                    f"for {center_streak} frame(s) -> XY ZERO"
                )
                return "CENTERED"

            # Preserve field-validated camera/body sign mapping.
            bx = clamp(-adj_ex * X_SIGN, -1.0, 1.0)
            by = clamp(-adj_ey * Y_SIGN, -1.0, 1.0)

            if filt_bx is None:
                filt_bx, filt_by = bx, by
            else:
                filt_bx = FILTER_ALPHA * bx + (1.0 - FILTER_ALPHA) * filt_bx
                filt_by = FILTER_ALPHA * by + (1.0 - FILTER_ALPHA) * filt_by

            radial = math.hypot(filt_bx, filt_by)
            speed, mus = fuzzy_speed_3state(radial, a)

            if mus[2] >= max(mus[0], mus[1]):
                phase = "FAR"
            elif mus[1] >= mus[0]:
                phase = "MID"
            else:
                phase = "MINIMUM"

            norm = math.hypot(filt_bx, filt_by)
            if norm > 1e-6:
                target_vx = speed * filt_bx / norm
                target_vy = speed * filt_by / norm
            else:
                target_vx = target_vy = 0.0

            vx_cmd = slew(vx_cmd, target_vx, XY_SLEW_MPS2, period)
            vy_cmd = slew(vy_cmd, target_vy, XY_SLEW_MPS2, period)

        elif not valid:
            center_streak = 0
            lost_since = lost_since or now
            vx_cmd = vy_cmd = 0.0
            phase = "TARGET LOST"
            zero_xy(test)
            if now - lost_since >= LOST_TIMEOUT_S:
                filt_bx = filt_by = None

        if now >= next_tx:
            send_body_xy(test, vx_cmd, vy_cmd)
            next_tx = now + period

        if now - last_print >= 0.20:
            ex_t = "--" if ex is None else f"{float(ex):+.3f}"
            ey_t = "--" if ey is None else f"{float(ey):+.3f}"
            conf_t = "--" if conf is None else f"{float(conf):.2f}"
            print(
                f"[FUZZY3XY] {phase:11s} conf={conf_t} rawEX={ex_t} "
                f"adjEX={(float(ex)-2.0*float(a.center_offset_x)):+.3f} ey={ey_t} r={radial:.3f} | "
                f"mu(MIN/MID/FAR)={mus[0]:.2f}/{mus[1]:.2f}/{mus[2]:.2f} "
                f"spd={speed:.2f} VX={vx_cmd:+.2f} VY={vy_cmd:+.2f} "
                f"center={center_streak}/{a.center_frames}"
            )
            last_print = now

        time.sleep(0.003)

    zero_xy(test)
    disable_ai(vision, "FUZZY3_TIMEOUT")
    print(f"[FUZZY3 TIMEOUT] {a.fuzzy_timeout:.1f}s -> XY ZERO; no LAND commanded")
    return "TIMEOUT"


def hold_for_manual_landing(test):
    print("[MANUAL LAND] Fuzzy test finished. XY ZERO. Land/change mode from laptop.")
    print("[MANUAL LAND] Script will stop commanding when FC leaves GUIDED.")
    next_zero = 0.0
    while True:
        test.poll(0.10)
        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost while waiting for manual landing")
        if test.mode != "GUIDED":
            print(f"[MANUAL LAND] FC mode changed to {test.mode}; releasing controller")
            return
        if not test.armed:
            print("[MANUAL LAND] vehicle DISARMED")
            return
        now = time.monotonic()
        if now >= next_zero:
            zero_xy(test)
            next_zero = now + 0.10


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
    disable_ai(vision, "FUZZY3_INIT")

    test = DirectFlightTest()
    flight_open = False

    print("============================================================")
    print(" KINGPHOENIX PURE FUZZY 3-STATE GUIDED VISION TEST")
    print("============================================================")
    print(f"[PLAN] LiDAR takeoff {a.takeoff_alt:.2f}m -> GUIDED bottom YOLO -> manual LAND")
    print("[FUZZY] EXACTLY 3 states: FAR / MID / MINIMUM; no Kp, no Kd, no trend state")
    print("[OUTPUT] Pi sends BODY XY velocity only; FC owns attitude + altitude")
    print("[LAND] no software LAND in this runner")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("shared Hailo YOLO26 did not become READY")
        wait_streams(state, float(a.stream_wait))

        test.open()
        flight_open = True
        test.collect_preflight(require_auto=False)

        require_word(
            f"Streams ready. Props clear. Takeoff to {a.takeoff_alt:.2f}m, then fuzzy cruise. Type GO: ",
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

        run_alignment(test, state, vision, a, detector_class_id)
        hold_for_manual_landing(test)
        return 0

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
        return 130

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt -> XY ZERO; no software LAND")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        return 130

    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
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
