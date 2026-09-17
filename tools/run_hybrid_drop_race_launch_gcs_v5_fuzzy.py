#!/usr/bin/env python3
"""Kingphoenix hybrid drop V5: V4 AUTO profile + fuzzy stick-like GUIDED vision.

V4 AUTO/race-launch behavior is intentionally preserved.  V5 changes only the
WP1 -> GUIDED -> BOTTOM YOLO alignment/drop controller.

The FC remains responsible for attitude/altitude stabilization.  The Pi sends
BODY XY velocity setpoints only.  Instead of PD magnitude = Kp*error + Kd*d,
V5 uses fuzzy error memberships to choose a stick-like speed:

  NEAR  -> final-cap
  MID   -> brake-cap
  FAR   -> far-cap

Memberships are blended continuously, so there are no hard speed steps.
A radial-error trend rule eases command speed while closing quickly and adds a
small boost if the target is diverging.  A near-centre sign reversal is treated
as overshoot and temporarily limits the reverse correction to a small bump.

Drop release gating, target-loss ZERO, LiDAR guards, servo logic, GCS flow,
1.70 m waypoint profile and 10 s V4 search/drop budget are preserved.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import tools.run_hybrid_drop_mission_v1 as hybrid_v1
import tools.run_hybrid_drop_race_launch_gcs_v3 as race_v3
import tools.run_hybrid_drop_race_launch_gcs_v4 as race_v4

# V5 fuzzy behaviour constants.  Existing CLI knobs still set the principal
# thresholds/speeds: drop-error, brake-error, far-error, final/brake/far-cap,
# xy-slew, filter-alpha, d-alpha and deadbands.
FUZZY_APPROACH_FAST_RATE = -0.30   # normalized radial error / s
FUZZY_DIVERGING_RATE = 0.18        # normalized radial error / s
FUZZY_APPROACH_FAST_SCALE = 0.72
FUZZY_DIVERGING_SCALE = 1.08
FUZZY_REVERSE_CAP_MPS = 0.18
FUZZY_REVERSE_HOLD_S = 0.30
FUZZY_CROSS_WINDOW = 0.35


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _smoothstep(lo: float, hi: float, x: float) -> float:
    if hi <= lo:
        return 1.0 if x >= hi else 0.0
    t = _clamp((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _fuzzy_speed(radial: float, radial_rate: float, a):
    """Weighted fuzzy speed from radial image error and closing/diverging trend."""
    drop_e = float(a.drop_error)
    brake_e = float(a.brake_error)
    far_e = float(a.far_error)

    # Error memberships.  These sum to ~1 through the two blend regions.
    mu_near = 1.0 - _smoothstep(drop_e, brake_e, radial)
    mu_far = _smoothstep(brake_e, far_e, radial)
    mu_mid = max(0.0, 1.0 - mu_near - mu_far)
    total = max(1e-6, mu_near + mu_mid + mu_far)

    speed = (
        mu_near * float(a.final_cap)
        + mu_mid * float(a.brake_cap)
        + mu_far * float(a.far_cap)
    ) / total

    # Fuzzy trend rule: if image error is collapsing quickly, ease early to
    # avoid sailing through centre; if it is growing, add only a mild boost.
    if radial_rate <= FUZZY_APPROACH_FAST_RATE:
        trend = "CLOSING_FAST"
        speed *= FUZZY_APPROACH_FAST_SCALE
    elif radial_rate >= FUZZY_DIVERGING_RATE:
        trend = "DIVERGING"
        speed *= FUZZY_DIVERGING_SCALE
    else:
        trend = "STABLE"

    speed = _clamp(speed, 0.0, float(a.far_cap))
    return speed, mu_near, mu_mid, mu_far, trend


def parse_args_v5():
    a = race_v4.parse_args_v4()
    print(
        "[V5 FUZZY] GUIDED vision only | "
        f"NEAR={a.final_cap:.2f} MID={a.brake_cap:.2f} FAR={a.far_cap:.2f}m/s | "
        f"drop/brake/far={a.drop_error:.2f}/{a.brake_error:.2f}/{a.far_error:.2f} | "
        f"reverse-cap={FUZZY_REVERSE_CAP_MPS:.2f}m/s"
    )
    print("[V5 FUZZY] AUTO navigation/race-launch logic is unchanged from V4")
    print("[V5 FUZZY] --kp-x/--kd-x/--kp-y/--kd-y accepted for compatibility but not used for speed magnitude")
    return a


def validate_v5(a):
    race_v4.validate_v4(a)
    if not 0.01 <= float(a.drop_error) < float(a.brake_error) < float(a.far_error) <= 1.0:
        raise SystemExit("V5 fuzzy requires drop-error < brake-error < far-error <= 1.0")
    if not 0.05 <= float(a.final_cap) <= float(a.brake_cap) <= float(a.far_cap) <= 2.5:
        raise SystemExit("V5 fuzzy requires final-cap <= brake-cap <= far-cap <= 2.5")


def run_fuzzy_drop(test, state, vision, a, detector_class_id: int, drop_start: float):
    """GUIDED bottom-YOLO stage using fuzzy stick-like BODY velocity commands."""
    hybrid_v1.activate_bottom_ai(vision, detector_class_id)

    # Keep the proven smooth AUTO -> GUIDED transition from V4/V3.
    test.set_mode("GUIDED")
    hybrid_v1.zero_xy(test)

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
    prev_radial = None
    prev_t = None
    radial_rate_f = 0.0
    vx_cmd = vy_cmd = 0.0
    outside_since = lidar_loss_since = None
    reverse_until = 0.0
    phase = "SEARCH"
    first_valid_elapsed = None

    while time.monotonic() < deadline:
        test.poll(0.02)
        hybrid_v1.check_flight_health(test)
        if test.mode != "GUIDED":
            try:
                hybrid_v1.zero_xy(test)
            except Exception:
                pass
            raise hybrid_v1.PilotTakeover(f"mode changed during fuzzy drop phase to {test.mode}")

        outside_since, lidar_loss_since = hybrid_v1.check_lidar_guard(
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

        mu_near = mu_mid = mu_far = 0.0
        commanded_speed = 0.0
        trend_name = "--"
        radial = 0.0

        if valid and frame_id != last_frame:
            last_frame = frame_id
            valid_streak += 1
            lost_since = None
            if first_valid_elapsed is None:
                first_valid_elapsed = now - drop_start
                print(f"[DROP TIMER] first valid detection = {first_valid_elapsed:.3f}s")

            raw_ex = float(ex)
            raw_ey = float(ey)

            # Preserve V4/V1 release semantics: raw image box, no speed gate.
            in_drop_box = (
                abs(raw_ex) <= float(a.drop_error)
                and abs(raw_ey) <= float(a.drop_error)
            )
            if valid_streak >= int(a.lock_frames) and in_drop_box:
                drop_streak += 1
            else:
                drop_streak = 0

            if valid_streak >= int(a.lock_frames) and drop_streak >= int(a.drop_frames):
                gate_elapsed = now - drop_start
                print(
                    f"[DROP GATE] t={gate_elapsed:.3f}s rawEX={raw_ex:+.3f} "
                    f"rawEY={raw_ey:+.3f} lock={valid_streak}"
                )
                hybrid_v1.zero_xy(test)
                vx_cmd = vy_cmd = 0.0
                servo_cmd_elapsed = time.monotonic() - drop_start
                print(f"[DROP TIMER] SERVO OPEN command @ {servo_cmd_elapsed:.3f}s")
                hybrid_v1.set_servo_pair(
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

            # Preserve field-validated X/Y sign mapping from V1.
            bx = hybrid_v1.clamp(-raw_ex * int(a.x_sign), -1.0, 1.0)
            by = hybrid_v1.clamp(-raw_ey * int(a.y_sign), -1.0, 1.0)

            if filt_bx is None:
                filt_bx, filt_by = bx, by
            else:
                alpha = float(a.filter_alpha)
                filt_bx = alpha * bx + (1.0 - alpha) * filt_bx
                filt_by = alpha * by + (1.0 - alpha) * filt_by

            ebx = hybrid_v1.deadband_remap(filt_bx, float(a.deadband_x))
            eby = hybrid_v1.deadband_remap(filt_by, float(a.deadband_y))
            radial = math.hypot(float(filt_bx), float(filt_by))

            if prev_t is not None and prev_radial is not None:
                dtv = max(1e-3, now - prev_t)
                raw_rr = (radial - prev_radial) / dtv
                da = float(a.d_alpha)
                radial_rate_f = da * raw_rr + (1.0 - da) * radial_rate_f
            else:
                radial_rate_f = 0.0

            # Detect centre crossing on either axis.  This is the small
            # "reverse stick bump" after an overshoot, not a full-speed chase.
            crossed_x = bool(
                prev_ebx is not None
                and prev_ebx * ebx < 0.0
                and max(abs(prev_ebx), abs(ebx)) <= FUZZY_CROSS_WINDOW
            )
            crossed_y = bool(
                prev_eby is not None
                and prev_eby * eby < 0.0
                and max(abs(prev_eby), abs(eby)) <= FUZZY_CROSS_WINDOW
            )
            if crossed_x or crossed_y:
                reverse_until = now + FUZZY_REVERSE_HOLD_S
                print(
                    f"[FUZZY OVERSHOOT] cross={'X' if crossed_x else ''}{'Y' if crossed_y else ''} "
                    f"-> reverse cap {FUZZY_REVERSE_CAP_MPS:.2f}m/s for {FUZZY_REVERSE_HOLD_S:.2f}s"
                )

            prev_ebx, prev_eby = ebx, eby
            prev_radial, prev_t = radial, now

            if valid_streak >= int(a.lock_frames):
                commanded_speed, mu_near, mu_mid, mu_far, trend_name = _fuzzy_speed(
                    radial, radial_rate_f, a
                )

                if now < reverse_until:
                    commanded_speed = min(commanded_speed, FUZZY_REVERSE_CAP_MPS)
                    phase = "FZ-REVERSE"
                elif mu_far >= max(mu_mid, mu_near):
                    phase = "FZ-FAR"
                elif mu_mid >= mu_near:
                    phase = "FZ-MID"
                else:
                    phase = "FZ-NEAR"

                direction_norm = math.hypot(ebx, eby)
                if direction_norm > 1e-6:
                    target_vx = commanded_speed * ebx / direction_norm
                    target_vy = commanded_speed * eby / direction_norm
                else:
                    target_vx = target_vy = 0.0

                # Slew stays as the final anti-jerk layer.
                vx_cmd = hybrid_v1.slew(vx_cmd, target_vx, float(a.xy_slew), period)
                vy_cmd = hybrid_v1.slew(vy_cmd, target_vy, float(a.xy_slew), period)
            else:
                phase = "LOCK"
                vx_cmd = vy_cmd = 0.0

        elif not valid:
            valid_streak = 0
            drop_streak = 0
            lost_since = lost_since or now
            vx_cmd = vy_cmd = 0.0
            phase = "TARGET LOST"
            hybrid_v1.zero_xy(test)
            if now - lost_since >= float(a.lost_timeout):
                filt_bx = filt_by = None
                prev_ebx = prev_eby = None
                prev_radial = prev_t = None
                radial_rate_f = 0.0
                reverse_until = 0.0

        if now >= next_tx:
            hybrid_v1.send_body_xy(test, vx_cmd, vy_cmd)
            next_tx = now + period

        if now - last_print >= 0.20:
            ex_t = "--" if ex is None else f"{float(ex):+.3f}"
            ey_t = "--" if ey is None else f"{float(ey):+.3f}"
            conf_t = "--" if conf is None else f"{float(conf):.2f}"
            remaining = max(0.0, deadline - now)
            print(
                f"[FZXY] {phase:11s} t={now-drop_start:.2f}s rem={remaining:.2f}s "
                f"conf={conf_t} ex={ex_t} ey={ey_t} r={radial:.3f} dr={radial_rate_f:+.2f} | "
                f"mu={mu_near:.2f}/{mu_mid:.2f}/{mu_far:.2f} {trend_name} "
                f"spd={commanded_speed:.2f} VX={vx_cmd:+.2f} VY={vy_cmd:+.2f} "
                f"lock={valid_streak} drop={drop_streak}/{a.drop_frames}"
            )
            last_print = now

        time.sleep(0.003)

    hybrid_v1.zero_xy(test)
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
    # Preserve V4's tested AUTO/race launch/GCS-servo flow.  Replace only the
    # GUIDED bottom-vision controller resolved by the V3 mission runner.
    race_v3.parse_args = parse_args_v5
    race_v3.validate = validate_v5
    race_v3.load_waypoint = race_v4.load_waypoint_v4
    race_v3.run_quick_drop = run_fuzzy_drop
    return race_v3.main()


if __name__ == "__main__":
    raise SystemExit(main())
