#!/usr/bin/env python3
"""Kingphoenix bottom-camera aggressive drop + dual-servo field runner.

Flow:
  servo pretest OPEN -> READY(CLOSE) -> GO
  LiDAR takeoff -> bottom-camera XY chase/brake/final
  center lock -> DROP (servo 7+8 OPEN) -> XY ZERO -> RC manual takeover

Field-validated camera sign corrections:
  X_SIGN=-1
  Y_SIGN=-1

Default horizontal gains/cap keep the current field-good conservative values.
No automatic LAND is sent by this runner.
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
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import tools.run_bottom_target_pd_tuning as base
from tools.run_bottom_target_pd_headless import HeadlessLoopbackGcs, wait_bottom_headless


class PilotTakeover(RuntimeError):
    pass


def parse_args():
    ap = argparse.ArgumentParser(
        description="Bottom aggressive XY alignment + dual-servo drop field runner"
    )

    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--control-hz", type=float, default=20.0)
    ap.add_argument("--kp-x", type=float, default=1.2)
    ap.add_argument("--kd-x", type=float, default=0.12)
    ap.add_argument("--kp-y", type=float, default=1.2)
    ap.add_argument("--kd-y", type=float, default=0.12)
    ap.add_argument("--xy-slew", type=float, default=1.5)

    ap.add_argument("--far-error", type=float, default=0.30)
    ap.add_argument("--brake-error", type=float, default=0.10)
    ap.add_argument("--drop-error", type=float, default=0.05)
    ap.add_argument("--far-cap", type=float, default=0.40)
    ap.add_argument("--brake-cap", type=float, default=0.25)
    ap.add_argument("--final-cap", type=float, default=0.12)
    ap.add_argument("--drop-command-speed", type=float, default=0.20)
    ap.add_argument("--drop-frames", type=int, default=3)

    ap.add_argument("--deadband-x", type=float, default=0.04)
    ap.add_argument("--deadband-y", type=float, default=0.04)
    ap.add_argument("--filter-alpha", type=float, default=0.40)
    ap.add_argument("--d-alpha", type=float, default=0.25)
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--lock-frames", type=int, default=3)
    ap.add_argument("--lost-timeout", type=float, default=0.15)

    ap.add_argument("--x-sign", type=int, choices=[-1, 1], default=-1)
    ap.add_argument("--y-sign", type=int, choices=[-1, 1], default=-1)

    ap.add_argument("--servo-a", type=int, default=7)
    ap.add_argument("--servo-b", type=int, default=8)
    ap.add_argument("--open-pwm", type=int, default=1100)
    ap.add_argument("--close-pwm", type=int, default=2000)
    ap.add_argument("--servo-settle", type=float, default=0.40)
    ap.add_argument(
        "--skip-servo-function-check",
        action="store_true",
        help="do not verify SERVOx_FUNCTION before sending DO_SET_SERVO",
    )

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
    return ap.parse_args()


def validate(a):
    if not 3 <= a.duration <= 180:
        raise SystemExit("--duration must be 3..180s")
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
    if not 0.01 <= a.drop_error < a.brake_error:
        raise SystemExit("require 0.01<=drop-error<brake-error")
    if not 0.05 <= a.final_cap <= a.brake_cap <= a.far_cap <= 2.5:
        raise SystemExit("require final-cap<=brake-cap<=far-cap<=2.5")
    if not 0.05 <= a.drop_command_speed <= 0.5:
        raise SystemExit("--drop-command-speed must be 0.05..0.5")
    if not 1 <= a.drop_frames <= 20:
        raise SystemExit("--drop-frames must be 1..20")
    if not 0.25 <= a.confidence <= 0.95:
        raise SystemExit("--confidence must be 0.25..0.95")
    if not 1 <= a.lock_frames <= 20:
        raise SystemExit("--lock-frames must be 1..20")
    if not 1 <= a.servo_a <= 32 or not 1 <= a.servo_b <= 32:
        raise SystemExit("servo channels must be 1..32")
    if a.servo_a == a.servo_b:
        raise SystemExit("--servo-a and --servo-b must differ")
    if not 900 <= a.open_pwm <= 2100 or not 900 <= a.close_pwm <= 2100:
        raise SystemExit("servo PWM must be 900..2100")
    if not 0.05 <= a.servo_settle <= 3.0:
        raise SystemExit("--servo-settle must be 0.05..3.0s")
    if not 0.50 <= a.alt <= 3.0:
        raise SystemExit("--alt must be 0.50..3.0m")
    if a.takeoff_ceiling <= a.alt:
        raise SystemExit("takeoff ceiling must be above target")
    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 2.0:
        raise SystemExit("invalid altitude guard")


def _param_name(msg) -> str:
    raw = getattr(msg, "param_id", "")
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("ascii", errors="ignore")
    return str(raw).rstrip("\x00").strip()


def read_param(test, name: str, timeout_s: float = 2.0):
    if test.master is None:
        raise base.FlightTestError("MAVLink not open")
    test.master.mav.param_request_read_send(
        test.master.target_system,
        test.master.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        msg = test.poll(0.20)
        if msg is not None and msg.get_type() == "PARAM_VALUE" and _param_name(msg) == name:
            return float(getattr(msg, "param_value"))
    return None


def verify_servo_functions(test, channels):
    def allowed(v):
        n = int(round(v))
        return n in {0, 1} or 51 <= n <= 66

    values = {}
    for ch in channels:
        name = f"SERVO{int(ch)}_FUNCTION"
        value = read_param(test, name)
        if value is None:
            raise base.FlightTestError(f"could not read {name}")
        values[ch] = int(round(value))
        if not allowed(value):
            raise base.FlightTestError(
                f"{name}={values[ch]} is not compatible with MAV_CMD_DO_SET_SERVO"
            )
    print(
        "[SERVO FUNCTION] "
        + " | ".join(f"SERVO{ch}_FUNCTION={values[ch]}" for ch in channels)
        + " -> OK"
    )


def set_servo(test, channel: int, pwm: int):
    if test.master is None:
        raise base.FlightTestError("MAVLink not open")
    test.master.mav.command_long_send(
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        float(channel),
        float(pwm),
        0.0, 0.0, 0.0, 0.0, 0.0,
    )


def set_servo_pair(test, a, b, pwm, label, settle_s):
    set_servo(test, int(a), int(pwm))
    set_servo(test, int(b), int(pwm))
    print(f"[SERVO] {label} | CH{a}={pwm}us CH{b}={pwm}us")
    time.sleep(float(settle_s))


def require_word(prompt: str, expected: str):
    ans = input(prompt).strip().upper()
    if ans != expected.upper():
        raise base.FlightTestError(
            f"operator cancelled: expected {expected.upper()}, got {ans or '<empty>'}"
        )


def main() -> int:
    a = parse_args()
    validate(a)

    cfg = base.bottom_config(float(a.confidence))
    stop_event = threading.Event()
    state = base.SharedState()
    events = base.EventBus()
    waypoints = base.WaypointManager(state)
    gcs = HeadlessLoopbackGcs(state, events, waypoints, stop_event, command_manager=None)
    vision = base.VisionStreamSession(cfg, state, stop_event)
    vision.workers = [vision.bottom]

    test = base.DirectFlightTest()
    flight_open = False
    servo_prepared = False
    drop_complete = False

    print("============================================================")
    print(" KINGPHOENIX BOTTOM AGGRESSIVE DROP - HEADLESS")
    print("============================================================")
    print(f"[FIELD SIGN] X={a.x_sign:+d} | Y={a.y_sign:+d}")
    print(
        f"[PD] X {a.kp_x:.2f}/{a.kd_x:.2f} | "
        f"Y {a.kp_y:.2f}/{a.kd_y:.2f} | slew={a.xy_slew:.2f}"
    )
    print(
        f"[ZONES] CHASE r>{a.far_error:.2f} cap={a.far_cap:.2f} | "
        f"BRAKE r>{a.brake_error:.2f} cap={a.brake_cap:.2f} | "
        f"FINAL cap={a.final_cap:.2f}"
    )
    print(
        f"[DROP] |ex|,|ey|<={a.drop_error:.2f} for {a.drop_frames} frames "
        f"and command speed<={a.drop_command_speed:.2f}m/s"
    )
    print(
        f"[SERVO] CH{a.servo_a}+CH{a.servo_b} "
        f"OPEN={a.open_pwm}us CLOSE={a.close_pwm}us"
    )
    print("[LANDING] AUTO LAND DISABLED -> RC manual takeover")
    print("[HARD OFF] VZ=0 | yaw=0")
    print("------------------------------------------------------------")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise base.FlightTestError("Hailo YOLO26 did not become READY")
        wait_bottom_headless(state, float(a.vision_wait))

        test.open()
        flight_open = True
        test.collect_preflight(require_auto=True)

        if not a.skip_servo_function_check:
            verify_servo_functions(test, (a.servo_a, a.servo_b))
        else:
            print("[SERVO FUNCTION] CHECK SKIPPED by operator")

        print()
        print("[GROUND SERVO PRETEST]")
        print(f"  OPEN means CH{a.servo_a}/CH{a.servo_b} -> {a.open_pwm}us")
        print(f"  CLOSE means CH{a.servo_a}/CH{a.servo_b} -> {a.close_pwm}us")
        require_word("Type OPEN to test opening both servos: ", "OPEN")
        set_servo_pair(test, a.servo_a, a.servo_b, a.open_pwm, "OPEN", a.servo_settle)

        require_word("Confirm mechanism is OPEN, then type READY to CLOSE: ", "READY")
        set_servo_pair(test, a.servo_a, a.servo_b, a.close_pwm, "CLOSE", a.servo_settle)
        servo_prepared = True

        require_word("Servos CLOSED. Safety pilot ready. Type GO to arm/takeoff: ", "GO")

        base.profiled_lidar_takeoff(
            test,
            float(a.alt),
            tolerance_m=float(a.tolerance),
            handoff_m=float(a.handoff),
            fast_rate_mps=float(a.climb_rate),
            near_rate_mps=float(a.near_rate),
            taper_zone_m=float(a.taper_zone),
            kick_alt_m=float(a.kick_alt),
            timeout_s=15.0,
            ceiling_m=float(a.takeoff_ceiling),
            zero_flush_s=float(a.zero_flush),
        )
        if test.mode != "GUIDED":
            test.set_mode("GUIDED")
        base.zero_xy(test)

        period = 1.0 / float(a.control_hz)
        end_time = time.monotonic() + float(a.duration)
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
        latest_conf = latest_ex = latest_ey = None

        while time.monotonic() < end_time:
            test.poll(0.02)
            base.check_flight_health(test)
            if test.mode != "GUIDED":
                try:
                    base.zero_xy(test)
                except Exception:
                    pass
                raise PilotTakeover(f"mode changed to {test.mode}")

            outside_since, lidar_loss_since = base.check_lidar_guard(
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
            latest_conf, latest_ex, latest_ey = conf, ex, ey
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

            if valid and frame_id != last_frame:
                last_frame = frame_id
                valid_streak += 1
                lost_since = None

                bx = base.clamp(-float(ex) * int(a.x_sign), -1.0, 1.0)
                by = base.clamp(-float(ey) * int(a.y_sign), -1.0, 1.0)

                if filt_bx is None:
                    filt_bx, filt_by = bx, by
                else:
                    alpha = float(a.filter_alpha)
                    filt_bx = alpha * bx + (1.0 - alpha) * filt_bx
                    filt_by = alpha * by + (1.0 - alpha) * filt_by

                ebx = base.response_curve(
                    base.deadband_remap(filt_bx, float(a.deadband_x)), "linear", 0.0
                )
                eby = base.response_curve(
                    base.deadband_remap(filt_by, float(a.deadband_y)), "linear", 0.0
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
                    target_vx, target_vy = base.vector_cap(target_vx, target_vy, cap)

                    vx_cmd = base.slew(vx_cmd, target_vx, float(a.xy_slew), period)
                    vy_cmd = base.slew(vy_cmd, target_vy, float(a.xy_slew), period)

                    centered = (
                        abs(float(filt_bx)) <= float(a.drop_error)
                        and abs(float(filt_by)) <= float(a.drop_error)
                    )
                    command_slow = math.hypot(vx_cmd, vy_cmd) <= float(a.drop_command_speed)
                    if centered and command_slow:
                        drop_streak += 1
                    else:
                        drop_streak = 0

                    if drop_streak >= int(a.drop_frames):
                        print(
                            f"[DROP COMMIT] centered {drop_streak} frames | "
                            f"EX={float(ex):+.3f} EY={float(ey):+.3f} | "
                            f"cmd={math.hypot(vx_cmd, vy_cmd):.2f}m/s"
                        )
                        base.zero_xy(test)
                        vx_cmd = vy_cmd = 0.0
                        set_servo_pair(
                            test, a.servo_a, a.servo_b, a.open_pwm, "DROP OPEN", a.servo_settle
                        )
                        drop_complete = True
                        break
                else:
                    phase = "LOCK"
                    vx_cmd = vy_cmd = 0.0

            elif not valid:
                valid_streak = 0
                drop_streak = 0
                lost_since = lost_since or now
                vx_cmd = vy_cmd = 0.0
                phase = "TARGET LOST"
                base.zero_xy(test)
                if now - lost_since >= float(a.lost_timeout):
                    filt_bx = filt_by = None
                    prev_ebx = prev_eby = prev_t = None
                    dfx = dfy = 0.0

            if now >= next_tx:
                base.send_body_xy(test, vx_cmd, vy_cmd)
                next_tx = now + period

            if now - last_print >= 0.20:
                conf_t = "--" if latest_conf is None else f"{float(latest_conf):.2f}"
                ex_t = "--" if latest_ex is None else f"{float(latest_ex):+.3f}"
                ey_t = "--" if latest_ey is None else f"{float(latest_ey):+.3f}"
                lidar_t = "--" if test.lidar is None else f"{float(test.lidar):.2f}"
                print(
                    f"[DROPXY] {phase:11s} conf={conf_t} ex={ex_t} ey={ey_t} | "
                    f"VX={vx_cmd:+.2f} VY={vy_cmd:+.2f} "
                    f"lock={valid_streak} drop={drop_streak}/{a.drop_frames} | "
                    f"LiDAR={lidar_t}m"
                )
                last_print = now

            time.sleep(0.003)

        base.zero_xy(test)
        if drop_complete:
            print("[RESULT] DROP COMPLETE -> XY ZERO -> RC TAKEOVER")
            print(f"[SERVO STATE] CH{a.servo_a}/CH{a.servo_b} left OPEN at {a.open_pwm}us")
        else:
            print("[RESULT] duration ended WITHOUT drop -> XY ZERO")
            if servo_prepared:
                print(f"[SERVO STATE] CH{a.servo_a}/CH{a.servo_b} remains CLOSED at {a.close_pwm}us")
            print("[RC TAKEOVER] AUTO LAND DISABLED; land manually")
        return 0

    except PilotTakeover as exc:
        print(f"[PILOT TAKEOVER] {exc}; XY ZERO; no software LAND")
        try:
            if flight_open:
                base.zero_xy(test)
        except Exception:
            pass
        return 0
    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt -> XY ZERO; no software LAND")
        try:
            if flight_open:
                base.zero_xy(test)
        except Exception:
            pass
        return 130
    except base.FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        try:
            if flight_open:
                base.zero_xy(test)
        except Exception:
            pass
        print("[RC TAKEOVER] software LAND suppressed")
        return 2
    except Exception as exc:
        print(f"[UNEXPECTED] {type(exc).__name__}: {exc}")
        try:
            if flight_open:
                base.zero_xy(test)
        except Exception:
            pass
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


if __name__ == "__main__":
    raise SystemExit(main())
