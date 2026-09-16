#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import DirectFlightTest, FlightTestError


def _fmt(value):
    return "--" if value is None else f"{float(value):.2f}"


def _send_velocity_ned(test: DirectFlightTest, vz_ned_mps: float):
    """Send velocity-only LOCAL_NED target.

    NED convention: negative Z velocity = climb/up.
    Position, acceleration, yaw and yaw-rate are ignored.
    """
    type_mask = (
        (1 << 0) | (1 << 1) | (1 << 2) |
        (1 << 6) | (1 << 7) | (1 << 8) |
        (1 << 10) | (1 << 11)
    )
    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0, 0.0, 0.0,
        0.0, 0.0, float(vz_ned_mps),
        0.0, 0.0, 0.0,
        0.0, 0.0,
    )


def _send_zero_velocity(test: DirectFlightTest):
    _send_velocity_ned(test, 0.0)


def lidar_authoritative_takeoff(
    test: DirectFlightTest,
    target_m: float,
    *,
    tolerance_m: float,
    handoff_m: float,
    climb_rate_mps: float,
    kick_alt_m: float,
    hold_s: float,
    timeout_s: float,
    ceiling_m: float,
):
    target_m = float(target_m)
    trigger_m = target_m - float(tolerance_m)
    handoff_m = float(handoff_m)
    climb_rate_mps = abs(float(climb_rate_mps))
    kick_alt_m = float(kick_alt_m)

    if not 0.50 <= target_m <= 3.0:
        raise FlightTestError("target altitude must be 0.50-3.00 m")
    if not 0.12 <= handoff_m < trigger_m:
        raise FlightTestError("handoff must be >=0.12m and below target trigger")
    if not 0.10 <= climb_rate_mps <= 0.80:
        raise FlightTestError("climb rate must be 0.10-0.80 m/s")
    if ceiling_m <= target_m:
        raise FlightTestError("ceiling must be above target")

    rel_alt0 = None if test.global_pos is None else test.global_pos[2]
    # Kick target is deliberately above the possibly-offset EKF/home altitude.
    # It is NOT the takeoff completion criterion; it only starts/spools the
    # ArduPilot TakeOff submode until raw LiDAR proves the aircraft is airborne.
    if rel_alt0 is not None:
        kick_alt_m = max(kick_alt_m, float(rel_alt0) + 1.50)
    kick_alt_m = min(kick_alt_m, 6.0)

    print(
        f"[PLAN] LiDAR target={target_m:.2f}m trigger={trigger_m:.2f}m | "
        f"handoff={handoff_m:.2f}m | climb={climb_rate_mps:.2f}m/s"
    )
    print(
        f"[PLAN] NAV_TAKEOFF kick={kick_alt_m:.2f}m ONLY FOR LAUNCH; "
        "completion is RAW LiDAR, not relAlt/localZ"
    )

    test.set_mode("GUIDED")
    test.arm()

    test.master.mav.command_long_send(
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, float("nan"), 0, 0, kick_alt_m,
    )

    deadline = time.monotonic() + float(timeout_s)
    lidar_loss_since = None
    last_print = 0.0
    handoff_done = False
    next_velocity_send = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()

        # After the aircraft is definitely off the ground, velocity mode takes
        # over from Guided TakeOff. 10 Hz is intentionally much faster than the
        # ArduPilot minimum refresh requirement for Guided velocity targets.
        if handoff_done and now >= next_velocity_send:
            _send_velocity_ned(test, -climb_rate_mps)
            next_velocity_send = now + 0.10

        test.poll(0.05)
        now = time.monotonic()

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during takeoff")
        if test.mode != "GUIDED":
            raise FlightTestError(f"flight mode changed during takeoff: {test.mode}")

        rel_alt = None if test.global_pos is None else test.global_pos[2]
        local_up = None if test.local is None else -float(test.local[2])

        if now - last_print >= 0.20:
            phase = "VEL-LIDAR" if handoff_done else "LAUNCH-KICK"
            print(
                f"[CLIMB:{phase}] LiDAR={_fmt(test.lidar)}m "
                f"relAlt={_fmt(rel_alt)}m localUp={_fmt(local_up)}m "
                f"target>={trigger_m:.2f}m"
            )
            last_print = now

        if not test.lidar_ok():
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                if handoff_done:
                    _send_zero_velocity(test)
                raise FlightTestError("LiDAR invalid/stale >0.60s during takeoff")
            continue

        lidar_loss_since = None
        lidar = float(test.lidar)

        if lidar >= float(ceiling_m):
            _send_zero_velocity(test)
            raise FlightTestError(f"LiDAR hard ceiling reached: {lidar:.2f}m")

        # If the launch kick happens to reach target before the handoff point is
        # processed, a zero-velocity target immediately changes Guided into the
        # velocity controller and terminates further climb.
        if lidar >= trigger_m:
            _send_zero_velocity(test)
            print(f"[TARGET] LiDAR={lidar:.2f}m -> ZERO VELOCITY")
            break

        if not handoff_done and lidar >= handoff_m:
            handoff_done = True
            _send_velocity_ned(test, -climb_rate_mps)
            next_velocity_send = now + 0.10
            print(
                f"[HANDOFF] LiDAR={lidar:.2f}m: NAV_TAKEOFF -> GUIDED VELOCITY "
                f"vz={-climb_rate_mps:.2f}m/s"
            )
    else:
        if handoff_done:
            _send_zero_velocity(test)
        raise FlightTestError(
            f"takeoff timeout before LiDAR target; LiDAR={_fmt(test.lidar)}m"
        )

    hold_deadline = time.monotonic() + float(hold_s)
    next_zero = 0.0
    last_print = 0.0
    lidar_loss_since = None

    while time.monotonic() < hold_deadline:
        now = time.monotonic()
        if now >= next_zero:
            _send_zero_velocity(test)
            next_zero = now + 0.10

        test.poll(0.05)
        now = time.monotonic()

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during LiDAR hold")
        if test.mode != "GUIDED":
            raise FlightTestError(f"flight mode changed during hold: {test.mode}")

        if not test.lidar_ok():
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR invalid/stale >0.60s during hold")
        else:
            lidar_loss_since = None
            if float(test.lidar) >= float(ceiling_m):
                raise FlightTestError(f"LiDAR exceeded ceiling in hold: {test.lidar:.2f}m")

        if now - last_print >= 0.25:
            rel_alt = None if test.global_pos is None else test.global_pos[2]
            print(
                f"[HOLD] LiDAR={_fmt(test.lidar)}m relAlt={_fmt(rel_alt)}m "
                f"remaining={max(0.0, hold_deadline-now):.1f}s"
            )
            last_print = now

    _send_zero_velocity(test)
    print(f"[TAKEOFF] LiDAR-authoritative hold complete ({hold_s:.1f}s)")


def main():
    ap = argparse.ArgumentParser(
        description="Kingphoenix DEV LiDAR-authoritative launch -> Guided velocity climb -> hold -> LAND"
    )
    ap.add_argument("--alt", type=float, default=1.2, help="raw LiDAR AGL target")
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--handoff", type=float, default=0.25, help="LiDAR height to leave NAV_TAKEOFF and enter velocity control")
    ap.add_argument("--climb-rate", type=float, default=0.30, help="vertical climb rate m/s after handoff")
    ap.add_argument("--kick-alt", type=float, default=3.0, help="temporary NAV_TAKEOFF target used only to launch")
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--ceiling", type=float, default=1.40)
    args = ap.parse_args()

    print("============================================================")
    print(" KINGPHOENIX LIDAR-AUTHORITATIVE TAKEOFF TEST")
    print("============================================================")
    print("Altitude completion source: RAW downward LiDAR only")
    print("relAlt/localZ: diagnostic display only, never completion criteria")

    test = DirectFlightTest()
    try:
        test.open()
        test.collect_preflight(require_auto=False)
        print(
            f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m mode={test.mode} armed={test.armed}"
        )
        lidar_authoritative_takeoff(
            test,
            args.alt,
            tolerance_m=args.tolerance,
            handoff_m=args.handoff,
            climb_rate_mps=args.climb_rate,
            kick_alt_m=args.kick_alt,
            hold_s=args.hold,
            timeout_s=args.timeout,
            ceiling_m=args.ceiling,
        )
        test.land_and_wait()
        print("[RESULT] LIDAR TAKEOFF -> HOLD -> LAND PASS")
        return 0
    except KeyboardInterrupt:
        test.land_best_effort("operator interrupted")
        return 130
    except FlightTestError as exc:
        test.land_best_effort(str(exc))
        print(f"[RESULT] BLOCKED/FAILED: {exc}")
        return 2
    except Exception as exc:
        test.land_best_effort(f"unexpected {type(exc).__name__}: {exc}")
        print(f"[RESULT] UNEXPECTED ERROR: {type(exc).__name__}: {exc}")
        return 3
    finally:
        test.close()


if __name__ == "__main__":
    raise SystemExit(main())
