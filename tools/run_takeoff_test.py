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


def _send_zero_velocity(test: DirectFlightTest):
    """Command zero XYZ velocity in GUIDED using LOCAL_NED.

    Position, acceleration, yaw and yaw-rate fields are ignored. Only
    vx/vy/vz are active, and all three are explicitly zero.
    """
    type_mask = (
        (1 << 0) | (1 << 1) | (1 << 2) |      # ignore position x/y/z
        (1 << 6) | (1 << 7) | (1 << 8) |      # ignore acceleration
        (1 << 10) | (1 << 11)                  # ignore yaw + yaw rate
    )
    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0, 0.0, 0.0,                         # x, y, z ignored
        0.0, 0.0, 0.0,                         # vx, vy, vz = STOP
        0.0, 0.0, 0.0,                         # acceleration ignored
        0.0, 0.0,                              # yaw/yaw-rate ignored
    )


def takeoff_then_zero_hold(
    test: DirectFlightTest,
    target_m: float,
    *,
    margin: float,
    tolerance: float,
    zero_hold_s: float,
    timeout: float,
    hard_ceiling_m: float,
    hard_floor_m: float,
):
    target_m = float(target_m)
    if not 0.50 <= target_m <= 3.0:
        raise FlightTestError("takeoff altitude must be 0.50-3.00 m")

    command_m = target_m * (1.0 + float(margin))
    trigger_m = target_m - float(tolerance)

    print(
        f"[TAKEOFF] LiDAR target={target_m:.2f}m trigger={trigger_m:.2f}m "
        f"FC command={command_m:.2f}m (+{margin*100:.0f}%)"
    )
    test.set_mode("GUIDED")
    test.arm()

    test.master.mav.command_long_send(
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, float("nan"), 0, 0, command_m,
    )

    deadline = time.monotonic() + float(timeout)
    lidar_loss_since = None
    last_print = 0.0
    takeoff_ack = None

    # Phase 1: let NAV_TAKEOFF climb, but terminate the climb from the
    # companion side as soon as the real LiDAR ground-clearance reaches the
    # requested threshold. We intentionally do NOT wait for FC altitude.
    while time.monotonic() < deadline:
        msg = test.poll(0.10)
        now = time.monotonic()

        if msg is not None and msg.get_type() == "COMMAND_ACK":
            command = int(getattr(msg, "command", -1))
            if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                takeoff_ack = int(getattr(msg, "result", -1))
                print(f"[ACK] NAV_TAKEOFF result={takeoff_ack}")
                if takeoff_ack not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                ):
                    raise FlightTestError(f"NAV_TAKEOFF rejected/result={takeoff_ack}")

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during takeoff")
        if test.mode != "GUIDED":
            raise FlightTestError(f"flight mode changed during takeoff: {test.mode}")

        rel_alt = None if test.global_pos is None else test.global_pos[2]
        local_up = None if test.local is None else -float(test.local[2])

        if now - last_print >= 0.20:
            print(
                f"[CLIMB] mode={test.mode:<8} LiDAR={_fmt(test.lidar)}m "
                f"relAlt={_fmt(rel_alt)}m localUp={_fmt(local_up)}m "
                f"trigger>={trigger_m:.2f}m ACK={takeoff_ack if takeoff_ack is not None else '--'}"
            )
            last_print = now

        if test.lidar_ok():
            lidar_loss_since = None
            if test.lidar >= float(hard_ceiling_m):
                raise FlightTestError(
                    f"LiDAR hard ceiling reached during climb: {test.lidar:.2f}m"
                )
            if test.lidar >= trigger_m:
                print(
                    f"[STOP] LiDAR={test.lidar:.2f}m reached trigger. "
                    "Sending vx=0 vy=0 vz=0."
                )
                break
        else:
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR invalid/stale >0.60 s during takeoff")
    else:
        raise FlightTestError(
            f"takeoff timeout before LiDAR trigger; LiDAR={_fmt(test.lidar)}m"
        )

    # Phase 2: zero-velocity hold. No LiDAR P/PID correction is applied here.
    # The companion only repeats a zero XYZ velocity setpoint and monitors
    # safety boundaries. This is deliberately conservative for the field test.
    hold_deadline = time.monotonic() + float(zero_hold_s)
    next_zero_send = 0.0
    next_print = 0.0
    lidar_loss_since = None

    while time.monotonic() < hold_deadline:
        now = time.monotonic()
        if now >= next_zero_send:
            _send_zero_velocity(test)
            next_zero_send = now + 0.10  # 10 Hz refresh

        test.poll(0.05)
        now = time.monotonic()

        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost during zero-velocity hold")
        if test.mode != "GUIDED":
            raise FlightTestError(f"flight mode changed during hold: {test.mode}")

        if not test.lidar_ok():
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                raise FlightTestError("LiDAR invalid/stale >0.60 s during hold")
        else:
            lidar_loss_since = None
            if test.lidar >= float(hard_ceiling_m):
                raise FlightTestError(
                    f"LiDAR exceeded hold ceiling: {test.lidar:.2f}m"
                )
            if test.lidar <= float(hard_floor_m):
                raise FlightTestError(
                    f"LiDAR fell below hold floor: {test.lidar:.2f}m"
                )

        if now >= next_print:
            rel_alt = None if test.global_pos is None else test.global_pos[2]
            local_up = None if test.local is None else -float(test.local[2])
            remaining = max(0.0, hold_deadline - now)
            print(
                f"[ZERO-HOLD] LiDAR={_fmt(test.lidar)}m relAlt={_fmt(rel_alt)}m "
                f"localUp={_fmt(local_up)}m remaining={remaining:.1f}s "
                "cmd(vx,vy,vz)=(0,0,0)"
            )
            next_print = now + 0.20

    # One final zero command immediately before handing over to LAND.
    _send_zero_velocity(test)
    print(f"[ZERO-HOLD] COMPLETE {zero_hold_s:.1f}s -> LAND")


def main():
    ap = argparse.ArgumentParser(
        description="Kingphoenix DEV LiDAR-triggered takeoff -> zero-velocity hold -> LAND test"
    )
    ap.add_argument("--alt", type=float, default=1.2, help="true LiDAR target, metres")
    ap.add_argument(
        "--margin", type=float, default=0.45,
        help="FC NAV_TAKEOFF compensation; default +45%%",
    )
    ap.add_argument(
        "--tolerance", type=float, default=0.05,
        help="stop-climb trigger below LiDAR target; default 0.05 m",
    )
    ap.add_argument(
        "--hold", type=float, default=2.0,
        help="zero-velocity hold duration before LAND; default 2 s",
    )
    ap.add_argument("--timeout", type=float, default=12.0, help="climb timeout")
    ap.add_argument(
        "--ceiling", type=float, default=1.35,
        help="LiDAR hard safety ceiling; default 1.35 m",
    )
    ap.add_argument(
        "--floor", type=float, default=0.70,
        help="LiDAR hard safety floor during zero hold; default 0.70 m",
    )
    args = ap.parse_args()

    if args.ceiling <= args.alt:
        raise SystemExit("--ceiling must be greater than --alt")
    if args.floor >= args.alt:
        raise SystemExit("--floor must be lower than --alt")

    test = DirectFlightTest()
    print("============================================================")
    print(" KINGPHOENIX DEV TAKEOFF -> ZERO VELOCITY 2s -> LAND TEST")
    print("============================================================")
    print(
        f"[PLAN] LiDAR target={args.alt:.2f}m | trigger={args.alt-args.tolerance:.2f}m | "
        f"FC command={args.alt*(1+args.margin):.2f}m | zero-hold={args.hold:.1f}s"
    )
    print(
        f"[GUARD] LiDAR hold range={args.floor:.2f}..{args.ceiling:.2f}m | "
        "NO LiDAR PID correction"
    )

    try:
        test.open()
        test.collect_preflight(require_auto=False)
        rel_alt = None if test.global_pos is None else test.global_pos[2]
        local_up = None if test.local is None else -float(test.local[2])
        print(
            f"[PREFLIGHT] PASS LiDAR={test.lidar:.2f}m mode={test.mode} armed={test.armed} "
            f"relAlt={_fmt(rel_alt)}m localUp={_fmt(local_up)}m"
        )
        takeoff_then_zero_hold(
            test,
            args.alt,
            margin=args.margin,
            tolerance=args.tolerance,
            zero_hold_s=args.hold,
            timeout=args.timeout,
            hard_ceiling_m=args.ceiling,
            hard_floor_m=args.floor,
        )
        test.land_and_wait()
        print("[RESULT] TAKEOFF -> ZERO HOLD -> LAND TEST PASS")
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
