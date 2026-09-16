#!/usr/bin/env python3
from __future__ import annotations

import time

from pymavlink import mavutil

from tools.flight_test_common import DirectFlightTest, FlightTestError


def _fmt(value):
    return "--" if value is None else f"{float(value):.2f}"


def send_velocity_ned(test: DirectFlightTest, vz_ned_mps: float):
    """Send velocity-only LOCAL_NED target. Negative Z means climb/up."""
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


def profiled_lidar_takeoff(
    test: DirectFlightTest,
    target_m: float,
    *,
    tolerance_m: float = 0.05,
    handoff_m: float = 0.25,
    fast_rate_mps: float = 0.45,
    near_rate_mps: float = 0.20,
    taper_zone_m: float = 0.25,
    kick_alt_m: float = 3.0,
    timeout_s: float = 15.0,
    ceiling_m: float = 1.45,
    zero_flush_s: float = 0.15,
):
    """LiDAR-authoritative mission takeoff with a low-momentum AUTO handoff.

    Important differences from the older diagnostic takeoff helper:
    - NAV_TAKEOFF kick is fixed after ARM and never derived from pre-ARM relAlt.
    - 0.45m/s is only the far-from-target maximum climb speed.
    - the final taper zone uses a slower vertical velocity to reduce overshoot.
    - there is no deliberate altitude hold; only a short zero-command flush so
      the previous upward velocity target is replaced before mode handoff.
    - raw downward LiDAR is the only completion criterion.
    """
    target_m = float(target_m)
    tolerance_m = float(tolerance_m)
    handoff_m = float(handoff_m)
    fast_rate_mps = abs(float(fast_rate_mps))
    near_rate_mps = abs(float(near_rate_mps))
    taper_zone_m = float(taper_zone_m)
    kick_alt_m = float(kick_alt_m)
    ceiling_m = float(ceiling_m)
    zero_flush_s = max(0.0, float(zero_flush_s))

    trigger_m = target_m - tolerance_m
    taper_start_m = max(handoff_m, trigger_m - taper_zone_m)

    if not 0.50 <= target_m <= 3.0:
        raise FlightTestError("target altitude must be 0.50-3.00 m")
    if not 0.12 <= handoff_m < trigger_m:
        raise FlightTestError("handoff must be >=0.12m and below target trigger")
    if not 0.10 <= fast_rate_mps <= 0.80:
        raise FlightTestError("fast climb rate must be 0.10-0.80 m/s")
    if not 0.08 <= near_rate_mps <= fast_rate_mps:
        raise FlightTestError("near climb rate must be 0.08m/s..fast climb rate")
    if not 0.05 <= taper_zone_m <= 0.60:
        raise FlightTestError("taper zone must be 0.05-0.60 m")
    if ceiling_m <= target_m:
        raise FlightTestError("ceiling must be above target")

    print(
        f"[TAKEOFF-PROFILE] target={target_m:.2f}m trigger={trigger_m:.2f}m | "
        f"handoff={handoff_m:.2f}m | fast={fast_rate_mps:.2f}m/s | "
        f"taper@{taper_start_m:.2f}m -> {near_rate_mps:.2f}m/s"
    )
    print(
        f"[TAKEOFF-PROFILE] fixed NAV_TAKEOFF kick={kick_alt_m:.2f}m after ARM; "
        "pre-ARM relAlt is NEVER used"
    )

    test.set_mode("GUIDED")
    test.arm()

    # Give the FC a brief moment to publish the post-arm HOME-relative state.
    arm_settle_deadline = time.monotonic() + 0.35
    while time.monotonic() < arm_settle_deadline:
        test.poll(0.05)

    rel_after_arm = None if test.global_pos is None else test.global_pos[2]
    print(
        f"[TAKEOFF-PROFILE] post-ARM LiDAR={_fmt(test.lidar)}m "
        f"FCrelAlt={_fmt(rel_after_arm)}m"
    )

    test.master.mav.command_long_send(
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, float("nan"), 0, 0, kick_alt_m,
    )

    deadline = time.monotonic() + float(timeout_s)
    handoff_done = False
    lidar_loss_since = None
    next_velocity_send = 0.0
    last_print = 0.0
    last_rate = None

    while time.monotonic() < deadline:
        now = time.monotonic()

        if handoff_done and now >= next_velocity_send:
            lidar_for_rate = float(test.lidar) if test.lidar_ok() else 0.0
            rate = near_rate_mps if lidar_for_rate >= taper_start_m else fast_rate_mps
            send_velocity_ned(test, -rate)
            last_rate = rate
            next_velocity_send = now + 0.10

        test.poll(0.05)
        now = time.monotonic()

        if not test.heartbeat_ok():
            if handoff_done:
                send_velocity_ned(test, 0.0)
            raise FlightTestError("heartbeat lost during profiled takeoff")
        if test.mode != "GUIDED":
            if handoff_done:
                send_velocity_ned(test, 0.0)
            raise FlightTestError(f"flight mode changed during profiled takeoff: {test.mode}")

        if not test.lidar_ok():
            lidar_loss_since = lidar_loss_since or now
            if now - lidar_loss_since > 0.60:
                if handoff_done:
                    send_velocity_ned(test, 0.0)
                raise FlightTestError("LiDAR invalid/stale >0.60s during profiled takeoff")
            continue

        lidar_loss_since = None
        lidar = float(test.lidar)
        rel_alt = None if test.global_pos is None else test.global_pos[2]

        if lidar >= ceiling_m:
            send_velocity_ned(test, 0.0)
            raise FlightTestError(f"LiDAR hard ceiling reached: {lidar:.2f}m")

        if now - last_print >= 0.20:
            if not handoff_done:
                phase = "LAUNCH-KICK"
                rate_text = "NAV"
            else:
                phase = "TAPER" if lidar >= taper_start_m else "FAST"
                rate_text = f"{(last_rate or 0.0):.2f}m/s"
            print(
                f"[CLIMB:{phase}] LiDAR={lidar:.2f}m relAlt={_fmt(rel_alt)}m "
                f"rate={rate_text} target>={trigger_m:.2f}m"
            )
            last_print = now

        if lidar >= trigger_m:
            send_velocity_ned(test, 0.0)
            print(f"[TARGET] LiDAR={lidar:.2f}m -> ZERO VELOCITY")
            break

        if not handoff_done and lidar >= handoff_m:
            handoff_done = True
            rate = near_rate_mps if lidar >= taper_start_m else fast_rate_mps
            send_velocity_ned(test, -rate)
            last_rate = rate
            next_velocity_send = now + 0.10
            print(
                f"[HANDOFF] LiDAR={lidar:.2f}m: NAV_TAKEOFF -> GUIDED VELOCITY "
                f"vz={-rate:.2f}m/s"
            )
    else:
        if handoff_done:
            send_velocity_ned(test, 0.0)
        raise FlightTestError(
            f"profiled takeoff timeout before LiDAR target; LiDAR={_fmt(test.lidar)}m"
        )

    # Not a hover/dwell. This only refreshes zero velocity a couple of times so
    # an old upward velocity setpoint cannot survive into the AUTO handoff.
    if zero_flush_s > 0.0:
        flush_deadline = time.monotonic() + zero_flush_s
        next_zero = 0.0
        while time.monotonic() < flush_deadline:
            now = time.monotonic()
            if now >= next_zero:
                send_velocity_ned(test, 0.0)
                next_zero = now + 0.05
            test.poll(0.03)
            if not test.lidar_ok():
                continue
            if float(test.lidar) >= ceiling_m:
                send_velocity_ned(test, 0.0)
                raise FlightTestError(
                    f"LiDAR exceeded ceiling during zero flush: {test.lidar:.2f}m"
                )

    send_velocity_ned(test, 0.0)
    print(
        f"[TAKEOFF-PROFILE] COMPLETE LiDAR={_fmt(test.lidar)}m | "
        f"zero-flush={zero_flush_s:.2f}s | NO HOLD"
    )
