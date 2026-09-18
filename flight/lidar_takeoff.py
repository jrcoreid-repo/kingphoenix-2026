from __future__ import annotations

import time

from safety.pilot_takeover import PilotTakeover


class LidarTakeoffError(RuntimeError):
    pass


class ProfiledLidarTakeoff:
    """Production form of the field-proven profiled LiDAR takeoff helper."""

    def __init__(self, state, command, tx, cfg: dict, log=print):
        self.state = state
        self.command = command
        self.tx = tx
        self.cfg = dict(cfg or {})
        self.log = log

    def _zero(self):
        self.tx.send_local_ned_velocity(0.0)

    def run(self, abort_event=None):
        c = self.cfg
        target = float(c["target_alt_m"])
        tolerance = float(c["tolerance_m"])
        trigger = target - tolerance
        handoff = float(c["handoff_m"])
        fast = abs(float(c["fast_vz_mps"]))
        near = abs(float(c["near_vz_mps"]))
        taper_zone = float(c["taper_zone_m"])
        taper_start = max(handoff, trigger - taper_zone)
        kick_alt = float(c["kick_alt_m"])
        ceiling = float(c["ceiling_m"])
        timeout = float(c["timeout_s"])
        stale_s = float(c.get("lidar_stale_s", 0.60))
        arm_settle = float(c.get("arm_settle_s", 0.35))
        zero_flush = max(0.0, float(c.get("zero_flush_s", 0.15)))
        command_hz = max(2.0, float(c.get("command_hz", 10.0)))
        command_period = 1.0 / command_hz

        self.log(
            f"[TAKEOFF] target={target:.2f}m | Vz={fast:.2f}m/s | "
            f"taper={near:.2f}m/s | trigger={trigger:.2f}m"
        )

        mode = self.command.set_mode("GUIDED")
        if not mode.ok:
            raise LidarTakeoffError(f"GUIDED failed: {mode.reason}")

        if not self.state.vehicle.armed:
            arm = self.command.arm()
            if not arm.ok:
                raise LidarTakeoffError(f"ARM failed: {arm.reason}")

        settle_end = time.monotonic() + arm_settle
        while time.monotonic() < settle_end:
            if abort_event is not None and abort_event.is_set():
                self._zero()
                raise LidarTakeoffError("aborted during ARM settle")
            time.sleep(0.02)

        if not self.tx.takeoff(kick_alt):
            raise LidarTakeoffError("NAV_TAKEOFF kick TX failed")

        deadline = time.monotonic() + timeout
        handoff_done = False
        lidar_loss_since = None
        next_tx = 0.0

        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                self._zero()
                raise LidarTakeoffError("aborted during takeoff")

            v = self.state.vehicle
            if not v.connected or v.heartbeat_age_s() > 2.5:
                self._zero()
                raise LidarTakeoffError("heartbeat lost during takeoff")
            if v.mode != "GUIDED":
                self._zero()
                raise PilotTakeover(f"operator changed mode during takeoff to {v.mode}")

            now = time.monotonic()
            if not v.rangefinder_is_fresh(stale_s):
                lidar_loss_since = lidar_loss_since or now
                if now - lidar_loss_since > stale_s:
                    self._zero()
                    raise LidarTakeoffError(f"LiDAR stale >{stale_s:.2f}s during takeoff")
                time.sleep(0.01)
                continue

            lidar_loss_since = None
            lidar = float(v.rangefinder_distance_m)
            if lidar >= ceiling:
                self._zero()
                raise LidarTakeoffError(f"LiDAR hard ceiling reached: {lidar:.2f}m")
            if lidar >= trigger:
                self._zero()
                break

            if not handoff_done and lidar >= handoff:
                handoff_done = True
                next_tx = 0.0
                self.log(f"[TAKEOFF] LiDAR {lidar:.2f}m -> GUIDED velocity handoff")

            if handoff_done and now >= next_tx:
                rate = near if lidar >= taper_start else fast
                self.tx.send_local_ned_velocity(-rate)
                next_tx = now + command_period

            time.sleep(0.01)
        else:
            self._zero()
            lidar = self.state.vehicle.rangefinder_distance_m
            raise LidarTakeoffError(f"takeoff timeout; LiDAR={lidar}")

        flush_end = time.monotonic() + zero_flush
        next_zero = 0.0
        while time.monotonic() < flush_end:
            now = time.monotonic()
            if now >= next_zero:
                self._zero()
                next_zero = now + 0.05
            if abort_event is not None and abort_event.is_set():
                raise LidarTakeoffError("aborted during zero flush")
            time.sleep(0.01)

        self._zero()
        lidar = self.state.vehicle.rangefinder_distance_m
        self.log(f"[TAKEOFF] COMPLETE | LiDAR={float(lidar):.2f}m | no dwell")
        return True
