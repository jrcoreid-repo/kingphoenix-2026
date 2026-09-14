from __future__ import annotations

import math
import time

from flight.vct import VerifiedCommandTransaction, VctResult


class CommandManager:
    """High-level critical-command API with VCT foundation."""

    def __init__(self, tx, state, events, freshness_policy=None, vct_cfg=None):
        self.tx = tx
        self.state = state
        self.events = events
        self.freshness = freshness_policy
        self.vct_cfg = vct_cfg or {}

    def _link_precheck(self, allow_land=False):
        v = self.state.vehicle
        if not v.connected:
            return False, "MAVLink not connected"
        if self.freshness is not None:
            ok, reason = self.freshness.require("HEARTBEAT")
            if not ok:
                return False, reason
        elif v.heartbeat_age_s() > 2.5:
            return False, "stale heartbeat"
        if v.mode == "LAND" and not allow_land:
            return False, "LAND has priority"
        if v.autopilot_system_id is not None and v.autopilot_system_id != self.tx.target_system:
            return False, f"unexpected FC sysid {v.autopilot_system_id}, expected {self.tx.target_system}"
        if v.autopilot_component_id is not None and v.autopilot_component_id != self.tx.target_component:
            return False, f"unexpected FC compid {v.autopilot_component_id}, expected {self.tx.target_component}"
        return True, ""

    def _vct_kwargs(self):
        backoff = self.vct_cfg.get("backoff_s", [0.25, 0.5, 0.9])
        return {
            "vehicle_state": self.state.vehicle,
            "expected_ack_system": self.tx.target_system,
            "expected_ack_component": self.tx.target_component,
            "ack_timeout_s": float(self.vct_cfg.get("ack_timeout_s", 0.8)),
            "state_timeout_s": float(self.vct_cfg.get("state_timeout_s", 1.5)),
            "attempts": int(self.vct_cfg.get("max_attempts", 3)),
            "backoff": tuple(backoff),
        }

    def set_mode(self, mode, trace_fn=None, require_disarmed=False):
        mode = str(mode).upper()

        def mode_precheck():
            ok, reason = self._link_precheck()
            if not ok:
                return ok, reason
            if require_disarmed and self.state.vehicle.armed:
                return False, "vehicle is armed; bench mode test requires DISARMED"
            return True, ""

        vct = VerifiedCommandTransaction(
            f"MODE_{mode}",
            send_fn=lambda: self.tx.send_mode(mode),
            verify_fn=lambda: self._link_precheck()[0] and self.state.vehicle.mode == mode,
            precheck_fn=mode_precheck,
            require_ack=False,
            trace_fn=trace_fn,
            **self._vct_kwargs(),
        )
        return vct.run()

    def arm(self, trace_fn=None):
        try:
            from pymavlink import mavutil
            cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        except Exception:
            cmd = 400
        terminal_rejects = tuple(self.vct_cfg.get("arm_terminal_reject_results", [1, 2, 3, 4, 6, 7, 8, 9]))
        sent_ns = [0]

        def send_arm():
            sent_ns[0] = time.monotonic_ns()
            return self.tx.arm()

        def armed_verified():
            v = self.state.vehicle
            return (self._link_precheck()[0] and v.armed and
                    (not sent_ns[0] or v.last_heartbeat_ns > sent_ns[0]))

        kwargs = self._vct_kwargs()
        kwargs["ack_timeout_s"] = float(self.vct_cfg.get("arm_ack_timeout_s", 2.0))
        kwargs["state_timeout_s"] = float(self.vct_cfg.get("arm_state_timeout_s", 8.0))
        kwargs["attempts"] = 1
        vct = VerifiedCommandTransaction(
            "ARM",
            send_fn=send_arm,
            verify_fn=armed_verified,
            precheck_fn=self._link_precheck,
            expected_command=cmd,
            require_ack=True,
            terminal_reject_results=terminal_rejects,
            trace_fn=trace_fn,
            abort_fn=self._link_precheck,
            progress_timeout_s=float(self.vct_cfg.get("arm_progress_timeout_s", 10.0)),
            **kwargs,
        )
        return vct.run()

    def disarm(self, trace_fn=None):
        try:
            from pymavlink import mavutil
            cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        except Exception:
            cmd = 400

        def disarm_precheck():
            return self._link_precheck(allow_land=True)

        terminal_rejects = tuple(self.vct_cfg.get("disarm_terminal_reject_results", [1, 2, 3, 4, 6, 7, 8, 9]))
        vct = VerifiedCommandTransaction(
            "DISARM",
            send_fn=self.tx.disarm,
            verify_fn=lambda: self._link_precheck(allow_land=True)[0] and not self.state.vehicle.armed,
            precheck_fn=disarm_precheck,
            expected_command=cmd,
            require_ack=True,
            terminal_reject_results=terminal_rejects,
            trace_fn=trace_fn,
            **self._vct_kwargs(),
        )
        return vct.run()

    def takeoff(self, altitude_m, trace_fn=None, min_alt_m=0.5, max_alt_m=3.0,
                verify_ratio=0.8, state_timeout_s=10.0):
        """Takeoff to LiDAR sensor-to-ground height, brake and verify stable hold.

        verify_ratio is retained only for old callers; LiDAR uses fixed 0.10m tolerance.
        All high-level takeoff callers share this implementation.
        """
        from flight.lidar_takeoff import LidarTakeoff
        try:
            values = [float(x) for x in (altitude_m, min_alt_m, max_alt_m, verify_ratio, state_timeout_s)]
        except (TypeError, ValueError):
            return VctResult(False, 0, "invalid takeoff target or verification limits")
        altitude_m, min_alt_m, max_alt_m, verify_ratio, state_timeout_s = values
        if (not all(math.isfinite(x) for x in values) or
                not 0 < min_alt_m <= altitude_m <= max_alt_m <= 3.0 or
                not 0 < verify_ratio <= 1 or not 0 < state_timeout_s <= 60):
            return VctResult(False, 0, "invalid takeoff target or verification limits")
        return LidarTakeoff(self, altitude_m, state_timeout_s, trace_fn).run()

    def land(self, trace_fn=None):
        return self.set_mode("LAND", trace_fn=trace_fn)


