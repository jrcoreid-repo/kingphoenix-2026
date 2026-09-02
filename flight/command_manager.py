from __future__ import annotations

from flight.vct import VerifiedCommandTransaction


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
            verify_fn=lambda: self.state.vehicle.mode == mode,
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
        vct = VerifiedCommandTransaction(
            "ARM",
            send_fn=self.tx.arm,
            verify_fn=lambda: self.state.vehicle.armed,
            precheck_fn=self._link_precheck,
            expected_command=cmd,
            require_ack=True,
            terminal_reject_results=terminal_rejects,
            trace_fn=trace_fn,
            **self._vct_kwargs(),
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
            verify_fn=lambda: not self.state.vehicle.armed,
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
        """Verified guided takeoff sequence: GUIDED -> ARM -> TAKEOFF -> altitude.

        This is intended for standalone flight tools first. It is deliberately
        not exposed through the GCS mission buttons yet.
        """
        altitude_m = float(altitude_m)
        if altitude_m < float(min_alt_m) or altitude_m > float(max_alt_m):
            from flight.vct import VctResult
            return VctResult(False, 0, f"takeoff altitude {altitude_m:.2f}m outside allowed {min_alt_m:.2f}-{max_alt_m:.2f}m")

        mode_result = self.set_mode("GUIDED", trace_fn=trace_fn)
        if not mode_result.ok:
            return mode_result

        if not self.state.vehicle.armed:
            arm_result = self.arm(trace_fn=trace_fn)
            if not arm_result.ok:
                return arm_result

        try:
            from pymavlink import mavutil
            cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        except Exception:
            cmd = 22

        threshold_m = max(float(min_alt_m) * 0.8, altitude_m * float(verify_ratio))

        def takeoff_precheck():
            ok, reason = self._link_precheck()
            if not ok:
                return ok, reason
            if self.state.vehicle.mode != "GUIDED":
                return False, f"mode is {self.state.vehicle.mode}, expected GUIDED"
            if not self.state.vehicle.armed:
                return False, "vehicle is not armed"
            return True, ""

        def altitude_verified():
            v = self.state.vehicle
            alt, source, fresh = v.altitude_reference(0.30)
            if alt is None:
                return False
            return float(alt) >= threshold_m

        kwargs = self._vct_kwargs()
        kwargs["state_timeout_s"] = float(state_timeout_s)
        kwargs["attempts"] = 1  # never repeat a physical takeoff command automatically

        return VerifiedCommandTransaction(
            f"TAKEOFF_{altitude_m:.2f}M",
            send_fn=lambda: self.tx.takeoff(altitude_m),
            verify_fn=altitude_verified,
            precheck_fn=takeoff_precheck,
            expected_command=cmd,
            require_ack=True,
            terminal_reject_results=(1, 2, 3, 4, 6, 7, 8, 9),
            trace_fn=trace_fn,
            **kwargs,
        ).run()

    def land(self, trace_fn=None):
        return self.set_mode("LAND", trace_fn=trace_fn)
