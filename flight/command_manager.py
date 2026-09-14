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
        """GUIDED -> verified ARM -> immediate TAKEOFF -> fresh target samples.

        No fixed post-arm delay or motor parameter override. FC owns spool-up.
        Success confirms target altitude, not stable hover. Flight-tool only.
        """
        altitude_m = float(altitude_m)
        if (not all(math.isfinite(float(x)) for x in
                    (altitude_m, min_alt_m, max_alt_m, verify_ratio, state_timeout_s))
                or not 0 < min_alt_m <= altitude_m <= max_alt_m
                or not 0 < verify_ratio <= 1 or state_timeout_s <= 0):
            return VctResult(False, 0, "invalid takeoff target or verification limits")

        v = self.state.vehicle
        freshness_s = 0.30

        def altitude_sample():
            # NAV_TAKEOFF target is relative to home; do not mix it with LiDAR AGL.
            alt = v.relative_alt_m
            ts = v.message_last_ns.get("GLOBAL_POSITION_INT", 0)
            if (alt is None or not math.isfinite(float(alt)) or not ts or
                    (time.monotonic_ns() - ts) / 1e9 > freshness_s):
                return None, ts
            return float(alt), ts

        ok, reason = self._link_precheck()
        if not ok:
            return VctResult(False, 0, reason)
        baseline, _ = altitude_sample()
        if baseline is None:
            return VctResult(False, 0, "fresh GLOBAL_POSITION_INT relative altitude required before ARM")
        if altitude_m - baseline < 0.2:
            return VctResult(False, 0, "target must be at least 0.2m above current relative altitude")

        mode_result = self.set_mode("GUIDED", trace_fn=trace_fn)
        if not mode_result.ok:
            return mode_result
        if not v.armed:
            arm_result = self.arm(trace_fn=trace_fn)
            if not arm_result.ok:
                return arm_result
        # No sleep here: dispatch as soon as armed is verified.
        sent_ns = [0]
        last_sample_ns = [0]
        target_samples = [0]
        rising_reported = [False]
        tolerance = min(0.15, altitude_m * (1.0 - verify_ratio))

        def takeoff_precheck():
            ok, reason = self._link_precheck()
            if not ok:
                return ok, reason
            if v.mode != "GUIDED":
                return False, f"mode is {v.mode}, expected GUIDED"
            if not v.armed:
                return False, "vehicle is not armed"
            return True, ""

        def send_takeoff():
            nonlocal baseline
            ok, _ = takeoff_precheck()
            baseline, _ = altitude_sample()
            if not ok or baseline is None or altitude_m - baseline < 0.2:
                return False
            sent_ns[0] = time.monotonic_ns()
            return self.tx.takeoff(altitude_m)

        def altitude_verified():
            alt, ts = altitude_sample()
            if not sent_ns[0] or alt is None or ts <= sent_ns[0]:
                target_samples[0] = 0
                return False
            if ts == last_sample_ns[0]:
                return False
            last_sample_ns[0] = ts
            if alt >= baseline + 0.2 and not rising_reported[0]:
                rising_reported[0] = True
                if trace_fn:
                    try:
                        trace_fn("CLIMB_OBSERVED", {"relative_alt_m": alt})
                    except Exception:
                        pass
            at_target = abs(alt - altitude_m) <= tolerance and alt >= baseline + 0.2
            target_samples[0] = target_samples[0] + 1 if at_target else 0
            return target_samples[0] >= 3

        kwargs = self._vct_kwargs()
        kwargs["ack_timeout_s"] = float(self.vct_cfg.get("takeoff_ack_timeout_s", 2.0))
        kwargs["state_timeout_s"] = float(state_timeout_s)
        kwargs["attempts"] = 1
        result = VerifiedCommandTransaction(
            f"TAKEOFF_{altitude_m:.2f}M",
            send_fn=send_takeoff,
            verify_fn=altitude_verified,
            precheck_fn=takeoff_precheck,
            abort_fn=takeoff_precheck,
            allow_already_satisfied=False,
            expected_command=22,
            require_ack=True,
            terminal_reject_results=(1, 2, 3, 4, 6, 7, 8, 9),
            trace_fn=trace_fn,
            **kwargs,
        ).run()
        if result.ok:
            result.reason = "TAKEOFF ACK accepted + 3 fresh relative-altitude samples at target; hover not assessed"
        return result

    def land(self, trace_fn=None):
        return self.set_mode("LAND", trace_fn=trace_fn)

