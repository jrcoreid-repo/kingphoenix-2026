"""LiDAR-triggered takeoff braking. No fallback to barometer for completion."""
import math
import time
from flight.vct import VerifiedCommandTransaction, VctResult


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class LidarTakeoff:
    # Fixed conservative test envelope; not an airborne tuning interface.
    freshness_s = .25
    tolerance_m = .10
    stable_span_s = .5
    max_vertical_speed = .15
    brake_timeout_s = 3.0

    def __init__(self, manager, target, timeout, trace=None):
        self.m = manager
        self.v = manager.state.vehicle
        self.target, self.timeout, self.trace = target, timeout, trace
        self.sensor_id = None
        self.sent_ns = 0
        self.brake_ns = 0
        self.last_ns = 0
        self.previous = None
        self.stable_since = None
        self.stable_count = 0
        self.command_target = None

    def emit(self, event, **data):
        if self.trace:
            try: self.trace(event, data)
            except Exception: pass

    def sample(self):
        v = self.v
        d, ts = v.rangefinder_distance_m, v.rangefinder_last_ns
        if (not finite(d) or not ts or not 0 <= v.rangefinder_age_s() <= self.freshness_s):
            raise ValueError('LiDAR missing, nonfinite or stale')
        if v.rangefinder_source != 'DISTANCE_SENSOR' or v.rangefinder_orientation != 25:
            raise ValueError('downward DISTANCE_SENSOR required; legacy rangefinder disabled')
        if self.sensor_id is not None and v.rangefinder_id != self.sensor_id:
            raise ValueError('LiDAR sensor ID changed')
        if (not finite(v.rangefinder_min_m) or not finite(v.rangefinder_max_m) or
                not 0 < v.rangefinder_min_m <= d <= v.rangefinder_max_m or
                self.target + self.tolerance_m > v.rangefinder_max_m):
            raise ValueError('LiDAR distance/target outside sensor range')
        quality = v.rangefinder_signal_quality
        # MAVLink: 0 is unknown, 1 is invalid, 100 is perfect.
        if quality is not None and (quality == 1 or not 0 <= quality <= 100):
            raise ValueError('LiDAR signal quality invalid')
        if (v.message_age_s('ATTITUDE') > .3 or not all(finite(x) for x in (v.roll, v.pitch)) or
                max(abs(v.roll), abs(v.pitch)) > math.radians(20)):
            raise ValueError('fresh attitude within 20 degrees required')
        # Sensor-to-ground vertical distance on approximately level terrain.
        return d * math.cos(v.roll) * math.cos(v.pitch), ts

    def guard(self, modes=None):
        ok, reason = self.m._link_precheck()
        if not ok: return ok, reason
        if getattr(self.m.state, 'system', {}).get('wp_test_abort'):
            return False, 'operator abort'
        if modes and self.v.mode not in modes:
            return False, 'pilot/FC mode override: ' + self.v.mode
        if (self.v.message_age_s('GPS_RAW_INT') > 1.5 or (self.v.gps_fix_type or 0) < 3):
            return False, 'fresh GPS 3D fix required for GUIDED/BRAKE'
        try: self.sample()
        except ValueError as exc: return False, str(exc)
        return True, ''

    def brake_allowed(self):
        v = self.v
        return (v.connected and v.heartbeat_age_s() <= 2.5 and v.armed and
                v.mode in ('GUIDED', 'BRAKE') and
                v.autopilot_system_id in (None, self.m.tx.target_system) and
                v.autopilot_component_id in (None, self.m.tx.target_component)), 'link lost / disarmed / mode override'

    def send_brake(self):
        if not self.brake_allowed()[0]: return False
        self.brake_ns = time.monotonic_ns()
        self.emit('LIDAR_BRAKE_REQUEST', target_agl_m=self.target)
        return bool(self.m.tx.send_mode('BRAKE'))

    def brake_confirmed(self):
        return (self.brake_ns > 0 and self.v.mode == 'BRAKE' and
                self.v.last_heartbeat_ns > self.brake_ns and self.v.heartbeat_age_s() <= 2.5)

    def recover(self):
        # Independent of LiDAR/GPS guards: still try to cancel climb on sensor loss.
        if self.brake_confirmed(): return 'BRAKE confirmed; RC mode switch to take over'
        if not self.sent_ns or not self.brake_allowed()[0]:
            return 'BRAKE not confirmed; preserve pilot/FC mode; RC takeover required'
        r = VerifiedCommandTransaction('TAKEOFF_ABORT_BRAKE', self.send_brake,
            self.brake_confirmed, precheck_fn=self.brake_allowed, abort_fn=self.brake_allowed,
            allow_already_satisfied=False, attempts=1, state_timeout_s=self.brake_timeout_s,
            trace_fn=self.trace).run()
        return ('BRAKE confirmed' if r.ok else 'BRAKE UNCONFIRMED; FC action may remain active') + '; RC mode switch to take over'

    def monitor(self):
        ok, reason = self.guard(('GUIDED', 'BRAKE') if self.brake_ns else ('GUIDED',))
        if not ok: return ok, reason
        if not self.v.armed: return False, 'disarmed during takeoff'
        if self.brake_ns and (time.monotonic_ns() - self.brake_ns) / 1e9 > self.brake_timeout_s and not self.brake_confirmed():
            return False, 'BRAKE heartbeat timeout'
        return True, ''

    def verify(self):
        alt, ts = self.sample()
        if ts <= self.sent_ns or ts <= self.last_ns: return False
        self.last_ns = ts
        if not self.brake_ns and alt >= self.target:
            if not self.send_brake(): raise RuntimeError('BRAKE transmission failed')
        # Count only independent samples measured after the mode request and its confirmation.
        if not self.brake_confirmed() or ts <= self.brake_ns:
            self.previous = (alt, ts)
            return False
        speed = float('inf')
        if self.previous is not None:
            old, old_ts = self.previous
            speed = abs(alt - old) / ((ts - old_ts) / 1e9)
        self.previous = (alt, ts)
        if abs(alt - self.target) > self.tolerance_m or speed > self.max_vertical_speed:
            self.stable_since = None
            self.stable_count = 0
            return False
        if self.stable_since is None: self.stable_since = ts
        self.stable_count += 1
        return self.stable_count >= 5 and (ts - self.stable_since) / 1e9 >= self.stable_span_s

    def run(self):
        ok, reason = self.guard()
        if not ok: return VctResult(False, 0, reason)
        baseline, _ = self.sample()
        self.sensor_id = self.v.rangefinder_id
        if self.sensor_id is None: return VctResult(False, 0, 'LiDAR sensor ID required')
        if self.target - baseline < .2:
            return VctResult(False, 0, 'LiDAR target must be at least 0.2m above current height')
        def relative_sample():
            v = self.v
            if not finite(v.relative_alt_m) or v.message_age_s('GLOBAL_POSITION_INT') > .3:
                raise ValueError('fresh FC relative altitude required to translate initial target')
            return v.relative_alt_m
        try: relative_sample()
        except ValueError as exc: return VctResult(False, 0, str(exc))
        initial_mode = self.v.mode
        mode_since = time.monotonic_ns()
        mode = VerifiedCommandTransaction('MODE_GUIDED', lambda: self.m.tx.send_mode('GUIDED'),
            lambda: self.v.mode == 'GUIDED' and self.v.last_heartbeat_ns > mode_since,
            precheck_fn=lambda: self.guard((initial_mode, 'GUIDED')),
            abort_fn=lambda: self.guard((initial_mode, 'GUIDED')), allow_already_satisfied=False,
            attempts=1, state_timeout_s=3, trace_fn=self.trace).run()
        if not mode.ok: return mode
        # ARM also observes sensor loss and pilot override; restore caller's hook afterwards.
        original = self.m._link_precheck
        def arm_guard(allow_land=False):
            # Avoid recursion through guard -> _link_precheck.
            ok, why = original(allow_land=allow_land)
            if not ok: return ok, why
            if self.v.mode != 'GUIDED': return False, 'mode override during ARM'
            if getattr(self.m.state, 'system', {}).get('wp_test_abort'): return False, 'operator abort'
            try: self.sample()
            except ValueError as exc: return False, str(exc)
            if self.v.message_age_s('GPS_RAW_INT') > 1.5 or (self.v.gps_fix_type or 0) < 3:
                return False, 'GPS lost during ARM'
            return True, ''
        self.m._link_precheck = arm_guard
        try:
            if not self.v.armed:
                arm = self.m.arm(trace_fn=self.trace)
                if not arm.ok: return arm
        finally: self.m._link_precheck = original

        def send():
            ok, _ = self.monitor()
            if not ok: return False
            agl, _ = self.sample()
            rise = self.target - agl
            if rise < .2: return False
            self.command_target = relative_sample() + rise * 1.05
            if not finite(self.command_target) or self.command_target <= 0:
                return False
            self.emit('TAKEOFF_TARGET', lidar_target_m=self.target, fc_relative_target_m=self.command_target)
            self.sent_ns = time.monotonic_ns()
            return self.m.tx.takeoff(self.command_target)
        kwargs = self.m._vct_kwargs()
        kwargs.update(attempts=1, state_timeout_s=self.timeout,
                      ack_timeout_s=float(self.m.vct_cfg.get('takeoff_ack_timeout_s', 2)))
        try:
            result = VerifiedCommandTransaction('LIDAR_TAKEOFF', send, self.verify,
                precheck_fn=self.monitor, abort_fn=self.monitor, allow_already_satisfied=False,
                expected_command=22, require_ack=True, terminal_reject_results=(1,2,3,4,6,7,8,9),
                trace_fn=self.trace, **kwargs).run()
        except (Exception, KeyboardInterrupt) as exc:
            result = VctResult(False, 1, str(exc) or 'operator interrupted', outcome='UNCONFIRMED')
        if result.ok:
            self.m.last_takeoff_lidar_id = self.sensor_id
            result.reason = 'TAKEOFF ACK accepted + BRAKE heartbeat + stable LiDAR height within 0.10m for >=0.5s; holding in BRAKE'
        else:
            try: result.reason += '; ' + self.recover()
            except Exception as exc: result.reason += '; BRAKE recovery unconfirmed: ' + str(exc)
        return result
