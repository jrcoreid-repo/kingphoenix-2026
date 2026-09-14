import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from flight.command_manager import CommandManager
from flight.vehicle_state import VehicleState


class Clock:
    def __init__(self):
        self.t = 1.0
        self.events = []
        self.tick = lambda: None
    def monotonic(self): return self.t
    def monotonic_ns(self): return int(self.t * 1e9)
    def later(self, delay, fn): self.events.append((self.t + delay, fn))
    def sleep(self, delay):
        self.t += delay
        due = [e for e in self.events if e[0] <= self.t]
        self.events = [e for e in self.events if e[0] > self.t]
        for _, fn in due: fn()
        self.tick()


class TakeoffTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.patches = [patch('time.' + name, getattr(self.clock, name))
                        for name in ('monotonic', 'monotonic_ns', 'sleep')]
        for p in self.patches: p.start()
        self.v = VehicleState(connected=True, mode='GUIDED', relative_alt_m=0.0)
        self.sent = []
        self.alt = 0.0
        self.lidar = .1
        self.publish_lidar = True
        self.lidar_stamp = 0
        self.sensor_id = 0
        self.modes = []
        self.brake_works = True
        self.publish_alt = True
        self.publish_heartbeat = True
        self.arm_delay = 0.12
        self.arm_ack = 0
        self.takeoff_ack = 0
        self.climb = True
        self.clock.tick = self.telemetry
        self.telemetry()
        self.tx = SimpleNamespace(target_system=1, target_component=1,
                                  arm=self.arm, takeoff=self.takeoff,
                                  send_mode=self.mode, disarm=lambda: True)
        self.manager = CommandManager(self.tx, SimpleNamespace(vehicle=self.v), None)
    def tearDown(self):
        for p in reversed(self.patches): p.stop()
    def telemetry(self):
        self.v.gps_fix_type = 3
        self.v.mark_message('GPS_RAW_INT')
        self.v.roll = self.v.pitch = 0.
        self.v.mark_message('ATTITUDE')
        if self.publish_lidar and self.clock.monotonic_ns() - self.lidar_stamp >= 100000000:
            self.lidar_stamp = self.clock.monotonic_ns()
            self.v.rangefinder_distance_m = self.lidar
            self.v.rangefinder_last_ns = self.lidar_stamp
            self.v.rangefinder_source = 'DISTANCE_SENSOR'
            self.v.rangefinder_orientation = 25
            self.v.rangefinder_id = self.sensor_id
            self.v.rangefinder_min_m = .1
            self.v.rangefinder_max_m = 12.
        if self.publish_heartbeat:
            self.v.last_heartbeat_ns = self.clock.monotonic_ns()
        if self.publish_alt:
            self.v.relative_alt_m = self.alt
            self.v.mark_message('GLOBAL_POSITION_INT')
    def ack(self, cmd, result, sysid=1):
        self.v.record_command_ack(cmd, result, sysid, 1)
    def mode(self, mode):
        self.modes.append(mode)
        if mode != 'BRAKE' or self.brake_works: self.v.mode = mode
        return True
    def armed(self):
        self.v.armed = True
        self.v.last_heartbeat_ns = self.clock.monotonic_ns()
        self.armed_at = self.clock.t
    def arm(self):
        self.sent.append(('arm', self.clock.t))
        if self.arm_ack is not None: self.ack(400, self.arm_ack)
        if self.arm_ack in (0, 5, None): self.clock.later(self.arm_delay, self.armed)
        return True
    def takeoff(self, alt):
        self.sent.append(('takeoff', self.clock.t))
        if self.takeoff_ack is not None: self.ack(22, self.takeoff_ack)
        self.fc_target = alt
        if self.climb:
            self.clock.later(0.1, lambda: (setattr(self, 'alt', alt), setattr(self, 'lidar', 1.2)))
        return True
    def run_takeoff(self): return self.manager.takeoff(1.2, state_timeout_s=2.0)
    def test_immediate_after_verified_arm(self):
        result = self.run_takeoff()
        self.assertTrue(result.ok, result)
        self.assertLessEqual(self.sent[1][1] - self.armed_at, 0.001)
        self.assertEqual([x[0] for x in self.sent], ['arm', 'takeoff'])
    def test_slow_arm(self):
        self.arm_delay = 3
        self.assertTrue(self.run_takeoff().ok)
        self.assertEqual(len(self.sent), 2)
    def test_progress_waits_for_final_ack(self):
        self.arm_ack = 5
        self.clock.later(2, lambda: self.ack(400, 0))
        self.assertTrue(self.run_takeoff().ok)
        self.assertGreaterEqual(self.sent[1][1], 3)
        self.assertEqual(len(self.sent), 2)
    def test_progress_without_final_ack_never_takeoff(self):
        self.arm_ack = 5
        r = self.run_takeoff()
        self.assertEqual(r.outcome, 'UNCONFIRMED')
        self.assertEqual(len(self.sent), 1)
    def test_arm_rejected(self):
        self.arm_ack = 2
        self.assertEqual(self.run_takeoff().outcome, 'REJECTED')
        self.assertEqual(len(self.sent), 1)
    def test_arm_ack_without_armed(self):
        self.arm_delay = 30
        self.assertFalse(self.run_takeoff().ok)
        self.assertEqual(len(self.sent), 1)
    def test_missing_arm_ack(self):
        self.arm_ack = None
        self.assertEqual(self.run_takeoff().outcome, 'UNCONFIRMED')
        self.assertEqual(len(self.sent), 1)
    def test_takeoff_rejected(self):
        self.takeoff_ack = 2
        self.assertEqual(self.run_takeoff().outcome, 'REJECTED')
        self.assertEqual(len(self.sent), 2)
    def test_missing_takeoff_ack(self):
        self.takeoff_ack = None
        self.assertEqual(self.run_takeoff().outcome, 'UNCONFIRMED')
        self.assertEqual(len(self.sent), 2)
    def test_accepted_no_climb(self):
        self.climb = False
        self.assertFalse(self.run_takeoff().ok)
    def test_stale_altitude_refuses_before_arm(self):
        self.v.message_last_ns['GLOBAL_POSITION_INT'] = 1
        self.assertFalse(self.run_takeoff().ok)
        self.assertEqual(self.sent, [])
    def test_frozen_target_sample_is_not_three_samples(self):
        def freeze():
            self.lidar = 1.2
            self.telemetry()
            self.publish_lidar = False
        self.clock.later(.24, freeze)
        self.assertFalse(self.run_takeoff().ok)
    def test_already_at_lidar_target_refuses_takeoff(self):
        self.climb = False
        self.v.rangefinder_distance_m = 1.2
        self.v.rangefinder_last_ns = self.clock.monotonic_ns()
        self.assertFalse(self.run_takeoff().ok)
    def test_mode_override_stops_monitor(self):
        self.clock.later(.16, lambda: setattr(self.v, 'mode', 'LAND'))
        self.assertFalse(self.run_takeoff().ok)
        self.assertEqual(len(self.sent), 2)
    def test_disarm_stops_monitor(self):
        self.clock.later(.16, lambda: setattr(self.v, 'armed', False))
        self.assertFalse(self.run_takeoff().ok)
    def test_wrong_source_ack(self):
        self.takeoff_ack = None
        self.clock.later(.16, lambda: self.ack(22, 0, 2))
        self.assertFalse(self.run_takeoff().ok)
    def test_invalid_targets_no_transmit(self):
        for x in (math.nan, math.inf, -1, 5):
            self.assertFalse(self.manager.takeoff(x).ok)
        self.assertEqual(self.sent, [])
    def test_progress_takeoff_then_accepted(self):
        self.takeoff_ack = 5
        self.clock.later(.5, lambda: self.ack(22, 0))
        self.assertTrue(self.run_takeoff().ok)
        self.assertEqual(len(self.sent), 2)
    def test_overshoot_not_target_success(self):
        self.climb = False
        self.clock.later(.24, lambda: setattr(self, 'lidar', 2.5))
        self.assertFalse(self.run_takeoff().ok)
    def test_heartbeat_loss(self):
        self.climb = False
        self.clock.later(.16, lambda: setattr(self.v, 'connected', False))
        self.assertFalse(self.run_takeoff().ok)


    def test_relative_altitude_not_completion_source(self):
        def wrong_baro():
            self.alt = 15.
        self.clock.later(.3, wrong_baro)
        result = self.run_takeoff()
        self.assertTrue(result.ok, result)
        self.assertEqual(self.v.mode, 'BRAKE')
        self.assertIn('BRAKE', self.modes)
        self.assertAlmostEqual(self.fc_target, (1.2-.1)*1.05)
    def test_frozen_relative_target_cannot_mask_stale_lidar(self):
        self.clock.later(.3, lambda: setattr(self, 'publish_lidar', False))
        result = self.run_takeoff()
        self.assertFalse(result.ok)
        self.assertIn('BRAKE', self.modes)
    def test_brake_rejected_never_done(self):
        self.brake_works = False
        result = self.run_takeoff()
        self.assertFalse(result.ok)
        self.assertIn('BRAKE', self.modes)
    def test_invalid_lidar_prevents_arm(self):
        for value in (float('nan'), float('inf'), 0., 20.):
            self.v.rangefinder_distance_m = value
            self.assertFalse(self.run_takeoff().ok)
        self.assertEqual(self.sent, [])
    def test_wrong_orientation_and_quality_prevent_arm(self):
        self.v.rangefinder_orientation = 0
        self.assertFalse(self.run_takeoff().ok)
        self.v.rangefinder_orientation = 25
        self.v.rangefinder_signal_quality = 1
        self.assertFalse(self.run_takeoff().ok)
        self.assertEqual(self.sent, [])
    def test_sensor_loss_during_arm_prevents_takeoff(self):
        self.arm_delay = 1.
        self.clock.later(.1, lambda: setattr(self, 'publish_lidar', False))
        self.assertFalse(self.run_takeoff().ok)
        self.assertEqual([x[0] for x in self.sent], ['arm'])
    def test_pilot_land_never_overridden_by_brake(self):
        self.clock.later(.16, lambda: setattr(self.v, 'mode', 'LAND'))
        self.assertFalse(self.run_takeoff().ok)
        self.assertNotIn('BRAKE', self.modes)
    def test_sensor_identity_change_aborts(self):
        self.clock.later(.3, lambda: setattr(self, 'sensor_id', 2))
        self.assertFalse(self.run_takeoff().ok)
        self.assertIn('BRAKE', self.modes)
    def test_baro_target_alone_does_not_finish(self):
        self.climb = False
        self.clock.later(.2, lambda: setattr(self, 'alt', 1.2))
        self.assertFalse(self.run_takeoff().ok)
    def test_missing_ack_still_brakes_at_lidar_target(self):
        self.takeoff_ack = None
        result = self.run_takeoff()
        self.assertFalse(result.ok)
        self.assertEqual(self.v.mode, 'BRAKE')
    def test_sensor_to_home_target_translation(self):
        self.alt = 2.
        self.telemetry()
        self.assertTrue(self.run_takeoff().ok)
        self.assertAlmostEqual(self.fc_target, 2.+(1.2-.1)*1.05)


if __name__ == '__main__': unittest.main()

