"""Exercise the real takeoff and VCT handoff with deterministic FC telemetry."""
import queue
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import test_verified_takeoff as fixtures
from runtestwp import FlightTest


class HandoffTests(unittest.TestCase):
    def run_scenario(self, lose_lidar=False):
        f = fixtures.TakeoffTests()
        f.setUp()
        self.addCleanup(f.tearDown)
        v = f.v
        state = f.manager.state
        state.system = {}
        v.global_lat_deg, v.global_lon_deg = -7.8, 110.3
        v.landed_state = 1
        def telemetry():
            f.telemetry()
            v.global_position_last_ns = f.clock.monotonic_ns()
            v.mark_message('EXTENDED_SYS_STATE')
        f.clock.tick = telemetry
        telemetry()
        calls = []
        transfer = NS(q=queue.Queue(), ids=lambda:(1,1), clear=lambda:None)
        def set_current(*args):
            calls.append('set_current')
            if lose_lidar: f.publish_lidar = False
            def confirmed():
                v.current_mission_item = 1
                v.mark_message('MISSION_CURRENT')
            f.clock.later(.5 if lose_lidar else .02, confirmed)
        transfer.send = set_current
        transfer.backup = lambda *a:calls.append('backup')
        transfer.upload = lambda *a:calls.append('upload')
        transfer.verify_uploaded = lambda *a:calls.append('readback')
        original_mode = f.mode
        def mode(name):
            calls.append(name)
            if name == 'AUTO':
                def done():
                    transfer.q.put(NS(get_type=lambda:'MISSION_ITEM_REACHED',seq=2))
                    v.armed = False
                    v.landed_state = 1
                f.clock.later(.2, done)
            return original_mode(name)
        f.tx.send_mode = mode
        runtime = NS(state=state, command=f.manager, tx=f.tx,
                     stop_event=NS(is_set=lambda:False), waypoints=NS(lock=lambda:None))
        test = FlightTest(runtime,NS(speed=1,goto_timeout=2,land_timeout=2))
        route = [dict(number=2,lat_deg=-7.8,lon_deg=110.3,alt_m=1.2)]
        with patch('flight.auto_test_mission.MissionTransfer',return_value=transfer):
            if lose_lidar:
                with self.assertRaisesRegex(RuntimeError,'mission start index unconfirmed'):
                    test.execute(route)
                self.assertNotIn('AUTO',calls)
                self.assertEqual(v.mode,'BRAKE')
            else:
                test.execute(route)
                self.assertEqual(calls, ['backup','upload','readback','GUIDED','BRAKE','set_current','AUTO'])
                self.assertFalse(v.armed)
    def test_takeoff_brake_auto_landing(self): self.run_scenario()
    def test_sensor_loss_before_auto_blocks_mission(self): self.run_scenario(True)
