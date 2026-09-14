import argparse
import queue
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from pathlib import Path
import runtestwp as wp
from flight.auto_test_mission import build_mission, MissionTransfer
from mission.waypoint_manager import WaypointManager
from flight.vehicle_state import VehicleState

class Tests(unittest.TestCase):
    def route(self):
        return [{'number':2,'lat_deg':-7.8,'lon_deg':110.3,'alt_m':1.2},
                {'number':3,'lat_deg':-7.80001,'lon_deg':110.30001,'alt_m':1.2}]
    def test_range(self):
        self.assertEqual(wp.waypoint_range('2-2'),[2])
        self.assertEqual(wp.waypoint_range('1-2'),[1,2])
        for x in ('0','2-1','1-13','1/2'):
            with self.assertRaises(argparse.ArgumentTypeError): wp.waypoint_range(x)
    def test_alias(self):
        a=wp.parser().parse_args(['--strat','r','-goto','2-2'])
        self.assertEqual((a.strat,a.goto,a.execute),('RIGHT',[2],False))
    def test_global_plan(self):
        data={'RIGHT':{'WP2':dict(self.route()[0],frame='GLOBAL_RELATIVE_ALT')}}
        p=wp.plan(data,'RIGHT',[2],(-7.8001,110.3))
        self.assertGreater(p[0]['leg_m'],10)
        self.assertEqual(p[0]['lat_deg'],-7.8)
        with self.assertRaises(ValueError): wp.plan(data,'RIGHT',[2],(0,0))
    def test_legacy_refused(self):
        with self.assertRaises(ValueError): wp.plan({'RIGHT':{'WP1':{'x':1,'y':2,'alt_m':1.2}}},'RIGHT',[1],(-7.8,110.3))
    def test_nonfinite_refused(self):
        for field in ('lat_deg','lon_deg','alt_m'):
            row=dict(self.route()[0],frame='GLOBAL_RELATIVE_ALT');row[field]=float('nan')
            with self.assertRaises(ValueError): wp.plan({'RIGHT':{'WP2':row}},'RIGHT',[2],(-7.8,110.3))
    def test_auto_mission(self):
        items=build_mission(self.route(),(-7.81,110.31),1)
        self.assertEqual([x['command'] for x in items],[16,178,16,16,21])
        self.assertEqual(items[2]['x'],round(-7.8*1e7))
        self.assertEqual(items[-1]['x'],items[-2]['x'])
        self.assertEqual(items[-1]['y'],items[-2]['y'])
        self.assertEqual(items[1]['param2'],1)
        self.assertNotIn(22,[x['command'] for x in items])
        self.assertNotIn(179,[x['command'] for x in items])
    def test_capture_global_without_local(self):
        import time
        v=VehicleState(connected=True,gps_fix_type=3,global_lat_deg=-7.8,global_lon_deg=110.3)
        v.last_heartbeat_ns=v.global_position_last_ns=time.monotonic_ns();v.mark_message('GPS_RAW_INT')
        state=NS(vehicle=v,system={})
        with patch('mission.waypoint_manager.load_json',side_effect=[{'RIGHT':{'WP2':{}}},{}]),patch('mission.waypoint_manager.save_json_atomic') as save:
            m=WaypointManager(state);m.unlock()
            self.assertTrue(m.save_current('RIGHT',2)[0]);save.assert_called_once()
            self.assertEqual(m.get('RIGHT',2)['lat_deg'],-7.8)
            state.system['wp_test_active']=True
            self.assertFalse(m.save_current('RIGHT',2)[0])
    def transfer(self):
        r=NS(rx=NS(),tx=NS(target_system=1,target_component=1))
        return MissionTransfer(r,lambda:(True,''))
    def msg(self,kind,**kw): return NS(get_type=lambda:kind,**kw)
    def test_upload_ack_and_requests(self):
        t=self.transfer();items=build_mission(self.route(),(-7.8,110.3),1);sent=[]
        def send(name,*args):
            sent.append(name)
            if name=='mission_count_send':
                for i in range(len(items)): t.q.put(self.msg('MISSION_REQUEST_INT',seq=i))
                t.q.put(self.msg('MISSION_ACK',type=0))
        t.send=send;t.upload(items)
        self.assertEqual(sent.count('mission_item_int_send'),len(items))
    def test_rejected_or_premature_upload(self):
        for code in (0,1):
            t=self.transfer()
            t.send=lambda *args:t.q.put(self.msg('MISSION_ACK',type=code))
            with self.assertRaises(RuntimeError): t.upload(build_mission(self.route(),(-7.8,110.3),1))
    def test_readback_wrong_target(self):
        t=self.transfer();items=build_mission(self.route(),(-7.8,110.3),1)
        def send(name,*args):
            if name=='mission_request_list_send':t.q.put(self.msg('MISSION_COUNT',count=len(items)))
            elif name=='mission_request_int_send':
                i=dict(items[args[-1]])
                if i['seq']==2:i['x']+=100
                t.q.put(self.msg('MISSION_ITEM_INT',**i))
        t.send=send
        with self.assertRaisesRegex(RuntimeError,'coordinate'):t.verify_uploaded(items)
    def test_readback_matches(self):
        t=self.transfer();items=build_mission(self.route(),(-7.8,110.3),1)
        def send(name,*args):
            if name=='mission_request_list_send':t.q.put(self.msg('MISSION_COUNT',count=len(items)))
            elif name=='mission_request_int_send':t.q.put(self.msg('MISSION_ITEM_INT',**items[args[-1]]))
        t.send=send;t.verify_uploaded(items)
    def test_backup_before_overwrite(self):
        t=self.transfer();calls=[]
        def send(name,*args):
            calls.append(name)
            if name=='mission_request_list_send':t.q.put(self.msg('MISSION_COUNT',count=0))
        t.send=send
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'backup.json';t.backup(path)
            self.assertIn('"count": 0',path.read_text())
        self.assertNotIn('mission_count_send',calls)

    def test_completion_requires_reached_landed_and_disarmed(self):
        import time
        for scenario in ('done', 'early_land', 'missing_reached', 'not_landed', 'override', 'abort'):
            v=VehicleState(connected=True,armed=True,mode='GUIDED',gps_fix_type=3,
                           global_lat_deg=-7.8,global_lon_deg=110.3,relative_alt_m=1.2)
            v.last_heartbeat_ns=v.global_position_last_ns=time.monotonic_ns()
            for kind in ('GPS_RAW_INT','GLOBAL_POSITION_INT','EXTENDED_SYS_STATE'):v.mark_message(kind)
            r=NS(state=NS(vehicle=v,system={}),stop_event=NS(is_set=lambda:False),tx=NS())
            t=NS(q=queue.Queue(),clear=lambda:None,ids=lambda:(1,1),send=lambda *a:None)
            r.tx.send_mode=lambda *a:True
            test=wp.FlightTest(r,NS(goto_timeout=1,land_timeout=1))
            def transaction(*a,**kw):
                if a[0]=='AUTO':
                    v.mode='AUTO';v.armed=False;v.landed_state=1
                    v.last_heartbeat_ns=time.monotonic_ns()
                    if scenario!='missing_reached':
                        for seq in (2,3):t.q.put(self.msg('MISSION_ITEM_REACHED',seq=seq))
                    if scenario=='early_land':v.mode='LAND';t.q=queue.Queue()
                    if scenario=='not_landed':v.landed_state=2
                    if scenario=='override':v.mode='ALTHOLD'
                    if scenario=='abort':r.state.system['wp_test_abort']=True
                return NS(run=lambda:NS(ok=True,reason=''))
            with patch('flight.vct.VerifiedCommandTransaction',side_effect=transaction):
                if scenario=='done':test.auto_and_land(self.route(),t)
                else:
                    with self.assertRaises(RuntimeError):test.auto_and_land(self.route(),t)

if __name__=='__main__':unittest.main()
