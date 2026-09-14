#!/usr/bin/env python3
"""GUIDED takeoff -> AUTO waypoint range -> LAND at final waypoint."""
import argparse
import math
import re
import time
from copy import deepcopy


def strategy(value):
    value = value.upper()
    if value in ('R', 'RIGHT'): return 'RIGHT'
    if value in ('L', 'LEFT'): return 'LEFT'
    raise argparse.ArgumentTypeError('use r/right or l/left')


def waypoint_range(value):
    m = re.fullmatch(r'(\d+)(?:-(\d+))?', value)
    if not m: raise argparse.ArgumentTypeError('use N or START-END, e.g. 2-2')
    a, b = int(m[1]), int(m[2] or m[1])
    if not 1 <= a <= b <= 12: raise argparse.ArgumentTypeError('range must ascend within 1..12')
    return list(range(a, b + 1))


def distance_m(lat1, lon1, lat2, lon2):
    p, q = math.radians(lat1), math.radians(lat2)
    h = math.sin((q-p)/2)**2 + math.cos(p)*math.cos(q)*math.sin(math.radians(lon2-lon1)/2)**2
    return 6371000 * 2 * math.asin(math.sqrt(min(1, max(0, h))))


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def plan(data, strat, ids, start, max_leg=100.0):
    result = []
    previous = start
    for n in ids:
        wp = deepcopy(data[strat][f'WP{n}'])
        lat, lon, alt = wp.get('lat_deg'), wp.get('lon_deg'), wp.get('alt_m')
        if (wp.get('frame') != 'GLOBAL_RELATIVE_ALT' or
                not all(finite(x) for x in (lat, lon, alt)) or
                not -90 <= lat <= 90 or not -180 <= lon <= 180 or not .5 <= alt <= 3):
            raise ValueError(f'WP{n}: requires valid global coordinates and altitude 0.5..3m; save again from updated Pi/GCS')
        leg = distance_m(*previous, lat, lon)
        if leg > max_leg: raise ValueError(f'WP{n}: leg {leg:.1f}m exceeds {max_leg:.1f}m')
        wp.update(number=n, leg_m=leg)
        result.append(wp)
        previous = lat, lon
    return result


class FlightTest:
    def __init__(self, runtime, args):
        self.r, self.args = runtime, args
        self.v = runtime.state.vehicle
        self.last_status = 0

    def navigation_ok(self, airborne=False):
        v = self.v
        if self.r.stop_event.is_set() or self.r.state.system.get('wp_test_abort'):
            return False, 'operator abort/runtime stopped'
        if not v.connected or v.heartbeat_age_s() > 2.5: return False, 'FC heartbeat lost/stale'
        if airborne and (not v.armed or v.mode not in ('GUIDED', 'AUTO', 'LAND')): return False, 'disarmed or pilot/FC mode override'
        if v.global_position_age_s() > .5 or v.message_age_s('GLOBAL_POSITION_INT') > .5:
            return False, 'global position stale'
        if v.message_age_s('GPS_RAW_INT') > 1.5 or (v.gps_fix_type or 0) < 3:
            return False, 'GPS fix unavailable/stale'
        if not all(finite(x) for x in (v.global_lat_deg, v.global_lon_deg, v.relative_alt_m)):
            return False, 'invalid position'
        return True, ''

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        while time.monotonic() < deadline:
            ok, reason = self.navigation_ok()
            if ok: return
            if self.r.state.system.get('wp_test_abort'): raise RuntimeError(reason)
            time.sleep(.1)
        raise RuntimeError('preflight: ' + reason)

    def auto_and_land(self, route, transfer):
        from flight.vct import VerifiedCommandTransaction
        v=self.v
        since=time.monotonic_ns()
        result=VerifiedCommandTransaction('MISSION_CURRENT_1',
            lambda: (transfer.send('mission_set_current_send',*transfer.ids(),1) or True),
            lambda: v.current_mission_item==1 and v.message_last_ns.get('MISSION_CURRENT',0)>since,
            precheck_fn=lambda:self.navigation_ok(True), abort_fn=lambda:self.navigation_ok(True),
            allow_already_satisfied=False, attempts=1,state_timeout_s=5,trace_fn=trace).run()
        if not result.ok: raise RuntimeError('mission start index unconfirmed')
        transfer.clear()
        def transition_ok():
            ok, reason=self.navigation_ok(True)
            if ok and v.mode not in ('GUIDED','AUTO'): return False,'mode overridden before AUTO'
            return ok,reason
        auto_since=time.monotonic_ns()
        result=VerifiedCommandTransaction('AUTO',lambda:self.r.tx.send_mode('AUTO'),
            lambda:v.mode=='AUTO' and v.last_heartbeat_ns>auto_since,
            precheck_fn=transition_ok,abort_fn=transition_ok,allow_already_satisfied=False,
            attempts=1,state_timeout_s=5,trace_fn=trace).run()
        if not result.ok: raise RuntimeError('AUTO unconfirmed: '+result.reason)
        required=set(range(2,2+len(route)))
        reached=set()
        deadline=time.monotonic()+self.args.goto_timeout
        landing_deadline=None
        while True:
            ok,reason=self.navigation_ok()
            if not ok: raise RuntimeError(reason)
            while True:
                try: msg=transfer.q.get_nowait()
                except __import__('queue').Empty: break
                if msg.get_type()=='MISSION_ITEM_REACHED':
                    seq=int(msg.seq)
                    if seq in required:
                        if seq not in reached:
                            print(f"[REACHED] WP{route[seq-2]['number']} (FC mission seq {seq})",flush=True)
                        reached.add(seq)
            if v.mode not in ('AUTO','LAND'): raise RuntimeError('pilot/FC mode override; mission monitoring stopped')
            if v.mode=='LAND' and reached!=required: raise RuntimeError('early LAND / failsafe; preserve FC mode')
            if not v.armed:
                if (reached==required and getattr(v,'landed_state',None)==1 and
                        v.message_age_s('EXTENDED_SYS_STATE')<=1 and v.last_heartbeat_ns>since):
                    print('[DONE] all waypoints reached + FC landed + disarmed verified',flush=True)
                    return
                raise RuntimeError('disarmed before mission completion could be verified')
            if reached==required and landing_deadline is None:
                landing_deadline=time.monotonic()+self.args.land_timeout
            if time.monotonic()>(landing_deadline if landing_deadline else deadline):
                raise RuntimeError('AUTO/landing timeout; FC mission may remain active')
            time.sleep(.05)

    def execute(self, route):
        self.r.state.system['wp_test_active'] = True
        self.r.waypoints.lock()
        if self.v.armed: raise RuntimeError('start disarmed on the ground')
        if (getattr(self.v, 'landed_state', None)!=1 or self.v.message_age_s('EXTENDED_SYS_STATE')>1):
            raise RuntimeError('fresh FC landed state ON_GROUND required')
        ok, reason=self.navigation_ok()
        if not ok: raise RuntimeError(reason)
        from flight.auto_test_mission import MissionTransfer, build_mission
        def upload_allowed():
            if self.v.armed: return False, 'armed during mission preparation'
            return self.navigation_ok()
        transfer=MissionTransfer(self.r,upload_allowed)
        backup_path=__import__('pathlib').Path(__file__).resolve().parent/'logs'/f'fc-mission-before-wptest-{time.time_ns()}.json'
        transfer.backup(backup_path)
        items=build_mission(route,(self.v.global_lat_deg,self.v.global_lon_deg),self.args.speed)
        transfer.upload(items)
        transfer.verify_uploaded(items)
        print('[MISSION] uploaded and read back; GUIDED takeoff next',flush=True)
        # Wrap the critical sequence so GPS loss / GCS abort also prevents TAKEOFF.
        original=self.r.command._link_precheck
        def guard(allow_land=False):
            ok, reason=self.navigation_ok()
            return original(allow_land=allow_land) if ok else (ok, reason)
        self.r.command._link_precheck=guard
        try:
            result=self.r.command.takeoff(route[0]['alt_m'], state_timeout_s=20, trace_fn=trace)
        finally:
            self.r.command._link_precheck=original
        if not result.ok: raise RuntimeError(f'TAKEOFF {result.outcome}: {result.reason}')
        self.auto_and_land(route,transfer)


def trace(event, data):
    print('[WPTEST]', event, data, flush=True)


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--strat', type=strategy, required=True)
    p.add_argument('--goto', '-goto', type=waypoint_range, required=True)
    p.add_argument('--execute', action='store_true', help='Enable real flight; otherwise preview only')
    p.add_argument('--confirm-flight-area-clear', action='store_true')
    p.add_argument('--speed', type=float, default=1.0)
    p.add_argument('--max-leg', type=float, default=100.0)
    p.add_argument('--ready-timeout', type=float, default=30)
    p.add_argument('--goto-timeout', type=float, default=120)
    p.add_argument('--land-timeout', type=float, default=60)
    return p


def main():
    p=parser(); args=p.parse_args()
    for name,lo,hi in [('speed',.2,3),('max_leg',1,200),
                       ('ready_timeout',1,120),('goto_timeout',5,300),('land_timeout',5,180)]:
        value=getattr(args,name)
        if not finite(value) or not lo<=value<=hi: p.error(f'{name} must be {lo}..{hi}')
    if args.execute and not args.confirm_flight_area_clear: p.error('--execute requires --confirm-flight-area-clear')
    # Import only after parsing: --help works without Pi dependencies.
    import main_v2_3_2  # install canonical telemetry patches
    from core.runtime import KingphoenixRuntime
    runtime=KingphoenixRuntime()
    runtime.state.system['wp_test_owner']=True
    runtime.state.system['wp_test_abort']=False
    runtime.gcs.command=None  # read/waypoint capture remain available; no bench flight commands
    runtime.mp_gateway.policy='RACE'
    test=FlightTest(runtime,args)
    try:
        runtime.start()
        test.wait_ready()
        runtime.tx.set_message_interval(245,5)
        deadline=time.monotonic()+5
        while getattr(test.v,'landed_state',None) is None and time.monotonic()<deadline:
            time.sleep(.1)
        route=plan(runtime.waypoints.data,args.strat,args.goto,
                   (test.v.global_lat_deg,test.v.global_lon_deg),args.max_leg)
        print('[PLAN]', args.strat, route, flush=True)
        if not args.execute:
            print('[PREVIEW ONLY] no ARM, TAKEOFF, GOTO or LAND sent')
            return 0
        test.execute(route)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'[STOPPED / NOT DONE] {exc}', flush=True)
        v=runtime.state.vehicle
        if runtime.state.system.get('wp_test_active') and v.armed and v.mode in ('GUIDED','AUTO') and v.connected and v.heartbeat_age_s()<=2.5:
            # Cancel further route execution by entering BRAKE; preserve any RC/FC override.
            try:
                from flight.vct import VerifiedCommandTransaction
                since=time.monotonic_ns()
                def brake_allowed():
                    return (v.connected and v.heartbeat_age_s()<=2.5 and v.armed and
                            v.mode in ('GUIDED','AUTO','BRAKE'), 'link lost or mode overridden')
                braking=VerifiedCommandTransaction('ABORT_BRAKE',lambda:runtime.tx.send_mode('BRAKE'),
                    lambda:v.mode=='BRAKE' and v.last_heartbeat_ns>since,
                    precheck_fn=brake_allowed,abort_fn=brake_allowed,allow_already_satisfied=False,
                    attempts=1,state_timeout_s=3,trace_fn=trace).run()
                print('[ABORT BRAKE]',braking.ok,braking.reason,flush=True)
            except Exception as brake_error: print('[BRAKE UNCONFIRMED]',brake_error,flush=True)
        print('[RC TAKEOVER] no automatic disarm; FC action may still be active if BRAKE unconfirmed',flush=True)
        return 2
    finally:
        runtime.stop()


if __name__=='__main__': raise SystemExit(main())
