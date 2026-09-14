"""Mission transfer over the runtime's single RX worker; no competing recv_match."""
import json
import math
import queue
import time
from pathlib import Path


def build_mission(route, start, speed):
    def item(seq, cmd, lat=0, lon=0, alt=0, params=(0,0,0,0), frame=6):
        return dict(seq=seq, command=cmd, frame=frame, current=0, autocontinue=1,
                    param1=params[0], param2=params[1], param3=params[2], param4=params[3],
                    x=round(lat*1e7), y=round(lon*1e7), z=alt)
    # seq 0 is protocol home placeholder, never executed. No DO_SET_HOME command.
    items=[item(0,16,*start,0)]
    items.append(item(1,178,params=(1,speed,-1,0),frame=2))
    for wp in route:
        items.append(item(len(items),16,wp['lat_deg'],wp['lon_deg'],wp['alt_m'],params=(1,1,0,0)))
    last=route[-1]
    items.append(item(len(items),21,last['lat_deg'],last['lon_deg'],0))
    return items


class MissionTransfer:
    def __init__(self, runtime, check, timeout=10):
        self.r, self.check, self.timeout=runtime, check, timeout
        self.q=queue.Queue(maxsize=1024)
        runtime.rx.mission_transfer_queue=self.q

    def send(self, name, *args):
        master=self.r.tx.with_master()
        if master is None: raise RuntimeError('FC link unavailable')
        with self.r.tx.lock: getattr(master.mav,name)(*args)

    def ids(self): return self.r.tx.target_system,self.r.tx.target_component

    def clear(self):
        while True:
            try: self.q.get_nowait()
            except queue.Empty: return

    def receive(self, kinds, deadline):
        while time.monotonic()<deadline:
            ok, reason=self.check()
            if not ok: raise RuntimeError(reason)
            try: msg=self.q.get(timeout=.1)
            except queue.Empty: continue
            if getattr(msg,'mission_type',0)!=0: continue
            if msg.get_type() in kinds: return msg
        raise RuntimeError('mission protocol timeout: '+str(kinds))

    def backup(self, path):
        self.clear()
        self.send('mission_request_list_send',*self.ids())
        count=int(self.receive({'MISSION_COUNT'},time.monotonic()+self.timeout).count)
        if not 0<=count<=500: raise RuntimeError('unexpected existing mission size')
        old=[]
        for seq in range(count):
            self.send('mission_request_int_send',*self.ids(),seq)
            deadline=time.monotonic()+self.timeout
            while True:
                msg=self.receive({'MISSION_ITEM_INT','MISSION_ITEM'},deadline)
                if int(msg.seq)==seq:
                    old.append(msg.to_dict()); break
        self.send('mission_ack_send',*self.ids(),0)
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(dict(count=count,items=old),indent=2))
        print('[MISSION BACKUP]',path,flush=True)

    def upload(self, items):
        self.clear()
        self.send('mission_count_send',*self.ids(),len(items))
        sent=set();deadline=time.monotonic()+max(30,len(items)*3)
        while True:
            msg=self.receive({'MISSION_REQUEST_INT','MISSION_REQUEST','MISSION_ACK'},deadline)
            if msg.get_type()=='MISSION_ACK':
                if int(msg.type)!=0: raise RuntimeError(f'mission upload rejected: {msg.type}')
                if len(sent)!=len(items): raise RuntimeError('premature mission ACK')
                return
            seq=int(msg.seq)
            if not 0<=seq<len(items): raise RuntimeError('invalid mission request index')
            i=items[seq];integer=msg.get_type()=='MISSION_REQUEST_INT'
            x,y=i['x'],i['y'];frame=i['frame']
            if not integer and frame==6: x,y,frame=x/1e7,y/1e7,3
            self.send('mission_item_int_send' if integer else 'mission_item_send',
                      *self.ids(),seq,frame,i['command'],i['current'],i['autocontinue'],
                      i['param1'],i['param2'],i['param3'],i['param4'],x,y,i['z'])
            sent.add(seq)

    def verify_uploaded(self, items):
        # Read back every item before arming; ACK alone is not sufficient.
        self.clear();self.send('mission_request_list_send',*self.ids())
        count=self.receive({'MISSION_COUNT'},time.monotonic()+self.timeout).count
        if count!=len(items): raise RuntimeError('mission readback count mismatch')
        for i in items:
            self.send('mission_request_int_send',*self.ids(),i['seq'])
            deadline=time.monotonic()+self.timeout
            while True:
                m=self.receive({'MISSION_ITEM_INT'},deadline)
                if m.seq==i['seq']: break
            # FC may replace seq 0 with its current home.
            if i['seq']==0: continue
            if any(not math.isfinite(float(getattr(m,k))) for k in ('x','y','z','param1','param2','param3','param4')):
                raise RuntimeError('nonfinite mission readback')
            if int(m.command)!=i['command']: raise RuntimeError('mission readback command mismatch')
            if i['command'] in (16,21):
                if int(m.frame) not in (3,6) or abs(int(m.x)-i['x'])>1 or abs(int(m.y)-i['y'])>1 or abs(float(m.z)-i['z'])>.01:
                    raise RuntimeError('mission readback coordinate/altitude mismatch')
            if any(abs(float(getattr(m,k))-i[k])>.01 for k in ('param1','param2','param3','param4')):
                raise RuntimeError('mission readback parameter mismatch')
        self.send('mission_ack_send',*self.ids(),0)
