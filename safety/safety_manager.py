import threading
from core.scheduler import Rate
from safety.abort_handler import AbortHandler
class SafetyManager(threading.Thread):
    daemon=True
    def __init__(self,state,events,stop_event): super().__init__(name='SafetyManager'); self.state=state; self.events=events; self.stop_event=stop_event; self.abort=AbortHandler(state); self.land_latched=False
    def run(self):
        r=Rate(50)
        while not self.stop_event.is_set():
            mode=self.state.vehicle.mode
            if mode=='LAND' and not self.land_latched:
                self.land_latched=True; self.events.publish('GLOBAL_ABORT',{'reason':'RC_LAND'}); self.abort.reset_to_standby('RC_LAND'); print('[SAFETY] RC LAND -> GLOBAL ABORT / STANDBY')
            elif mode!='LAND': self.land_latched=False
            r.sleep()
