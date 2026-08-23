import threading
from core.scheduler import Rate
class MissionManager(threading.Thread):
    daemon=True
    def __init__(self,state,events,command,loader,waypoints,stop_event): super().__init__(name='MissionManager'); self.state=state; self.events=events; self.command=command; self.loader=loader; self.waypoints=waypoints; self.stop_event=stop_event
    def run(self):
        r=Rate(50)
        while not self.stop_event.is_set(): r.sleep()
