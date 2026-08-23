from config import load_json,save_json_atomic
class WaypointManager:
    def __init__(self,state): self.state=state; self.data=load_json('waypoints.json'); self.edit_unlocked=False
    def unlock(self): self.edit_unlocked=True
    def lock(self): self.edit_unlocked=False
    def get(self,strategy,wp): return self.data[strategy.upper()][f'WP{int(wp)}']
    def save_current(self,strategy,wp):
        if not self.edit_unlocked:return False,'EDIT MODE LOCKED'
        v=self.state.vehicle
        if None in (v.x,v.y,v.z):return False,'LOCAL_POSITION_NED unavailable'
        self.data[strategy.upper()][f'WP{int(wp)}']={'x':v.x,'y':v.y,'z':v.z}; save_json_atomic('waypoints.json',self.data)
        return True,f'WP{wp} {strategy.upper()} saved'
