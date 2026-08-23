from dataclasses import dataclass
import time
@dataclass
class MissionState:
    status:str='STANDBY'; step_index:int=0; strategy:str='LEFT'; active_ai_camera:str='none'; abort_reason:str=''
def now_ns(): return time.monotonic_ns()
