from config import load_json
class ControlProfiles:
    def __init__(self): self.data=load_json('control_params.json')
    def get(self,name): return dict(self.data[name])
