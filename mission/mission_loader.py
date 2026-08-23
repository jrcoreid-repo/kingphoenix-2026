from config import load_json
class MissionLoader:
    def __init__(self): self.data=load_json('mission.json')
    def steps(self): return list(self.data['sequence'])
    def validate(self):
        ids=[s['id'] for s in self.steps()]; return ids==list(range(1,len(ids)+1))
