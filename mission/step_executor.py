class StepExecutor:
    def __init__(self,command,waypoints,state): self.command=command; self.waypoints=waypoints; self.state=state
    def execute(self,step,strategy):
        k=step['type']
        if k=='TAKEOFF':return self.command.takeoff(step['altitude_m'])
        if k=='LAND':return self.command.land()
        if k=='WAYPOINT':return {'ok':True,'intent':'AUTO_WAYPOINT','waypoint':self.waypoints.get(strategy,step['wp'])}
        if k=='VISION_GATE':return {'ok':True,'intent':'GUIDED_VISION_GATE','step':step}
        if k=='DROP_ALIGNMENT':return {'ok':True,'intent':'GUIDED_DROP_ALIGNMENT','step':step}
        if k=='DROP_LOGISTICS':return {'ok':True,'intent':'DROP_LOGISTICS'}
        return {'ok':False,'reason':'unknown step'}
