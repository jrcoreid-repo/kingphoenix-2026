class Movement:
    def __init__(self,tx): self.tx=tx
    def velocity(self,vx,vy,vz,yaw_rate=0): return self.tx.send_guided_velocity(vx,vy,vz,yaw_rate)
    def hold(self): return self.velocity(0,0,0,0)
    def brake(self): return self.hold()
