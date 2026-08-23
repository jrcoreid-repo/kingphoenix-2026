import time
class Rate:
    def __init__(self,hz): self.p=1.0/float(hz); self.n=time.monotonic()
    def sleep(self):
        self.n+=self.p; d=self.n-time.monotonic()
        if d>0: time.sleep(d)
        else:self.n=time.monotonic()
