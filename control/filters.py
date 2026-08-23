class LowPass:
    def __init__(self,alpha=.25): self.alpha=float(alpha); self.value=None
    def update(self,x): self.value=x if self.value is None else self.alpha*x+(1-self.alpha)*self.value; return self.value
