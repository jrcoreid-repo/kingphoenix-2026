from __future__ import annotations
from dataclasses import dataclass

def clamp(v,lo,hi): return max(lo,min(hi,float(v)))
def db(v,d):
    v=float(v); d=abs(float(d))
    if abs(v)<=d: return 0.0
    s=1.0 if v>=0 else -1.0
    return s*(abs(v)-d)/max(1.0-d,1e-6)

@dataclass
class BboxControlResult:
    valid: bool
    ex: float|None=None
    ey: float|None=None
    bbox_area_ratio: float|None=None
    bbox_width_ratio: float|None=None
    bbox_height_ratio: float|None=None
    phase: str="SEARCH"
    intent: dict|None=None
    def as_dict(self):
        return self.__dict__.copy()

class BboxControlEstimator:
    """Lightweight bbox estimator. Never transmits MAVLink."""
    def __init__(self,camera,cfg,w=1920,h=1200):
        self.camera=str(camera).lower(); self.cfg=cfg or {}; self.role=self.cfg.get(self.camera,{})
        self.w=float(w); self.h=float(h)
        self.dbx=float(self.cfg.get("deadband_x",.035)); self.dby=float(self.cfg.get("deadband_y",.035))
        self.conf=float(self.cfg.get("confidence_min",.25))
    def estimate(self,bbox,confidence=None):
        if bbox is None or len(bbox)<4: return BboxControlResult(False,phase="SEARCH")
        if confidence is not None and float(confidence)<self.conf: return BboxControlResult(False,phase="LOW_CONF")
        x1,y1,x2,y2=map(float,bbox[:4]); bw=max(0.,x2-x1); bh=max(0.,y2-y1)
        if bw<=1 or bh<=1: return BboxControlResult(False,phase="INVALID_BBOX")
        cx=(x1+x2)/2; cy=(y1+y2)/2
        ex=clamp((cx-self.w/2)/(self.w/2),-1,1); ey=clamp((cy-self.h/2)/(self.h/2),-1,1)
        exd=db(ex,self.dbx); eyd=db(ey,self.dby)
        wr=bw/self.w; hr=bh/self.h; area=bw*bh/(self.w*self.h)
        return self.front(ex,ey,exd,wr,hr,area) if self.camera=="front" else self.bottom(ex,ey,exd,eyd,wr,hr,area)
    def front(self,ex,ey,exd,wr,hr,area):
        r=self.role
        ml=float(r.get("max_lateral_mps",1.5)); my=float(r.get("max_yaw_rate_dps",25))
        lat=clamp(float(r.get("kp_lateral",1.2))*exd,-ml,ml)
        yaw=clamp(float(r.get("kp_yaw_rate",20))*exd,-my,my)
        vmax=float(r.get("forward_max_mps",5)); vmin=float(r.get("forward_min_mps",1.2))
        a0=float(r.get("slow_area_ratio",.08)); a1=float(r.get("commit_area_ratio",.22))
        if area<=a0: fwd=vmax
        elif area>=a1: fwd=vmin
        else:
            t=(area-a0)/max(a1-a0,1e-6); fwd=vmax+(vmin-vmax)*t
        aligned=abs(ex)<=float(r.get("align_error_max",.12))
        phase="COMMIT" if area>=a1 and aligned else ("APPROACH" if aligned else "ALIGN")
        return BboxControlResult(True,round(ex,5),round(ey,5),round(area,6),round(wr,6),round(hr,6),phase,
            {"forward_mps":round(fwd,4),"lateral_mps":round(lat,4),"yaw_rate_dps":round(yaw,4),
             "vertical_source":"RANGEFINDER","tx_enabled":False})
    def bottom(self,ex,ey,exd,eyd,wr,hr,area):
        r=self.role; kp=float(r.get("kp_xy",1)); m=float(r.get("max_xy_mps",1))
        right=clamp(kp*exd,-m,m); forward=clamp(-kp*eyd,-m,m)
        centered=abs(ex)<=float(r.get("center_error_max",.05)) and abs(ey)<=float(r.get("center_error_max",.05))
        return BboxControlResult(True,round(ex,5),round(ey,5),round(area,6),round(wr,6),round(hr,6),
            "CENTERED" if centered else "ALIGN",
            {"forward_mps":round(forward,4),"right_mps":round(right,4),
             "vertical_source":"RANGEFINDER","tx_enabled":False})
