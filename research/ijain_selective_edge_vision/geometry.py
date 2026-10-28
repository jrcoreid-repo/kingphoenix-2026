"""Optional offline gate geometry helpers; never transmits aircraft commands."""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Pose:
    valid: bool
    translation_m: tuple | None = None
    reprojection_rmse_px: float | None = None
    ambiguity_resolved: bool = False
    reason: str = ""

def mean_diagonal(corners):
    if len(corners) != 4:
        raise ValueError("four physical corners required (TL, TR, BR, BL)")
    return (math.dist(corners[0],corners[2])+math.dist(corners[1],corners[3]))/2

def normalized_residual(observed,predicted):
    if len(observed)!=4 or len(predicted)!=4:
        raise ValueError("four ordered correspondences required")
    diagonal=mean_diagonal(observed)
    if diagonal<=1e-9: raise ValueError("degenerate gate quadrilateral")
    return math.sqrt(sum(math.dist(a,b)**2 for a,b in zip(observed,predicted))/4)/diagonal

def causal_corner_prediction(prev2,t2,prev1,t1,now):
    if len(prev2)!=4 or len(prev1)!=4 or not t2<t1<=now:
        raise ValueError("two chronological corner observations required")
    ratio=(now-t1)/(t1-t2)
    return [(b[0]+ratio*(b[0]-a[0]),b[1]+ratio*(b[1]-a[1])) for a,b in zip(prev2,prev1)]

def find_gate_quad(frame,roi_xyxy,margin=0.15):
    """Find a physical quadrilateral candidate inside expanded detector ROI.
    Box corners are never used as physical corner correspondences.
    """
    import cv2
    import numpy as np
    h,w=frame.shape[:2]
    x1,y1,x2,y2=map(float,roi_xyxy)
    dx,dy=max(1.,x2-x1),max(1.,y2-y1)
    xa,xb=max(0,int(x1-margin*dx)),min(w,int(x2+margin*dx))
    ya,yb=max(0,int(y1-margin*dy)),min(h,int(y2+margin*dy))
    if xa>=xb or ya>=yb: return None
    roi=cv2.GaussianBlur(cv2.cvtColor(frame[ya:yb,xa:xb],cv2.COLOR_BGR2GRAY),(5,5),0)
    edges=cv2.Canny(roi,60,150)
    contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    candidates=[]
    for c in contours:
        area=cv2.contourArea(c)
        if area<0.005*(xb-xa)*(yb-ya): continue
        q=cv2.approxPolyDP(c,0.025*cv2.arcLength(c,True),True)
        if len(q)==4 and cv2.isContourConvex(q): candidates.append((area,q.reshape(4,2)))
    if not candidates: return None
    pts=np.asarray(max(candidates,key=lambda pair:pair[0])[1],float)
    by_y=pts[np.argsort(pts[:,1])]
    top=by_y[:2][np.argsort(by_y[:2,0])]
    bottom=by_y[2:][np.argsort(by_y[2:,0])]
    return (np.array([top[0],top[1],bottom[1],bottom[0]])+[xa,ya]).tolist()

def estimate_planar_pose(corners,camera_matrix,distortion,gate_width_m,gate_height_m,ambiguity_gap_px=0.5):
    """IPPE with positive depth/ambiguity checks; genuine camera calibration required."""
    import cv2
    import numpy as np
    pts=np.asarray(corners,np.float64)
    if pts.shape!=(4,2) or not np.isfinite(pts).all(): return Pose(False,reason="invalid_corners")
    if gate_width_m<=0 or gate_height_m<=0: return Pose(False,reason="invalid_dimensions")
    K=np.asarray(camera_matrix,np.float64).reshape(3,3)
    D=np.asarray(distortion,np.float64)
    if not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0: return Pose(False,reason="uncalibrated_camera")
    a,b=gate_width_m/2,gate_height_m/2
    obj=np.array([[-a,-b,0],[a,-b,0],[a,b,0],[-a,b,0]],np.float64)
    ok,rvecs,tvecs,_=cv2.solvePnPGeneric(obj,pts,K,D,flags=cv2.SOLVEPNP_IPPE)
    if not ok: return Pose(False,reason="pnp_failed")
    solutions=[]
    for r,t in zip(rvecs,tvecs):
        if float(t[2,0])<=0: continue
        projected,_=cv2.projectPoints(obj,r,t,K,D)
        err=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-pts)**2,axis=1))))
        solutions.append((err,t.reshape(3)))
    if not solutions: return Pose(False,reason="non_positive_depth")
    solutions.sort(key=lambda pair:pair[0])
    if len(solutions)>1 and solutions[1][0]-solutions[0][0]<ambiguity_gap_px:
        return Pose(False,reprojection_rmse_px=solutions[0][0],reason="ambiguous_planar_pose")
    return Pose(True,tuple(float(v) for v in solutions[0][1]),solutions[0][0],True,"ok")
