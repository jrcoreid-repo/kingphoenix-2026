"""Offline common-trace replay: no HEF calls, no MAVLink, no UAV actuation."""
import argparse
import csv
import json
import math
from pathlib import Path
from .scheduler import Scheduler, Observation, POLICIES
from .geometry import causal_corner_prediction, normalized_residual

def f(row,key):
    raw=row.get(key,"")
    if raw is None or str(raw).strip()=="": return None
    value=float(raw)
    return value if math.isfinite(value) else None

def yes(value): return str(value).strip().lower() in ("1","true","yes","y")

def corners(row):
    vals=[f(row,axis+str(i)) for i in range(1,5) for axis in ("x","y")]
    return None if any(v is None for v in vals) else [vals[j:j+2] for j in range(0,8,2)]

def evaluate_trace(rows,policy="full_reliability",inference_ms=43.0):
    scheduler=Scheduler(policy,inference_s=inference_ms/1000)
    events,history=[],[]
    for idx,row in enumerate(rows):
        t=f(row,"capture_s")
        if t is None: raise ValueError("capture_s missing in row "+str(idx+2))
        active=yes(row.get("mission_active","0"))
        pts=corners(row)
        valid=active and pts is not None
        temp=None
        if valid and len(history)>=2:
            (t2,p2),(t1,p1)=history[-2:]
            if t2<t1<=t:
                temp=normalized_residual(pts,causal_corner_prediction(p2,t2,p1,t1,t))
        age=f(row,"feedback_age_s")
        o=Observation(int(row.get("frame_id") or idx),t,active,valid,
                      f(row,"reprojection_rmse_px"),f(row,"geo_normalized"),temp,
                      yes(row.get("positive_depth","0")),yes(row.get("ambiguity_resolved","0")),
                      age if age is not None else float("inf"),f(row,"detector_confidence"))
        d=scheduler.step(o)
        if d.accepted and pts is not None: history.append((t,pts))
        vx,vy,rx,ry=(f(row,k) for k in ("visual_x_m","visual_y_m","rtk_x_m","rtk_y_m"))
        pair=d.accepted and yes(row.get("rtk_valid","0")) and all(x is not None for x in (vx,vy,rx,ry))
        error=math.hypot(vx-rx,vy-ry) if pair else None
        events.append(dict(policy=policy,frame_id=o.frame_id,capture_s=t,mission_active=int(active),
                           refresh_requested=int(d.refresh_requested),inference_executed=int(d.inference_executed),
                           coalesced=int(d.coalesced),measurement_accepted=int(d.accepted),
                           detector_age_s=d.detector_age_s,trigger_reason=d.reason,
                           temporal_normalized=temp,rtk_paired=int(pair),localization_error_m=error))
    n=sum(e["mission_active"] for e in events)
    executed=sum(e["inference_executed"] for e in events)
    accepted=sum(e["measurement_accepted"] for e in events)
    errors=[e["localization_error_m"] for e in events if e["rtk_paired"]]
    summary=dict(policy=policy,offered_frames=n,detector_requests=sum(e["refresh_requested"] for e in events),
                 detector_executions=executed,coalesced_requests=sum(e["coalesced"] for e in events),
                 detector_utilization_pct=round(100*executed/n,3) if n else None,
                 accepted_measurements=accepted,measurement_coverage_pct=round(100*accepted/n,3) if n else None,
                 valid_rtk_pairs=len(errors),
                 localization_rmse_m=round(math.sqrt(sum(e*e for e in errors)/len(errors)),5) if errors else None,
                 high_error_acceptance_pct=round(100*sum(e>.20 for e in errors)/len(errors),3) if errors else None,
                 limitation="Trace-level model only; not physical detector execution or validated flight logs")
    return events,summary

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",required=True)
    parser.add_argument("--out-dir",required=True)
    parser.add_argument("--policy",choices=(*POLICIES,"all"),default="all")
    parser.add_argument("--inference-ms",type=float,default=43.0,
                        help="Assumed detector busy window; use measured timing when available")
    args=parser.parse_args(argv)
    with open(args.input,newline="",encoding="utf-8") as fh:
        rows=list(csv.DictReader(fh))
    destination=Path(args.out_dir)
    destination.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for policy in (POLICIES if args.policy=="all" else (args.policy,)):
        events,summary=evaluate_trace(rows,policy,args.inference_ms)
        if events:
            with (destination/("events_"+policy+".csv")).open("w",newline="",encoding="utf-8") as fh:
                writer=csv.DictWriter(fh,fieldnames=list(events[0]))
                writer.writeheader()
                writer.writerows(events)
        summaries.append(summary)
    (destination/"summary.json").write_text(json.dumps(summaries,indent=2),encoding="utf-8")
    print(json.dumps(summaries,indent=2))

if __name__=="__main__": main()
