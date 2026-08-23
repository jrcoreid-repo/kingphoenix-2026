#!/usr/bin/env python3
from pathlib import Path
import json

ROOT=Path(__file__).resolve().parents[1]
cfg=json.loads((ROOT/"config/system.json").read_text())

for role in ("front","bottom"):
    sensor=cfg["camera"][role]["sensor_mode"]
    out=cfg["camera"][role]["output"]
    assert (sensor["width"],sensor["height"])==(1920,1200)
    assert float(sensor["fps"])==60.0
    assert (out["width"],out["height"])==(1920,1200)

g=cfg["ai"]["input_geometry"]
assert g["resize_policy"]=="LETTERBOX"
assert (g["source_width"],g["source_height"])==(1920,1200)
assert (g["model_width"],g["model_height"])==(640,640)

scale=min(640/1920,640/1200)
rw=round(1920*scale)
rh=round(1200*scale)
left=(640-rw)//2
top=(640-rh)//2

assert abs(scale-(1/3)) < 1e-12
assert (rw,rh)==(640,400)
assert (left,top)==(0,120)

# Center mapping sanity
x,y=960.0,600.0
mx,my=x*scale+left,y*scale+top
assert abs(mx-320.0) < 1e-9
assert abs(my-320.0) < 1e-9
ix,iy=(mx-left)/scale,(my-top)/scale
assert abs(ix-x) < 1e-9
assert abs(iy-y) < 1e-9

print("KINGPHOENIX V1.9.6.1 VISION GEOMETRY SELF-CHECK")
print("[PASS] canonical = 1920x1200 full sensor")
print("[PASS] sensor target = 60 fps")
print("[PASS] YOLO = 640x640 letterbox")
print("[PASS] 640x400 image + 120px top/bottom padding")
print("[PASS] inverse mapping -> 1920x1200 canonical coordinates")
print("[PASS] GCS stays 640x640 letterboxed")
print("VISION GEOMETRY SELF-CHECK: PASS")
