#!/usr/bin/env python3
from pathlib import Path
import json

ROOT=Path(__file__).resolve().parents[1]
cfg=json.loads((ROOT/"config/system.json").read_text())
cam=cfg["camera"]

assert cam["backend"]=="picamera2"
assert cam["bottom"]["camera_num"]==0
assert cam["front"]["camera_num"]==1

for name in ("front","bottom"):
    sensor=cam[name]["sensor_mode"]
    output=cam[name]["output"]
    assert sensor["width"]==1920 and sensor["height"]==1200
    assert float(sensor["fps"])==60.0
    assert output["width"]==1920 and output["height"]==1200
    assert output["format"]=="RGB888"

print("KINGPHOENIX V1.9.4 CAMERA CONFIG SELF-CHECK")
print("[PASS] camera_num 0 -> BOTTOM")
print("[PASS] camera_num 1 -> FRONT")
print("[PASS] sensor target 1920x1200 @ 60fps")
print("[PASS] PiSP main output 1920x1200 RGB888 full sensor")
print("CAMERA CONFIG SELF-CHECK: PASS")
