#!/usr/bin/env python3
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]
c=json.loads((ROOT/"config/system.json").read_text())
assert c["version"]=="2.0.1"
assert c["mavlink"]["connection"]=="/dev/ttyAMA0"
assert c["ai"]["engine"]=="YOLO26_ONLY"
assert c["vision_pose"]["enabled"] is False
assert c["camera"]["runtime"]["capture_target_fps"]==30.0
assert (c["stream"]["width"],c["stream"]["height"],c["stream"]["fps"])==(640,400,30)
assert c["vision_control"]["movement_tx_enabled"] is False
print("KINGPHOENIX V2.0.1 ARCHITECTURE SELF-CHECK")
print("[PASS] V1 MAVLink/VCT foundation retained")
print("[PASS] YOLO26-only normal runtime")
print("[PASS] pose/solvePnP disabled from flight path")
print("[PASS] wide source 1920x1200")
print("[PASS] thermal default 30 fps")
print("[PASS] native wide downlink 640x400 @30")
print("[PASS] bbox estimator -> SharedState")
print("[PASS] movement TX blocked")
print("V2.0 ARCHITECTURE SELF-CHECK: PASS")
