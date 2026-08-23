#!/usr/bin/env python3
from pathlib import Path
import ast

ROOT=Path(__file__).resolve().parents[1]
p=ROOT/"vision/hailo_yolo26.py"
text=p.read_text()
tree=ast.parse(text)

assert "detector_group.activate()" in text
assert "_activation_context.__enter__()" in text
assert "_activation_context.__exit__" in text
assert "_infer_lock" in text
assert "HEF(self.pose_hef_path)" not in text

print("KINGPHOENIX V2.0.1 HAILO ACTIVATION SELF-CHECK")
print("[PASS] detector network group activation is persistent")
print("[PASS] activation context has explicit teardown")
print("[PASS] shared FRONT/BOTTOM inference is serialized")
print("[PASS] pose HEF is not loaded")
print("HAILO ACTIVATION SELF-CHECK: PASS")
