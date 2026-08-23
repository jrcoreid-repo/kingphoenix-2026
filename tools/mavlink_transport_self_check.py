#!/usr/bin/env python3
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
cfg = json.loads(
    (ROOT / "config" / "system.json").read_text(encoding="utf-8")
)
mav = cfg["mavlink"]

assert mav["connection"] == "/dev/ttyAMA0"
assert mav["device"] == "/dev/ttyAMA0"
assert int(mav["baud"]) == 921600
assert mav["transport"] == "SERIAL_DIRECT"
assert int(mav["source_system"]) == 1
assert int(mav["source_component"]) == 191
assert int(mav["target_system"]) == 1
assert int(mav["target_component"]) == 1

print("KINGPHOENIX V1.9.5 MAVLINK TRANSPORT SELF-CHECK")
print("[PASS] direct serial /dev/ttyAMA0")
print("[PASS] baud 921600")
print("[PASS] source 1:191 -> target 1:1")
print("[PASS] MAVProxy not required by runtime config")
print("MAVLINK TRANSPORT SELF-CHECK: PASS")
