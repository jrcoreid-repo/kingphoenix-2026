#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import load_json


def main():
    cfg = load_json("system.json")
    ai = cfg["ai"]

    pose = Path(ai["front"]["primary"]["hef"])
    detect = Path(ai["front"]["fallback"]["hef"])

    print("======================================================")
    print(" KINGPHOENIX V1.9.3.1 - AI CONFIG CHECK")
    print("======================================================")
    print("AI enabled :", ai.get("enabled"))
    print("AI camera  :", ai.get("active_camera"))
    print("POSE HEF   :", pose)
    print("POSE exists:", pose.exists())
    print("DET HEF    :", detect)
    print("DET exists :", detect.exists())

    try:
        import hailo_platform  # noqa: F401
        hailo_ok = True
    except Exception as exc:
        hailo_ok = False
        print("Hailo SDK  : ERROR", repr(exc))
    else:
        print("Hailo SDK  : OK")

    ok = (
        bool(ai.get("enabled"))
        and ai.get("active_camera") == "front"
        and pose.exists()
        and detect.exists()
        and hailo_ok
    )

    print("======================================================")
    print(" AI CONFIG CHECK:", "PASS" if ok else "FAIL")
    print("======================================================")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
