#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import json
import sys
import time

import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vision.camera_manager import CameraManager

ROOT=Path(__file__).resolve().parents[1]

def build(name,camera_cfg,stream_cfg):
    role=camera_cfg[name]
    return CameraManager(
        name,
        backend=camera_cfg.get("backend","picamera2"),
        camera_num=role["camera_num"],
        sensor_mode=role.get("sensor_mode",{}),
        output=role.get("output",{}),
        runtime=camera_cfg.get("runtime",{}),
        width=stream_cfg.get("width",640),
        height=stream_cfg.get("height",640),
        fps=stream_cfg.get("fps",30),
    )

def main():
    cfg=json.loads((ROOT/"config/system.json").read_text())
    camera_cfg=cfg["camera"]
    stream_cfg=cfg["stream"]

    print("="*64)
    print(" KINGPHOENIX V1.9.4 - DUAL AR0234 PICAMERA2 TEST")
    print("="*64)
    print("BOTTOM -> camera_num 0")
    print("FRONT  -> camera_num 1")

    try:
        from picamera2 import Picamera2
    except Exception as exc:
        print(f"[FAIL] Picamera2 import: {exc}")
        print("Install: sudo apt install python3-picamera2")
        return 2

    info=Picamera2.global_camera_info()
    print(f"Detected cameras: {len(info)}")
    for i,item in enumerate(info):
        print(f" [{i}] {item.get('Model','--')} | {item.get('Id','--')}")

    if len(info)<2:
        print("[FAIL] Need two cameras.")
        return 3

    cams={
        "bottom":build("bottom",camera_cfg,stream_cfg),
        "front":build("front",camera_cfg,stream_cfg),
    }

    try:
        cams["bottom"].open()
        cams["front"].open()
        print("[PASS] both cameras opened simultaneously")

        samples={}
        counts={"bottom":0,"front":0}
        duration=4.0
        start=time.monotonic()

        while time.monotonic()-start<duration:
            for name in ("bottom","front"):
                p=cams[name].read()
                if p is not None:
                    counts[name]+=1
                    samples[name]=p.frame.copy()

        elapsed=time.monotonic()-start
        for name in ("bottom","front"):
            print(
                f"{name.upper():6s}: {counts[name]} frames "
                f"/ {elapsed:.2f}s = {counts[name]/max(elapsed,1e-9):.1f} fps"
            )

        outdir=ROOT/"logs"
        outdir.mkdir(exist_ok=True)
        for name,frame in samples.items():
            p=outdir/f"camera_test_{name}.jpg"
            cv2.imwrite(str(p),frame)
            print("saved:",p)

        if len(samples)==2:
            print("DUAL AR0234 CAMERA TEST: PASS")
            return 0
        print("DUAL AR0234 CAMERA TEST: FAIL")
        return 4
    finally:
        for cam in cams.values():
            cam.close()

if __name__=="__main__":
    raise SystemExit(main())
