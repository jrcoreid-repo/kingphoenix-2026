#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import json
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

ROOT = Path(__file__).resolve().parents[1]


def check(name, condition):
    if not condition:
        raise AssertionError(name)
    print("[PASS]", name)


def main():
    cfg = json.loads(
        (ROOT / "config" / "system.json").read_text(encoding="utf-8")
    )
    color = cfg["vision_color"]

    print("======================================================")
    print(" KINGPHOENIX V1.9.5 - VISION COLOR CONTRACT SELF-CHECK")
    print("======================================================")

    check(
        "canonical internal order = BGR",
        color["canonical_internal_order"] == "BGR",
    )
    check(
        "camera conversion = NONE",
        color["camera_conversion"] == "NONE",
    )
    check(
        "FFmpeg input = bgr24",
        color["ffmpeg_input_pix_fmt"] == "bgr24",
    )
    check(
        "Hailo tensor order = RGB",
        color["hailo_tensor_order"] == "RGB",
    )

    # Synthetic BGR red pixel: [B,G,R] = [0,0,255].
    bgr = np.array([[[0, 0, 255]]], dtype=np.uint8)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    check(
        "single BGR->RGB conversion maps red correctly",
        rgb[0, 0].tolist() == [255, 0, 0],
    )

    cam_src = (
        ROOT / "vision" / "camera_manager.py"
    ).read_text(encoding="utf-8")
    hailo_src = (
        ROOT / "vision" / "hailo_front_ai.py"
    ).read_text(encoding="utf-8")
    ffmpeg_src = (
        ROOT / "streaming" / "video_streamer.py"
    ).read_text(encoding="utf-8")

    check(
        "CameraManager has no cvtColor in Picamera2 reader",
        "cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)" not in cam_src
        and "cv2.COLOR_RGB2BGR" not in cam_src,
    )
    check(
        "Hailo performs BGR->RGB",
        "cv2.COLOR_BGR2RGB" in hailo_src,
    )
    check(
        "FFmpeg consumes bgr24",
        '"-pix_fmt", "bgr24"' in ffmpeg_src,
    )

    print("======================================================")
    print(" VISION COLOR CONTRACT SELF-CHECK: PASS")
    print("======================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
