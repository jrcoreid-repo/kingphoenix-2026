#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision.camera_manager import CameraManager


def build_camera(camera_name: str, cfg: dict) -> CameraManager:
    camera_cfg = cfg["camera"]
    role_cfg = camera_cfg[camera_name]
    stream_cfg = cfg["stream"]

    return CameraManager(
        camera_name,
        backend=camera_cfg.get("backend", "picamera2"),
        camera_num=role_cfg.get("camera_num"),
        sensor_mode=role_cfg.get("sensor_mode", {}),
        output=role_cfg.get("output", {}),
        runtime=camera_cfg.get("runtime", {}),
        width=stream_cfg.get("width", 640),
        height=stream_cfg.get("height", 640),
        fps=stream_cfg.get("fps", 30),
    )


def main():
    ap = argparse.ArgumentParser(
        description="Capture checkerboard frames from the real Kingphoenix camera."
    )
    ap.add_argument("--camera", choices=["front", "bottom"], required=True)
    ap.add_argument("--cols", type=int, required=True,
                    help="checkerboard INNER corners horizontally")
    ap.add_argument("--rows", type=int, required=True,
                    help="checkerboard INNER corners vertically")
    ap.add_argument("--count", type=int, default=25)
    args = ap.parse_args()

    cfg = json.loads((ROOT / "config" / "system.json").read_text(encoding="utf-8"))
    cam = build_camera(args.camera, cfg)

    out_dir = ROOT / "calibration" / "captures" / args.camera
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 66)
    print(" KINGPHOENIX CAMERA CALIBRATION CAPTURE")
    print("=" * 66)
    print(f"Camera       : {args.camera.upper()}")
    print(f"camera_num   : {cfg['camera'][args.camera]['camera_num']}")
    print(f"Inner corners: {args.cols} x {args.rows}")
    print(f"Target images: {args.count}")
    print("SPACE = save when corners detected")
    print("Q / ESC = quit")
    print("Move board around center, edges, corners, near/far, and tilt it.")
    print("=" * 66)

    saved = 0
    cam.open()

    try:
        while saved < args.count:
            packet = cam.read()
            if packet is None:
                continue

            frame = packet.frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            ok, corners = cv2.findChessboardCorners(
                gray,
                (args.cols, args.rows),
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_NORMALIZE_IMAGE
                + cv2.CALIB_CB_FAST_CHECK,
            )

            preview = frame.copy()

            if ok:
                cv2.drawChessboardCorners(
                    preview,
                    (args.cols, args.rows),
                    corners,
                    ok,
                )

            status = "DETECTED" if ok else "SEARCH"
            cv2.putText(
                preview,
                f"{args.camera.upper()} | {status} | {saved}/{args.count}",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("Kingphoenix Camera Calibration", preview)
            key = cv2.waitKey(1) & 0xFF

            if key == 32:  # SPACE
                if not ok:
                    print("[SKIP] Checkerboard not detected.")
                    continue

                path = out_dir / f"{args.camera}_{saved:03d}.jpg"
                if cv2.imwrite(str(path), frame):
                    print("[SAVE]", path)
                    saved += 1

            elif key in (ord("q"), 27):
                break

    finally:
        cam.close()
        cv2.destroyAllWindows()

    print(f"[DONE] {saved} images saved in {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
