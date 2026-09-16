#!/usr/bin/env python3
"""Kingphoenix front-gate vision SHADOW test.

Camera FRONT -> YOLO26/Hailo-8 -> gate bbox -> alignment decision -> terminal/CSV.

SAFETY CONTRACT
---------------
This tool intentionally imports NO pymavlink module and sends NO flight command.
It is the perception/decision validation phase before any GUIDED vision control.

Canonical geometry is the real front-camera frame (normally 1920x1200). The
existing HailoYolo26 engine letterboxes this to 640x640 internally and maps the
selected detection back to canonical camera coordinates.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from config import load_json
from vision.bbox_control import BboxControlEstimator
from vision.camera_manager import CameraManager
from vision.hailo_yolo26 import HailoYolo26


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def direction_x(ex: float | None, deadband: float) -> str:
    if ex is None:
        return "SEARCH"
    if abs(float(ex)) <= abs(float(deadband)):
        return "CENTER"
    return "RIGHT" if float(ex) > 0.0 else "LEFT"


def direction_y(ey: float | None, deadband: float) -> str:
    if ey is None:
        return "SEARCH"
    if abs(float(ey)) <= abs(float(deadband)):
        return "CENTER"
    # Image Y grows downward.
    return "DOWN" if float(ey) > 0.0 else "UP"


def make_camera(system_cfg: dict, video_path: str | None) -> CameraManager:
    cam_cfg = system_cfg["camera"]
    front = cam_cfg["front"]

    if video_path:
        return CameraManager(
            "front",
            video_path,
            backend="opencv",
            width=int(front["output"].get("width", 1920)),
            height=int(front["output"].get("height", 1200)),
            fps=float(cam_cfg["runtime"].get("capture_target_fps", 30.0)),
            loop_file=True,
        )

    return CameraManager(
        "front",
        backend=cam_cfg.get("backend", "picamera2"),
        camera_num=front.get("camera_num"),
        sensor_mode=front.get("sensor_mode", {}),
        output=front.get("output", {}),
        runtime=cam_cfg.get("runtime", {}),
        width=int(front["output"].get("width", 1920)),
        height=int(front["output"].get("height", 1200)),
        fps=float(cam_cfg["runtime"].get("capture_target_fps", 30.0)),
    )


def open_csv(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    f = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow(
        [
            "monotonic_s",
            "frame_id",
            "vision_valid",
            "confidence",
            "x1",
            "y1",
            "x2",
            "y2",
            "cx",
            "cy",
            "ex",
            "ey",
            "bbox_area_ratio",
            "phase",
            "suggest_lateral_mps",
            "suggest_yaw_rate_dps",
            "suggest_forward_mps",
            "aligned",
            "stable_hits",
            "stable_window",
            "stable",
            "model_ms",
            "model_fps",
        ]
    )
    return f, writer


def main() -> int:
    system_cfg = load_json("system.json")
    ai_cfg = system_cfg["ai"]
    vc_cfg = system_cfg["vision_control"]
    front_cfg = system_cfg["camera"]["front"]

    ap = argparse.ArgumentParser(
        description="Front camera YOLO26 gate SHADOW mode; NO MAVLink TX"
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="seconds to run; 0 = until Ctrl+C",
    )
    ap.add_argument(
        "--print-hz",
        type=float,
        default=5.0,
        help="terminal status refresh rate",
    )
    ap.add_argument(
        "--confidence",
        type=float,
        default=None,
        help="override detector confidence threshold (default from system.json)",
    )
    ap.add_argument(
        "--class-id",
        type=int,
        default=None,
        help="override front detector class id (default from system.json)",
    )
    ap.add_argument(
        "--stable-window",
        type=int,
        default=20,
        help="rolling frames used for center stability",
    )
    ap.add_argument(
        "--stable-required",
        type=int,
        default=18,
        help="aligned frames required inside rolling window",
    )
    ap.add_argument(
        "--lateral-cap",
        type=float,
        default=0.20,
        help="shadow suggestion cap for later phase-2 lateral test",
    )
    ap.add_argument(
        "--yaw-cap",
        type=float,
        default=8.0,
        help="shadow suggestion cap for later phase-2 yaw test, deg/s",
    )
    ap.add_argument(
        "--video",
        default=None,
        help="optional video file instead of physical front camera",
    )
    ap.add_argument(
        "--csv",
        default=None,
        help="CSV output path; default logs/front_gate_shadow_<timestamp>.csv",
    )
    ap.add_argument(
        "--no-csv",
        action="store_true",
        help="disable CSV logging",
    )
    args = ap.parse_args()

    if args.duration < 0:
        raise SystemExit("--duration must be >=0")
    if args.print_hz <= 0 or args.print_hz > 30:
        raise SystemExit("--print-hz must be >0 and <=30")
    if args.stable_window < 1:
        raise SystemExit("--stable-window must be >=1")
    if args.stable_required < 1 or args.stable_required > args.stable_window:
        raise SystemExit("--stable-required must be 1..stable-window")
    if args.lateral_cap <= 0 or args.lateral_cap > 1.0:
        raise SystemExit("--lateral-cap must be >0 and <=1.0m/s")
    if args.yaw_cap <= 0 or args.yaw_cap > 45.0:
        raise SystemExit("--yaw-cap must be >0 and <=45deg/s")
    if args.confidence is not None and not 0.0 < args.confidence <= 1.0:
        raise SystemExit("--confidence must be >0 and <=1")

    if str(ai_cfg.get("engine", "")).upper() != "YOLO26_ONLY":
        raise SystemExit(
            f"AI engine is {ai_cfg.get('engine')!r}; expected YOLO26_ONLY for this tool"
        )

    canonical_w = int(front_cfg["output"].get("width", 1920))
    canonical_h = int(front_cfg["output"].get("height", 1200))
    detector_class_id = int(
        args.class_id
        if args.class_id is not None
        else ai_cfg.get("front", {}).get("detector_class_id", 1)
    )
    confidence_min = float(
        args.confidence
        if args.confidence is not None
        else ai_cfg.get("detector", {}).get("confidence_threshold", 0.25)
    )

    # BboxControlEstimator remains an estimator only. This script additionally
    # caps displayed suggestions to conservative phase-2 values and forces FWD=0.
    estimator = BboxControlEstimator(
        "front",
        vc_cfg,
        w=canonical_w,
        h=canonical_h,
    )
    estimator.conf = confidence_min

    align_limit = float(
        vc_cfg.get("front", {}).get("align_error_max", 0.12)
    )
    deadband_x = float(vc_cfg.get("deadband_x", 0.035))
    deadband_y = float(vc_cfg.get("deadband_y", 0.035))

    camera = make_camera(system_cfg, args.video)
    engine = HailoYolo26(ai_cfg, frame_w=canonical_w, frame_h=canonical_h)
    engine.det_conf = confidence_min

    stable_history = deque(maxlen=int(args.stable_window))
    last_print = 0.0
    started = time.monotonic()
    frames = 0
    detections = 0
    last_detect_s = None

    csv_file = None
    csv_writer = None
    csv_path = None
    if not args.no_csv:
        csv_path = Path(args.csv) if args.csv else (
            PROJECT_ROOT
            / "logs"
            / f"front_gate_shadow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        csv_file, csv_writer = open_csv(csv_path)

    print("============================================================")
    print(" KINGPHOENIX FRONT GATE AI - SHADOW MODE")
    print("============================================================")
    print("[SAFETY] COMMAND TX : DISABLED")
    print("[SAFETY] MAVLINK    : NOT IMPORTED / NOT OPENED")
    print(f"[CAM] front geometry: {canonical_w}x{canonical_h}")
    print(f"[AI] engine         : YOLO26_ONLY / Hailo-8")
    print(f"[AI] gate class     : {detector_class_id}")
    print(f"[AI] confidence min : {confidence_min:.2f}")
    print(
        f"[LOCK] stable       : {args.stable_required}/{args.stable_window} "
        f"frames with |ex| <= {align_limit:.3f}"
    )
    print(
        f"[NEXT-PHASE CAPS] lateral={args.lateral_cap:.2f}m/s "
        f"yaw={args.yaw_cap:.1f}deg/s forward=0.00m/s"
    )
    if csv_path is not None:
        print(f"[LOG] {csv_path}")
    if args.video:
        print(f"[SOURCE] VIDEO FILE {args.video}")
    else:
        print(f"[SOURCE] PHYSICAL FRONT CAMERA camera_num={front_cfg.get('camera_num')}")
    print("------------------------------------------------------------")

    try:
        camera.open()
        engine.open()

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                break

            packet = camera.read()
            if packet is None:
                time.sleep(0.01)
                continue

            result = engine.process(
                packet.frame,
                policy="YOLO26_ONLY",
                detector_class_id=detector_class_id,
            )
            frames += 1

            ctrl = estimator.estimate(result.bbox, result.confidence)
            valid = bool(result.vision_valid and ctrl.valid)

            bbox = None
            cx = cy = None
            if valid and result.bbox is not None and len(result.bbox) >= 4:
                bbox = [float(v) for v in result.bbox[:4]]
                cx = (bbox[0] + bbox[2]) * 0.5
                cy = (bbox[1] + bbox[3]) * 0.5
                detections += 1
                last_detect_s = now

            ex = float(ctrl.ex) if valid and ctrl.ex is not None else None
            ey = float(ctrl.ey) if valid and ctrl.ey is not None else None
            area = (
                float(ctrl.bbox_area_ratio)
                if valid and ctrl.bbox_area_ratio is not None
                else None
            )

            aligned = bool(valid and ex is not None and abs(ex) <= align_limit)
            stable_history.append(1 if aligned else 0)
            stable_hits = int(sum(stable_history))
            stable = bool(
                len(stable_history) == int(args.stable_window)
                and stable_hits >= int(args.stable_required)
            )

            raw_intent = ctrl.intent or {}
            raw_lat = float(raw_intent.get("lateral_mps", 0.0)) if valid else 0.0
            raw_yaw = float(raw_intent.get("yaw_rate_dps", 0.0)) if valid else 0.0

            # Conservative display-only suggestion for the NEXT flight phase.
            suggest_lat = clamp(raw_lat, -args.lateral_cap, args.lateral_cap)
            suggest_yaw = clamp(raw_yaw, -args.yaw_cap, args.yaw_cap)
            suggest_fwd = 0.0

            if csv_writer is not None:
                b = bbox or [None, None, None, None]
                csv_writer.writerow(
                    [
                        f"{now:.6f}",
                        int(packet.frame_id),
                        int(valid),
                        None if result.confidence is None else f"{float(result.confidence):.6f}",
                        *[None if v is None else f"{float(v):.3f}" for v in b],
                        None if cx is None else f"{cx:.3f}",
                        None if cy is None else f"{cy:.3f}",
                        None if ex is None else f"{ex:.6f}",
                        None if ey is None else f"{ey:.6f}",
                        None if area is None else f"{area:.8f}",
                        str(ctrl.phase),
                        f"{suggest_lat:.6f}",
                        f"{suggest_yaw:.6f}",
                        f"{suggest_fwd:.6f}",
                        int(aligned),
                        stable_hits,
                        len(stable_history),
                        int(stable),
                        f"{float(result.model_ms):.3f}",
                        f"{float(result.model_fps):.3f}",
                    ]
                )

            if now - last_print >= 1.0 / float(args.print_hz):
                if valid:
                    hdir = direction_x(ex, deadband_x)
                    vdir = direction_y(ey, deadband_y)
                    conf_text = (
                        "--"
                        if result.confidence is None
                        else f"{float(result.confidence):.3f}"
                    )
                    bbox_text = (
                        f"[{bbox[0]:.0f},{bbox[1]:.0f},{bbox[2]:.0f},{bbox[3]:.0f}]"
                        if bbox is not None
                        else "--"
                    )
                    print(
                        f"[GATE] LOCKED frame={packet.frame_id} conf={conf_text} "
                        f"model={result.model_ms:.1f}ms/{result.model_fps:.1f}fps"
                    )
                    print(
                        f"       bbox={bbox_text} center=({cx:.0f},{cy:.0f}) "
                        f"ex={ex:+.3f}({hdir}) ey={ey:+.3f}({vdir}) area={area:.4f}"
                    )
                    print(
                        f"       SHADOW LAT={suggest_lat:+.2f}m/s "
                        f"YAW={suggest_yaw:+.1f}deg/s FWD=0.00 | "
                        f"stable={stable_hits}/{args.stable_window} "
                        f"{'YES' if stable else 'NO'} | COMMAND=DISABLED"
                    )
                else:
                    lost_age = (
                        float("inf")
                        if last_detect_s is None
                        else max(0.0, now - last_detect_s)
                    )
                    lost_text = "never" if not math.isfinite(lost_age) else f"{lost_age:.2f}s"
                    print(
                        f"[GATE] SEARCH frame={packet.frame_id} "
                        f"vision={result.vision_mode} conf={result.confidence} "
                        f"lost={lost_text} stable={stable_hits}/{args.stable_window} "
                        "COMMAND=DISABLED"
                    )
                last_print = now

    except KeyboardInterrupt:
        print("\n[SHADOW] operator stop")
    finally:
        try:
            engine.close()
        except Exception:
            pass
        try:
            camera.close()
        except Exception:
            pass
        if csv_file is not None:
            try:
                csv_file.flush()
                csv_file.close()
            except Exception:
                pass

    elapsed = max(time.monotonic() - started, 1e-6)
    detection_ratio = detections / max(frames, 1)
    print("------------------------------------------------------------")
    print(
        f"[RESULT] frames={frames} detections={detections} "
        f"detect_ratio={detection_ratio*100.0:.1f}% loop_fps={frames/elapsed:.1f}"
    )
    if csv_path is not None:
        print(f"[RESULT] CSV={csv_path}")
    print("[RESULT] SHADOW COMPLETE - ZERO FLIGHT COMMANDS SENT")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
