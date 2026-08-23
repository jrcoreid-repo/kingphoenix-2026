#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import argparse
import json
import math
from datetime import datetime, timezone

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(
        description="Calibrate one Kingphoenix camera and write project-compatible JSON."
    )
    ap.add_argument("--camera", choices=["front", "bottom"], required=True)
    ap.add_argument("--cols", type=int, required=True,
                    help="checkerboard INNER corners horizontally")
    ap.add_argument("--rows", type=int, required=True,
                    help="checkerboard INNER corners vertically")
    ap.add_argument("--square-mm", type=float, required=True,
                    help="physical size of ONE checker square in mm")
    ap.add_argument("--min-images", type=int, default=12)
    args = ap.parse_args()

    capture_dir = ROOT / "calibration" / "captures" / args.camera
    paths = sorted(capture_dir.glob("*.jpg")) + sorted(capture_dir.glob("*.png"))

    if not paths:
        raise SystemExit(f"No calibration images found in {capture_dir}")

    object_template = np.zeros(
        (args.rows * args.cols, 3),
        dtype=np.float32,
    )
    object_template[:, :2] = (
        np.mgrid[0:args.cols, 0:args.rows]
        .T.reshape(-1, 2)
    )
    object_template *= float(args.square_mm)

    object_points = []
    image_points = []
    used_images = []
    image_size = None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])

        ok, corners = cv2.findChessboardCorners(
            gray,
            (args.cols, args.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if not ok:
            print("[REJECT]", path.name, "- checkerboard not found")
            continue

        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria,
        )

        object_points.append(object_template.copy())
        image_points.append(corners)
        used_images.append(path.name)

    if len(used_images) < args.min_images:
        raise SystemExit(
            f"Only {len(used_images)} valid images; need >= {args.min_images}"
        )

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    per_image_errors = []
    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs,
        )
        error = cv2.norm(
            image_points[i],
            projected,
            cv2.NORM_L2,
        ) / len(projected)
        per_image_errors.append(float(error))

    width, height = image_size
    if (width, height) != (1920, 1200):
        raise SystemExit(
            f"V1.9.6.1 requires 1920x1200 full-sensor captures; got "
            f"{width}x{height}. Recapture calibration images."
        )
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])

    hfov_deg = math.degrees(
        2.0 * math.atan(width / (2.0 * fx))
    )
    vfov_deg = math.degrees(
        2.0 * math.atan(height / (2.0 * fy))
    )

    config_path = (
        ROOT / "config" / "calibration" / f"{args.camera}_camera.json"
    )
    config_path.parent.mkdir(parents=True, exist_ok=True)

    existing = {}
    if config_path.exists():
        try:
            existing = json.loads(config_path.read_text(encoding="utf-8"))
        except Exception:
            existing = {}

    # Preserve camera-to-body extrinsics if they have already been measured.
    camera_to_body = existing.get(
        "camera_to_body",
        {
            "position_m": [0.0, 0.0, 0.0],
            "rpy_deg": [0.0, 0.0, 0.0],
        },
    )
    legacy_extrinsics = existing.get(
        "extrinsics",
        {
            "position_body_m": [0.0, 0.0, 0.0],
            "rotation_body_deg": [0.0, 0.0, 0.0],
        },
    )

    matrix_list = camera_matrix.tolist()
    dist_list = dist_coeffs.reshape(-1).tolist()

    output = {
        "schema": "kingphoenix.camera_calibration.v1",
        "camera_name": args.camera,
        "camera": args.camera,
        "valid": True,

        # Fields consumed directly by GatePoseSolver.
        "image_width": int(width),
        "image_height": int(height),
        "camera_matrix": matrix_list,
        "dist_coeffs": dist_list,

        # Structured mirror for future modules/tools.
        "intrinsics": {
            "resolution": [int(width), int(height)],
            "camera_matrix": matrix_list,
            "dist_coeffs": dist_list,
            "distortion_model": "opencv_plumb_bob",
        },

        "camera_to_body": camera_to_body,
        "extrinsics": legacy_extrinsics,

        "calibration_quality": {
            "rms_reprojection_error_px": float(rms),
            "mean_reprojection_error_px": float(np.mean(per_image_errors)),
            "max_per_image_error_px": float(np.max(per_image_errors)),
            "valid_images": len(used_images),
            "total_images": len(paths),
        },

        "hfov_deg": float(hfov_deg),
        "vfov_deg": float(vfov_deg),

        "board": {
            "type": "checkerboard",
            "inner_corners": [args.cols, args.rows],
            "square_size_mm": float(args.square_mm),
        },

        "runtime_contract": {
            "resolution": [int(width), int(height)],
            "color_order": "BGR",
            "note": (
                "Intrinsic calibration is valid for the exact Kingphoenix "
                "runtime camera geometry/crop used during capture."
            ),
        },

        "used_images": used_images,
        "calibrated_at_utc": datetime.now(timezone.utc).isoformat(),
    }

    config_path.write_text(
        json.dumps(output, indent=2) + "\n",
        encoding="utf-8",
    )

    print("=" * 66)
    print(f" KINGPHOENIX {args.camera.upper()} CAMERA CALIBRATION")
    print("=" * 66)
    print(f"Valid images : {len(used_images)}/{len(paths)}")
    print(f"Resolution   : {width}x{height}")
    print(f"RMS error    : {rms:.4f} px")
    print(f"Mean error   : {np.mean(per_image_errors):.4f} px")
    print(f"Max error    : {np.max(per_image_errors):.4f} px")
    print(f"fx / fy      : {camera_matrix[0,0]:.3f} / {camera_matrix[1,1]:.3f}")
    print(f"cx / cy      : {camera_matrix[0,2]:.3f} / {camera_matrix[1,2]:.3f}")
    print(f"HFOV / VFOV  : {hfov_deg:.2f} / {vfov_deg:.2f} deg")
    print(f"Output JSON  : {config_path}")

    if rms <= 0.5:
        print("[QUALITY] EXCELLENT")
    elif rms <= 1.0:
        print("[QUALITY] GOOD")
    else:
        print("[QUALITY] REVIEW - capture more sharp/diverse board poses")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
