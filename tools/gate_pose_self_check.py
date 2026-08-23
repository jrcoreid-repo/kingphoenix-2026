#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import json

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.gate_pose import GatePoseSolver


def check(name, cond):
    if not cond:
        raise AssertionError(name)
    print("[PASS]", name)


def main():
    print("======================================================")
    print(" KINGPHOENIX V1.9.3.1 GATE POSE SELF-CHECK")
    print("======================================================")

    K = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 320.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    D = np.zeros((5, 1), dtype=np.float64)

    object_points = np.array([
        [-1.0, -1.0, 0.0],
        [ 1.0, -1.0, 0.0],
        [ 1.0,  1.0, 0.0],
        [-1.0,  1.0, 0.0],
    ], dtype=np.float64)

    rvec = np.array([[0.08], [-0.12], [0.04]], dtype=np.float64)
    tvec = np.array([[0.25], [-0.10], [5.0]], dtype=np.float64)

    image_points, _ = cv2.projectPoints(
        object_points, rvec, tvec, K, D
    )
    image_points = image_points.reshape(-1, 2)

    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "front_camera.json"
        p.write_text(json.dumps({
            "valid": True,
            "camera_matrix": K.tolist(),
            "dist_coeffs": D.reshape(-1).tolist(),
        }))

        solver = GatePoseSolver(
            p,
            frame_w=1920,
            frame_h=1200,
            gate_width_m=2.0,
            gate_height_m=2.0,
            test_estimate_cfg={},
            run_profile="FULL_MISSION",
        )
        result = solver.solve(image_points.tolist())

    check("pose valid", result.valid)
    check("real calibration source", result.pose_source == "REAL_CALIBRATION")
    check("metric pose usable", result.metric_pose_usable)
    check("positive Z", result.xyz is not None and result.xyz[2] > 0)
    check("Z accuracy", abs(result.xyz[2] - 5.0) < 0.05)
    check(
        "distance accuracy",
        result.distance_m is not None
        and abs(result.distance_m - float(np.linalg.norm(tvec))) < 0.05,
    )
    check(
        "low reprojection error",
        result.reprojection_error_px is not None
        and result.reprojection_error_px < 0.5,
    )

    print("======================================================")
    print(" GATE POSE SELF-CHECK: PASS")
    print("======================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
