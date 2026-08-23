#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import argparse
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", choices=["front", "bottom"], required=True)
    args = ap.parse_args()

    path = (
        ROOT / "config" / "calibration" / f"{args.camera}_camera.json"
    )
    data = json.loads(path.read_text(encoding="utf-8"))

    matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
    dist = np.asarray(data["dist_coeffs"], dtype=np.float64)

    assert data.get("valid") is True, "Calibration valid flag is not true"
    assert matrix.shape == (3, 3), "camera_matrix must be 3x3"
    assert np.isfinite(matrix).all(), "camera_matrix contains invalid values"
    assert matrix[0, 0] > 1.0 and matrix[1, 1] > 1.0, "invalid focal length"
    assert dist.size >= 4, "dist_coeffs too short"
    assert np.isfinite(dist).all(), "dist_coeffs contains invalid values"
    assert (
        int(data["image_width"]),
        int(data["image_height"]),
    ) == (1920, 1200), (
        f"Expected Kingphoenix 1920x1200 full-sensor calibration, got "
        f"{data['image_width']}x{data['image_height']}"
    )

    print("=" * 66)
    print(" KINGPHOENIX CAMERA CALIBRATION VALIDATION")
    print("=" * 66)
    print("[PASS]", path)
    print("Role :", args.camera.upper())
    print("K:")
    print(matrix)
    print("D:", dist.tolist())
    print(
        "RMS:",
        data.get("calibration_quality", {}).get(
            "rms_reprojection_error_px"
        ),
    )
    print("valid=true -> GatePoseSolver may use REAL_CALIBRATION")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
