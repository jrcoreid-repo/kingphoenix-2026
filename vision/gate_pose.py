from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import cv2
import numpy as np


@dataclass
class GatePoseResult:
    valid: bool
    xyz: list | None = None
    rpy: list | None = None
    distance_m: float | None = None
    rvec: list | None = None
    pose_source: str = "NONE"
    metric_pose_usable: bool = False
    reprojection_error_px: float | None = None
    error: str | None = None

    def as_metadata(self):
        return {
            "pose_3d_valid": bool(self.valid),
            "xyz": self.xyz,
            "rpy": self.rpy,
            "distance_m": self.distance_m,
            "rvec": self.rvec,
            "pose_source": self.pose_source,
            "metric_pose_usable": bool(self.metric_pose_usable),
            "reprojection_error_px": self.reprojection_error_px,
            "pose_error": self.error,
        }


class GatePoseSolver:
    """4-point planar gate pose solver.

    Keypoint order is fixed by config:
      top-left, top-right, bottom-right, bottom-left.

    REAL_CALIBRATION results may later be consumed by movement.
    TEST_ESTIMATE results are visualization/debug only.
    """

    def __init__(
        self,
        calibration_path,
        *,
        frame_w,
        frame_h,
        gate_width_m,
        gate_height_m,
        test_estimate_cfg=None,
        run_profile="UNKNOWN",
    ):
        self.calibration_path = Path(calibration_path)
        self.frame_w = int(frame_w)
        self.frame_h = int(frame_h)
        self.gate_width_m = float(gate_width_m)
        self.gate_height_m = float(gate_height_m)
        self.test_estimate_cfg = test_estimate_cfg or {}
        self.run_profile = str(run_profile).upper()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.pose_source = "NONE"
        self.metric_pose_usable = False
        self._load_calibration()

    def _load_calibration(self):
        real_ok = False
        try:
            data = json.loads(
                self.calibration_path.read_text(encoding="utf-8")
            )
            if bool(data.get("valid", False)):
                cal_w = int(data.get("image_width", 0) or 0)
                cal_h = int(data.get("image_height", 0) or 0)
                if (cal_w, cal_h) != (self.frame_w, self.frame_h):
                    raise ValueError(
                        "calibration resolution mismatch: "
                        f"{cal_w}x{cal_h} != runtime "
                        f"{self.frame_w}x{self.frame_h}"
                    )

                matrix = np.asarray(
                    data.get("camera_matrix"),
                    dtype=np.float64,
                ).reshape(3, 3)
                dist = np.asarray(
                    data.get("dist_coeffs", [0, 0, 0, 0, 0]),
                    dtype=np.float64,
                ).reshape(-1, 1)

                if (
                    np.isfinite(matrix).all()
                    and matrix[0, 0] > 1.0
                    and matrix[1, 1] > 1.0
                ):
                    self.camera_matrix = matrix
                    self.dist_coeffs = dist
                    self.pose_source = "REAL_CALIBRATION"
                    self.metric_pose_usable = True
                    real_ok = True
        except Exception:
            real_ok = False

        if real_ok:
            print(
                f"[POSE] front calibration REAL: "
                f"{self.calibration_path}"
            )
            return

        allowed = {
            str(x).upper()
            for x in self.test_estimate_cfg.get(
                "allowed_profiles",
                [],
            )
        }
        estimate_enabled = bool(
            self.test_estimate_cfg.get("enabled", False)
        )

        if (
            estimate_enabled
            and self.run_profile in allowed
        ):
            hfov_deg = float(
                self.test_estimate_cfg.get("hfov_deg", 90.0)
            )
            hfov_rad = math.radians(hfov_deg)
            fx = self.frame_w / (
                2.0 * math.tan(hfov_rad / 2.0)
            )
            fy = fx
            cx = self.frame_w / 2.0
            cy = self.frame_h / 2.0

            self.camera_matrix = np.array(
                [
                    [fx, 0.0, cx],
                    [0.0, fy, cy],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            self.dist_coeffs = np.asarray(
                self.test_estimate_cfg.get(
                    "dist_coeffs",
                    [0, 0, 0, 0, 0],
                ),
                dtype=np.float64,
            ).reshape(-1, 1)

            self.pose_source = "TEST_ESTIMATE"
            self.metric_pose_usable = False

            print(
                "[POSE] TEST_ESTIMATE calibration active "
                f"(HFOV={hfov_deg:.1f} deg). "
                "Visualization only; movement use BLOCKED."
            )
            return

        self.pose_source = "NONE"
        self.metric_pose_usable = False
        print(
            "[POSE] 3D pose disabled: no valid real calibration "
            "and no permitted test estimate."
        )

    @property
    def enabled(self):
        return self.camera_matrix is not None

    def _object_points(self):
        hw = self.gate_width_m / 2.0
        hh = self.gate_height_m / 2.0

        # Mirrors the proven historical Kingphoenix implementation:
        # TL, TR, BR, BL on Z=0.
        return np.array(
            [
                [-hw, -hh, 0.0],
                [ hw, -hh, 0.0],
                [ hw,  hh, 0.0],
                [-hw,  hh, 0.0],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _image_points(keypoints):
        pts = []
        for point in keypoints[:4]:
            if point is None or len(point) < 2:
                raise ValueError("invalid keypoint")
            pts.append(
                [float(point[0]), float(point[1])]
            )
        return np.asarray(pts, dtype=np.float64)

    @staticmethod
    def _rpy_from_rvec(rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        euler, _, _, _, _, _ = cv2.RQDecomp3x3(
            rotation_matrix
        )
        return [
            float(euler[0]),
            float(euler[1]),
            float(euler[2]),
        ]

    def solve(self, keypoints):
        if not self.enabled:
            return GatePoseResult(
                valid=False,
                pose_source=self.pose_source,
                metric_pose_usable=False,
                error="calibration unavailable",
            )

        try:
            image_points = self._image_points(keypoints)
            object_points = self._object_points()

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )

            if not success:
                return GatePoseResult(
                    valid=False,
                    pose_source=self.pose_source,
                    metric_pose_usable=self.metric_pose_usable,
                    error="solvePnP failed",
                )

            xyz = [
                float(tvec[0, 0]),
                float(tvec[1, 0]),
                float(tvec[2, 0]),
            ]
            if xyz[2] <= 0.01:
                return GatePoseResult(
                    valid=False,
                    pose_source=self.pose_source,
                    metric_pose_usable=self.metric_pose_usable,
                    error="negative/near-zero Z",
                )

            rpy = self._rpy_from_rvec(rvec)
            distance = float(np.linalg.norm(tvec))

            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs,
            )
            projected = projected.reshape(-1, 2)
            reproj = float(
                np.mean(
                    np.linalg.norm(
                        projected - image_points,
                        axis=1,
                    )
                )
            )

            return GatePoseResult(
                valid=True,
                xyz=[round(v, 5) for v in xyz],
                rpy=[round(v, 3) for v in rpy],
                distance_m=round(distance, 5),
                rvec=[
                    round(float(v), 7)
                    for v in rvec.reshape(-1)
                ],
                pose_source=self.pose_source,
                metric_pose_usable=self.metric_pose_usable,
                reprojection_error_px=round(reproj, 4),
                error=None,
            )

        except Exception as exc:
            return GatePoseResult(
                valid=False,
                pose_source=self.pose_source,
                metric_pose_usable=self.metric_pose_usable,
                error=repr(exc),
            )
