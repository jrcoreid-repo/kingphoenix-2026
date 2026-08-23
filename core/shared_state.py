from __future__ import annotations

from copy import deepcopy
from threading import RLock
import time

from core.state_types import MissionState
from flight.vehicle_state import VehicleState


def _camera_state(ai_state, ai_task):
    return {
        "video_state": "OFFLINE",
        "ai_state": ai_state,
        "ai_task": ai_task,
        "frame_id": None,
        "frame_timestamp_ns": 0,
        "fps": 0.0,
        "vision_mode": "SEARCH",
        "model_mode": "OFF",
        "ai_model": None,
        "detector_class_id": None,
        "vision_valid": False,
        "vision_timestamp_ns": 0,
        "vision_error": None,
        "model_ms": 0.0,
        "confidence": None,
        "target": None,
        "bbox": None,
        "keypoints": None,
        "xyz": None,
        "rpy": None,
        "distance_m": None,
        "pose_3d_valid": False,
        "pose_source": "NONE",
        "metric_pose_usable": False,
        "reprojection_error_px": None,
        "overlay_type": "NONE",
        "pose_valid": False,
        "detector_valid": False,
        "metadata_tx_ok": False,
        "metadata_tx_count": 0,
        "metadata_tx_error": None,
    }


class SharedState:
    """Central shared state.

    Vision producers write through update_vision().
    Future mission/movement code reads immutable copies via
    get_vision_snapshot(); it must never consume GCS UDP metadata.
    """

    def __init__(self):
        self.lock = RLock()
        self.vehicle = VehicleState()
        self.mission = MissionState()
        self.system = {
            "running": False,
            "gcs_connected": False,
            "gcs_endpoint": None,
            "run_profile": "UNKNOWN",
            "pi_health_status": "UNKNOWN",
            "pi_health": {
                "timestamp_ns": 0,
                "status": "UNKNOWN",
                "cpu_temp_c": None,
                "cpu_load_pct": None,
                "ram_used_pct": None,
                "disk_free_gb": None,
                "uptime_s": None,
                "throttled_raw": None,
                "throttled_hex": None,
                "undervoltage_now": False,
                "frequency_capped_now": False,
                "throttling_now": False,
                "soft_temp_limit_now": False,
                "undervoltage_occurred": False,
                "frequency_capped_occurred": False,
                "throttling_occurred": False,
                "soft_temp_limit_occurred": False,
                "checks": {},
            },
        }
        self.vision = {
            "active_ai_camera": "front",
            "ai_route": {
                "camera": "front",
                "model_mode": "HYBRID",
                "task": "GATE_NAV",
                "detector_class_id": 1,
                "source": "INIT",
                "generation": 0,
                "timestamp_ns": 0,
            },
            "front": _camera_state("ACTIVE", "GATE_NAV"),
            "bottom": _camera_state("STANDBY", "DROP_ALIGNMENT"),
        }

    def set_ai_route(self, route):
        route = dict(route)
        with self.lock:
            self.vision["ai_route"] = deepcopy(route)
            self.vision["active_ai_camera"] = route.get("camera", "none")

            for camera in ("front", "bottom"):
                active = (
                    route.get("camera") == camera
                    and route.get("model_mode") != "OFF"
                )
                self.vision[camera]["ai_state"] = (
                    "ACTIVE" if active else "STANDBY"
                )
                if active:
                    self.vision[camera]["ai_task"] = route.get(
                        "task",
                        self.vision[camera].get("ai_task", "--"),
                    )
                    self.vision[camera]["model_mode"] = route.get(
                        "model_mode",
                        "OFF",
                    )
                    self.vision[camera]["detector_class_id"] = route.get(
                        "detector_class_id"
                    )
                else:
                    self.vision[camera]["model_mode"] = "OFF"

    def update_vision(self, camera, **fields):
        camera = str(camera).lower()
        if camera not in ("front", "bottom"):
            raise ValueError(camera)

        with self.lock:
            target = self.vision[camera]
            target.update(deepcopy(fields))

    def get_vision_snapshot(self, camera=None, max_age_s=None):
        """Mission/movement-safe copy with optional freshness annotation."""
        now_ns = time.monotonic_ns()

        with self.lock:
            if camera is None:
                result = deepcopy(self.vision)
            else:
                camera = str(camera).lower()
                result = deepcopy(self.vision[camera])

        def annotate(item):
            ts = int(item.get("vision_timestamp_ns") or 0)
            if ts <= 0:
                age_s = float("inf")
            else:
                age_s = (now_ns - ts) / 1e9
            item["vision_age_s"] = age_s
            if max_age_s is None:
                item["fresh"] = ts > 0
            else:
                item["fresh"] = (
                    ts > 0 and age_s <= float(max_age_s)
                )
            item["usable"] = bool(
                item.get("vision_valid")
                and item.get("fresh")
                and item.get("ai_state") == "ACTIVE"
            )
            item["metric_pose_ready"] = bool(
                item.get("usable")
                and item.get("pose_3d_valid")
                and item.get("metric_pose_usable")
            )
            return item

        if camera is not None:
            return annotate(result)

        for name in ("front", "bottom"):
            annotate(result[name])
        return result
