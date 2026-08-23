from __future__ import annotations

from dataclasses import dataclass, asdict
import threading
import time


@dataclass(frozen=True)
class AiRoute:
    camera: str
    model_mode: str
    task: str
    detector_class_id: int | None
    source: str
    generation: int
    timestamp_ns: int

    def as_dict(self):
        return asdict(self)


class AiRouter:
    """Thread-safe owner of the current AI assignment.

    The same API is used by:
      - V1.9 bench timer switching;
      - future MissionManager switching;
      - future manual diagnostics.

    Mission code must call set_route() instead of talking directly to Hailo.
    """

    VALID_CAMERAS = {"none", "front", "bottom"}
    VALID_MODEL_MODES = {"OFF", "POSE_ONLY", "YOLO26_ONLY", "HYBRID"}

    def __init__(
        self,
        active="front",
        model_mode="HYBRID",
        task="GATE_NAV",
        detector_class_id=1,
        source="INIT",
    ):
        self._lock = threading.RLock()
        self._generation = 0
        self._route = None
        self.set_route(
            active,
            model_mode=model_mode,
            task=task,
            detector_class_id=detector_class_id,
            source=source,
        )

    def set_route(
        self,
        camera,
        *,
        model_mode="HYBRID",
        task="--",
        detector_class_id=None,
        source="MISSION",
    ):
        camera = str(camera).lower()
        model_mode = str(model_mode).upper()
        task = str(task).upper()
        source = str(source).upper()

        if camera not in self.VALID_CAMERAS:
            raise ValueError(f"invalid AI camera: {camera}")
        if model_mode not in self.VALID_MODEL_MODES:
            raise ValueError(f"invalid AI model mode: {model_mode}")

        if camera == "none":
            model_mode = "OFF"
            detector_class_id = None

        with self._lock:
            self._generation += 1
            self._route = AiRoute(
                camera=camera,
                model_mode=model_mode,
                task=task,
                detector_class_id=(
                    None
                    if detector_class_id is None
                    else int(detector_class_id)
                ),
                source=source,
                generation=self._generation,
                timestamp_ns=time.monotonic_ns(),
            )
            return self._route

    def disable(self, source="MISSION"):
        return self.set_route(
            "none",
            model_mode="OFF",
            task="NONE",
            detector_class_id=None,
            source=source,
        )

    def snapshot(self):
        with self._lock:
            return self._route

    @property
    def active(self):
        return self.snapshot().camera

    def state_for(self, camera):
        camera = str(camera).lower()
        route = self.snapshot()
        if route.camera == camera and route.model_mode != "OFF":
            return "ACTIVE"
        return "STANDBY"

    def model_mode_for(self, camera):
        camera = str(camera).lower()
        route = self.snapshot()
        if route.camera != camera:
            return "OFF"
        return route.model_mode
