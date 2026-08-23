from __future__ import annotations

import threading
import time


class AiTestSwitcher(threading.Thread):
    """Bench-only camera/model router driven by timed config sequence."""

    daemon = True

    def __init__(self, router, state, stop_event, cfg):
        super().__init__(name="AITestSwitcher")
        self.router = router
        self.state = state
        self.stop_event = stop_event
        self.cfg = cfg or {}
        self.sequence = list(self.cfg.get("sequence", []))
        self.loop = bool(self.cfg.get("loop", True))

    def _apply(self, item, index):
        route = self.router.set_route(
            item.get("camera", "none"),
            model_mode=item.get("model_mode", "OFF"),
            task=item.get("task", "--"),
            detector_class_id=item.get("detector_class_id"),
            source="TEST_TIMER",
        )
        self.state.set_ai_route(route.as_dict())
        print(
            "[AI-ROUTER:TEST] "
            f"step={index + 1}/{len(self.sequence)} "
            f"camera={route.camera} "
            f"model={route.model_mode} "
            f"task={route.task} "
            f"class={route.detector_class_id} "
            f"duration={float(item.get('duration_s', 5.0)):.1f}s"
        )
        return float(item.get("duration_s", 5.0))

    def run(self):
        if not self.sequence:
            print("[AI-ROUTER:TEST] no sequence configured")
            return

        while not self.stop_event.is_set():
            for index, item in enumerate(self.sequence):
                duration = self._apply(item, index)
                deadline = time.monotonic() + max(0.05, duration)

                while (
                    not self.stop_event.is_set()
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)

                if self.stop_event.is_set():
                    return

            if not self.loop:
                return
