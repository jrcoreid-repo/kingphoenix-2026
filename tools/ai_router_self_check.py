#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.shared_state import SharedState
from vision.ai_router import AiRouter


def check(name, condition):
    if not condition:
        raise AssertionError(name)
    print("[PASS]", name)


def main():
    print("======================================================")
    print(" KINGPHOENIX V1.9.3.1 AI ROUTER SELF-CHECK")
    print("======================================================")

    state = SharedState()
    router = AiRouter(
        "front",
        model_mode="HYBRID",
        task="GATE_NAV",
        detector_class_id=1,
        source="TEST",
    )

    state.set_ai_route(router.snapshot().as_dict())
    check("front active", router.state_for("front") == "ACTIVE")
    check("bottom standby", router.state_for("bottom") == "STANDBY")
    check("front hybrid", router.model_mode_for("front") == "HYBRID")

    route = router.set_route(
        "bottom",
        model_mode="YOLO26_ONLY",
        task="DROP_ALIGNMENT",
        detector_class_id=2,
        source="TEST",
    )
    state.set_ai_route(route.as_dict())

    check("bottom active", router.state_for("bottom") == "ACTIVE")
    check("front standby after switch", router.state_for("front") == "STANDBY")
    check("bottom class 2", route.detector_class_id == 2)
    check("bottom detector only", route.model_mode == "YOLO26_ONLY")

    route = router.set_route(
        "front",
        model_mode="YOLO26_ONLY",
        task="GATE_NAV",
        detector_class_id=1,
        source="MISSION",
    )
    state.set_ai_route(route.as_dict())
    check("mission can select detector-only", route.model_mode == "YOLO26_ONLY")

    state.update_vision(
        "front",
        ai_state="ACTIVE",
        vision_valid=True,
        vision_timestamp_ns=time.monotonic_ns(),
        target=[320.0, 240.0],
        confidence=0.91,
    )
    snap = state.get_vision_snapshot("front", max_age_s=0.2)
    check("global vision snapshot fresh", snap["fresh"])
    check("global vision snapshot usable", snap["usable"])
    check("global target accessible", snap["target"] == [320.0, 240.0])

    route = router.disable(source="MISSION")
    state.set_ai_route(route.as_dict())
    check("AI can be fully disabled", router.active == "none")

    print("======================================================")
    print(" AI ROUTER SELF-CHECK: PASS")
    print("======================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
