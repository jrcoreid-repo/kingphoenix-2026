#!/usr/bin/env python3
"""Kingphoenix V2.2 Startup Supervisor / System Readiness.

Keeps V2.1.6 RSSI + selective read-only MAVLink telemetry, adds robust Hailo
startup retries with a real inference sanity test, camera-open checks, and a
single system.ready_for_mission authority. Diagnostic services stay alive when
critical hardware fails; mission readiness stays locked.
"""

from __future__ import annotations

import time

# Preserve the validated V2.1.6 RSSI and selective MAVLink telemetry patches.
import main_v2_1_3  # noqa: F401

from core.runtime import KingphoenixRuntime
from streaming.vision_stream_session import VisionStreamSession, CameraStreamWorker


_original_system_metadata_snapshot = CameraStreamWorker._system_metadata_snapshot


def _system_metadata_snapshot_v22(self):
    out = _original_system_metadata_snapshot(self)
    with self.state.lock:
        out["ai_health"] = dict(self.state.system.get("ai_health") or {})
        out["readiness"] = dict(self.state.system.get("readiness") or {})
        out["ready_for_mission"] = bool(self.state.system.get("ready_for_mission", False))
        out["startup_state"] = str(self.state.system.get("startup_state", "UNKNOWN"))
    return out


CameraStreamWorker._system_metadata_snapshot = _system_metadata_snapshot_v22


def _set_ai_health(session, **fields):
    with session.state.lock:
        health = dict(session.state.system.get("ai_health") or {})
        health.update(fields)
        health["timestamp_ns"] = time.monotonic_ns()
        session.state.system["ai_health"] = health


def _install_camera_open_probe(session, worker):
    original_open = worker.camera.open

    def checked_open():
        try:
            result = original_open()
            session.state.update_vision(
                worker.camera_name,
                camera_open=True,
                vision_error=None,
            )
            print(f"[STARTUP] {worker.camera_name.upper()} CAMERA PASS")
            return result
        except Exception as exc:
            session.state.update_vision(
                worker.camera_name,
                camera_open=False,
                vision_error=repr(exc),
            )
            print(f"[STARTUP] {worker.camera_name.upper()} CAMERA FAIL | {exc}")
            raise

    worker.camera.open = checked_open


def _vision_start_v22(self):
    ai_cfg = self.cfg.get("ai", {})
    startup_cfg = self.cfg.get("readiness", {}).get("hailo", {})
    max_attempts = max(1, int(startup_cfg.get("max_attempts", 3)))
    retry_delay_s = max(0.0, float(startup_cfg.get("retry_delay_s", 1.0)))

    _set_ai_health(
        self,
        status="INITIALIZING",
        device="CHECKING",
        hef="CHECKING",
        network_group="CHECKING",
        inference="CHECKING",
        attempt=0,
        last_error=None,
        detail="Hailo startup self-test",
    )

    if self.shared_ai is None:
        _set_ai_health(
            self,
            status="FAIL",
            device="DISABLED",
            hef="DISABLED",
            network_group="DISABLED",
            inference="DISABLED",
            detail="AI disabled in configuration",
        )
    else:
        last_error = None
        for attempt in range(1, max_attempts + 1):
            _set_ai_health(self, attempt=attempt, status="INITIALIZING")
            print(f"[STARTUP] HAILO attempt {attempt}/{max_attempts}")
            try:
                self.shared_ai.open()
                _set_ai_health(
                    self,
                    device="PASS",
                    hef="PASS",
                    network_group="PASS",
                    detail="device + HEF + network group active; testing inference",
                )

                # Real Hailo inference, not only /dev/hailo0 existence. A blank
                # canonical frame is sufficient to prove the entire inference
                # path can accept a tensor and return normally.
                import numpy as np

                camera_runtime = self.cfg.get("camera", {}).get("runtime", {})
                w = int(camera_runtime.get("canonical_width", 1920))
                h = int(camera_runtime.get("canonical_height", 1200))
                dummy = np.zeros((h, w, 3), dtype=np.uint8)
                detector_class_id = self.ai_router.snapshot().detector_class_id
                self.shared_ai.process(
                    dummy,
                    policy="YOLO26_ONLY",
                    detector_class_id=detector_class_id,
                )

                _set_ai_health(
                    self,
                    status="PASS",
                    device="PASS",
                    hef="PASS",
                    network_group="PASS",
                    inference="PASS",
                    last_error=None,
                    detail=f"Hailo READY after attempt {attempt}; real inference PASS",
                )
                print("[STARTUP] HAILO DEVICE PASS")
                print("[STARTUP] YOLO26 HEF PASS")
                print("[STARTUP] HAILO NETWORK GROUP PASS")
                print("[STARTUP] HAILO REAL INFERENCE PASS")
                break
            except Exception as exc:
                last_error = repr(exc)
                print(f"[STARTUP] HAILO FAIL attempt {attempt}/{max_attempts} | {exc}")
                try:
                    self.shared_ai.close()
                except Exception:
                    pass
                _set_ai_health(
                    self,
                    status="RECOVERING" if attempt < max_attempts else "FAIL",
                    device="FAIL",
                    hef="UNKNOWN",
                    network_group="FAIL",
                    inference="FAIL",
                    last_error=last_error,
                    detail=f"Hailo initialization failed on attempt {attempt}",
                )
                if attempt < max_attempts:
                    print(f"[STARTUP] HAILO RECOVERY retry in {retry_delay_s:.1f}s")
                    time.sleep(retry_delay_s)
        else:
            _set_ai_health(
                self,
                status="FAIL",
                last_error=last_error,
                detail=f"Hailo OFFLINE after {max_attempts} attempts",
            )

    # Camera workers still run even if Hailo failed so the operator keeps
    # diagnostic video. Wrap camera.open() to expose actual boot status.
    for worker in self.workers:
        _install_camera_open_probe(self, worker)
        worker.start()

    if self.test_switcher is not None:
        self.test_switcher.start()


VisionStreamSession.start = _vision_start_v22


def main():
    print("============================================================")
    print(" KINGPHOENIX V2.2 - STARTUP SUPERVISOR / SYSTEM READINESS")
    print("============================================================")
    print("[V2.2] Critical hardware must PASS before READY FOR MISSION")
    print("[V2.2] Hailo check includes REAL inference + retry recovery")
    print("[V2.2] Runtime/GCS/video diagnostics stay alive on hardware FAIL")
    print("[V2.2] Flight movement/control TX remains DISABLED")

    runtime = KingphoenixRuntime()
    try:
        runtime.start()
        runtime.wait()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
