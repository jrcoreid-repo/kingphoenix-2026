#!/usr/bin/env python3
from __future__ import annotations

import json
import signal
import socket
import sys
import time

from config import load_json
from vision.bbox_control import BboxControlEstimator
from vision.camera_manager import CameraManager
from vision.hailo_yolo26 import HailoYolo26


class DropVisionDaemon:
    """Standalone bottom-camera + Hailo perception process.

    Owns camera/Hailo only. It has no MAVLink, servo, mission, or flight-control
    authority. Latest detection state is published over localhost UDP.
    """

    def __init__(self):
        self.cfg = load_json("vision_daemon.json")
        self.system = load_json("system.json")
        self.stop = False

        camera_root = self.system["camera"]
        camera_cfg = camera_root["bottom"]
        runtime_cfg = dict(camera_root.get("runtime", {}) or {})
        output_cfg = dict(camera_cfg.get("output", {}) or {})

        self.frame_w = int(output_cfg.get("width", 1920))
        self.frame_h = int(output_cfg.get("height", 1200))

        self.camera = CameraManager(
            "bottom",
            backend=str(camera_root.get("backend", "picamera2")),
            camera_num=camera_cfg.get("camera_num"),
            sensor_mode=camera_cfg.get("sensor_mode", {}),
            output=output_cfg,
            runtime=runtime_cfg,
            width=self.frame_w,
            height=self.frame_h,
            fps=float(runtime_cfg.get("capture_target_fps", 30)),
            loop_file=False,
        )

        ai_cfg = dict(self.system["ai"])
        detector_cfg = dict(ai_cfg.get("detector", {}) or {})
        detector_cfg["confidence_threshold"] = float(
            self.cfg.get(
                "confidence_threshold",
                detector_cfg.get("confidence_threshold", 0.25),
            )
        )
        ai_cfg["detector"] = detector_cfg

        self.detector_class_id = int(
            self.cfg.get(
                "detector_class_id",
                ai_cfg.get("bottom", {}).get("detector_class_id", 0),
            )
        )
        self.model_mode = str(
            self.cfg.get("model_mode", "YOLO26_ONLY")
        ).upper()

        self.ai = HailoYolo26(
            ai_cfg,
            frame_w=self.frame_w,
            frame_h=self.frame_h,
        )

        control_cfg = dict(self.system.get("vision_control", {}) or {})
        control_cfg["confidence_min"] = float(
            self.cfg.get("confidence_threshold", 0.25)
        )
        self.estimator = BboxControlEstimator(
            "bottom",
            control_cfg,
            w=self.frame_w,
            h=self.frame_h,
        )

        ipc = self.cfg["ipc"]
        self.publish_addr = (
            str(ipc.get("host", "127.0.0.1")),
            int(ipc.get("port", 14600)),
        )
        self.publish_hz = max(1.0, float(ipc.get("publish_hz", 20.0)))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        display = self.cfg.get("display", {})
        self.display_hz = max(0.5, float(display.get("refresh_hz", 4.0)))
        self.use_ansi = bool(display.get("ansi", True)) and sys.stdout.isatty()

        self.retry_s = max(0.5, float(self.cfg.get("retry_s", 2.0)))
        self.infer_reset_after = max(
            1, int(self.cfg.get("infer_reset_after_errors", 3))
        )
        self.coarse_limit = float(
            self.cfg.get("coarse_center_error_max", 0.35)
        )
        self.fine_limit = float(
            self.cfg.get("fine_center_error_max", 0.05)
        )

        self.camera_ready = False
        self.hailo_ready = False
        self.last_error = None
        self.last_camera_retry = 0.0
        self.last_hailo_retry = 0.0
        self.infer_errors = 0

        self.frame_id = None
        self.frame_timestamp_ns = 0
        self.last_capture_ns = 0
        self.capture_fps = 0.0
        self.model_ms = 0.0
        self.model_fps = 0.0

        self.target_detected = False
        self.confidence = None
        self.bbox = None
        self.target = None
        self.ex = None
        self.ey = None
        self.bbox_area_ratio = None
        self.control_phase = "SEARCH"
        self.velocity_intent = None
        self.coarse_centered = False
        self.fine_centered = False
        self.coarse_streak = 0

        self.last_result_ns = 0
        self.last_publish_s = 0.0
        self.last_display_s = 0.0
        self.publish_count = 0

    def _signal_stop(self, *_):
        self.stop = True

    def install_signals(self):
        signal.signal(signal.SIGINT, self._signal_stop)
        signal.signal(signal.SIGTERM, self._signal_stop)

    def ensure_camera(self):
        if self.camera_ready:
            return True
        now = time.monotonic()
        if now - self.last_camera_retry < self.retry_s:
            return False
        self.last_camera_retry = now
        try:
            self.camera.close()
            self.camera.open()
            self.camera_ready = True
            self.last_error = None
            return True
        except Exception as exc:
            self.camera_ready = False
            self.last_error = f"CAMERA: {type(exc).__name__}: {exc}"
            return False

    def ensure_hailo(self):
        if self.hailo_ready and self.ai.ready:
            return True
        now = time.monotonic()
        if now - self.last_hailo_retry < self.retry_s:
            return False
        self.last_hailo_retry = now
        try:
            self.ai.close()
            self.ai.open()
            self.ai.set_route_context(
                "bottom",
                self.model_mode,
                self.detector_class_id,
            )
            self.hailo_ready = True
            self.infer_errors = 0
            self.last_error = None
            return True
        except Exception as exc:
            self.hailo_ready = False
            self.last_error = f"HAILO: {type(exc).__name__}: {exc}"
            return False

    def update_capture_fps(self, timestamp_ns):
        if self.last_capture_ns:
            dt = (timestamp_ns - self.last_capture_ns) / 1e9
            if dt > 0:
                instant = 1.0 / dt
                self.capture_fps = (
                    instant
                    if self.capture_fps <= 0
                    else 0.90 * self.capture_fps + 0.10 * instant
                )
        self.last_capture_ns = timestamp_ns

    def clear_detection(self):
        self.target_detected = False
        self.confidence = None
        self.bbox = None
        self.target = None
        self.ex = None
        self.ey = None
        self.bbox_area_ratio = None
        self.control_phase = "SEARCH"
        self.velocity_intent = None
        self.coarse_centered = False
        self.fine_centered = False
        self.coarse_streak = 0

    def process_frame(self):
        try:
            packet = self.camera.read()
        except Exception as exc:
            self.camera_ready = False
            self.last_error = f"CAMERA READ: {type(exc).__name__}: {exc}"
            try:
                self.camera.close()
            except Exception:
                pass
            return

        if packet is None:
            self.camera_ready = False
            self.last_error = "CAMERA READ: empty frame"
            try:
                self.camera.close()
            except Exception:
                pass
            return

        self.frame_id = int(packet.frame_id)
        self.frame_timestamp_ns = int(packet.timestamp_ns)
        self.update_capture_fps(packet.timestamp_ns)

        try:
            result = self.ai.process(
                packet.frame,
                policy=self.model_mode,
                detector_class_id=self.detector_class_id,
            )
            self.infer_errors = 0
            self.last_error = None
        except Exception as exc:
            self.infer_errors += 1
            self.last_error = f"INFERENCE: {type(exc).__name__}: {exc}"
            self.clear_detection()
            if self.infer_errors >= self.infer_reset_after:
                self.hailo_ready = False
                try:
                    self.ai.close()
                except Exception:
                    pass
            return

        self.model_ms = float(result.model_ms or 0.0)
        self.model_fps = float(result.model_fps or 0.0)
        self.last_result_ns = time.monotonic_ns()

        if not result.vision_valid or result.bbox is None:
            self.clear_detection()
            return

        ctrl = self.estimator.estimate(result.bbox, result.confidence)
        self.target_detected = bool(ctrl.valid)
        self.confidence = (
            None if result.confidence is None else float(result.confidence)
        )
        self.bbox = [float(x) for x in result.bbox]
        self.target = (
            None if result.target is None else [float(x) for x in result.target]
        )
        self.ex = None if ctrl.ex is None else float(ctrl.ex)
        self.ey = None if ctrl.ey is None else float(ctrl.ey)
        self.bbox_area_ratio = (
            None
            if ctrl.bbox_area_ratio is None
            else float(ctrl.bbox_area_ratio)
        )
        self.control_phase = str(ctrl.phase)
        self.velocity_intent = ctrl.intent

        if self.ex is not None and self.ey is not None:
            self.coarse_centered = (
                abs(self.ex) <= self.coarse_limit
                and abs(self.ey) <= self.coarse_limit
            )
            self.fine_centered = (
                abs(self.ex) <= self.fine_limit
                and abs(self.ey) <= self.fine_limit
            )
        else:
            self.coarse_centered = False
            self.fine_centered = False

        if self.target_detected and self.coarse_centered:
            self.coarse_streak += 1
        else:
            self.coarse_streak = 0

    def result_age_ms(self):
        if self.last_result_ns <= 0:
            return None
        return max(
            0.0,
            (time.monotonic_ns() - self.last_result_ns) / 1e6,
        )

    def packet(self):
        age_ms = self.result_age_ms()
        ready = bool(self.camera_ready and self.hailo_ready and self.ai.ready)
        return {
            "schema": "kingphoenix.vision.drop.v1",
            "sent_monotonic_ns": time.monotonic_ns(),
            "ready": ready,
            "camera_ready": bool(self.camera_ready),
            "hailo_ready": bool(self.hailo_ready and self.ai.ready),
            "model_mode": self.model_mode,
            "detector_class_id": self.detector_class_id,
            "frame_id": self.frame_id,
            "frame_timestamp_ns": self.frame_timestamp_ns,
            "result_age_ms": age_ms,
            "capture_fps": round(self.capture_fps, 3),
            "model_ms": round(self.model_ms, 3),
            "model_fps": round(self.model_fps, 3),
            "target_detected": bool(self.target_detected),
            "confidence": self.confidence,
            "bbox": self.bbox,
            "target": self.target,
            "ex": self.ex,
            "ey": self.ey,
            "bbox_area_ratio": self.bbox_area_ratio,
            "control_phase": self.control_phase,
            "velocity_intent": self.velocity_intent,
            "coarse_centered": bool(self.coarse_centered),
            "fine_centered": bool(self.fine_centered),
            "coarse_streak_frames": int(self.coarse_streak),
            "error": self.last_error,
        }

    def publish_if_due(self):
        now = time.monotonic()
        if now - self.last_publish_s < 1.0 / self.publish_hz:
            return
        self.last_publish_s = now
        payload = json.dumps(
            self.packet(),
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        try:
            self.sock.sendto(payload, self.publish_addr)
            self.publish_count += 1
        except (BlockingIOError, OSError) as exc:
            self.last_error = f"IPC: {type(exc).__name__}: {exc}"

    @staticmethod
    def fmt(value, digits=3, fallback="-"):
        if value is None:
            return fallback
        try:
            return f"{float(value):.{digits}f}"
        except Exception:
            return str(value)

    def dashboard(self):
        age = self.result_age_ms()
        target = "DETECTED" if self.target_detected else "SEARCHING"
        health = "READY" if self.camera_ready and self.hailo_ready else "WAIT"
        bbox = "-"
        if self.bbox is not None and len(self.bbox) >= 4:
            bbox = (
                f"[{self.bbox[0]:.0f},{self.bbox[1]:.0f}] "
                f"[{self.bbox[2]:.0f},{self.bbox[3]:.0f}]"
            )

        lines = [
            "============================================================",
            " KINGPHOENIX DROP VISION DAEMON",
            "============================================================",
            (
                f" HEALTH : {health:<6} | CAMERA: "
                f"{'READY' if self.camera_ready else 'WAIT':<5} | HAILO: "
                f"{'READY' if self.hailo_ready else 'WAIT':<5}"
            ),
            (
                f" MODEL  : {self.model_mode:<12} | CLASS: {self.detector_class_id} "
                f"| IPC: {self.publish_addr[0]}:{self.publish_addr[1]}"
            ),
            (
                f" FPS    : CAP {self.capture_fps:5.1f} | AI {self.model_fps:5.1f} "
                f"| MODEL {self.model_ms:5.1f} ms | AGE "
                f"{'-' if age is None else f'{age:.0f} ms'}"
            ),
            "------------------------------------------------------------",
            f" TARGET : {target:<9} | CONF: {self.fmt(self.confidence, 2)}",
            f" BBOX   : {bbox}",
            (
                f" ERROR  : ex={self.fmt(self.ex, 3):>7} "
                f"ey={self.fmt(self.ey, 3):>7} "
                f"area={self.fmt(self.bbox_area_ratio, 4)}"
            ),
            (
                f" CENTER : COARSE={'YES' if self.coarse_centered else 'NO ':<3} "
                f"(±{self.coarse_limit:.2f}) | "
                f"FINE={'YES' if self.fine_centered else 'NO ':<3} "
                f"(±{self.fine_limit:.2f}) | "
                f"STREAK={self.coarse_streak}"
            ),
            f" PHASE  : {self.control_phase}",
            "------------------------------------------------------------",
            f" IPC TX : {self.publish_count} packets",
            f" ERROR  : {self.last_error or '-'}",
            "============================================================",
            " Ctrl+C to stop",
        ]
        return "\n".join(lines)

    def display_if_due(self):
        now = time.monotonic()
        if now - self.last_display_s < 1.0 / self.display_hz:
            return
        self.last_display_s = now
        text = self.dashboard()
        if self.use_ansi:
            sys.stdout.write("\033[2J\033[H" + text + "\n")
            sys.stdout.flush()
        else:
            print(
                f"[VISION] CAM={'OK' if self.camera_ready else 'WAIT'} "
                f"HAILO={'OK' if self.hailo_ready else 'WAIT'} "
                f"TARGET={'YES' if self.target_detected else 'NO'} "
                f"CONF={self.fmt(self.confidence, 2)} "
                f"ex={self.fmt(self.ex, 3)} ey={self.fmt(self.ey, 3)} "
                f"AI={self.model_fps:.1f}fps "
                f"ERR={self.last_error or '-'}"
            )

    def close(self):
        try:
            self.camera.close()
        except Exception:
            pass
        try:
            self.ai.close()
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass

    def run(self):
        self.install_signals()
        print("[VISION] starting standalone bottom-camera drop daemon")
        print("[VISION] no MAVLink / servo / mission authority")

        try:
            while not self.stop:
                camera_ok = self.ensure_camera()
                hailo_ok = self.ensure_hailo()

                if camera_ok and hailo_ok:
                    self.process_frame()
                else:
                    self.clear_detection()
                    time.sleep(0.03)

                self.publish_if_due()
                self.display_if_due()
        finally:
            self.close()
            print("\n[VISION] daemon stopped")


def main():
    DropVisionDaemon().run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
