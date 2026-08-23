from __future__ import annotations

from pathlib import Path

import threading
import time

from streaming.barcode import draw_frame_id_barcode
from streaming.watermark import draw_test_watermark, watermark_text
from streaming.metadata_streamer import MetadataStreamer, build_camera_metadata
from streaming.video_streamer import VideoStreamer
from vision.camera_manager import CameraManager
from vision.ai_router import AiRouter
from vision.hailo_yolo26 import HailoYolo26
from vision.ai_test_switcher import AiTestSwitcher
from vision.bbox_control import BboxControlEstimator


class CameraStreamWorker(threading.Thread):
    daemon = True

    def __init__(
        self,
        camera_name,
        source,
        stop_event,
        state,
        destination_getter,
        video_port_key,
        metadata_port_key,
        *,
        width=640,
        height=640,
        fps=30,
        bitrate_kbps=3000,
        keyint=30,
        ai_router=None,
        ai_task="--",
        ai_engine=None,
        pose_solver=None,
        run_mode_cfg=None,
        run_profile="UNKNOWN",
        vision_control_cfg=None,
        loop_file=True,
    ):
        super().__init__(name=f"VisionStream-{camera_name}")
        self.camera_name = str(camera_name).lower()
        self.stop_event = stop_event
        self.state = state
        self.destination_getter = destination_getter
        self.video_port_key = video_port_key
        self.metadata_port_key = metadata_port_key
        self.fps = float(fps)
        self.ai_router = ai_router
        self.ai_task = str(ai_task).upper()
        self.ai_engine = ai_engine
        self.pose_solver = pose_solver
        self.ai_error = None
        self.run_mode_cfg = run_mode_cfg or {}
        self.run_profile = str(run_profile).upper()
        self.bbox_estimator = BboxControlEstimator(
            self.camera_name, vision_control_cfg or {}, w=1920, h=1200
        )

        if isinstance(source, dict):
            camera_source_cfg = dict(source)
            self.camera = CameraManager(
                self.camera_name,
                backend=camera_source_cfg.get("backend", "picamera2"),
                camera_num=camera_source_cfg.get("camera_num"),
                sensor_mode=camera_source_cfg.get("sensor_mode", {}),
                output=camera_source_cfg.get("output", {}),
                runtime=camera_source_cfg.get("runtime", {}),
                width=width,
                height=height,
                fps=fps,
                loop_file=loop_file,
            )
        else:
            self.camera = CameraManager(
                self.camera_name,
                source,
                backend="opencv",
                width=width,
                height=height,
                fps=fps,
                loop_file=loop_file,
            )

        self.video = None
        self.meta = None
        self.width = width
        self.height = height
        self.bitrate_kbps = bitrate_kbps
        self.keyint = keyint

        self._last_frame_ns = 0
        self._fps_ema = 0.0
        self._last_diag_s = 0.0

    @staticmethod
    def _letterbox_frame(frame, out_w, out_h):
        import cv2
        import numpy as np
        src_h, src_w = frame.shape[:2]
        scale = min(out_w/src_w, out_h/src_h)
        new_w = max(1, int(round(src_w*scale)))
        new_h = max(1, int(round(src_h*scale)))
        resized = cv2.resize(
            frame, (new_w,new_h),
            interpolation=cv2.INTER_AREA,
        )
        canvas = np.full((out_h,out_w,3),114,dtype=np.uint8)
        left = (out_w-new_w)//2
        top = (out_h-new_h)//2
        canvas[top:top+new_h,left:left+new_w] = resized
        return canvas, {
            "scale":float(scale),
            "left":float(left),
            "top":float(top),
            "src_w":int(src_w),
            "src_h":int(src_h),
            "dst_w":int(out_w),
            "dst_h":int(out_h),
        }

    @staticmethod
    def _map_point_to_display(point, t):
        if point is None or len(point)<2:
            return point
        x=float(point[0])*t["scale"]+t["left"]
        y=float(point[1])*t["scale"]+t["top"]
        if len(point)>=3:
            return [round(x,2),round(y,2),point[2]]
        return [round(x,2),round(y,2)]

    @classmethod
    def _metadata_for_display(cls, metadata, t):
        out=dict(metadata)
        if out.get("keypoints") is not None:
            out["keypoints"]=[
                cls._map_point_to_display(p,t)
                for p in out["keypoints"]
            ]
        bbox=out.get("bbox")
        if bbox is not None and len(bbox)>=4:
            p1=cls._map_point_to_display([bbox[0],bbox[1]],t)
            p2=cls._map_point_to_display([bbox[2],bbox[3]],t)
            out["bbox"]=[p1[0],p1[1],p2[0],p2[1]]
        if out.get("target") is not None:
            out["target"]=cls._map_point_to_display(out["target"],t)
        out["coordinate_space"]="DISPLAY_640x400_WIDE"
        out["canonical_coordinate_space"]="CAMERA_1920x1200"
        return out

    def _ensure_destination(self):
        endpoint = self.destination_getter()
        if not endpoint:
            return None

        ip = endpoint["ip"]
        vport = int(endpoint[self.video_port_key])
        mport = int(endpoint[self.metadata_port_key])

        if self.video is None:
            self.video = VideoStreamer(
                ip,
                vport,
                width=self.width,
                height=self.height,
                fps=self.fps,
                bitrate_kbps=self.bitrate_kbps,
                keyint=self.keyint,
            )
        else:
            self.video.set_destination(ip, vport)

        if self.meta is None:
            self.meta = MetadataStreamer(ip, mport)
        else:
            self.meta.set_destination(ip, mport)

        return endpoint

    def _update_fps(self, now_ns):
        if self._last_frame_ns:
            dt = (now_ns - self._last_frame_ns) / 1e9
            if dt > 0:
                inst = 1.0 / dt
                self._fps_ema = inst if self._fps_ema <= 0 else (
                    0.90 * self._fps_ema + 0.10 * inst
                )
        self._last_frame_ns = now_ns

    def _telemetry_snapshot(self):
        v = self.state.vehicle
        with self.state.lock:
            pi_health = dict(self.state.system.get("pi_health") or {})
        return {
            "connected": bool(v.connected),
            "mode": v.mode,
            "armed": bool(v.armed),
            "battery_v": v.battery_v,
            "relative_alt_m": v.relative_alt_m,
            "rangefinder_m": v.rangefinder_distance_m,
            "pi_health": pi_health,
        }

    def _system_metadata_snapshot(self):
        with self.state.lock:
            health = dict(self.state.system.get("pi_health") or {})
        return {
            "pi_health": health,
            "pi_health_status": health.get("status", "UNKNOWN"),
            "cpu_temp": health.get("cpu_temp_c"),
            "cpu_load_pct": health.get("cpu_load_pct"),
            "ram_used_pct": health.get("ram_used_pct"),
            "disk_free_gb": health.get("disk_free_gb"),
            "throttled_hex": health.get("throttled_hex"),
            "undervoltage_now": bool(health.get("undervoltage_now", False)),
            "throttling_now": bool(health.get("throttling_now", False)),
            "rangefinder_m": self.state.vehicle.rangefinder_distance_m,
        }

    def run(self):
        interval = 1.0 / max(self.fps, 1.0)

        try:
            self.camera.open()
            print(f"[VISION:{self.camera_name}] source opened: {self.camera.source}")

        except Exception as exc:
            print(f"[VISION:{self.camera_name}] source error: {exc}")
            self.state.vision[self.camera_name]["video_state"] = "OFFLINE"
            return

        try:
            while not self.stop_event.is_set():
                start = time.monotonic()

                if not self.state.system.get("gcs_connected", False):
                    time.sleep(0.05)
                    continue

                if self._ensure_destination() is None:
                    time.sleep(0.05)
                    continue

                packet = self.camera.read()
                if packet is None:
                    self.state.vision[self.camera_name]["video_state"] = "OFFLINE"
                    time.sleep(0.05)
                    continue

                self._update_fps(packet.timestamp_ns)

                canonical_frame = packet.frame
                frame, display_transform = self._letterbox_frame(
                    canonical_frame,
                    self.width,
                    self.height,
                )
                draw_test_watermark(
                    frame,
                    watermark_text(
                        self.run_mode_cfg,
                        self.run_profile,
                    ),
                )
                draw_frame_id_barcode(frame, packet.frame_id)

                route = (
                    self.ai_router.snapshot()
                    if self.ai_router is not None
                    else None
                )
                ai_state = (
                    self.ai_router.state_for(self.camera_name)
                    if self.ai_router is not None
                    else "STANDBY"
                )
                route_model_mode = (
                    "OFF"
                    if route is None or route.camera != self.camera_name
                    else route.model_mode
                )
                route_task = (
                    self.ai_task
                    if route is None or route.camera != self.camera_name
                    else route.task
                )
                route_class_id = (
                    None
                    if route is None or route.camera != self.camera_name
                    else route.detector_class_id
                )

                ai_extra = {
                    "mode": "STREAM_ONLY",
                    "ai_model": None,
                    "model_mode": route_model_mode,
                    "detector_class_id": route_class_id,
                    "target": None,
                    "bbox": None,
                    "keypoints": None,
                    "confidence": None,
                    "conf": None,
                    "vision_valid": False,
                    "vision_error": self.ai_error,
                    "model_ms": 0.0,
                    "model_fps": 0.0,
                    "xyz": None,
                    "rpy": None,
                    "overlay_type": "NONE",
                    "pose_valid": False,
                    "detector_valid": False,
                    "pose_3d_valid": False,
                    "distance_m": None,
                    "rvec": None,
                    "pose_source": "NONE",
                    "metric_pose_usable": False,
                    "reprojection_error_px": None,
                    "pose_error": None,
                }
                vision_mode = "STREAM_ONLY"

                if (
                    ai_state == "ACTIVE"
                    and route_model_mode != "OFF"
                    and self.ai_engine is not None
                    and self.ai_engine.ready
                ):
                    try:
                        self.ai_engine.set_route_context(
                            self.camera_name,
                            route_model_mode,
                            route_class_id,
                        )
                        ai_result = self.ai_engine.process(
                            canonical_frame,
                            policy=route_model_mode,
                            detector_class_id=route_class_id,
                        )
                        ai_extra.update(ai_result.as_metadata())
                        ctrl = self.bbox_estimator.estimate(
                            ai_result.bbox, ai_result.confidence
                        )
                        ai_extra["bbox_control"] = ctrl.as_dict()
                        ai_extra["ex"] = ctrl.ex
                        ai_extra["ey"] = ctrl.ey
                        ai_extra["bbox_area_ratio"] = ctrl.bbox_area_ratio
                        ai_extra["control_phase"] = ctrl.phase
                        ai_extra["velocity_intent"] = ctrl.intent
                        ai_extra["model_mode"] = route_model_mode
                        ai_extra["detector_class_id"] = route_class_id
                        vision_mode = str(ai_result.vision_mode).upper()

                        # V1.9.2.1 explicit GCS overlay contract.
                        # This avoids the GUI having to infer overlay type from
                        # model names or transient router state.
                        if (
                            vision_mode == "POSE"
                            and ai_result.keypoints is not None
                            and len(ai_result.keypoints) >= 4
                        ):
                            ai_extra["overlay_type"] = "POSE"
                            ai_extra["pose_valid"] = True
                            ai_extra["detector_valid"] = False
                        elif (
                            vision_mode == "YOLO26"
                            and ai_result.bbox is not None
                            and len(ai_result.bbox) >= 4
                        ):
                            ai_extra["overlay_type"] = "DETECT"
                            ai_extra["pose_valid"] = False
                            ai_extra["detector_valid"] = True
                        else:
                            ai_extra["overlay_type"] = "NONE"
                            ai_extra["pose_valid"] = False
                            ai_extra["detector_valid"] = False

                        self.ai_error = None

                        # V1.9.3: solve 3D pose from the exact-frame
                        # 4-keypoint result. TEST_ESTIMATE is draw/debug only;
                        # REAL_CALIBRATION is the only metric pose allowed for
                        # future movement.
                        if (
                            self.camera_name == "front"
                            and vision_mode == "POSE"
                            and ai_result.keypoints is not None
                            and len(ai_result.keypoints) >= 4
                            and self.pose_solver is not None
                        ):
                            pose3d = self.pose_solver.solve(
                                ai_result.keypoints
                            )
                            ai_extra.update(
                                pose3d.as_metadata()
                            )
                    except Exception as ai_exc:
                        self.ai_error = repr(ai_exc)
                        ai_extra["vision_error"] = self.ai_error
                        vision_mode = "SEARCH"
                        print(
                            f"[AI:{self.camera_name}] "
                            f"inference error: {ai_exc}"
                        )

                if self.ai_error is not None and ai_state == "ACTIVE":
                    ai_state_for_meta = "DISABLED"
                else:
                    ai_state_for_meta = ai_state

                system_extra = self._system_metadata_snapshot()
                system_extra.update(ai_extra)

                metadata = build_camera_metadata(
                    camera=self.camera_name,
                    frame_id=packet.frame_id,
                    timestamp_ns=packet.timestamp_ns,
                    ai_state=ai_state_for_meta,
                    ai_task=route_task,
                    video_state="LIVE",
                    vision_mode=vision_mode,
                    fps=self._fps_ema,
                    telemetry=self._telemetry_snapshot(),
                    extra=system_extra,
                )
                display_metadata = self._metadata_for_display(
                    metadata,
                    display_transform,
                )

                # Same frame_id is used in the image barcode and metadata.
                video_ok = self.video.write(frame)
                meta_ok = False
                if video_ok:
                    meta_ok = self.meta.send(display_metadata)

                self.state.update_vision(
                    self.camera_name,
                    video_state="LIVE" if video_ok else "OFFLINE",
                    ai_state=ai_state_for_meta,
                    ai_task=route_task,
                    model_mode=route_model_mode,
                    detector_class_id=route_class_id,
                    vision_mode=metadata.get("vision_mode"),
                    ai_model=metadata.get("ai_model"),
                    vision_valid=bool(metadata.get("vision_valid")),
                    vision_timestamp_ns=(
                        packet.timestamp_ns
                        if metadata.get("vision_valid")
                        else 0
                    ),
                    frame_timestamp_ns=packet.timestamp_ns,
                    vision_error=metadata.get("vision_error"),
                    model_ms=float(metadata.get("model_ms") or 0.0),
                    confidence=metadata.get("confidence"),
                    target=metadata.get("target"),
                    bbox=metadata.get("bbox"),
                    keypoints=metadata.get("keypoints"),
                    xyz=metadata.get("xyz"),
                    rpy=metadata.get("rpy"),
                    distance_m=metadata.get("distance_m"),
                    pose_3d_valid=bool(metadata.get("pose_3d_valid", False)),
                    pose_source=metadata.get("pose_source", "NONE"),
                    metric_pose_usable=bool(
                        metadata.get("metric_pose_usable", False)
                    ),
                    reprojection_error_px=metadata.get(
                        "reprojection_error_px"
                    ),
                    overlay_type=metadata.get("overlay_type", "NONE"),
                    pose_valid=bool(metadata.get("pose_valid", False)),
                    detector_valid=bool(metadata.get("detector_valid", False)),
                    bbox_control=metadata.get("bbox_control"),
                    ex=metadata.get("ex"),
                    ey=metadata.get("ey"),
                    bbox_area_ratio=metadata.get("bbox_area_ratio"),
                    control_phase=metadata.get("control_phase"),
                    velocity_intent=metadata.get("velocity_intent"),
                    frame_id=packet.frame_id,
                    fps=self._fps_ema,
                    metadata_tx_ok=bool(meta_ok),
                    metadata_tx_count=(
                        0 if self.meta is None else self.meta.sent_packets
                    ),
                    metadata_tx_error=(
                        None if self.meta is None else self.meta.last_error
                    ),
                )

                now_s = time.monotonic()
                if now_s - self._last_diag_s >= 2.0:
                    self._last_diag_s = now_s
                    print(
                        f"[META-TX:{self.camera_name}] "
                        f"dst={self.meta.destination_ip}:{self.meta.port} "
                        f"ok={meta_ok} packets={self.meta.sent_packets} "
                        f"error={self.meta.last_error or '--'} "
                        f"AI={metadata.get('ai_model') or '--'} "
                        f"mode={metadata.get('vision_mode') or '--'} "
                        f"overlay={metadata.get('overlay_type') or '--'} "
                        f"kp={0 if metadata.get('keypoints') is None else len(metadata.get('keypoints'))} "
                        f"bbox={'Y' if metadata.get('bbox') is not None else 'N'} "
                        f"P3D={metadata.get('pose_3d_valid', False)} "
                        f"SRC={metadata.get('pose_source', 'NONE')} "
                        f"Z={None if metadata.get('xyz') is None else metadata.get('xyz')[2]}"
                    )

                elapsed = time.monotonic() - start
                remaining = interval - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        finally:
            self.camera.close()
            if self.video is not None:
                self.video.stop()
            if self.meta is not None:
                self.meta.close()


class VisionStreamSession:
    def __init__(self, cfg, state, stop_event):
        project_root = Path(__file__).resolve().parents[1]
        self.cfg = cfg
        self.state = state
        self.stop_event = stop_event

        camera_cfg = cfg["camera"]
        stream_cfg = cfg["stream"]

        camera_backend = str(
            camera_cfg.get("backend", "opencv")
        ).lower()
        camera_runtime = camera_cfg.get("runtime", {})

        if camera_backend == "picamera2":
            front_source = dict(camera_cfg["front"])
            front_source["backend"] = "picamera2"
            front_source["runtime"] = camera_runtime

            bottom_source = dict(camera_cfg["bottom"])
            bottom_source["backend"] = "picamera2"
            bottom_source["runtime"] = camera_runtime
        else:
            fallback = camera_cfg.get(
                "legacy_file_fallback",
                camera_cfg,
            )
            front_source = fallback.get("front_source", "Gate6.mp4")
            bottom_source = fallback.get("bottom_source", "Gate6.mp4")

        ai_cfg = cfg.get("ai", {})
        route_cfg = ai_cfg.get("route", {})
        active_camera = str(
            route_cfg.get(
                "camera",
                ai_cfg.get("active_camera", "front"),
            )
        ).lower()

        self.ai_router = AiRouter(
            active=active_camera,
            model_mode=route_cfg.get("model_mode", "HYBRID"),
            task=route_cfg.get("task", "GATE_NAV"),
            detector_class_id=route_cfg.get("detector_class_id", 1),
            source=route_cfg.get("source", "CONFIG"),
        )
        self.state.set_ai_route(
            self.ai_router.snapshot().as_dict()
        )

        self.shared_ai = None
        if bool(ai_cfg.get("enabled", True)):
            canonical_w = int(
                camera_cfg.get("runtime", {}).get(
                    "canonical_width", 1280
                )
            )
            canonical_h = int(
                camera_cfg.get("runtime", {}).get(
                    "canonical_height", 720
                )
            )
            self.shared_ai = HailoYolo26(
                ai_cfg,
                frame_w=canonical_w,
                frame_h=canonical_h,
            )

        self.test_switcher = None
        test_switch_cfg = ai_cfg.get("test_switch", {})
        if bool(test_switch_cfg.get("enabled", False)):
            self.test_switcher = AiTestSwitcher(
                self.ai_router,
                self.state,
                stop_event,
                test_switch_cfg,
            )

        run_mode_cfg = cfg.get("run_mode", {})
        self.run_profile = str(
            run_mode_cfg.get("profile", "VISION_AI_TEST")
        ).upper()
        self.state.system["run_profile"] = self.run_profile

        # V2 normal runtime intentionally has no pose solver.
        self.front_pose_solver = None

        self.state.system.setdefault("gcs_connected", False)
        self.state.system.setdefault("gcs_endpoint", None)

        def endpoint():
            return self.state.system.get("gcs_endpoint")

        self.front = CameraStreamWorker(
            "front",
            front_source,
            stop_event,
            state,
            endpoint,
            "front_video_port",
            "front_metadata_port",
            width=stream_cfg.get("width", 640),
            height=stream_cfg.get("height", 640),
            fps=stream_cfg.get("fps", 30),
            bitrate_kbps=stream_cfg.get("front_bitrate_kbps", 3000),
            keyint=stream_cfg.get("keyint", 30),
            ai_router=self.ai_router,
            ai_task=ai_cfg.get("front", {}).get(
                "task",
                camera_cfg.get("front", {}).get("ai_task", "GATE_NAV"),
            ),
            ai_engine=self.shared_ai,
            pose_solver=self.front_pose_solver,
            run_mode_cfg=run_mode_cfg,
            run_profile=self.run_profile,
            vision_control_cfg=cfg.get("vision_control", {}),
            loop_file=camera_cfg.get("legacy_file_fallback", {}).get("loop_file_sources", True),
        )

        self.bottom = CameraStreamWorker(
            "bottom",
            bottom_source,
            stop_event,
            state,
            endpoint,
            "bottom_video_port",
            "bottom_metadata_port",
            width=stream_cfg.get("width", 640),
            height=stream_cfg.get("height", 640),
            fps=stream_cfg.get("fps", 30),
            bitrate_kbps=stream_cfg.get("bottom_bitrate_kbps", 2500),
            keyint=stream_cfg.get("keyint", 30),
            ai_router=self.ai_router,
            ai_task=ai_cfg.get("bottom", {}).get(
                "task",
                camera_cfg.get("bottom", {}).get("ai_task", "DROP_ALIGNMENT"),
            ),
            ai_engine=self.shared_ai,
            run_mode_cfg=run_mode_cfg,
            run_profile=self.run_profile,
            vision_control_cfg=cfg.get("vision_control", {}),
            loop_file=camera_cfg.get("legacy_file_fallback", {}).get("loop_file_sources", True),
        )

        self.workers = [self.front, self.bottom]

    def start(self):
        if self.shared_ai is not None:
            try:
                self.shared_ai.open()
            except Exception as exc:
                print(f"[AI] shared Hailo DISABLED: {exc}")

        for worker in self.workers:
            worker.start()

        if self.test_switcher is not None:
            self.test_switcher.start()

    def set_ai_route(
        self,
        camera,
        *,
        model_mode,
        task,
        detector_class_id=None,
        source="MISSION",
    ):
        """Public mission-ready AI switching API."""
        route = self.ai_router.set_route(
            camera,
            model_mode=model_mode,
            task=task,
            detector_class_id=detector_class_id,
            source=source,
        )
        self.state.set_ai_route(route.as_dict())
        return route

    def disable_ai(self, source="MISSION"):
        route = self.ai_router.disable(source=source)
        self.state.set_ai_route(route.as_dict())
        return route

    def join(self, timeout=1.0):
        for worker in self.workers:
            worker.join(timeout=timeout)

        if self.test_switcher is not None:
            self.test_switcher.join(timeout=timeout)

        if self.shared_ai is not None:
            try:
                self.shared_ai.close()
            except Exception:
                pass
