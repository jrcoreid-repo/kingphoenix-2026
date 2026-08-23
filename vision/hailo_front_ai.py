from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np


@dataclass
class FrontAiResult:
    vision_mode: str = "SEARCH"
    ai_model: str = "NONE"
    confidence: float | None = None
    keypoints: list | None = None
    bbox: list | None = None
    target: list | None = None
    model_ms: float = 0.0
    model_fps: float = 0.0
    vision_valid: bool = False
    error: str | None = None

    def as_metadata(self):
        return {
            "mode": self.vision_mode,
            "vision_mode": self.vision_mode,
            "ai_model": self.ai_model,
            "confidence": self.confidence,
            "conf": self.confidence,
            "keypoints": self.keypoints,
            "bbox": self.bbox,
            "target": self.target,
            "model_ms": round(float(self.model_ms), 3),
            "model_fps": round(float(self.model_fps), 3),
            "vision_valid": bool(self.vision_valid),
            "vision_error": self.error,
            # Pose X/Y/Z and R/P/Y are intentionally disabled until the
            # configured front camera calibration is marked valid.
            "xyz": None,
            "rpy": None,
        }


def _sigmoid(x):
    x = np.asarray(x, dtype=np.float32)
    return 1.0 / (1.0 + np.exp(-np.clip(x, -60.0, 60.0)))


def _softmax(x):
    x = np.asarray(x, dtype=np.float32)
    x = x - np.max(x, axis=-1, keepdims=True)
    exp = np.exp(x)
    return exp / np.maximum(np.sum(exp, axis=-1, keepdims=True), 1e-9)


def _nms_xyxy(boxes, scores, threshold):
    if len(boxes) == 0:
        return []

    boxes = np.asarray(boxes, dtype=np.float32)
    scores = np.asarray(scores, dtype=np.float32)

    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        union = areas[i] + areas[order[1:]] - inter + 1e-9
        iou = inter / union

        remaining = np.where(iou <= float(threshold))[0]
        order = order[remaining + 1]

    return keep


class _HailoModelSwitcher:
    def __init__(self, pose_group, detector_group):
        self.pose_group = pose_group
        self.detector_group = detector_group
        self.stack = ExitStack()
        self.active = None

    def switch(self, model):
        if self.active == model:
            return

        self.stack.close()
        self.stack = ExitStack()

        if model == "POSE":
            self.stack.enter_context(self.pose_group.activate())
        elif model == "YOLO26":
            self.stack.enter_context(self.detector_group.activate())
        else:
            raise ValueError(model)

        self.active = model

    def close(self):
        self.stack.close()
        self.active = None


class FrontHailoAi:
    """Front gate AI: YOLOv8 Pose primary, YOLO26 fallback.

    This is a modularization of the previously proven Kingphoenix Hailo
    inference/post-processing path. It owns NO MAVLink and cannot move the
    aircraft.
    """

    REG_MAX = 15

    def __init__(self, cfg, frame_w=640, frame_h=640):
        self.cfg = cfg
        self.frame_w = int(frame_w)
        self.frame_h = int(frame_h)
        self.model_size = int(cfg.get("model_size", 640))

        front_cfg = cfg["front"]
        primary = front_cfg["primary"]
        fallback = front_cfg["fallback"]

        self.pose_hef_path = str(primary["hef"])
        self.detector_hef_path = str(fallback["hef"])

        self.num_keypoints = int(primary.get("num_keypoints", 4))
        self.pose_conf = float(primary.get("confidence_threshold", 0.25))
        self.keypoint_conf = float(
            primary.get("keypoint_confidence_threshold", 0.20)
        )
        self.pose_iou = float(primary.get("iou_threshold", 0.60))

        self.det_conf = float(fallback.get("confidence_threshold", 0.25))
        self.det_iou = float(fallback.get("iou_threshold", 0.45))
        self.gate_class_id = int(fallback.get("gate_class_id", 1))

        self.pose_lost_timeout_s = (
            float(front_cfg.get("pose_lost_timeout_ms", 200)) / 1000.0
        )
        self.pose_probe_interval_s = (
            float(front_cfg.get("pose_probe_interval_ms", 100)) / 1000.0
        )
        self.fallback_bbox_max_age_s = (
            float(front_cfg.get("fallback_bbox_max_age_ms", 250)) / 1000.0
        )

        self.device = None
        self.pose_group = None
        self.detector_group = None
        self.pose_pipeline = None
        self.detector_pipeline = None
        self.switcher = None

        self._pose_context = None
        self._detector_context = None

        self.state = "POSE"
        self.pose_lost_since = None
        self.last_pose_probe = 0.0
        self.last_detection = None
        self.last_detection_time = 0.0
        self.ready = False
        self.route_context = None

    def open(self):
        if self.ready:
            return

        for path in (self.pose_hef_path, self.detector_hef_path):
            if not Path(path).exists():
                raise FileNotFoundError(path)

        # Lazy import keeps no-Hailo development tools runnable elsewhere.
        from hailo_platform import (
            VDevice,
            HEF,
            ConfigureParams,
            InputVStreamParams,
            OutputVStreamParams,
            InferVStreams,
            HailoStreamInterface,
            FormatType,
        )

        pose_hef = HEF(self.pose_hef_path)
        detector_hef = HEF(self.detector_hef_path)

        self.device = VDevice()

        pose_config = ConfigureParams.create_from_hef(
            pose_hef,
            interface=HailoStreamInterface.PCIe,
        )
        self.pose_group = self.device.configure(pose_hef, pose_config)[0]
        pose_input = InputVStreamParams.make(self.pose_group)
        pose_output = OutputVStreamParams.make(
            self.pose_group,
            format_type=FormatType.FLOAT32,
        )

        detector_config = ConfigureParams.create_from_hef(
            detector_hef,
            interface=HailoStreamInterface.PCIe,
        )
        self.detector_group = self.device.configure(
            detector_hef,
            detector_config,
        )[0]
        detector_input = InputVStreamParams.make(self.detector_group)
        detector_output = OutputVStreamParams.make(
            self.detector_group,
            format_type=FormatType.FLOAT32,
        )

        self._pose_context = InferVStreams(
            self.pose_group,
            pose_input,
            pose_output,
        )
        self.pose_pipeline = self._pose_context.__enter__()

        self._detector_context = InferVStreams(
            self.detector_group,
            detector_input,
            detector_output,
        )
        self.detector_pipeline = self._detector_context.__enter__()

        self.switcher = _HailoModelSwitcher(
            self.pose_group,
            self.detector_group,
        )
        self.switcher.switch("POSE")

        self.state = "POSE"
        self.pose_lost_since = None
        self.last_pose_probe = 0.0
        self.last_detection = None
        self.last_detection_time = 0.0
        self.ready = True

        print("[AI:front] Hailo READY")
        print(f"[AI:front] primary : {self.pose_hef_path}")
        print(f"[AI:front] fallback: {self.detector_hef_path}")

    def close(self):
        if self.switcher is not None:
            try:
                self.switcher.close()
            except Exception:
                pass
            self.switcher = None

        if self._detector_context is not None:
            try:
                self._detector_context.__exit__(None, None, None)
            except Exception:
                pass
            self._detector_context = None
            self.detector_pipeline = None

        if self._pose_context is not None:
            try:
                self._pose_context.__exit__(None, None, None)
            except Exception:
                pass
            self._pose_context = None
            self.pose_pipeline = None

        if self.device is not None:
            try:
                close_fn = getattr(self.device, "close", None)
                if callable(close_fn):
                    close_fn()
            except Exception:
                pass
            self.device = None

        self.ready = False

    def _preprocess(self, frame):
        import cv2

        if frame is None:
            raise ValueError("AI preprocess received None frame")
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(
                f"AI preprocess expected HxWx3 BGR frame, got {frame.shape}"
            )

        src_h, src_w = frame.shape[:2]
        model_w = self.model_size
        model_h = self.model_size

        scale = min(model_w / src_w, model_h / src_h)
        resized_w = max(1, int(round(src_w * scale)))
        resized_h = max(1, int(round(src_h * scale)))

        resized = cv2.resize(
            frame,
            (resized_w, resized_h),
            interpolation=cv2.INTER_LINEAR,
        )

        pad_left = (model_w - resized_w) // 2
        pad_right = model_w - resized_w - pad_left
        pad_top = (model_h - resized_h) // 2
        pad_bottom = model_h - resized_h - pad_top

        letterboxed = cv2.copyMakeBorder(
            resized,
            pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        self._letterbox = {
            "src_w": float(src_w),
            "src_h": float(src_h),
            "scale": float(scale),
            "pad_left": float(pad_left),
            "pad_top": float(pad_top),
        }

        rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(
            np.expand_dims(rgb, axis=0).astype(np.uint8)
        )

    def _model_xy_to_frame(self, x, y):
        t = getattr(self, "_letterbox", None)
        if not t:
            return float(x), float(y)
        scale = max(float(t["scale"]), 1e-9)
        fx = (float(x) - float(t["pad_left"])) / scale
        fy = (float(y) - float(t["pad_top"])) / scale
        fx = min(max(fx, 0.0), float(t["src_w"]) - 1.0)
        fy = min(max(fy, 0.0), float(t["src_h"]) - 1.0)
        return fx, fy

    def _map_pose_to_frame(self, pose):
        if pose is None:
            return None
        mapped = dict(pose)
        points = np.asarray(pose["keypoints"], dtype=np.float32).copy()
        for i in range(points.shape[0]):
            x, y = self._model_xy_to_frame(points[i,0], points[i,1])
            points[i,0] = x
            points[i,1] = y
        mapped["keypoints"] = points
        if "box" in mapped:
            box = np.asarray(mapped["box"], dtype=np.float32).copy()
            x1,y1 = self._model_xy_to_frame(box[0],box[1])
            x2,y2 = self._model_xy_to_frame(box[2],box[3])
            mapped["box"] = np.asarray([x1,y1,x2,y2], dtype=np.float32)
        return mapped

    def _map_detection_to_frame(self, det):
        if det is None:
            return None
        mapped = dict(det)
        x1,y1 = self._model_xy_to_frame(det["x1"],det["y1"])
        x2,y2 = self._model_xy_to_frame(det["x2"],det["y2"])
        mapped["x1"],mapped["y1"] = x1,y1
        mapped["x2"],mapped["y2"] = x2,y2
        return mapped

    @staticmethod
    def _find_output_by_shape(outputs, grid, channels):
        expected = (1, grid, grid, channels)
        for _, tensor in outputs.items():
            if tuple(tensor.shape) == expected:
                return tensor

        available = {
            name: tuple(tensor.shape)
            for name, tensor in outputs.items()
        }
        raise RuntimeError(
            f"missing Hailo tensor {expected}; available={available}"
        )

    def _decode_pose_boxes(self, tensor, stride):
        batch, h, w, _ = tensor.shape
        tensor = tensor.reshape(
            batch,
            h * w,
            4,
            self.REG_MAX + 1,
        )
        probability = _softmax(tensor)
        bins = np.arange(self.REG_MAX + 1, dtype=np.float32)
        distance = np.sum(probability * bins, axis=-1) * stride

        grid_y, grid_x = np.meshgrid(
            np.arange(h),
            np.arange(w),
            indexing="ij",
        )
        center_x = (grid_x.reshape(-1) + 0.5) * stride
        center_y = (grid_y.reshape(-1) + 0.5) * stride

        left = distance[0, :, 0]
        top = distance[0, :, 1]
        right = distance[0, :, 2]
        bottom = distance[0, :, 3]

        return np.stack(
            [
                center_x - left,
                center_y - top,
                center_x + right,
                center_y + bottom,
            ],
            axis=1,
        )

    def _decode_pose_keypoints(self, tensor, stride):
        batch, h, w, channels = tensor.shape
        expected = self.num_keypoints * 3
        if channels != expected:
            raise RuntimeError(
                f"expected {expected} pose keypoint channels, got {channels}"
            )

        raw = tensor.reshape(
            batch,
            h * w,
            self.num_keypoints,
            3,
        )

        grid_y, grid_x = np.meshgrid(
            np.arange(h),
            np.arange(w),
            indexing="ij",
        )

        center_x = (
            (grid_x.reshape(1, h * w, 1) + 0.5) * stride
        )
        center_y = (
            (grid_y.reshape(1, h * w, 1) + 0.5) * stride
        )

        x = stride * ((raw[..., 0] * 2.0) - 0.5) + center_x
        y = stride * ((raw[..., 1] * 2.0) - 0.5) + center_y
        confidence = _sigmoid(raw[..., 2])

        return np.stack([x, y, confidence], axis=-1)[0]

    def _decode_pose(self, outputs):
        all_boxes = []
        all_scores = []
        all_keypoints = []

        for grid, stride in ((80, 8), (40, 16), (20, 32)):
            bbox_tensor = self._find_output_by_shape(outputs, grid, 64)
            score_tensor = self._find_output_by_shape(outputs, grid, 1)
            keypoint_tensor = self._find_output_by_shape(
                outputs,
                grid,
                self.num_keypoints * 3,
            )

            boxes = self._decode_pose_boxes(bbox_tensor, stride)
            scores = score_tensor.reshape(-1)
            if np.min(scores) < 0.0 or np.max(scores) > 1.0:
                scores = _sigmoid(scores)

            keypoints = self._decode_pose_keypoints(
                keypoint_tensor,
                stride,
            )

            all_boxes.append(boxes)
            all_scores.append(scores)
            all_keypoints.append(keypoints)

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        keypoints = np.concatenate(all_keypoints, axis=0)

        mask = scores >= self.pose_conf
        boxes = boxes[mask]
        scores = scores[mask]
        keypoints = keypoints[mask]

        if len(boxes) == 0:
            return None

        boxes[:, [0, 2]] = np.clip(
            boxes[:, [0, 2]],
            0,
            self.frame_w - 1,
        )
        boxes[:, [1, 3]] = np.clip(
            boxes[:, [1, 3]],
            0,
            self.frame_h - 1,
        )
        keypoints[:, :, 0] = np.clip(
            keypoints[:, :, 0],
            0,
            self.frame_w - 1,
        )
        keypoints[:, :, 1] = np.clip(
            keypoints[:, :, 1],
            0,
            self.frame_h - 1,
        )

        valid = (
            (boxes[:, 2] > boxes[:, 0])
            & (boxes[:, 3] > boxes[:, 1])
        )
        boxes = boxes[valid]
        scores = scores[valid]
        keypoints = keypoints[valid]

        if len(boxes) == 0:
            return None

        keep = _nms_xyxy(boxes, scores, self.pose_iou)
        if not keep:
            return None

        index = keep[0]
        candidate = {
            "conf": float(scores[index]),
            "box": boxes[index].copy(),
            "keypoints": keypoints[index].copy(),
        }

        if candidate["keypoints"].shape != (
            self.num_keypoints,
            3,
        ):
            return None

        if np.any(
            candidate["keypoints"][:, 2]
            < self.keypoint_conf
        ):
            return None

        return candidate

    @staticmethod
    def _map_yolo26_outputs(outputs):
        mapped = {}

        for _, tensor in outputs.items():
            shape = tuple(tensor.shape)
            if len(shape) != 4 or shape[0] != 1:
                continue

            grid_h, grid_w, channels = (
                shape[1],
                shape[2],
                shape[3],
            )

            if grid_h != grid_w or grid_h not in (80, 40, 20):
                continue

            if channels == 4:
                mapped[f"reg_{grid_h}"] = tensor[0]
            else:
                mapped[f"cls_{grid_h}"] = tensor[0]

        required = [
            "reg_80", "reg_40", "reg_20",
            "cls_80", "cls_40", "cls_20",
        ]
        missing = [key for key in required if key not in mapped]
        if missing:
            available = {
                name: tuple(tensor.shape)
                for name, tensor in outputs.items()
            }
            raise RuntimeError(
                f"YOLO26 missing {missing}; available={available}"
            )

        return mapped

    def _decode_yolo26(self, outputs):
        mapped = self._map_yolo26_outputs(outputs)

        boxes_all = []
        scores_all = []
        classes_all = []

        for grid, stride in ((80, 8), (40, 16), (20, 32)):
            cls_data = mapped[f"cls_{grid}"]
            reg_data = mapped[f"reg_{grid}"]

            if np.min(cls_data) < 0.0 or np.max(cls_data) > 1.0:
                cls_prob = _sigmoid(cls_data)
            else:
                cls_prob = cls_data

            scores = cls_prob.max(axis=2)
            classes = cls_prob.argmax(axis=2)
            mask = scores >= self.det_conf

            if not np.any(mask):
                continue

            rows, cols = np.where(mask)
            regression = reg_data[rows, cols]
            selected_scores = scores[rows, cols]
            selected_classes = classes[rows, cols]

            left = regression[:, 0]
            top = regression[:, 1]
            right = regression[:, 2]
            bottom = regression[:, 3]

            boxes_all.append(
                np.stack(
                    [
                        (cols + 0.5 - left) * stride,
                        (rows + 0.5 - top) * stride,
                        (cols + 0.5 + right) * stride,
                        (rows + 0.5 + bottom) * stride,
                    ],
                    axis=1,
                )
            )
            scores_all.append(selected_scores)
            classes_all.append(selected_classes)

        if not boxes_all:
            return []

        boxes = np.concatenate(boxes_all, axis=0)
        scores = np.concatenate(scores_all, axis=0)
        classes = np.concatenate(classes_all, axis=0)

        boxes[:, [0, 2]] = np.clip(
            boxes[:, [0, 2]],
            0,
            self.frame_w - 1,
        )
        boxes[:, [1, 3]] = np.clip(
            boxes[:, [1, 3]],
            0,
            self.frame_h - 1,
        )

        keep = _nms_xyxy(boxes, scores, self.det_iou)
        detections = []

        for index in keep:
            detections.append(
                {
                    "x1": float(boxes[index, 0]),
                    "y1": float(boxes[index, 1]),
                    "x2": float(boxes[index, 2]),
                    "y2": float(boxes[index, 3]),
                    "conf": float(scores[index]),
                    "cls_id": int(classes[index]),
                }
            )

        return detections

    def _select_class(self, detections, class_id=None):
        if class_id is None:
            class_id = self.gate_class_id
        candidates = [
            det
            for det in detections
            if det["cls_id"] == int(class_id)
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda det: det["conf"])

    @staticmethod
    def _pose_center(pose):
        center = np.mean(
            pose["keypoints"][:, :2],
            axis=0,
        )
        return [float(center[0]), float(center[1])]

    @staticmethod
    def _bbox_center(det):
        return [
            float((det["x1"] + det["x2"]) / 2.0),
            float((det["y1"] + det["y2"]) / 2.0),
        ]

    @staticmethod
    def _pose_keypoints_json(pose):
        if pose is None:
            return None
        return [
            [
                round(float(p[0]), 2),
                round(float(p[1]), 2),
                round(float(p[2]), 4),
            ]
            for p in pose["keypoints"]
        ]

    @staticmethod
    def _bbox_json(det):
        if det is None:
            return None
        return [
            round(float(det["x1"]), 2),
            round(float(det["y1"]), 2),
            round(float(det["x2"]), 2),
            round(float(det["y2"]), 2),
        ]

    def _run_pose(self, input_tensor):
        self.switcher.switch("POSE")
        infer_start = time.perf_counter()
        outputs = self.pose_pipeline.infer(input_tensor)
        infer_sec = time.perf_counter() - infer_start
        return self._map_pose_to_frame(
            self._decode_pose(outputs)
        ), infer_sec

    def _run_detector(self, input_tensor, detector_class_id):
        self.switcher.switch("YOLO26")
        infer_start = time.perf_counter()
        outputs = self.detector_pipeline.infer(input_tensor)
        infer_sec = time.perf_counter() - infer_start
        detections = self._decode_yolo26(outputs)
        selected = self._select_class(
            detections,
            detector_class_id,
        )
        return self._map_detection_to_frame(selected), infer_sec

    def _pose_result(self, pose, infer_sec):
        result = FrontAiResult(
            vision_mode="POSE",
            ai_model="YOLO8_POSE",
            confidence=float(pose["conf"]),
            keypoints=self._pose_keypoints_json(pose),
            target=[
                round(v, 2)
                for v in self._pose_center(pose)
            ],
            model_ms=infer_sec * 1000.0,
            model_fps=1.0 / max(infer_sec, 1e-9),
            vision_valid=True,
        )
        return result

    def _detector_result(self, detection, infer_sec):
        if detection is None:
            return FrontAiResult(
                vision_mode="SEARCH",
                ai_model="YOLO26",
                model_ms=infer_sec * 1000.0,
                model_fps=1.0 / max(infer_sec, 1e-9),
                vision_valid=False,
            )

        return FrontAiResult(
            vision_mode="YOLO26",
            ai_model="YOLO26",
            confidence=float(detection["conf"]),
            bbox=self._bbox_json(detection),
            target=[
                round(v, 2)
                for v in self._bbox_center(detection)
            ],
            model_ms=infer_sec * 1000.0,
            model_fps=1.0 / max(infer_sec, 1e-9),
            vision_valid=True,
        )

    def set_route_context(self, camera, policy, detector_class_id):
        """Reset transient model/fallback state when the mission route changes."""
        context = (
            str(camera).lower(),
            str(policy).upper(),
            None if detector_class_id is None else int(detector_class_id),
        )
        if context == self.route_context:
            return

        self.route_context = context
        self.state = "POSE" if context[1] in ("POSE_ONLY", "HYBRID") else "FALLBACK"
        self.pose_lost_since = None
        self.last_pose_probe = 0.0
        self.last_detection = None
        self.last_detection_time = 0.0
    def process(
        self,
        frame,
        *,
        policy="HYBRID",
        detector_class_id=None,
    ):
        """Run the requested model policy on one frame.

        policy:
          OFF          no inference
          POSE_ONLY    only YOLOv8 pose
          YOLO26_ONLY  only detector, class selectable per route
          HYBRID       pose primary + YOLO26 fallback
        """
        if not self.ready:
            raise RuntimeError("FrontHailoAi is not open")

        policy = str(policy).upper()
        if policy not in {
            "OFF",
            "POSE_ONLY",
            "YOLO26_ONLY",
            "HYBRID",
        }:
            raise ValueError(f"invalid AI policy: {policy}")

        if policy == "OFF":
            return FrontAiResult(
                vision_mode="OFF",
                ai_model="NONE",
                vision_valid=False,
            )

        input_tensor = self._preprocess(frame)
        now = time.monotonic()

        if policy == "POSE_ONLY":
            pose, infer_sec = self._run_pose(input_tensor)
            self.state = "POSE"
            self.pose_lost_since = None
            self.last_detection = None

            if pose is None:
                return FrontAiResult(
                    vision_mode="SEARCH",
                    ai_model="YOLO8_POSE",
                    model_ms=infer_sec * 1000.0,
                    model_fps=1.0 / max(infer_sec, 1e-9),
                    vision_valid=False,
                )
            return self._pose_result(pose, infer_sec)

        if policy == "YOLO26_ONLY":
            detection, infer_sec = self._run_detector(
                input_tensor,
                detector_class_id,
            )
            self.state = "FALLBACK"

            if detection is not None:
                self.last_detection = detection
                self.last_detection_time = now
            else:
                self.last_detection = None

            return self._detector_result(
                detection,
                infer_sec,
            )

        # HYBRID: existing pose-first behavior.
        pose_probe = False
        if self.state == "POSE":
            active_model = "POSE"
        else:
            if now - self.last_pose_probe >= self.pose_probe_interval_s:
                active_model = "POSE"
                pose_probe = True
            else:
                active_model = "YOLO26"

        if active_model == "POSE":
            pose, infer_sec = self._run_pose(input_tensor)

            if pose is not None:
                self.state = "POSE"
                self.pose_lost_since = None
                self.last_detection = None
                return self._pose_result(pose, infer_sec)

            if pose_probe:
                self.state = "FALLBACK"
                self.last_pose_probe = now

                age = now - self.last_detection_time
                if (
                    self.last_detection is not None
                    and age <= self.fallback_bbox_max_age_s
                ):
                    return self._detector_result(
                        self.last_detection,
                        infer_sec,
                    )
            else:
                if self.pose_lost_since is None:
                    self.pose_lost_since = now

                if (
                    now - self.pose_lost_since
                    >= self.pose_lost_timeout_s
                ):
                    self.state = "FALLBACK"
                    self.last_pose_probe = now

            return FrontAiResult(
                vision_mode="SEARCH",
                ai_model="YOLO8_POSE",
                model_ms=infer_sec * 1000.0,
                model_fps=1.0 / max(infer_sec, 1e-9),
                vision_valid=False,
            )

        detection, infer_sec = self._run_detector(
            input_tensor,
            detector_class_id,
        )

        if detection is None:
            self.last_detection = None
            return self._detector_result(
                None,
                infer_sec,
            )

        self.last_detection = detection
        self.last_detection_time = now
        return self._detector_result(
            detection,
            infer_sec,
        )
