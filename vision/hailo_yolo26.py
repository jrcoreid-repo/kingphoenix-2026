from __future__ import annotations
from pathlib import Path
import time
import threading

from vision.hailo_front_ai import FrontHailoAi, FrontAiResult

class HailoYolo26(FrontHailoAi):
    """V2 detector-only runtime. Does not load or activate the pose HEF."""
    def __init__(self,cfg,frame_w=1920,frame_h=1200):
        detector=cfg["detector"]
        legacy={
            "model_size":cfg.get("model_size",640),
            "front":{
                "primary":{
                    "hef":"__V2_POSE_DISABLED__",
                    "num_keypoints":4,
                    "confidence_threshold":1.0,
                    "keypoint_confidence_threshold":1.0,
                    "iou_threshold":0.6,
                },
                "fallback":{
                    "hef":detector["hef"],
                    "gate_class_id":cfg.get("front",{}).get("detector_class_id",1),
                    "confidence_threshold":detector.get("confidence_threshold",.25),
                    "iou_threshold":detector.get("iou_threshold",.45),
                }
            }
        }
        super().__init__(legacy,frame_w=frame_w,frame_h=frame_h)
        self._activation_context = None
        self._infer_lock = threading.Lock()

    def open(self):
        if self.ready: return
        if not Path(self.detector_hef_path).exists():
            raise FileNotFoundError(self.detector_hef_path)
        from hailo_platform import (
            VDevice,HEF,ConfigureParams,InputVStreamParams,OutputVStreamParams,
            InferVStreams,HailoStreamInterface,FormatType
        )
        hef=HEF(self.detector_hef_path)
        self.device=VDevice()
        params=ConfigureParams.create_from_hef(hef,interface=HailoStreamInterface.PCIe)
        self.detector_group=self.device.configure(hef,params)[0]
        inp=InputVStreamParams.make(self.detector_group)
        out=OutputVStreamParams.make(self.detector_group,format_type=FormatType.FLOAT32)
        self._detector_context=InferVStreams(self.detector_group,inp,out)
        self.detector_pipeline=self._detector_context.__enter__()

        # HailoRT requires the configured network group to stay ACTIVATED
        # for the whole lifetime of InferVStreams inference.
        self._activation_context = self.detector_group.activate()
        self._activation_context.__enter__()

        self.pose_group=None; self.pose_pipeline=None; self.switcher=None
        self.ready=True
        print("[AI] V2 YOLO26 ONLY READY")
        print("[AI] pose HEF: NOT LOADED")
        print("[AI] network group: ACTIVATED")
        print(f"[AI] detector: {self.detector_hef_path}")

    def close(self):
        self.ready=False

        # Stop new inference before tearing down Hailo resources.
        with self._infer_lock:
            if self._activation_context is not None:
                try:
                    self._activation_context.__exit__(None,None,None)
                except Exception:
                    pass
                self._activation_context=None

            if self._detector_context is not None:
                try:
                    self._detector_context.__exit__(None,None,None)
                except Exception:
                    pass
                self._detector_context=None
                self.detector_pipeline=None

        self.detector_group=None
        if self.device is not None:
            try:
                f=getattr(self.device,"close",None)
                if callable(f):
                    f()
            except Exception:
                pass
            self.device=None

    def _run_detector(self,input_tensor,detector_class_id):
        # One shared detector is used by FRONT/BOTTOM. Router normally keeps
        # only one camera ACTIVE, while this lock also protects the exact
        # transition instant between cameras.
        with self._infer_lock:
            if not self.ready or self._activation_context is None:
                raise RuntimeError("YOLO26 network group is not active")
            start=time.perf_counter()
            outputs=self.detector_pipeline.infer(input_tensor)
            elapsed=time.perf_counter()-start

        detections=self._decode_yolo26(outputs)
        selected=self._select_class(detections,detector_class_id)
        return self._map_detection_to_frame(selected),elapsed

    def process(self,frame,*,policy="YOLO26_ONLY",detector_class_id=None):
        if not self.ready: raise RuntimeError("V2 YOLO26 engine is not open")
        policy=str(policy).upper()
        if policy=="OFF":
            return FrontAiResult(vision_mode="OFF",ai_model="NONE",vision_valid=False)
        if policy!="YOLO26_ONLY":
            raise ValueError(f"V2 supports OFF/YOLO26_ONLY only, got {policy}")
        tensor=self._preprocess(frame)
        det,sec=self._run_detector(tensor,detector_class_id)
        return self._detector_result(det,sec)
