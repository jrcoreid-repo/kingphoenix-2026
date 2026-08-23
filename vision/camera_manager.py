from __future__ import annotations

from dataclasses import dataclass
import threading
import time


@dataclass
class FramePacket:
    camera: str
    frame_id: int
    timestamp_ns: int
    frame: object


class CameraManager:
    """Real/bench camera abstraction. Owns no MAVLink.

    V1.9.5 canonical frame contract:
      CameraManager output -> BGR uint8 HxWx3
      OpenCV              -> BGR
      FFmpeg raw input    -> bgr24
      Hailo preprocess    -> converts BGR to RGB exactly once
    """

    def __init__(
        self,
        camera_name,
        source=None,
        *,
        backend="opencv",
        width=640,
        height=640,
        fps=30,
        loop_file=True,
        camera_num=None,
        sensor_mode=None,
        output=None,
        runtime=None,
    ):
        self.camera_name = str(camera_name).lower()
        self.backend = str(backend).lower()
        self.source = source
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.loop_file = bool(loop_file)

        self.camera_num = None if camera_num is None else int(camera_num)
        self.sensor_mode = dict(sensor_mode or {})
        self.output_cfg = dict(output or {})
        self.runtime_cfg = dict(runtime or {})

        self._cap = None
        self._picam2 = None
        self._frame_id = 0
        self._lock = threading.RLock()
        self._latest = None
        self._opened = False
        self._last_read_ns = 0

        self.actual_camera_id = None
        self.selected_sensor_mode = None
        self.capture_fps = 0.0

    @staticmethod
    def _normalize_source(source):
        if source is None:
            return None
        s = str(source)
        return int(s) if s.isdigit() else source

    @staticmethod
    def _mode_size(mode):
        size = mode.get("size")
        try:
            return int(size[0]), int(size[1])
        except Exception:
            return None

    def _select_sensor_mode(self, picam2):
        wanted_w = int(self.sensor_mode.get("width", 1280))
        wanted_h = int(self.sensor_mode.get("height", 720))
        wanted_fps = float(self.sensor_mode.get("fps", 120.0))
        wanted_depth = int(self.sensor_mode.get("bit_depth", 10))

        modes = list(getattr(picam2, "sensor_modes", []) or [])
        if not modes:
            return None

        def score(mode):
            size = self._mode_size(mode) or (0, 0)
            fps = float(mode.get("fps", 0.0) or 0.0)
            depth = int(mode.get("bit_depth", 0) or 0)
            return (
                abs(size[0]-wanted_w)*5
                + abs(size[1]-wanted_h)*5
                + abs(fps-wanted_fps)*20
                + abs(depth-wanted_depth)*100
            )

        mode = min(modes, key=score)
        self.selected_sensor_mode = {
            "size": list(self._mode_size(mode) or (0,0)),
            "fps": float(mode.get("fps",0.0) or 0.0),
            "bit_depth": int(mode.get("bit_depth",0) or 0),
            "format": str(mode.get("format","")),
        }
        return mode

    def _open_picamera2(self):
        from picamera2 import Picamera2

        if self.camera_num is None:
            raise RuntimeError(f"{self.camera_name}: camera_num required")

        info = Picamera2.global_camera_info()
        if not 0 <= self.camera_num < len(info):
            raise RuntimeError(
                f"{self.camera_name}: camera_num={self.camera_num} unavailable; "
                f"detected={len(info)}"
            )

        item = info[self.camera_num]
        self.actual_camera_id = str(
            item.get("Id", item.get("Model", self.camera_num))
        )
        picam2 = Picamera2(camera_num=self.camera_num)

        out_w = int(self.output_cfg.get("width", self.width))
        out_h = int(self.output_cfg.get("height", self.height))
        out_fmt = str(self.output_cfg.get("format","RGB888")).upper()

        target_fps = float(
            self.runtime_cfg.get(
                "capture_target_fps",
                self.sensor_mode.get("fps",self.fps)
            )
        )
        queue = bool(self.runtime_cfg.get("queue",False))
        buffer_count = int(self.runtime_cfg.get("buffer_count",4))

        selected = self._select_sensor_mode(picam2)

        main = {
            "size": (out_w,out_h),
            "format": out_fmt,
            "preserve_ar": bool(
                self.output_cfg.get("preserve_aspect",True)
            ),
        }
        controls = {"FrameRate": target_fps}

        raw = None
        if selected is not None:
            size = self._mode_size(selected)
            fmt = selected.get("format")
            if size and fmt:
                raw={"size":size,"format":fmt}

        try:
            config = picam2.create_video_configuration(
                main=main,
                raw=raw,
                controls=controls,
                buffer_count=buffer_count,
                queue=queue,
            )
            picam2.configure(config)
        except Exception as exc:
            print(
                f"[CAM:{self.camera_name}] explicit raw-mode fallback: {exc}"
            )
            config = picam2.create_video_configuration(
                main=main,
                controls=controls,
                buffer_count=buffer_count,
                queue=queue,
            )
            picam2.configure(config)

        picam2.start()
        warmup=float(self.runtime_cfg.get("warmup_s",0.5))
        if warmup>0:
            time.sleep(warmup)

        self._picam2=picam2
        self._opened=True
        self.width=out_w
        self.height=out_h
        self.capture_fps=target_fps

        print(
            f"[CAM:{self.camera_name}] OPEN camera_num={self.camera_num} "
            f"id={self.actual_camera_id} "
            f"sensor={self.selected_sensor_mode} "
            f"main={out_w}x{out_h} {out_fmt} "
            f"target={target_fps:.1f}fps"
        )
        return True

    def _open_opencv(self):
        import cv2
        source=self._normalize_source(self.source)
        cap=cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(
                f"{self.camera_name}: cannot open source {self.source!r}"
            )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
        cap.set(cv2.CAP_PROP_FPS,self.fps)
        self._cap=cap
        self._opened=True
        self.actual_camera_id=str(self.source)
        self.capture_fps=self.fps
        return True

    def open(self):
        if self._opened:
            return True
        if self.backend=="picamera2":
            return self._open_picamera2()
        if self.backend=="opencv":
            return self._open_opencv()
        raise ValueError(f"unsupported backend {self.backend!r}")

    def close(self):
        with self._lock:
            if self._cap is not None:
                self._cap.release()
            self._cap=None

            if self._picam2 is not None:
                try: self._picam2.stop()
                except Exception: pass
                try: self._picam2.close()
                except Exception: pass
            self._picam2=None
            self._opened=False
            self._latest=None

    def _read_picamera2(self):
        frame = self._picam2.capture_array("main")
        if frame is None:
            return None

        # V1.9.5 COLOR CONTRACT
        #
        # On the validated Pi 5 + Arducam B0353/Pivariety stack,
        # Picamera2 "RGB888" capture_array() is already correct when consumed
        # as the OpenCV/FFmpeg BGR byte layout used by Kingphoenix.
        #
        # DO NOT apply RGB<->BGR conversion here. Doing so swaps red/blue,
        # producing visibly wrong colors and feeding the wrong colors to AI.
        #
        # Canonical internal frame order from this point onward: BGR.
        return frame

    def _read_opencv(self):
        import cv2
        ok,frame=self._cap.read()
        if (
            not ok
            and self.loop_file
            and not isinstance(self._normalize_source(self.source),int)
        ):
            self._cap.set(cv2.CAP_PROP_POS_FRAMES,0)
            ok,frame=self._cap.read()

        if not ok or frame is None:
            return None

        if frame.shape[1]!=self.width or frame.shape[0]!=self.height:
            frame=cv2.resize(
                frame,(self.width,self.height),
                interpolation=cv2.INTER_AREA
            )
        return frame

    def read(self):
        if not self._opened:
            self.open()

        frame=(
            self._read_picamera2()
            if self.backend=="picamera2"
            else self._read_opencv()
        )
        if frame is None:
            return None

        self._frame_id=(self._frame_id+1)&0xFFFFFFFF
        now=time.monotonic_ns()
        packet=FramePacket(
            camera=self.camera_name,
            frame_id=self._frame_id,
            timestamp_ns=now,
            frame=frame,
        )
        with self._lock:
            self._latest=packet
            self._last_read_ns=now
        return packet

    def latest(self):
        with self._lock:
            return self._latest

    def age_s(self):
        if not self._last_read_ns:
            return float("inf")
        return (time.monotonic_ns()-self._last_read_ns)/1e9
