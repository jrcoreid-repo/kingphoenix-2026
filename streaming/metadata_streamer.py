from __future__ import annotations

import json
import socket
import time


class MetadataStreamer:
    def __init__(self, destination_ip, port):
        self.destination_ip = str(destination_ip)
        self.port = int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent_packets = 0
        self.sent_bytes = 0
        self.last_error = None

    def set_destination(self, ip, port=None):
        self.destination_ip = str(ip)
        if port is not None:
            self.port = int(port)

    def send(self, metadata):
        payload = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
        try:
            sent = self.sock.sendto(payload, (self.destination_ip, self.port))
            self.sent_packets += 1
            self.sent_bytes += int(sent)
            self.last_error = None
            return sent == len(payload)
        except Exception as exc:
            self.last_error = repr(exc)
            return False

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def build_camera_metadata(
    *,
    camera,
    frame_id,
    timestamp_ns,
    ai_state,
    ai_task,
    video_state="LIVE",
    vision_mode="STREAM_ONLY",
    fps=0.0,
    telemetry=None,
    extra=None,
):
    meta = {
        "camera": str(camera).lower(),
        "frame_id": int(frame_id) & 0xFFFFFFFF,
        "timestamp_ns": int(timestamp_ns),
        "monotonic_s": time.monotonic(),
        "video_state": str(video_state).upper(),
        "ai_state": str(ai_state).upper(),
        "ai_task": str(ai_task).upper(),
        "vision_mode": str(vision_mode).upper(),
        "fps": float(fps),
        "telemetry": telemetry or {},
    }
    if extra:
        meta.update(extra)
    return meta
