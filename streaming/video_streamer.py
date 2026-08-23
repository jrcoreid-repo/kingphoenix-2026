from __future__ import annotations

import shutil
import subprocess
import threading


class VideoStreamer:
    """FFmpeg MPEG-TS/H264/UDP backend for V1.9 bench integration.

    Input frame contract is canonical Kingphoenix BGR HxWx3 uint8.
    FFmpeg therefore MUST receive -pix_fmt bgr24.

    The wire format intentionally matches the existing Windows PyAV GCS.
    """

    def __init__(
        self,
        destination_ip,
        port,
        width=640,
        height=640,
        fps=30,
        bitrate_kbps=3000,
        keyint=30,
    ):
        self.destination_ip = str(destination_ip)
        self.port = int(port)
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.bitrate_kbps = int(bitrate_kbps)
        self.keyint = int(keyint)

        self._proc = None
        self._lock = threading.RLock()

    @staticmethod
    def ffmpeg_available():
        return shutil.which("ffmpeg") is not None

    def set_destination(self, ip, port=None):
        with self._lock:
            changed = str(ip) != self.destination_ip or (
                port is not None and int(port) != self.port
            )
            self.destination_ip = str(ip)
            if port is not None:
                self.port = int(port)
            if changed:
                self.stop()

    def _command(self):
        target = f"udp://{self.destination_ip}:{self.port}?pkt_size=1316"
        return [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s:v", f"{self.width}x{self.height}",
            "-r", str(self.fps),
            "-i", "pipe:0",
            "-an",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-b:v", f"{self.bitrate_kbps}k",
            "-maxrate", f"{self.bitrate_kbps}k",
            "-bufsize", f"{max(1000, self.bitrate_kbps * 2)}k",
            "-g", str(self.keyint),
            "-keyint_min", str(self.keyint),
            "-pix_fmt", "yuv420p",
            "-f", "mpegts",
            "-mpegts_flags", "+resend_headers",
            target,
        ]

    def start(self):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return True
            if not self.ffmpeg_available():
                raise RuntimeError("ffmpeg not found in PATH")

            print(
                f"[VIDEO-TX] MPEG-TS/H264 -> "
                f"{self.destination_ip}:{self.port} "
                f"{self.width}x{self.height}@{self.fps:g} "
                f"{self.bitrate_kbps} kbps"
            )
            self._proc = subprocess.Popen(
                self._command(),
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
            return True

    def write(self, frame):
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self.start()
            try:
                self._proc.stdin.write(frame.tobytes())
                return True
            except (BrokenPipeError, OSError, AttributeError):
                self.stop()
                return False

    def stop(self):
        with self._lock:
            proc = self._proc
            self._proc = None

        if proc is None:
            return

        try:
            if proc.stdin:
                proc.stdin.close()
        except Exception:
            pass

        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
