#!/usr/bin/env python3
from __future__ import annotations

import argparse
import selectors
import signal
import socket
import sys
import time

import serial
from pymavlink import mavutil

from config import load_json


class FastMavBridge:
    """Minimal raw MAVLink router for race runtime."""

    def __init__(self, cfg: dict, mode: str = "race"):
        self.cfg = cfg
        self.mode = str(mode).lower()
        if self.mode not in {"race", "setup"}:
            raise ValueError(f"unsupported bridge mode: {self.mode}")
        self.stop = False
        self.serial = None
        self.selector = selectors.DefaultSelector()

        runner = cfg["runner"]
        self.runner_addr = (
            str(runner.get("dest_ip", "127.0.0.1")),
            int(runner.get("dest_port", 14550)),
        )
        self.runner_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.runner_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.runner_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)
        self.runner_sock.bind((
            str(runner.get("bind_ip", "127.0.0.1")),
            int(runner.get("bind_port", 14551)),
        ))
        self.runner_sock.setblocking(False)

        mp = cfg["mission_planner"]
        self.mp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.mp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)
        self.mp_sock.bind((
            str(mp.get("bind_ip", "0.0.0.0")),
            int(mp.get("bind_port", 14555)),
        ))
        self.mp_sock.setblocking(False)

        fixed_ip = str(mp.get("fixed_ip", "") or "").strip()
        self.mp_peer = (
            (fixed_ip, int(mp.get("fixed_port", 14555)))
            if fixed_ip
            else None
        )
        self.learn_mp_peer = bool(mp.get("learn_peer", True))

        self.mp_parser = mavutil.mavlink.MAVLink(None)
        self.allowed_mp_types = set(
            str(x).upper()
            for x in mp.get(
                "fc_uplink_allow_types",
                ["GPS_RTCM_DATA", "GPS_INJECT_DATA"],
            )
        )

        self.stats = {
            "fc_rx": 0,
            "runner_tx": 0,
            "mp_tx": 0,
            "runner_rx": 0,
            "mp_rx": 0,
            "mp_allowed": 0,
            "mp_blocked": 0,
        }
        self.stats_interval_s = float(cfg.get("stats_interval_s", 5.0))
        self.last_stats = time.monotonic()

    def _signal_stop(self, *_):
        self.stop = True

    def open(self):
        signal.signal(signal.SIGINT, self._signal_stop)
        signal.signal(signal.SIGTERM, self._signal_stop)

        fc = self.cfg["fc"]
        device = str(fc.get("device", "/dev/ttyAMA0"))
        baud = int(fc.get("baud", 921600))

        self.serial = serial.Serial(
            port=device,
            baudrate=baud,
            timeout=0,
            write_timeout=0,
            exclusive=True,
        )

        self.selector.register(self.serial.fileno(), selectors.EVENT_READ, "fc")
        self.selector.register(self.runner_sock, selectors.EVENT_READ, "runner")
        self.selector.register(self.mp_sock, selectors.EVENT_READ, "mp")

        print("============================================================")
        print(" KINGPHOENIX FAST MAVLINK BRIDGE")
        print(f" FC      : {device} @ {baud}")
        print(
            f" RUNNER  : {self.runner_sock.getsockname()[0]}:"
            f"{self.runner_sock.getsockname()[1]} -> "
            f"{self.runner_addr[0]}:{self.runner_addr[1]}"
        )
        uplink_policy = "RTCM_ONLY" if self.mode == "race" else "FULL_ACCESS"
        print(
            f" MP      : UDP {self.mp_sock.getsockname()[1]} | "
            f"uplink policy={uplink_policy}"
        )
        print(f" MODE    : {self.mode.upper()}")
        print(" Priority: FC -> RUNNER -> MP")
        print("============================================================")

    def close(self):
        try:
            self.selector.close()
        except Exception:
            pass
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        for sock in (self.runner_sock, self.mp_sock):
            try:
                sock.close()
            except Exception:
                pass

    def _serial_write(self, payload: bytes):
        if not payload or self.serial is None:
            return
        view = memoryview(payload)
        while view and not self.stop:
            try:
                n = self.serial.write(view)
            except serial.SerialTimeoutException:
                continue
            if not n:
                break
            view = view[n:]

    def _fc_to_clients(self):
        data = self.serial.read(8192)
        if not data:
            return
        self.stats["fc_rx"] += len(data)

        try:
            self.runner_sock.sendto(data, self.runner_addr)
            self.stats["runner_tx"] += len(data)
        except (BlockingIOError, OSError):
            pass

        if self.mp_peer is not None:
            try:
                self.mp_sock.sendto(data, self.mp_peer)
                self.stats["mp_tx"] += len(data)
            except (BlockingIOError, OSError):
                pass

    def _runner_to_fc(self):
        try:
            data, _peer = self.runner_sock.recvfrom(65535)
        except BlockingIOError:
            return
        if not data:
            return
        self.stats["runner_rx"] += len(data)
        self._serial_write(data)

    def _filter_mp_uplink(self, data: bytes) -> bytes:
        allowed = bytearray()
        try:
            messages = self.mp_parser.parse_buffer(data) or []
        except Exception:
            self.stats["mp_blocked"] += 1
            return b""

        for msg in messages:
            msg_type = str(msg.get_type()).upper()
            raw = bytes(msg.get_msgbuf())
            if msg_type in self.allowed_mp_types:
                allowed.extend(raw)
                self.stats["mp_allowed"] += 1
            else:
                self.stats["mp_blocked"] += 1
        return bytes(allowed)

    def _mp_to_fc(self):
        try:
            data, peer = self.mp_sock.recvfrom(65535)
        except BlockingIOError:
            return
        if not data:
            return

        self.stats["mp_rx"] += len(data)
        if self.learn_mp_peer and self.mp_peer != peer:
            self.mp_peer = peer
            print(f"[MP] peer learned {peer[0]}:{peer[1]}")

        if self.mode == "setup":
            # Full Mission Planner access for bench/setup only.
            self._serial_write(data)
            self.stats["mp_allowed"] += 1
            return

        filtered = self._filter_mp_uplink(data)
        if filtered:
            self._serial_write(filtered)

    def _print_stats(self):
        now = time.monotonic()
        if now - self.last_stats < self.stats_interval_s:
            return
        dt = max(0.001, now - self.last_stats)
        fc_kbps = (self.stats["fc_rx"] * 8.0 / 1000.0) / dt
        peer = (
            f"{self.mp_peer[0]}:{self.mp_peer[1]}"
            if self.mp_peer is not None
            else "WAIT"
        )
        uplink_label = "MP_UP" if self.mode == "setup" else "RTCM"
        print(
            f"[BRIDGE:{self.mode.upper()}] FC={fc_kbps:.1f}kbps | MP={peer} | "
            f"{uplink_label}={self.stats['mp_allowed']} | "
            f"MP_BLOCKED={self.stats['mp_blocked']}"
        )
        for key in self.stats:
            self.stats[key] = 0
        self.last_stats = now

    def run(self):
        self.open()
        try:
            while not self.stop:
                events = self.selector.select(timeout=0.02)
                ready = {key.data for key, _mask in events}
                if "fc" in ready:
                    self._fc_to_clients()
                if "runner" in ready:
                    self._runner_to_fc()
                if "mp" in ready:
                    self._mp_to_fc()
                self._print_stats()
        finally:
            self.close()
            print("[BRIDGE] stopped")


def main():
    parser = argparse.ArgumentParser(description="Kingphoenix fast MAVLink race bridge")
    parser.add_argument(
        "--config",
        default="mav_bridge.json",
        help="config filename inside config/ (default: mav_bridge.json)",
    )
    parser.add_argument(
        "--mode",
        choices=("race", "setup"),
        default=None,
        help="race=MP monitor+RTK only, setup=full MP uplink",
    )
    args = parser.parse_args()
    cfg = load_json(args.config)
    mode = args.mode or str(cfg.get("default_mode", "race")).lower()
    FastMavBridge(cfg, mode=mode).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
