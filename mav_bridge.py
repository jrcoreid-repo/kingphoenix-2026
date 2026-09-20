#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
import selectors
import signal
import socket
import sys
import time

import serial
from pymavlink import mavutil

from config import load_json


class FastMavBridge:
    """Raw MAVLink router plus race-mode telemetry keeper."""

    def __init__(self, cfg: dict, mode: str = "race"):
        self.cfg = cfg
        self.mode = str(mode).lower()
        if self.mode not in {"race", "setup"}:
            raise ValueError(f"unsupported bridge mode: {self.mode}")

        self.stop = False
        self.serial = None
        self.selector = selectors.DefaultSelector()
        self.started_s = time.monotonic()

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
            if fixed_ip else None
        )
        self.learn_mp_peer = bool(mp.get("learn_peer", True))
        self.allowed_mp_types = set(
            str(x).upper()
            for x in mp.get("fc_uplink_allow_types", ["GPS_RTCM_DATA", "GPS_INJECT_DATA"])
        )

        self.mp_parser = mavutil.mavlink.MAVLink(None)
        self.fc_parser = mavutil.mavlink.MAVLink(None)
        self.tx_mav = mavutil.mavlink.MAVLink(None)
        self.tx_mav.srcSystem = int(cfg.get("source_system", 254))
        self.tx_mav.srcComponent = int(cfg.get("source_component", 190))

        policy_root = dict(cfg.get("telemetry_policy", {}) or {})
        self.policy = dict(policy_root.get(self.mode, {}) or {})
        self.telemetry_enabled = bool(self.policy.get("enabled", False))
        self.target_system = int(self.policy.get("target_system", 1))
        self.target_component = int(self.policy.get("target_component", 1))
        self.rates_hz = dict(self.policy.get("rates_hz", {}) or {})
        self.reassert_interval_s = float(self.policy.get("reassert_interval_s", 10.0))
        self.request_on_fc_heartbeat = bool(
            self.policy.get("request_on_fc_heartbeat", True)
        )

        lidar_cfg = dict(self.policy.get("lidar", {}) or {})
        self.lidar_message = str(lidar_cfg.get("message", "DISTANCE_SENSOR")).upper()
        self.lidar_preferred_hz = float(lidar_cfg.get("preferred_hz", 30.0))
        self.lidar_minimum_hz = float(lidar_cfg.get("minimum_hz", 15.0))
        self.lidar_max_age_s = float(lidar_cfg.get("max_age_s", 0.10))

        self.use_ansi = bool(self.policy.get("ansi", True)) and sys.stdout.isatty()
        self.display_hz = max(1.0, float(self.policy.get("display_hz", 5.0)))
        self.last_display_s = 0.0

        self.msg_last_ns = {}
        self.msg_rx_times = {}
        self.fc_heartbeat_seen = False
        self.stream_policy_applied = False
        self.last_reassert_s = 0.0
        self.last_policy_action = "WAIT FC HEARTBEAT" if self.telemetry_enabled else "DISABLED"
        self.event_log = deque(maxlen=4)

        self.stats = {
            "fc_rx": 0,
            "runner_tx": 0,
            "mp_tx": 0,
            "runner_rx": 0,
            "mp_rx": 0,
            "mp_allowed": 0,
            "mp_blocked": 0,
        }
        self.total = {key: 0 for key in self.stats}
        self.rate_bytes = 0
        self.fc_kbps = 0.0
        self.last_rate_s = time.monotonic()
        self.stats_interval_s = float(cfg.get("stats_interval_s", 5.0))
        self.last_stats = time.monotonic()

    def _signal_stop(self, *_):
        self.stop = True

    def _event(self, text):
        self.event_log.append(str(text))
        self.last_policy_action = str(text)

    def _message_id(self, name):
        name = str(name).upper()
        fallback = {"DISTANCE_SENSOR": 132, "RANGEFINDER": 173}
        if name in fallback:
            return fallback[name]
        return getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)

    def _message_age_s(self, name):
        ts = self.msg_last_ns.get(str(name).upper(), 0)
        if not ts:
            return float("inf")
        return (time.monotonic_ns() - ts) / 1e9

    def _message_rate_hz(self, name):
        q = self.msg_rx_times.get(str(name).upper())
        if not q or len(q) < 2:
            return 0.0
        span_s = (q[-1] - q[0]) / 1e9
        return 0.0 if span_s <= 0 else (len(q) - 1) / span_s

    def _mark_message(self, name, now_ns):
        name = str(name).upper()
        self.msg_last_ns[name] = now_ns
        q = self.msg_rx_times.get(name)
        if q is None:
            q = deque(maxlen=256)
            self.msg_rx_times[name] = q
        q.append(now_ns)
        cutoff = now_ns - int(5e9)
        while q and q[0] < cutoff:
            q.popleft()

    def _observe_fc(self, data):
        try:
            messages = self.fc_parser.parse_buffer(data) or []
        except Exception:
            return
        now_ns = time.monotonic_ns()
        for msg in messages:
            msg_type = str(msg.get_type()).upper()
            if msg_type == "BAD_DATA":
                continue
            self._mark_message(msg_type, now_ns)
            if msg_type == "HEARTBEAT":
                try:
                    src_sys = int(msg.get_srcSystem())
                    src_comp = int(msg.get_srcComponent())
                except Exception:
                    src_sys, src_comp = 0, 0
                if src_sys == self.target_system and (
                    src_comp == self.target_component or self.target_component <= 0
                ):
                    first = not self.fc_heartbeat_seen
                    self.fc_heartbeat_seen = True
                    if first:
                        self._event(f"FC HEARTBEAT {src_sys}:{src_comp}")

    def _send_message_interval(self, name, rate_hz):
        mid = self._message_id(name)
        if mid is None or self.serial is None:
            return False
        interval_us = -1 if float(rate_hz) <= 0 else int(round(1_000_000.0 / float(rate_hz)))
        try:
            msg = self.tx_mav.command_long_encode(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mid),
                float(interval_us),
                0.0, 0.0, 0.0, 0.0, 0.0,
            )
            self._serial_write(msg.pack(self.tx_mav))
            return True
        except Exception:
            return False

    def _request_policy(self, lidar_only=False, reason="POLICY"):
        if not self.telemetry_enabled or not self.fc_heartbeat_seen:
            return
        items = (
            [(self.lidar_message, self.rates_hz.get(self.lidar_message, 50))]
            if lidar_only
            else list(self.rates_hz.items())
        )
        ok_count = 0
        for name, rate in items:
            if self._send_message_interval(name, rate):
                ok_count += 1
        self.stream_policy_applied = True
        self.last_reassert_s = time.monotonic()
        label = "LiDAR" if lidar_only else "ALL"
        self._event(f"{reason}: {label} {ok_count}/{len(items)} requested")

    def _maintain_telemetry(self):
        if not self.telemetry_enabled:
            return
        if (
            self.request_on_fc_heartbeat
            and self.fc_heartbeat_seen
            and not self.stream_policy_applied
        ):
            self._request_policy(reason="WARMUP")
            return
        if not self.stream_policy_applied:
            return

        now = time.monotonic()
        if now - self.last_reassert_s < self.reassert_interval_s:
            return

        rate = self._message_rate_hz(self.lidar_message)
        age = self._message_age_s(self.lidar_message)
        if rate < self.lidar_preferred_hz or age > self.lidar_max_age_s:
            self._request_policy(lidar_only=True, reason="REASSERT")
        else:
            self.last_reassert_s = now

    def open(self):
        signal.signal(signal.SIGINT, self._signal_stop)
        signal.signal(signal.SIGTERM, self._signal_stop)

        fc = self.cfg["fc"]
        device = str(fc.get("device", "/dev/ttyAMA0"))
        baud = int(fc.get("baud", 921600))
        self.device = device
        self.baud = baud

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
        self._event("BRIDGE OPEN")

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
        n = len(data)
        self.stats["fc_rx"] += n
        self.total["fc_rx"] += n
        self.rate_bytes += n
        self._observe_fc(data)

        try:
            self.runner_sock.sendto(data, self.runner_addr)
            self.stats["runner_tx"] += n
            self.total["runner_tx"] += n
        except (BlockingIOError, OSError):
            pass

        if self.mp_peer is not None:
            try:
                self.mp_sock.sendto(data, self.mp_peer)
                self.stats["mp_tx"] += n
                self.total["mp_tx"] += n
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
        self.total["runner_rx"] += len(data)
        self._serial_write(data)

    def _filter_mp_uplink(self, data: bytes) -> bytes:
        allowed = bytearray()
        try:
            messages = self.mp_parser.parse_buffer(data) or []
        except Exception:
            self.stats["mp_blocked"] += 1
            self.total["mp_blocked"] += 1
            return b""

        for msg in messages:
            msg_type = str(msg.get_type()).upper()
            raw = bytes(msg.get_msgbuf())
            if msg_type in self.allowed_mp_types:
                allowed.extend(raw)
                self.stats["mp_allowed"] += 1
                self.total["mp_allowed"] += 1
            else:
                self.stats["mp_blocked"] += 1
                self.total["mp_blocked"] += 1
        return bytes(allowed)

    def _mp_to_fc(self):
        try:
            data, peer = self.mp_sock.recvfrom(65535)
        except BlockingIOError:
            return
        if not data:
            return

        self.stats["mp_rx"] += len(data)
        self.total["mp_rx"] += len(data)
        if self.learn_mp_peer and self.mp_peer != peer:
            self.mp_peer = peer
            self._event(f"MP PEER {peer[0]}:{peer[1]}")

        if self.mode == "setup":
            self._serial_write(data)
            self.stats["mp_allowed"] += 1
            self.total["mp_allowed"] += 1
            return

        filtered = self._filter_mp_uplink(data)
        if filtered:
            self._serial_write(filtered)

    def _update_rates(self):
        now = time.monotonic()
        dt = now - self.last_rate_s
        if dt >= 1.0:
            self.fc_kbps = (self.rate_bytes * 8.0 / 1000.0) / max(0.001, dt)
            self.rate_bytes = 0
            self.last_rate_s = now

    def _lidar_quality(self):
        rate = self._message_rate_hz(self.lidar_message)
        age = self._message_age_s(self.lidar_message)
        if age == float("inf"):
            return "WAIT", rate, None
        if age > self.lidar_max_age_s or rate < self.lidar_minimum_hz:
            return "FAIL", rate, age
        if rate < self.lidar_preferred_hz:
            return "WARN", rate, age
        return "GOOD", rate, age

    def dashboard(self):
        quality, lidar_hz, lidar_age = self._lidar_quality()
        peer = (
            f"{self.mp_peer[0]}:{self.mp_peer[1]}"
            if self.mp_peer is not None else "WAIT"
        )
        runner_age = self._message_age_s("HEARTBEAT")
        hb_text = "-" if runner_age == float("inf") else f"{runner_age * 1000.0:.0f}ms"
        lidar_age_text = "-" if lidar_age is None else f"{lidar_age * 1000.0:.0f}ms"
        gps_hz = self._message_rate_hz("GPS_RAW_INT")
        gpos_hz = self._message_rate_hz("GLOBAL_POSITION_INT")
        mission_hz = self._message_rate_hz("MISSION_CURRENT")
        uplink = "RTCM_ONLY" if self.mode == "race" else "FULL_ACCESS"
        keeper = "ON" if self.telemetry_enabled else "OFF"

        lines = [
            "======================================================================",
            " KINGPHOENIX AR-1 | FAST MAVLINK BRIDGE",
            "======================================================================",
            (
                f" MODE   : {self.mode.upper():<6} | FC: {self.device} @ {self.baud} | "
                f"UPTIME={time.monotonic() - self.started_s:6.1f}s"
            ),
            (
                f" LINK   : FC={self.fc_kbps:6.1f} kbps | HB AGE={hb_text:<6} | "
                f"RUNNER -> {self.runner_addr[0]}:{self.runner_addr[1]}"
            ),
            (
                f" MP     : {peer:<21} | UDP={self.mp_sock.getsockname()[1]} | "
                f"UPLINK={uplink}"
            ),
            "----------------------------------------------------------------------",
            (
                f" KEEPER : {keeper:<3} | POLICY={'APPLIED' if self.stream_policy_applied else 'WARMING':<7} | "
                f"TARGET LiDAR={float(self.rates_hz.get(self.lidar_message, 0)):4.0f}Hz"
            ),
            (
                f" LIDAR  : {quality:<4} | {lidar_hz:5.1f}Hz | AGE={lidar_age_text:>6} | "
                f"SRC={self.lidar_message}"
            ),
            (
                f" AUX    : GPS={gps_hz:4.1f}Hz | GPOS={gpos_hz:4.1f}Hz | "
                f"MISSION={mission_hz:4.1f}Hz"
            ),
            "----------------------------------------------------------------------",
            (
                f" TRAFFIC: FC->RUNNER={self.total['runner_tx']} B | "
                f"RUNNER->FC={self.total['runner_rx']} B | MP->FC={self.total['mp_rx']} B"
            ),
            (
                f" MP ACL : allowed={self.total['mp_allowed']} | "
                f"blocked={self.total['mp_blocked']}"
            ),
            f" STATUS : {self.last_policy_action}",
            " EVENTS :",
        ]
        events = list(self.event_log) or ["-"]
        lines.extend(f"   {x}" for x in events[-4:])
        lines.extend([
            "======================================================================",
            " Ctrl+C = stop bridge",
        ])
        return "\n".join(lines)

    def display_if_due(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_display_s < 1.0 / self.display_hz:
            return
        self.last_display_s = now
        text = self.dashboard()
        if self.use_ansi:
            sys.stdout.write("\033[2J\033[H" + text + "\n")
            sys.stdout.flush()
        elif force or now - self.last_stats >= self.stats_interval_s:
            sys.stdout.write(text + "\n")
            sys.stdout.flush()
            self.last_stats = now

    def run(self):
        self.open()
        self.display_if_due(force=True)
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
                self._update_rates()
                self._maintain_telemetry()
                self.display_if_due()
        finally:
            self.close()
            if not self.use_ansi:
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
        help="race=telemetry keeper + MP RTK only, setup=full MP uplink",
    )
    args = parser.parse_args()
    cfg = load_json(args.config)
    mode = args.mode or str(cfg.get("default_mode", "race")).lower()
    FastMavBridge(cfg, mode=mode).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
