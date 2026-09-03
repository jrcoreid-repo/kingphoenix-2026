from __future__ import annotations

from dataclasses import dataclass, field
from collections import deque
import threading
import time
from typing import Dict, Optional


@dataclass
class AckRecord:
    command: int
    result: int
    source_system: int
    source_component: int
    timestamp_ns: int
    progress: Optional[int] = None
    result_param2: Optional[int] = None


@dataclass
class StatusTextRecord:
    text: str
    severity: int
    source_system: int
    source_component: int
    timestamp_ns: int
    message_id: int | None = None
    chunk_seq: int | None = None


@dataclass
class VehicleState:
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"

    # Local position / attitude
    x: float | None = None
    y: float | None = None
    z: float | None = None
    relative_alt_m: float | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    groundspeed_mps: float | None = None

    # EKF/global vehicle position from GLOBAL_POSITION_INT.
    # Kept separate from raw receiver GPS so the GCS Current Location has a
    # single unambiguous source that matches navigation/mission state.
    global_lat_deg: float | None = None
    global_lon_deg: float | None = None
    global_alt_m: float | None = None
    global_position_last_ns: int = 0

    # Power / raw GPS / mission
    battery_v: float | None = None
    battery_current_a: float | None = None
    battery_remaining: int | None = None
    gps_satellites: int | None = None
    gps_fix_type: int | None = None
    gps_lat_deg: float | None = None
    gps_lon_deg: float | None = None
    gps_alt_m: float | None = None
    gps_hdop: float | None = None
    gps_vdop: float | None = None
    gps_cog_deg: float | None = None
    current_mission_item: int | None = None

    # Rangefinder / LiDAR state
    rangefinder_distance_m: float | None = None
    rangefinder_min_m: float | None = None
    rangefinder_max_m: float | None = None
    rangefinder_orientation: int | None = None
    rangefinder_id: int | None = None
    rangefinder_covariance: int | None = None
    rangefinder_signal_quality: int | None = None
    rangefinder_source: str | None = None
    rangefinder_last_ns: int = 0

    # Identity discovered from FC heartbeat
    autopilot_system_id: int | None = None
    autopilot_component_id: int | None = None

    # Common timestamps
    last_heartbeat_ns: int = 0
    last_position_ns: int = 0
    last_attitude_ns: int = 0

    # Message health
    message_last_ns: Dict[str, int] = field(default_factory=dict)
    message_rx_times: Dict[str, deque] = field(default_factory=dict)

    # ACK and STATUSTEXT histories
    latest_command_ack: AckRecord | None = None
    command_ack_history: deque = field(default_factory=lambda: deque(maxlen=64))
    status_text_history: deque = field(default_factory=lambda: deque(maxlen=128))

    ack_lock: threading.RLock = field(default_factory=threading.RLock, repr=False)
    status_text_lock: threading.RLock = field(default_factory=threading.RLock, repr=False)

    def heartbeat_age_s(self):
        if not self.last_heartbeat_ns:
            return float("inf")
        return (time.monotonic_ns() - self.last_heartbeat_ns) / 1e9

    def global_position_age_s(self):
        if not self.global_position_last_ns:
            return float("inf")
        return (time.monotonic_ns() - self.global_position_last_ns) / 1e9

    def mark_message(self, message_name: str, timestamp_ns: int | None = None):
        name = str(message_name).upper()
        now = int(timestamp_ns or time.monotonic_ns())
        self.message_last_ns[name] = now
        q = self.message_rx_times.get(name)
        if q is None:
            q = deque(maxlen=256)
            self.message_rx_times[name] = q
        q.append(now)
        cutoff = now - int(5e9)
        while q and q[0] < cutoff:
            q.popleft()

    def message_age_s(self, message_name: str) -> float:
        name = str(message_name).upper()
        ts = self.message_last_ns.get(name, 0)
        if not ts:
            return float("inf")
        return (time.monotonic_ns() - ts) / 1e9

    def message_rate_hz(self, message_name: str) -> float:
        name = str(message_name).upper()
        q = self.message_rx_times.get(name)
        if not q or len(q) < 2:
            return 0.0
        span_s = (q[-1] - q[0]) / 1e9
        if span_s <= 0:
            return 0.0
        return (len(q) - 1) / span_s

    def is_message_fresh(self, message_name: str, max_age_s: float) -> bool:
        return self.message_age_s(message_name) <= float(max_age_s)

    def message_status(self, message_name: str, max_age_s: float) -> str:
        age = self.message_age_s(message_name)
        if age == float("inf"):
            return "MISSING"
        return "FRESH" if age <= float(max_age_s) else "STALE"

    def record_command_ack(self, command: int, result: int, source_system: int,
                           source_component: int, timestamp_ns: int | None = None,
                           progress: int | None = None,
                           result_param2: int | None = None) -> AckRecord:
        record = AckRecord(int(command), int(result), int(source_system), int(source_component),
                           int(timestamp_ns or time.monotonic_ns()),
                           None if progress is None else int(progress),
                           None if result_param2 is None else int(result_param2))
        with self.ack_lock:
            self.latest_command_ack = record
            self.command_ack_history.append(record)
        return record

    def find_command_ack(self, command: int, since_ns: int,
                         source_system: int | None = None,
                         source_component: int | None = None) -> AckRecord | None:
        with self.ack_lock:
            for ack in reversed(self.command_ack_history):
                if ack.timestamp_ns < int(since_ns): break
                if ack.command != int(command): continue
                if source_system is not None and ack.source_system != int(source_system): continue
                if source_component is not None and ack.source_component != int(source_component): continue
                return ack
        return None

    def record_status_text(self, text: str, severity: int, source_system: int,
                           source_component: int, timestamp_ns: int | None = None,
                           message_id: int | None = None,
                           chunk_seq: int | None = None) -> StatusTextRecord:
        record = StatusTextRecord(str(text).strip(), int(severity), int(source_system), int(source_component),
                                  int(timestamp_ns or time.monotonic_ns()),
                                  None if message_id is None else int(message_id),
                                  None if chunk_seq is None else int(chunk_seq))
        with self.status_text_lock:
            self.status_text_history.append(record)
        return record

    def recent_status_texts(self, since_ns: int = 0, source_system: int | None = None,
                            source_component: int | None = None):
        with self.status_text_lock:
            out = []
            for item in self.status_text_history:
                if item.timestamp_ns < int(since_ns): continue
                if source_system is not None and item.source_system != int(source_system): continue
                if source_component is not None and item.source_component != int(source_component): continue
                out.append(item)
            return out

    def rangefinder_age_s(self) -> float:
        if not self.rangefinder_last_ns:
            return float("inf")
        return (time.monotonic_ns() - self.rangefinder_last_ns) / 1e9

    def rangefinder_is_fresh(self, max_age_s: float = 0.15) -> bool:
        return (self.rangefinder_distance_m is not None and self.rangefinder_distance_m > 0.0
                and self.rangefinder_age_s() <= float(max_age_s))

    def rangefinder_altitude_m(self, max_age_s: float = 0.15) -> float | None:
        if not self.rangefinder_is_fresh(max_age_s): return None
        return self.rangefinder_distance_m

    def altitude_reference(self, rangefinder_max_age_s: float = 0.15):
        rf = self.rangefinder_altitude_m(rangefinder_max_age_s)
        if rf is not None: return rf, "RANGEFINDER", True
        if self.relative_alt_m is not None: return self.relative_alt_m, "REL_ALT_FALLBACK", False
        return None, "NONE", False
