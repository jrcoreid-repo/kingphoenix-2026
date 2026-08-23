from __future__ import annotations

import threading
import time


class MavlinkRxWorker(threading.Thread):
    """The ONLY worker allowed to call recv_match()."""

    daemon = True

    def __init__(
        self,
        connection,
        state,
        events,
        stop_event,
        heartbeat_timeout_s=2.5,
    ):
        super().__init__(name="MAVLinkRX")
        self.connection = connection
        self.state = state
        self.events = events
        self.stop_event = stop_event
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.heartbeat_lost_latched = False

    def _check_heartbeat_loss(self):
        v = self.state.vehicle
        if v.connected and v.heartbeat_age_s() > self.heartbeat_timeout_s:
            if not self.heartbeat_lost_latched:
                self.heartbeat_lost_latched = True
                v.connected = False
                self.events.publish(
                    "HEARTBEAT_LOST",
                    {"age_s": v.heartbeat_age_s()},
                )

    def run(self):
        while not self.stop_event.is_set():
            master = self.connection.master

            if master is None:
                time.sleep(0.2)
                continue

            try:
                msg = master.recv_match(blocking=True, timeout=0.2)
            except Exception:
                self._check_heartbeat_loss()
                time.sleep(0.02)
                continue

            if msg is None:
                self._check_heartbeat_loss()
                continue

            now = time.monotonic_ns()
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

            v = self.state.vehicle
            v.mark_message(msg_type, now)

            try:
                src_sys = int(msg.get_srcSystem())
                src_comp = int(msg.get_srcComponent())
            except Exception:
                src_sys, src_comp = 0, 0

            if msg_type == "HEARTBEAT":
                if (
                    src_sys == self.connection.target_system
                    and src_comp == self.connection.target_component
                ):
                    v.connected = True
                    v.last_heartbeat_ns = now
                    v.autopilot_system_id = src_sys
                    v.autopilot_component_id = src_comp
                    self.heartbeat_lost_latched = False

                    old_armed = v.armed
                    try:
                        v.armed = bool(msg.base_mode & 128)
                    except Exception:
                        pass

                    if old_armed != v.armed:
                        self.events.publish(
                            "ARMED_CHANGED",
                            {"old": old_armed, "new": v.armed},
                        )

                    try:
                        mode = master.flightmode
                        if mode and mode != v.mode:
                            old_mode = v.mode
                            v.mode = mode
                            self.events.publish(
                                "MODE_CHANGED",
                                {"old": old_mode, "new": mode},
                            )
                    except Exception:
                        pass

            elif msg_type == "LOCAL_POSITION_NED":
                v.x = float(msg.x)
                v.y = float(msg.y)
                v.z = float(msg.z)
                v.last_position_ns = now

            elif msg_type == "GLOBAL_POSITION_INT":
                v.relative_alt_m = float(msg.relative_alt) / 1000.0

            elif msg_type == "ATTITUDE":
                v.roll = float(msg.roll)
                v.pitch = float(msg.pitch)
                v.yaw = float(msg.yaw)
                v.last_attitude_ns = now

            elif msg_type == "VFR_HUD":
                v.groundspeed_mps = float(msg.groundspeed)

            elif msg_type == "SYS_STATUS":
                voltage = getattr(msg, "voltage_battery", 0)
                if voltage not in (0, 65535):
                    v.battery_v = float(voltage) / 1000.0
                remaining = getattr(msg, "battery_remaining", -1)
                if remaining >= 0:
                    v.battery_remaining = int(remaining)

            elif msg_type == "GPS_RAW_INT":
                sat = getattr(msg, "satellites_visible", 255)
                if sat != 255:
                    v.gps_satellites = int(sat)
                fix_type = getattr(msg, "fix_type", None)
                if fix_type is not None:
                    v.gps_fix_type = int(fix_type)

            elif msg_type == "DISTANCE_SENSOR":
                current_cm = int(getattr(msg, "current_distance", 0))
                min_cm = int(getattr(msg, "min_distance", 0))
                max_cm = int(getattr(msg, "max_distance", 0))
                orientation = int(getattr(msg, "orientation", 0))
                sensor_id = int(getattr(msg, "id", 0))

                if current_cm > 0:
                    v.rangefinder_distance_m = current_cm / 100.0
                    v.rangefinder_min_m = min_cm / 100.0 if min_cm > 0 else None
                    v.rangefinder_max_m = max_cm / 100.0 if max_cm > 0 else None
                    v.rangefinder_orientation = orientation
                    v.rangefinder_id = sensor_id

                    covariance = getattr(msg, "covariance", None)
                    v.rangefinder_covariance = None if covariance is None else int(covariance)

                    quality = getattr(msg, "signal_quality", None)
                    v.rangefinder_signal_quality = None if quality is None else int(quality)

                    v.rangefinder_source = "DISTANCE_SENSOR"
                    v.rangefinder_last_ns = now

            elif msg_type == "RANGEFINDER":
                distance = float(getattr(msg, "distance", 0.0))
                if distance > 0.0:
                    if v.message_age_s("DISTANCE_SENSOR") > 0.15:
                        v.rangefinder_distance_m = distance
                        v.rangefinder_source = "RANGEFINDER"
                        v.rangefinder_last_ns = now

            elif msg_type == "MISSION_CURRENT":
                v.current_mission_item = int(msg.seq)

            elif msg_type == "MISSION_ITEM_REACHED":
                self.events.publish(
                    "MISSION_ITEM_REACHED",
                    {
                        "seq": int(msg.seq),
                        "source_system": src_sys,
                        "source_component": src_comp,
                    },
                )

            elif msg_type == "STATUSTEXT":
                raw_text = getattr(msg, "text", "")
                if isinstance(raw_text, (bytes, bytearray)):
                    raw_text = raw_text.decode("utf-8", errors="replace")
                text = str(raw_text).rstrip("\x00").strip()

                record = v.record_status_text(
                    text=text,
                    severity=int(getattr(msg, "severity", 255)),
                    source_system=src_sys,
                    source_component=src_comp,
                    timestamp_ns=now,
                    message_id=getattr(msg, "id", None),
                    chunk_seq=getattr(msg, "chunk_seq", None),
                )

                self.events.publish(
                    "STATUSTEXT",
                    {
                        "text": record.text,
                        "severity": record.severity,
                        "source_system": record.source_system,
                        "source_component": record.source_component,
                        "timestamp_ns": record.timestamp_ns,
                    },
                )

            elif msg_type == "COMMAND_ACK":
                record = v.record_command_ack(
                    command=int(msg.command),
                    result=int(msg.result),
                    source_system=src_sys,
                    source_component=src_comp,
                    timestamp_ns=now,
                    progress=getattr(msg, "progress", None),
                    result_param2=getattr(msg, "result_param2", None),
                )

                self.events.publish(
                    "COMMAND_ACK",
                    {
                        "command": record.command,
                        "result": record.result,
                        "source_system": record.source_system,
                        "source_component": record.source_component,
                        "timestamp_ns": record.timestamp_ns,
                    },
                )

            self._check_heartbeat_loss()
