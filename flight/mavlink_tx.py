from __future__ import annotations

import threading


class MavlinkTx:
    """Single coordinated MAVLink TX path."""

    def __init__(self, connection):
        self.connection = connection
        self.lock = threading.Lock()

    @property
    def target_system(self):
        return self.connection.target_system

    @property
    def target_component(self):
        return self.connection.target_component

    def with_master(self):
        return self.connection.master

    def send_message(self, msg):
        master = self.with_master()
        if master is None or msg is None:
            return False
        try:
            with self.lock:
                master.mav.send(msg)
            return True
        except Exception:
            return False

    def send_mode(self, mode_name: str):
        master = self.with_master()
        if master is None:
            return False
        with self.lock:
            mapping = master.mode_mapping()
            if not mapping:
                return False
            mode_id = mapping.get(str(mode_name).upper())
            if mode_id is None:
                return False
            master.mav.set_mode_send(
                self.target_system,
                1,
                mode_id,
            )
        return True

    def command_long(self, command, params=None, confirmation=0):
        master = self.with_master()
        if master is None:
            return False
        p = list(params or [])
        p = (p + [0.0] * 7)[:7]
        with self.lock:
            master.mav.command_long_send(
                self.target_system,
                self.target_component,
                int(command),
                int(confirmation),
                float(p[0]), float(p[1]), float(p[2]), float(p[3]),
                float(p[4]), float(p[5]), float(p[6]),
            )
        return True

    def set_message_interval(self, message_id: int, rate_hz: float) -> bool:
        rate_hz = float(rate_hz)
        interval_us = -1 if rate_hz <= 0 else int(round(1_000_000.0 / rate_hz))
        return self.command_long(511, [float(message_id), float(interval_us), 0, 0, 0, 0, 0])

    def arm(self):
        try:
            from pymavlink import mavutil
            return self.command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [1.0, 0.0, 0, 0, 0, 0, 0],
            )
        except Exception:
            return False

    def disarm(self):
        try:
            from pymavlink import mavutil
            return self.command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0.0, 0.0, 0, 0, 0, 0, 0],
            )
        except Exception:
            return False

    def takeoff(self, altitude_m: float):
        """Send MAV_CMD_NAV_TAKEOFF in GUIDED mode.

        param7 is target altitude in metres. ArduCopter interprets this as the
        takeoff altitude for guided takeoff.
        """
        try:
            from pymavlink import mavutil
            return self.command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(altitude_m)],
            )
        except Exception:
            return False

    def send_guided_velocity(self, vx, vy, vz, yaw_rate=0.0):
        master = self.with_master()
        if master is None:
            return False
        # Intentionally remains disabled until the dedicated velocity tool is
        # bench/flight validated.
        return True
