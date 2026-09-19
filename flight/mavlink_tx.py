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
        try:
            with self.lock:
                mapping = master.mode_mapping()
                if not mapping:
                    return False
                mode_id = mapping.get(str(mode_name).upper())
                if mode_id is None:
                    return False
                master.mav.set_mode_send(self.target_system, 1, mode_id)
            return True
        except Exception:
            return False

    def command_long(self, command, params=None, confirmation=0):
        master = self.with_master()
        if master is None:
            return False
        p = list(params or [])
        p = (p + [0.0] * 7)[:7]
        try:
            with self.lock:
                master.mav.command_long_send(
                    self.target_system, self.target_component, int(command), int(confirmation),
                    float(p[0]), float(p[1]), float(p[2]), float(p[3]),
                    float(p[4]), float(p[5]), float(p[6]),
                )
            return True
        except Exception:
            return False

    def set_message_interval(self, message_id: int, rate_hz: float) -> bool:
        rate_hz = float(rate_hz)
        interval_us = -1 if rate_hz <= 0 else int(round(1_000_000.0 / rate_hz))
        return self.command_long(511, [float(message_id), float(interval_us), 0, 0, 0, 0, 0])

    def arm(self):
        try:
            from pymavlink import mavutil
            return self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0, 0, 0, 0, 0])
        except Exception:
            return False

    def disarm(self):
        try:
            from pymavlink import mavutil
            return self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [0.0, 0.0, 0, 0, 0, 0, 0])
        except Exception:
            return False

    def takeoff(self, altitude_m: float):
        try:
            from pymavlink import mavutil
            return self.command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                [0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, float(altitude_m)],
            )
        except Exception:
            return False

    def send_local_ned_velocity(self, vz_ned_mps: float):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            type_mask = (
                (1 << 0) | (1 << 1) | (1 << 2)
                | (1 << 6) | (1 << 7) | (1 << 8)
                | (1 << 10) | (1 << 11)
            )
            with self.lock:
                master.mav.set_position_target_local_ned_send(
                    0, self.target_system, self.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
                    0.0, 0.0, 0.0, 0.0, 0.0, float(vz_ned_mps),
                    0.0, 0.0, 0.0, 0.0, 0.0,
                )
            return True
        except Exception:
            return False

    def send_body_velocity(self, vx, vy, vz=0.0, yaw_rate=0.0):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            with self.lock:
                master.mav.set_position_target_local_ned_send(
                    0, self.target_system, self.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 1479,
                    0.0, 0.0, 0.0, float(vx), float(vy), float(vz),
                    0.0, 0.0, 0.0, 0.0, float(yaw_rate),
                )
            return True
        except Exception:
            return False

    def send_guided_velocity(self, vx, vy, vz, yaw_rate=0.0):
        return self.send_body_velocity(vx, vy, vz, yaw_rate)

    def set_servo(self, channel: int, pwm: int):
        try:
            from pymavlink import mavutil
            return self.command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                [float(channel), float(pwm), 0, 0, 0, 0, 0],
            )
        except Exception:
            return False

    def mission_request_list(self):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            with self.lock:
                try:
                    master.mav.mission_request_list_send(
                        self.target_system, self.target_component,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                except TypeError:
                    master.mav.mission_request_list_send(self.target_system, self.target_component)
            return True
        except Exception:
            return False

    def mission_request_int(self, seq: int):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            with self.lock:
                try:
                    master.mav.mission_request_int_send(
                        self.target_system, self.target_component, int(seq),
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                except TypeError:
                    master.mav.mission_request_int_send(self.target_system, self.target_component, int(seq))
            return True
        except Exception:
            return False

    def mission_clear_all(self):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            with self.lock:
                try:
                    master.mav.mission_clear_all_send(
                        self.target_system, self.target_component,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                except TypeError:
                    master.mav.mission_clear_all_send(self.target_system, self.target_component)
            return True
        except Exception:
            return False

    def mission_count(self, count: int):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            with self.lock:
                try:
                    master.mav.mission_count_send(
                        self.target_system, self.target_component, int(count),
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                except TypeError:
                    master.mav.mission_count_send(self.target_system, self.target_component, int(count))
            return True
        except Exception:
            return False

    def mission_item_int(self, *, seq: int, lat: float, lon: float, alt_m: float,
                         acceptance_radius_m: float, is_home: bool = False):
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            args = [
                self.target_system, self.target_component, int(seq),
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0,
                0.0 if is_home else float(acceptance_radius_m), 0.0,
                0.0 if is_home else float("nan"),
                int(round(float(lat) * 1e7)), int(round(float(lon) * 1e7)),
                float(alt_m),
            ]
            with self.lock:
                try:
                    master.mav.mission_item_int_send(*args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                except TypeError:
                    master.mav.mission_item_int_send(*args)
            return True
        except Exception:
            return False

    def mission_item_int_raw(
        self,
        *,
        seq: int,
        frame: int,
        command: int,
        current: int = 0,
        autocontinue: int = 1,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = float("nan"),
        lat: float = 0.0,
        lon: float = 0.0,
        alt_m: float = 0.0,
    ):
        """Send an arbitrary MISSION_ITEM_INT while preserving mission semantics."""
        master = self.with_master()
        if master is None:
            return False
        try:
            from pymavlink import mavutil
            args = [
                self.target_system,
                self.target_component,
                int(seq),
                int(frame),
                int(command),
                int(current),
                int(autocontinue),
                float(param1),
                float(param2),
                float(param3),
                float(param4),
                int(round(float(lat) * 1e7)),
                int(round(float(lon) * 1e7)),
                float(alt_m),
            ]
            with self.lock:
                try:
                    master.mav.mission_item_int_send(
                        *args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                except TypeError:
                    master.mav.mission_item_int_send(*args)
            return True
        except Exception:
            return False

    def mission_set_current(self, seq: int):
        master = self.with_master()
        if master is None:
            return False
        try:
            with self.lock:
                master.mav.mission_set_current_send(self.target_system, self.target_component, int(seq))
            return True
        except Exception:
            return False
