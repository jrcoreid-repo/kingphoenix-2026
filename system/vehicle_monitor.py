from __future__ import annotations

import math
import threading


WATCH_MESSAGES = [
    "HEARTBEAT",
    "ATTITUDE",
    "LOCAL_POSITION_NED",
    "GLOBAL_POSITION_INT",
    "GPS_RAW_INT",
    "VFR_HUD",
    "SYS_STATUS",
    "MISSION_CURRENT",
    "DISTANCE_SENSOR",
    "RANGEFINDER",
]


class VehicleMonitor(threading.Thread):
    """Read-only terminal observability for central VehicleState.

    V1.3 adds centralized freshness-policy status. This worker never calls
    recv_match(); flight/mavlink_rx.py remains the sole MAVLink reader.
    """

    daemon = True

    def __init__(
        self,
        state,
        stop_event,
        requested_rates_hz=None,
        freshness_policy=None,
        hz=2.0,
    ):
        super().__init__(name="VehicleMonitor")
        self.state = state
        self.stop_event = stop_event
        self.period = 1.0 / float(hz)
        self.freshness = freshness_policy
        self.requested_rates_hz = {
            str(k).upper(): float(v)
            for k, v in (requested_rates_hz or {}).items()
        }

    @staticmethod
    def _deg(value):
        return None if value is None else math.degrees(value)

    @staticmethod
    def _fmt(value, digits=2):
        if value is None:
            return "--"
        return f"{value:.{digits}f}"

    @staticmethod
    def _age(value):
        if math.isinf(value):
            return "--"
        return f"{value:.3f}s"

    def _freshness_status(self, name):
        if self.freshness is None:
            return "--", None
        result = self.freshness.check(name)
        if result.max_age_s == float("inf"):
            return "NO-POLICY", None
        if result.age_s == float("inf"):
            return "MISSING", result.max_age_s
        return ("FRESH" if result.ok else "STALE"), result.max_age_s

    def run(self):
        while not self.stop_event.is_set():
            v = self.state.vehicle

            print("\n================ VEHICLE STATE ================")
            print(f"CONNECTED : {v.connected}")
            print(f"HB AGE    : {self._age(v.heartbeat_age_s())}")
            print(f"MODE      : {v.mode}")
            print(f"ARMED     : {v.armed}")

            print("\n--- MAVLINK IDENTITY ---")
            print(
                "AUTOPILOT : "
                + (
                    "--"
                    if v.autopilot_system_id is None
                    else f"{v.autopilot_system_id}:{v.autopilot_component_id}"
                )
            )
            print(
                f"KINGPHOENIX: "
                f"{self.state.system.get('mav_source_system', '--')}:"
                f"{self.state.system.get('mav_source_component', '--')}"
            )
            print(
                f"TARGET    : "
                f"{self.state.system.get('mav_target_system', '--')}:"
                f"{self.state.system.get('mav_target_component', '--')}"
            )

            print("\n--- GPS / POSITION ---")
            print(f"GPS FIX   : {v.gps_fix_type if v.gps_fix_type is not None else '--'}")
            print(f"SAT       : {v.gps_satellites if v.gps_satellites is not None else '--'}")
            print(f"X/Y/Z     : {self._fmt(v.x)} / {self._fmt(v.y)} / {self._fmt(v.z)} m")
            print(f"REL ALT   : {self._fmt(v.relative_alt_m)} m")

            print("\n--- ATTITUDE ---")
            print(f"ROLL      : {self._fmt(self._deg(v.roll))} deg")
            print(f"PITCH     : {self._fmt(self._deg(v.pitch))} deg")
            print(f"YAW       : {self._fmt(self._deg(v.yaw))} deg")
            print(f"GROUNDSPD : {self._fmt(v.groundspeed_mps)} m/s")
            print(f"WP        : {v.current_mission_item if v.current_mission_item is not None else '--'}")

            print("\n--- RANGEFINDER / ALTITUDE ---")
            rf_age = v.rangefinder_age_s()
            print(
                f"DISTANCE  : "
                f"{self._fmt(v.rangefinder_distance_m)} m"
            )
            print(
                f"SOURCE    : "
                f"{v.rangefinder_source if v.rangefinder_source else '--'}"
            )
            print(
                f"RF AGE    : {self._age(rf_age)}"
            )
            print(
                f"ORIENT    : "
                f"{v.rangefinder_orientation if v.rangefinder_orientation is not None else '--'}"
            )
            print(
                f"SENSOR ID : "
                f"{v.rangefinder_id if v.rangefinder_id is not None else '--'}"
            )
            alt_value, alt_source, alt_primary = v.altitude_reference(0.15)
            print(
                f"ALT REF   : {self._fmt(alt_value)} m  "
                f"[{alt_source}]"
            )

            print("\n--- POWER ---")
            print(f"BAT V     : {self._fmt(v.battery_v)} V")
            print(f"BAT %     : {v.battery_remaining if v.battery_remaining is not None else '--'}")

            print("\n--- MAVLINK RX HEALTH / FRESHNESS ---")
            print("MESSAGE                ACTUAL    TARGET    AGE      LIMIT    STATUS")
            for name in WATCH_MESSAGES:
                actual = v.message_rate_hz(name)
                target = self.requested_rates_hz.get(name)
                age = v.message_age_s(name)
                status, limit = self._freshness_status(name)

                actual_s = "--" if actual <= 0 else f"{actual:5.1f}Hz"
                target_s = "--" if target is None else f"{target:5.1f}Hz"
                limit_s = "--" if limit is None else f"{limit:.3f}s"

                print(
                    f"{name:<22} "
                    f"{actual_s:>8}  "
                    f"{target_s:>8}  "
                    f"{self._age(age):>8}  "
                    f"{limit_s:>8}  "
                    f"{status:>8}"
                )

            print("===============================================")
            self.stop_event.wait(self.period)
