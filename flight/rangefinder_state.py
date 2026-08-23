from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RangefinderCheck:
    ok: bool
    distance_m: float | None
    source: str
    age_s: float
    reason: str


class RangefinderStatePolicy:
    """Validates the central VehicleState rangefinder measurement.

    The TFmini Plus is expected to be connected to the flight controller
    through I2C. Kingphoenix therefore reads the LiDAR through MAVLink, not
    directly from the Raspberry Pi I2C bus.
    """

    def __init__(
        self,
        vehicle_state,
        *,
        max_age_s=0.15,
        expected_orientation=25,
        allow_legacy_fallback=True,
    ):
        self.vehicle_state = vehicle_state
        self.max_age_s = float(max_age_s)
        self.expected_orientation = int(expected_orientation)
        self.allow_legacy_fallback = bool(allow_legacy_fallback)

    def check(self) -> RangefinderCheck:
        v = self.vehicle_state
        age = v.rangefinder_age_s()
        distance = v.rangefinder_distance_m
        source = v.rangefinder_source or "NONE"

        if distance is None or distance <= 0:
            return RangefinderCheck(False, distance, source, age, "no valid distance")

        if age > self.max_age_s:
            return RangefinderCheck(
                False, distance, source, age,
                f"stale ({age:.3f}s > {self.max_age_s:.3f}s)"
            )

        if source == "DISTANCE_SENSOR":
            if (
                v.rangefinder_orientation is not None
                and v.rangefinder_orientation != self.expected_orientation
            ):
                return RangefinderCheck(
                    False, distance, source, age,
                    f"unexpected orientation {v.rangefinder_orientation}, "
                    f"expected {self.expected_orientation}"
                )

            if v.rangefinder_min_m is not None and distance < v.rangefinder_min_m:
                return RangefinderCheck(False, distance, source, age, "below sensor minimum")

            if v.rangefinder_max_m is not None and distance > v.rangefinder_max_m:
                return RangefinderCheck(False, distance, source, age, "above sensor maximum")

            return RangefinderCheck(True, distance, source, age, "valid")

        if source == "RANGEFINDER":
            if self.allow_legacy_fallback:
                return RangefinderCheck(True, distance, source, age, "valid legacy fallback")
            return RangefinderCheck(False, distance, source, age, "legacy fallback disabled")

        return RangefinderCheck(False, distance, source, age, "unknown source")
