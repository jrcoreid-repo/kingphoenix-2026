from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple


@dataclass(frozen=True)
class FreshnessResult:
    message: str
    ok: bool
    age_s: float
    max_age_s: float
    reason: str


class FreshnessPolicy:
    """Central MAVLink freshness policy.

    This module never reads MAVLink directly. It evaluates the timestamps
    maintained by the dedicated MAVLink RX thread in VehicleState.

    Future VCT pre-checks should use this policy rather than hard-coded ages.
    """

    def __init__(self, vehicle_state, max_age_s: Dict[str, float] | None = None):
        self.vehicle_state = vehicle_state
        self.max_age_s = {
            str(name).upper(): float(value)
            for name, value in (max_age_s or {}).items()
        }

    def limit_s(self, message_name: str) -> float | None:
        return self.max_age_s.get(str(message_name).upper())

    def check(self, message_name: str, max_age_s: float | None = None) -> FreshnessResult:
        name = str(message_name).upper()
        limit = self.limit_s(name) if max_age_s is None else float(max_age_s)

        if limit is None:
            return FreshnessResult(
                message=name,
                ok=False,
                age_s=float("inf"),
                max_age_s=float("inf"),
                reason="no freshness policy",
            )

        age = self.vehicle_state.message_age_s(name)

        if age == float("inf"):
            return FreshnessResult(
                message=name,
                ok=False,
                age_s=age,
                max_age_s=limit,
                reason="message never received",
            )

        if age > limit:
            return FreshnessResult(
                message=name,
                ok=False,
                age_s=age,
                max_age_s=limit,
                reason=f"stale ({age:.3f}s > {limit:.3f}s)",
            )

        return FreshnessResult(
            message=name,
            ok=True,
            age_s=age,
            max_age_s=limit,
            reason="fresh",
        )

    def is_fresh(self, message_name: str, max_age_s: float | None = None) -> bool:
        return self.check(message_name, max_age_s).ok

    def require(self, *message_names: str) -> Tuple[bool, str]:
        """Require all named messages to satisfy the configured policy."""
        failures = []
        for name in message_names:
            result = self.check(name)
            if not result.ok:
                failures.append(f"{result.message}: {result.reason}")

        if failures:
            return False, "; ".join(failures)
        return True, ""

    def snapshot(self, message_names: Iterable[str] | None = None):
        names = (
            [str(x).upper() for x in message_names]
            if message_names is not None
            else list(self.max_age_s.keys())
        )
        return {name: self.check(name) for name in names}
