from __future__ import annotations

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
    9: "UNSUPPORTED_MAV_FRAME",
}


def mav_result_name(result: int | None) -> str:
    if result is None:
        return "NONE"
    try:
        return MAV_RESULT_NAMES.get(int(result), f"UNKNOWN_{int(result)}")
    except Exception:
        return "UNKNOWN"


def format_mav_result(result: int | None) -> str:
    if result is None:
        return "--"
    return f"{int(result)} ({mav_result_name(result)})"
