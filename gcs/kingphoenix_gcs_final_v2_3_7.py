#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.7 - operator controls TX to Raspberry Pi.

Extends V2.3.6 without redesigning the frozen operator layout.

This release closes the last UI-only control gaps:
- START L / START R -> SET_STRATEGY + START_MISSION over UDP control
- ABORT -> ABORT_MISSION over UDP control
- strategy selector -> SET_STRATEGY over UDP control
- SERVO OPEN / CLOSE / TEST -> SERVO_CONTROL over UDP control

The Raspberry Pi remains authoritative. A positive transport ACK means the Pi
accepted the request; it does not imply that a backend which is explicitly
reported as PENDING has actuated hardware.
"""

from __future__ import annotations

import threading

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_3_5 as v235
import kingphoenix_gcs_final_v2_3_6 as v236


_mission_lock = threading.Lock()
_mission_busy = False
_servo_lock = threading.Lock()
_servo_busy = False


def _safe_set(tag, value):
    try:
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)
    except Exception:
        pass


def _require_ack(response, action):
    if not isinstance(response, dict):
        raise RuntimeError(f"{action}: invalid Pi response")
    if not bool(response.get("ok", False)):
        raise RuntimeError(str(response.get("message") or f"{action} rejected"))
    return str(response.get("message") or "ACK")


def _mission_worker(action):
    global _mission_busy
    try:
        action = str(action).upper()

        if action in ("START_L", "START_R"):
            strategy = "LEFT" if action == "START_L" else "RIGHT"
            _safe_set("mission_selected_strategy", strategy)
            _safe_set("mission_state", f"SETTING STRATEGY {strategy} ...")

            response, sender = v235._control_transaction(
                {"type": "SET_STRATEGY", "strategy": strategy},
                timeout_s=3.0,
            )
            _require_ack(response, "SET_STRATEGY")

            _safe_set("mission_state", f"START {strategy} - WAITING PI ACK")
            response, sender = v235._control_transaction(
                {"type": "START_MISSION"},
                timeout_s=3.0,
            )
            message = _require_ack(response, "START_MISSION")
            _safe_set("mission_state", f"START {strategy} ACK - {message}")
            print(
                f"[GCS-MISSION] START {strategy} ACK via Pi "
                f"{sender[0]}:{sender[1]} | {message}"
            )
            return

        if action == "ABORT":
            _safe_set("mission_state", "ABORT - WAITING PI ACK")
            response, sender = v235._control_transaction(
                {"type": "ABORT_MISSION"},
                timeout_s=3.0,
            )
            message = _require_ack(response, "ABORT_MISSION")
            _safe_set("mission_state", f"ABORT ACK - {message}")
            print(
                f"[GCS-MISSION] ABORT ACK via Pi "
                f"{sender[0]}:{sender[1]} | {message}"
            )
            return

    except Exception as exc:
        _safe_set("mission_state", f"MISSION TX FAILED - {exc}")
        print(f"[GCS-MISSION] ERROR {type(exc).__name__}: {exc}")
    finally:
        with _mission_lock:
            _mission_busy = False


def _mission_control_callback_v237(action):
    global _mission_busy
    action = str(action).upper()

    if action == "PREFLIGHT":
        # V2.3.4 patches this symbol to the live Pi-backed implementation.
        gcs.run_preflight_check()
        return

    if getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("mission_state", "BLOCKED - EXIT SETUP MODE")
        return
    if not getattr(gcs, "is_connected", False):
        _safe_set("mission_state", "BLOCKED - GCS NOT CONNECTED")
        return
    if action not in ("START_L", "START_R", "ABORT"):
        _safe_set("mission_state", f"UNKNOWN MISSION ACTION {action}")
        return

    with _mission_lock:
        if _mission_busy:
            _safe_set("mission_state", "MISSION COMMAND BUSY")
            return
        _mission_busy = True

    threading.Thread(
        target=_mission_worker,
        args=(action,),
        name=f"GCS-Mission-{action}",
        daemon=True,
    ).start()


def _strategy_worker(strategy):
    global _mission_busy
    try:
        response, sender = v235._control_transaction(
            {"type": "SET_STRATEGY", "strategy": strategy},
            timeout_s=3.0,
        )
        message = _require_ack(response, "SET_STRATEGY")
        _safe_set("mission_selected_strategy", strategy)
        _safe_set("mission_state", f"STRAT {strategy} ACK - {message}")
        print(
            f"[GCS-MISSION] STRATEGY {strategy} ACK via Pi "
            f"{sender[0]}:{sender[1]}"
        )
    except Exception as exc:
        _safe_set("mission_state", f"STRATEGY TX FAILED - {exc}")
        print(f"[GCS-MISSION] STRATEGY ERROR {type(exc).__name__}: {exc}")
    finally:
        with _mission_lock:
            _mission_busy = False


def _mission_strategy_callback_v237(strategy):
    global _mission_busy
    strategy = str(strategy).upper().strip()
    if strategy not in ("LEFT", "RIGHT"):
        _safe_set("mission_state", "INVALID STRATEGY")
        return
    if not getattr(gcs, "is_connected", False):
        _safe_set("mission_state", "STRATEGY BLOCKED - GCS NOT CONNECTED")
        return

    with _mission_lock:
        if _mission_busy:
            _safe_set("mission_state", "MISSION COMMAND BUSY")
            return
        _mission_busy = True

    threading.Thread(
        target=_strategy_worker,
        args=(strategy,),
        name=f"GCS-Strategy-{strategy}",
        daemon=True,
    ).start()


def _servo_worker(action):
    global _servo_busy
    try:
        _safe_set("servo_state", f"{action} - WAITING PI ACK")
        response, sender = v235._control_transaction(
            {"type": "SERVO_CONTROL", "action": action},
            timeout_s=3.0,
        )
        message = _require_ack(response, f"SERVO_{action}")
        data = response.get("data") or {}
        executed = bool(data.get("executed", False))
        backend = str(data.get("backend") or "UNKNOWN")

        if executed:
            state = f"{action} EXECUTED - {message}"
        else:
            state = f"{action} RX BY PI - BACKEND {backend}"
        _safe_set("servo_state", state)
        print(
            f"[GCS-SERVO] {action} ACK via Pi {sender[0]}:{sender[1]} "
            f"executed={executed} backend={backend} | {message}"
        )
    except Exception as exc:
        _safe_set("servo_state", f"SERVO TX FAILED - {exc}")
        print(f"[GCS-SERVO] ERROR {type(exc).__name__}: {exc}")
    finally:
        with _servo_lock:
            _servo_busy = False


def _servo_control_callback_v237(action):
    global _servo_busy
    action = str(action).upper().strip()

    if not getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("servo_state", "BLOCKED - ENTER SETUP MODE")
        return
    if not getattr(gcs, "is_connected", False):
        _safe_set("servo_state", "BLOCKED - GCS NOT CONNECTED")
        return
    if action not in ("OPEN", "CLOSE", "TEST"):
        _safe_set("servo_state", f"INVALID SERVO ACTION {action}")
        return

    with _servo_lock:
        if _servo_busy:
            _safe_set("servo_state", "SERVO COMMAND BUSY")
            return
        _servo_busy = True

    threading.Thread(
        target=_servo_worker,
        args=(action,),
        name=f"GCS-Servo-{action}",
        daemon=True,
    ).start()


def main():
    # The frozen GUI callbacks resolve these module globals at click time.
    # Patch them before the full V2.3.6 wrapper chain creates the GUI.
    gcs.mission_control_callback = _mission_control_callback_v237
    gcs.mission_strategy_callback = _mission_strategy_callback_v237
    gcs.servo_control_callback = _servo_control_callback_v237

    print("[GCS] V2.3.7 FULL OPERATOR CONTROL TX")
    print("[GCS] START L/R -> Pi SET_STRATEGY + START_MISSION")
    print("[GCS] ABORT -> Pi ABORT_MISSION")
    print("[GCS] SERVO OPEN/CLOSE/TEST -> Pi MAV_CMD_DO_SET_SERVO backend")
    print("[GCS] Mission Planner + RTK transport: direct MAVProxy path")

    v236.main()


if __name__ == "__main__":
    main()
