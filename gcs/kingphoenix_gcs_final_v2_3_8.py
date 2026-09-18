#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.8 - persistent waypoint edit session.

Final full-mission workflow:
- ENTER SETUP -> Pi UNLOCK_EDIT once
- SAVE WAYPOINT -> JSON only, no FC mission upload
- EXIT SETUP -> Pi LOCK_EDIT + queue FC mission compare/sync
- WP selector limited to 1..11
"""

from __future__ import annotations

import threading

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_3_5 as v235
import kingphoenix_gcs_final_v2_3_6 as v236
import kingphoenix_gcs_final_v2_3_7 as v237


_setup_lock = threading.Lock()
_setup_busy = False
_wp_lock = threading.Lock()
_wp_busy = False
_active_edit_strategy = None


def _safe_set(tag, value):
    try:
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)
    except Exception:
        pass


def _ack(response, action):
    if not isinstance(response, dict) or not bool(response.get("ok", False)):
        raise RuntimeError(str((response or {}).get("message") or f"{action} rejected"))
    return str(response.get("message") or "ACK")


def _strategy():
    try:
        value = str(dpg.get_value("pos_strat_selector") or "LEFT").upper().strip()
    except Exception:
        value = "LEFT"
    return value if value in ("LEFT", "RIGHT") else "LEFT"


def _setup_worker(entering):
    global _setup_busy, _active_edit_strategy
    try:
        strategy = _strategy() if entering else (_active_edit_strategy or _strategy())
        if entering:
            _safe_set("waypoint_save_status", f"UNLOCKING {strategy} EDIT...")
            response, _ = v235._control_transaction(
                {"type": "UNLOCK_EDIT", "strategy": strategy}, timeout_s=3.0
            )
            message = _ack(response, "UNLOCK_EDIT")
            _active_edit_strategy = strategy
            gcs.edit_mode_unlocked = True
            gcs._set_operator_mode(True)
            try:
                dpg.configure_item("pos_strat_selector", enabled=False)
            except Exception:
                pass
            _safe_set("waypoint_save_status", message + " | STRATEGY LOCKED FOR THIS EDIT SESSION")
        else:
            _safe_set("waypoint_save_status", f"LOCKING {strategy} + SYNC QUEUE...")
            response, _ = v235._control_transaction(
                {"type": "LOCK_EDIT", "strategy": strategy}, timeout_s=3.0
            )
            message = _ack(response, "LOCK_EDIT")
            gcs.edit_mode_unlocked = False
            gcs._set_operator_mode(False)
            _active_edit_strategy = None
            try:
                dpg.configure_item("pos_strat_selector", enabled=True)
            except Exception:
                pass
            _safe_set("waypoint_save_status", message)
    except Exception as exc:
        _safe_set("waypoint_save_status", f"SETUP TX FAILED - {exc}")
        print(f"[GCS-SETUP] ERROR {type(exc).__name__}: {exc}")
    finally:
        with _setup_lock:
            _setup_busy = False


def _toggle_setup_mode_v238():
    global _setup_busy
    if not getattr(gcs, "is_connected", False):
        _safe_set("waypoint_save_status", "BLOCKED - GCS NOT CONNECTED")
        return
    entering = not bool(getattr(gcs, "edit_mode_unlocked", False))
    with _setup_lock:
        if _setup_busy:
            _safe_set("waypoint_save_status", "SETUP COMMAND BUSY")
            return
        _setup_busy = True
    threading.Thread(
        target=_setup_worker,
        args=(entering,),
        name="GCS-SetupMode",
        daemon=True,
    ).start()


def _save_worker(strategy, wp):
    global _wp_busy
    try:
        _safe_set("waypoint_save_status", f"SAVING WP {wp} / {strategy} JSON...")
        response, _ = v235._control_transaction(
            {"type": "SAVE_WAYPOINT", "strategy": strategy, "wp": int(wp)},
            timeout_s=3.0,
        )
        message = _ack(response, "SAVE_WAYPOINT")
        _safe_set("waypoint_save_status", message + " | FC SYNC ON EXIT SETUP")
        print(f"[GCS-WP] {message}")
    except Exception as exc:
        _safe_set("waypoint_save_status", f"SAVE FAILED - {exc}")
        print(f"[GCS-WP] ERROR {type(exc).__name__}: {exc}")
    finally:
        with _wp_lock:
            _wp_busy = False


def _save_waypoint_v238():
    global _wp_busy
    if not getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("waypoint_save_status", "BLOCKED - ENTER SETUP MODE")
        return
    if not getattr(gcs, "is_connected", False):
        _safe_set("waypoint_save_status", "BLOCKED - GCS NOT CONNECTED")
        return
    try:
        wp = int(str(dpg.get_value("waypoint_selector") or "").strip())
    except Exception:
        _safe_set("waypoint_save_status", "BLOCKED - INVALID WAYPOINT")
        return
    strategy = _active_edit_strategy or _strategy()
    if not 1 <= wp <= 11:
        _safe_set("waypoint_save_status", "BLOCKED - WP MUST BE 1..11")
        return
    with _wp_lock:
        if _wp_busy:
            _safe_set("waypoint_save_status", "BUSY - WAIT FOR CURRENT SAVE")
            return
        _wp_busy = True
    threading.Thread(
        target=_save_worker,
        args=(strategy, wp),
        name=f"GCS-WP-{strategy}-{wp}",
        daemon=True,
    ).start()


def main():
    gcs.WP_OPTIONS = [str(i) for i in range(1, 12)]
    gcs.unlock_edit_mode_callback = _toggle_setup_mode_v238
    v236._save_waypoint_callback_v236 = _save_waypoint_v238
    print("[GCS] V2.3.8 FULL-MISSION WAYPOINT EDIT WORKFLOW")
    print("[GCS] SETUP unlock persists; SAVE writes JSON; EXIT SETUP queues FC sync")
    v237.main()


if __name__ == "__main__":
    main()
