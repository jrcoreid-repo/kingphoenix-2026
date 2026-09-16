#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.6 - live global waypoint capture.

Extends V2.3.5. The existing SAVE WAYPOINT button now writes the current
ArduPilot GLOBAL_POSITION_INT latitude/longitude to the Raspberry Pi runtime.
The Pi remains authoritative for freshness/validation and persists the point
atomically in config/waypoints.json.

For safety the button performs one atomic edit transaction:
UNLOCK_EDIT -> SAVE_WAYPOINT -> LOCK_EDIT. The Pi is therefore relocked even
while the GCS stays visually in SETUP MODE.
"""

from __future__ import annotations

import threading

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_3_5 as v235


_wp_lock = threading.Lock()
_wp_busy = False


def _safe_set(tag, value):
    try:
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)
    except Exception:
        pass


def _ack_ok(response, action):
    if not bool((response or {}).get("ok", False)):
        message = str((response or {}).get("message") or "unknown error")
        raise RuntimeError(f"{action}: {message}")
    return str((response or {}).get("message") or "OK")


def _save_waypoint_worker(strategy, wp):
    global _wp_busy
    relock_error = None

    try:
        _safe_set("waypoint_save_status", f"SAVING WP {wp} / {strategy} ...")

        response, _ = v235._control_transaction(
            {"type": "UNLOCK_EDIT"}, timeout_s=2.5
        )
        _ack_ok(response, "UNLOCK_EDIT")

        response, sender = v235._control_transaction(
            {
                "type": "SAVE_WAYPOINT",
                "strategy": strategy,
                "wp": int(wp),
            },
            timeout_s=3.0,
        )
        message = _ack_ok(response, "SAVE_WAYPOINT")

        print(
            f"[GCS-WP] SAVE OK strategy={strategy} wp={wp} "
            f"via Pi {sender[0]}:{sender[1]} | {message}"
        )
        _safe_set("waypoint_save_status", message)

    except Exception as exc:
        print(f"[GCS-WP] SAVE FAILED {type(exc).__name__}: {exc}")
        _safe_set("waypoint_save_status", f"SAVE FAILED - {exc}")

    finally:
        # Always try to return the Pi-side waypoint editor to LOCKED state.
        try:
            response, _ = v235._control_transaction(
                {"type": "LOCK_EDIT"}, timeout_s=1.5
            )
            _ack_ok(response, "LOCK_EDIT")
        except Exception as exc:
            relock_error = exc
            print(f"[GCS-WP] WARNING relock failed: {exc}")

        if relock_error is not None:
            current = ""
            try:
                current = str(dpg.get_value("waypoint_save_status") or "")
            except Exception:
                pass
            _safe_set(
                "waypoint_save_status",
                (current + " | WARNING: PI EDIT RELOCK ACK LOST").strip(" |"),
            )

        with _wp_lock:
            _wp_busy = False


def _save_waypoint_callback_v236():
    """Real SAVE WAYPOINT callback used by the packaged Kingphoenix GCS app."""
    global _wp_busy

    if not getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("waypoint_save_status", "BLOCKED - ENTER SETUP MODE")
        return

    if not getattr(gcs, "is_connected", False):
        _safe_set("waypoint_save_status", "BLOCKED - GCS NOT CONNECTED")
        return

    try:
        wp = int(str(dpg.get_value("waypoint_selector") or "").strip())
        strategy = str(dpg.get_value("pos_strat_selector") or "").strip().upper()
    except Exception:
        _safe_set("waypoint_save_status", "BLOCKED - INVALID WAYPOINT SELECTION")
        return

    if wp < 1 or wp > 12 or strategy not in ("LEFT", "RIGHT"):
        _safe_set("waypoint_save_status", "BLOCKED - INVALID WAYPOINT/STRATEGY")
        return

    with _wp_lock:
        if _wp_busy:
            _safe_set("waypoint_save_status", "BUSY - WAIT FOR CURRENT SAVE")
            return
        _wp_busy = True

    threading.Thread(
        target=_save_waypoint_worker,
        args=(strategy, wp),
        name=f"GCS-WP-{strategy}-{wp}",
        daemon=True,
    ).start()


def main():
    # The frozen base create_gui resolves this module-global callback when the
    # SAVE WAYPOINT button is created. Patch it before the V2.3.5 wrapper chain
    # enters gcs.main(). No layout redesign is required.
    gcs.save_waypoint_callback = _save_waypoint_callback_v236

    print("[GCS] V2.3.6 LIVE GLOBAL WAYPOINT CAPTURE")
    print("[GCS] SAVE WAYPOINT -> Pi main.py -> GLOBAL_POSITION_INT lat/lon")
    print("[GCS] Pi edit transaction: UNLOCK -> SAVE -> LOCK")
    print("[GCS] Existing V2.3.5 MP bridge / FC controls remain unchanged")

    v235.main()


if __name__ == "__main__":
    main()
