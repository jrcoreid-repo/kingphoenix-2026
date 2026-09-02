#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.4 - live Pi-backed PRE-FLIGHT CHECK.

Extends V2.3.3. PRE-FLIGHT no longer uses mock PASS rows. Clicking the button
actively requests a point-in-time snapshot from the Raspberry Pi control
server. Hardware/runtime items are populated from live Pi/FC state. Mission
File and Strategy Data remain explicit WARN/TEST placeholders until their
validation is integrated.
"""

from __future__ import annotations

import json
import threading
import time

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_3_3 as v233


ROW_NAMES = [
    "MAVLink Link",
    "FC Heartbeat",
    "Vehicle State",
    "GPS",
    "LiDAR / Rangefinder",
    "Front Camera",
    "Bottom Camera",
    "YOLO26 / Hailo",
    "Mission File",
    "Strategy Data",
    "Battery",
    "Setup Mode",
    "Pi Thermal",
    "Pi Power",
    "Pi Throttling",
    "Pi CPU Load",
    "Pi RAM",
    "Pi Disk",
]


def _set_row(index, status, detail):
    status = str(status or "WARN").upper()
    detail = str(detail or "--")
    try:
        dpg.set_value(f"preflight_status_{index}", status)
        dpg.set_value(f"preflight_detail_{index}", detail)
        if status == "PASS":
            color = (95, 230, 120, 255)
        elif status == "WARN":
            color = (245, 190, 70, 255)
        else:
            color = (245, 80, 80, 255)
        dpg.configure_item(f"preflight_status_{index}", color=color)
    except Exception:
        pass


def _show_checking_state():
    for i, name in enumerate(ROW_NAMES):
        if name in ("Mission File", "Strategy Data"):
            _set_row(i, "WARN", "TEST STATUS / validation not integrated yet")
        elif name == "Setup Mode":
            setup = bool(getattr(gcs, "edit_mode_unlocked", False))
            _set_row(
                i,
                "FAIL" if setup else "PASS",
                "ON / exit SETUP before flight" if setup else "OFF / mission-safe",
            )
        else:
            _set_row(i, "WARN", "REQUESTING LIVE DATA FROM RASPBERRY PI...")

    try:
        dpg.set_value("preflight_verdict", "STATUS: CHECKING")
        dpg.configure_item("preflight_verdict", color=(110, 190, 255, 255))
        dpg.set_value("preflight_message", "REQUESTING LIVE READINESS SNAPSHOT")
        dpg.configure_item("preflight_message", color=(110, 190, 255, 255))
        dpg.configure_item("preflight_modal", show=True)
        gcs._center_modal("preflight_modal", 710, 720)
    except Exception:
        pass


def _apply_snapshot(snapshot):
    checks = (snapshot or {}).get("checks") or {}

    statuses = []
    for i, name in enumerate(ROW_NAMES):
        if name == "Setup Mode":
            setup = bool(getattr(gcs, "edit_mode_unlocked", False))
            status = "FAIL" if setup else "PASS"
            detail = "ON / exit SETUP before flight" if setup else "OFF / mission-safe"
        else:
            item = checks.get(name) or {}
            status = str(item.get("status", "WARN")).upper()
            detail = str(item.get("detail", "No live result"))

        statuses.append(status)
        _set_row(i, status, detail)

    has_fail = any(x == "FAIL" for x in statuses)
    has_warn = any(x == "WARN" for x in statuses)

    try:
        if has_fail:
            verdict = "STATUS: NOT READY"
            message = "CHECK FAILED LIVE ITEMS BEFORE START"
            color = (245, 80, 80, 255)
            mission_state = "PREFLIGHT NOT READY"
        elif has_warn:
            # Expected while mission/strategy validators are still TEST status.
            verdict = "STATUS: TEST / PARTIAL READY"
            message = "LIVE HARDWARE PASS / TEST ITEMS REMAIN"
            color = (245, 190, 70, 255)
            mission_state = "PREFLIGHT LIVE PASS / TEST WARNINGS"
        else:
            verdict = "STATUS: READY"
            message = "READY TO ROCK AND ROLL"
            color = (95, 230, 120, 255)
            mission_state = "PREFLIGHT READY"

        dpg.set_value("preflight_verdict", verdict)
        dpg.configure_item("preflight_verdict", color=color)
        dpg.set_value("preflight_message", message)
        dpg.configure_item("preflight_message", color=color)
        dpg.set_value("mission_state", mission_state)
    except Exception:
        pass


def _apply_transport_failure(reason):
    # If GCS cannot reach the Pi, every item that must come from Pi/FC is FAIL,
    # not a fake PASS. TEST placeholders stay WARN.
    for i, name in enumerate(ROW_NAMES):
        if name in ("Mission File", "Strategy Data"):
            _set_row(i, "WARN", "TEST STATUS / validation not integrated yet")
        elif name == "Setup Mode":
            setup = bool(getattr(gcs, "edit_mode_unlocked", False))
            _set_row(i, "FAIL" if setup else "PASS",
                     "ON / exit SETUP before flight" if setup else "OFF / mission-safe")
        else:
            _set_row(i, "FAIL", reason)

    try:
        dpg.set_value("preflight_verdict", "STATUS: NOT READY")
        dpg.configure_item("preflight_verdict", color=(245, 80, 80, 255))
        dpg.set_value("preflight_message", "RASPBERRY PI LIVE PREFLIGHT UNAVAILABLE")
        dpg.configure_item("preflight_message", color=(245, 80, 80, 255))
        dpg.set_value("mission_state", "PREFLIGHT LINK FAILED")
    except Exception:
        pass


def _request_live_snapshot():
    if not getattr(gcs, "is_connected", False):
        _apply_transport_failure("GCS not connected to Raspberry Pi")
        return

    tx_id = int(time.time_ns() & 0x7FFFFFFFFFFFFFFF)
    request = {"type": "GET_PREFLIGHT_STATUS", "tx_id": tx_id}

    old_timeout = None
    try:
        with gcs.control_socket_lock:
            try:
                old_timeout = gcs.control_socket.gettimeout()
            except Exception:
                old_timeout = None

            gcs.control_socket.settimeout(3.0)
            gcs.control_socket.sendto(
                json.dumps(request).encode("utf-8"),
                (gcs.drone_ip, gcs.CONTROL_PORT),
            )

            while True:
                data, sender = gcs.control_socket.recvfrom(65535)
                response = json.loads(data.decode("utf-8"))
                if response.get("type") != "ACK":
                    continue
                if response.get("tx_id") != tx_id:
                    continue
                break

        if not bool(response.get("ok", False)):
            raise RuntimeError(str(response.get("message") or "Pi rejected preflight request"))

        snapshot = response.get("data") or {}
        _apply_snapshot(snapshot)
        print(
            "[PREFLIGHT-LIVE] snapshot received "
            f"from={sender[0]}:{sender[1]} "
            f"readiness={snapshot.get('readiness_status')} "
            f"ready={snapshot.get('ready_for_mission')}"
        )

    except Exception as exc:
        print(f"[PREFLIGHT-LIVE] ERROR {type(exc).__name__}: {exc}")
        _apply_transport_failure(f"Pi request failed: {exc}")
    finally:
        try:
            with gcs.control_socket_lock:
                gcs.control_socket.settimeout(old_timeout)
        except Exception:
            pass


def run_preflight_check_live():
    """Operator-triggered live snapshot request to Raspberry Pi."""
    if bool(getattr(gcs, "edit_mode_unlocked", False)):
        # Still show the complete diagnostic modal, but Setup Mode itself will
        # be a real FAIL rather than silently blocking the diagnostic request.
        print("[PREFLIGHT-LIVE] SETUP MODE active; diagnostic will report FAIL")

    print("[PREFLIGHT-LIVE] REQUEST START")
    try:
        dpg.set_value("mission_state", "PRE-FLIGHT LIVE CHECK RUNNING")
    except Exception:
        pass
    _show_checking_state()

    threading.Thread(
        target=_request_live_snapshot,
        name="GCS-LivePreflight",
        daemon=True,
    ).start()


def main():
    # mission_control_callback in the frozen GCS resolves run_preflight_check
    # from its module global at call time, so replacing this hook upgrades the
    # existing PRE-FLIGHT button without changing its layout.
    gcs.run_preflight_check = run_preflight_check_live

    print("[GCS] V2.3.4 LIVE PI-BACKED PRE-FLIGHT CHECK")
    print("[GCS] PRE-FLIGHT button now requests GET_PREFLIGHT_STATUS from Raspberry Pi")
    print("[GCS] No MAVLink/FC connection -> real FAIL, never mock PASS")
    print("[GCS] Mission File + Strategy Data intentionally remain WARN / TEST STATUS")

    v233.main()


if __name__ == "__main__":
    main()
