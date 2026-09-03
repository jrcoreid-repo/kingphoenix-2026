#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.5 - explicit Mission Planner laptop destination.

Extends V2.3.4 without redesigning the frozen operator layout. Adds a compact
Mission Planner bridge control inside the existing SETUP / WAYPOINT card so
Mission Planner may run on a different laptop from Kingphoenix GCS.

The GCS sends only the requested Mission Planner destination IP/port to the Pi.
The Pi gateway remains authoritative for filtering: in RACE policy FC telemetry
is mirrored outward while only GPS_RTCM_DATA is accepted back toward the FC.
"""

from __future__ import annotations

import ipaddress
import json
import threading
import time

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_3_3 as v233
import kingphoenix_gcs_final_v2_3_4 as v234


_original_create_gui_v233 = v233._create_gui_v233
_mp_command_lock = threading.Lock()
_mp_command_busy = False


def _safe_set(tag, value):
    try:
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)
    except Exception:
        pass


def _valid_ipv4(text):
    try:
        ip = ipaddress.ip_address(str(text or "").strip())
    except ValueError:
        return None
    if ip.version != 4 or ip.is_unspecified or ip.is_multicast:
        return None
    if str(ip) == "255.255.255.255":
        return None
    return str(ip)


def _control_transaction(payload, timeout_s=3.0):
    tx_id = int(time.time_ns() & 0x7FFFFFFFFFFFFFFF)
    message = dict(payload)
    message["tx_id"] = tx_id

    old_timeout = None
    with gcs.control_socket_lock:
        try:
            old_timeout = gcs.control_socket.gettimeout()
        except Exception:
            old_timeout = None

        try:
            gcs.control_socket.settimeout(timeout_s)
            gcs.control_socket.sendto(
                json.dumps(message).encode("utf-8"),
                (gcs.drone_ip, gcs.CONTROL_PORT),
            )
            while True:
                data, sender = gcs.control_socket.recvfrom(8192)
                response = json.loads(data.decode("utf-8"))
                if response.get("type") != "ACK":
                    continue
                if response.get("tx_id") != tx_id:
                    continue
                return response, sender
        finally:
            try:
                gcs.control_socket.settimeout(old_timeout)
            except Exception:
                pass


def _set_mp_destination_worker(ip):
    global _mp_command_busy
    try:
        _safe_set("mp_bridge_status", "SETTING...")
        response, sender = _control_transaction(
            {"type": "SET_MP_DESTINATION", "ip": ip, "port": 14555},
            timeout_s=3.0,
        )
        ok = bool(response.get("ok", False))
        message = str(response.get("message") or "--")
        if ok:
            data = response.get("data") or {}
            shown_ip = str(data.get("ip") or ip)
            port = int(data.get("port") or 14555)
            _safe_set("mp_ip_input", shown_ip)
            _safe_set("mp_bridge_status", f"ACTIVE -> {shown_ip}:{port} / RTCM ONLY IN")
            print(
                f"[GCS-MP] destination set {shown_ip}:{port} "
                f"via Pi {sender[0]}:{sender[1]}"
            )
        else:
            _safe_set("mp_bridge_status", f"FAILED - {message}")
            print(f"[GCS-MP] SET FAILED | {message}")
    except Exception as exc:
        _safe_set("mp_bridge_status", f"ERROR - {exc}")
        print(f"[GCS-MP] ERROR {type(exc).__name__}: {exc}")
    finally:
        with _mp_command_lock:
            _mp_command_busy = False


def _set_mp_destination_requested():
    global _mp_command_busy

    if not getattr(gcs, "is_connected", False):
        _safe_set("mp_bridge_status", "BLOCKED - GCS NOT CONNECTED")
        return

    try:
        raw = dpg.get_value("mp_ip_input")
    except Exception:
        raw = ""

    ip = _valid_ipv4(raw)
    if ip is None:
        _safe_set("mp_bridge_status", "INVALID IPv4")
        return

    with _mp_command_lock:
        if _mp_command_busy:
            _safe_set("mp_bridge_status", "BUSY")
            return
        _mp_command_busy = True

    threading.Thread(
        target=_set_mp_destination_worker,
        args=(ip,),
        name="GCS-MP-Destination",
        daemon=True,
    ).start()


def _clear_mp_destination_worker():
    global _mp_command_busy
    try:
        _safe_set("mp_bridge_status", "SWITCHING TO AUTO...")
        response, _ = _control_transaction(
            {"type": "CLEAR_MP_DESTINATION"}, timeout_s=3.0
        )
        if bool(response.get("ok", False)):
            _safe_set("mp_bridge_status", "AUTO / GCS HELLO OR MP PEER")
            print("[GCS-MP] runtime Mission Planner destination override cleared")
        else:
            _safe_set(
                "mp_bridge_status",
                "FAILED - " + str(response.get("message") or "Pi rejected request"),
            )
    except Exception as exc:
        _safe_set("mp_bridge_status", f"ERROR - {exc}")
        print(f"[GCS-MP] AUTO ERROR {type(exc).__name__}: {exc}")
    finally:
        with _mp_command_lock:
            _mp_command_busy = False


def _clear_mp_destination_requested():
    global _mp_command_busy
    if not getattr(gcs, "is_connected", False):
        _safe_set("mp_bridge_status", "BLOCKED - GCS NOT CONNECTED")
        return
    with _mp_command_lock:
        if _mp_command_busy:
            _safe_set("mp_bridge_status", "BUSY")
            return
        _mp_command_busy = True
    threading.Thread(
        target=_clear_mp_destination_worker,
        name="GCS-MP-Auto",
        daemon=True,
    ).start()


def _create_gui_v235():
    _original_create_gui_v233()

    try:
        # Keep the frozen layout intact by adding only one compact network
        # control at the bottom of the existing SETUP / WAYPOINT card.
        dpg.add_separator(parent="waypoint_control_card")
        dpg.add_text(
            "MISSION PLANNER BRIDGE",
            parent="waypoint_control_card",
            color=(145, 180, 220, 255),
        )
        with dpg.group(horizontal=True, parent="waypoint_control_card"):
            dpg.add_input_text(
                tag="mp_ip_input",
                default_value="",
                hint="Mission Planner IP",
                width=155,
            )
            dpg.add_button(
                label="SET",
                width=58,
                callback=_set_mp_destination_requested,
            )
            dpg.add_button(
                label="AUTO",
                width=62,
                callback=_clear_mp_destination_requested,
            )
        dpg.add_text(
            "AUTO / SET IP FOR SEPARATE MP LAPTOP",
            tag="mp_bridge_status",
            parent="waypoint_control_card",
            color=(180, 190, 200, 255),
        )
    except Exception as exc:
        print(f"[GCS] V2.3.5 Mission Planner bridge widget warning: {exc}")


def main():
    # v234.main() eventually calls v233.main(). Patch v233's GUI hook first so
    # all V2.3.4 preflight and V2.3.3 FC-message behavior is preserved.
    v233._create_gui_v233 = _create_gui_v235

    print("[GCS] V2.3.5 MISSION PLANNER DESTINATION CONTROL")
    print("[GCS] SETUP / WAYPOINT card can SET a separate Mission Planner laptop IP")
    print("[GCS] Pi RACE gateway remains FC telemetry OUT + GPS_RTCM_DATA ONLY IN")

    v234.main()


if __name__ == "__main__":
    main()
