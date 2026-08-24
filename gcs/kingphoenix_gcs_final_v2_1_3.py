#!/usr/bin/env python3
"""Kingphoenix GCS V2.1.3.

Keeps the frozen final operator layout, preserves the V2.1.2 safe-set guard,
and adds a compact wlan1 RSSI indicator beside the connection message.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs


_original_set_value = dpg.set_value
_original_create_gui = gcs.create_gui
_original_update_info_panel = gcs.update_info_panel


def _safe_set_value(item, value):
    try:
        if item in (None, 0, ""):
            return None
        if isinstance(item, str) and not dpg.does_item_exist(item):
            return None
    except Exception:
        return None
    return _original_set_value(item, value)


def _create_gui_with_rssi():
    _original_create_gui()
    try:
        dpg.add_spacer(width=12, parent="connection_strip")
        dpg.add_text("WLAN1", parent="connection_strip", color=(145, 180, 220, 255))
        dpg.add_text(
            "[----] -- dBm",
            tag="wifi_rssi_badge",
            parent="connection_strip",
            color=(150, 155, 165, 255),
        )
    except Exception as exc:
        print(f"[GCS] RSSI widget create warning: {exc}")


def _rssi_visual(rssi):
    if rssi is None:
        return "[----] -- dBm", (150, 155, 165, 255)

    rssi = float(rssi)
    if rssi >= -60:
        bars, color = "[####]", (95, 230, 120, 255)
    elif rssi >= -67:
        bars, color = "[###-]", (95, 230, 120, 255)
    elif rssi >= -74:
        bars, color = "[##--]", (245, 190, 70, 255)
    elif rssi >= -82:
        bars, color = "[#---]", (245, 105, 70, 255)
    else:
        bars, color = "[!...]", (245, 80, 80, 255)

    return f"{bars} {rssi:.0f} dBm", color


def _update_info_panel_with_rssi():
    _original_update_info_panel()

    try:
        meta = gcs.get_active_metadata()
        health = (meta or {}).get("pi_health") or {}
        rssi = health.get("wifi_rssi_dbm")
        text, color = _rssi_visual(rssi)

        if dpg.does_item_exist("wifi_rssi_badge"):
            _original_set_value("wifi_rssi_badge", text)
            dpg.configure_item("wifi_rssi_badge", color=color)
    except Exception as exc:
        print(f"[GCS] RSSI update warning: {exc}")


def main():
    dpg.set_value = _safe_set_value
    gcs.dpg.set_value = _safe_set_value
    gcs.create_gui = _create_gui_with_rssi
    gcs.update_info_panel = _update_info_panel_with_rssi

    print("[GCS] V2.1.3 wlan1 RSSI indicator enabled")
    print("[GCS] RSSI thresholds: green >= -67, yellow >= -74, red < -74 dBm")
    gcs.main()


if __name__ == "__main__":
    main()
