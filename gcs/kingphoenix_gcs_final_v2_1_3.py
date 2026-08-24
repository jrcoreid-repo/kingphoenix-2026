#!/usr/bin/env python3
"""Kingphoenix GCS V2.1.5.

Keeps the frozen final operator layout, preserves the safe-set guard,
and shows wlan1 RSSI as a longer color-coded progress bar beside the
connection message.
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


def _create_rssi_themes():
    """Create RSSI progress-bar themes once; switch theme at runtime."""
    specs = {
        "wifi_rssi_theme_green": (55, 190, 85, 255),
        "wifi_rssi_theme_yellow": (225, 175, 35, 255),
        "wifi_rssi_theme_red": (215, 60, 55, 255),
        "wifi_rssi_theme_gray": (95, 95, 100, 255),
    }

    for tag, color in specs.items():
        if dpg.does_item_exist(tag):
            continue
        with dpg.theme(tag=tag):
            with dpg.theme_component(dpg.mvProgressBar):
                dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram, color)
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (28, 32, 36, 255))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 3)


def _create_gui_with_rssi():
    _original_create_gui()
    try:
        _create_rssi_themes()
        dpg.add_spacer(width=12, parent="connection_strip")
        dpg.add_text("WLAN1", parent="connection_strip", color=(145, 180, 220, 255))
        dpg.add_progress_bar(
            default_value=0.0,
            tag="wifi_rssi_progress",
            parent="connection_strip",
            width=240,
            height=18,
            overlay="-- dBm",
        )
        dpg.bind_item_theme("wifi_rssi_progress", "wifi_rssi_theme_gray")
    except Exception as exc:
        print(f"[GCS] RSSI progress widget create warning: {exc}")


def _rssi_to_fraction(rssi):
    """Map -90..-50 dBm into 0..100% for a field-friendly bar."""
    if rssi is None:
        return 0.0

    rssi = float(rssi)
    return max(0.0, min(1.0, (rssi + 90.0) / 40.0))


def _rssi_theme_tag(rssi):
    if rssi is None:
        return "wifi_rssi_theme_gray"

    rssi = float(rssi)
    if rssi >= -67.0:
        return "wifi_rssi_theme_green"
    if rssi >= -75.0:
        return "wifi_rssi_theme_yellow"
    return "wifi_rssi_theme_red"


def _update_info_panel_with_rssi():
    _original_update_info_panel()

    try:
        meta = gcs.get_active_metadata()
        health = (meta or {}).get("pi_health") or {}
        rssi = health.get("wifi_rssi_dbm")

        if dpg.does_item_exist("wifi_rssi_progress"):
            fraction = _rssi_to_fraction(rssi)
            if rssi is None:
                overlay = "-- dBm"
            else:
                overlay = f"{float(rssi):.0f} dBm   {fraction * 100.0:.0f}%"

            _original_set_value("wifi_rssi_progress", fraction)
            dpg.configure_item("wifi_rssi_progress", overlay=overlay)
            dpg.bind_item_theme("wifi_rssi_progress", _rssi_theme_tag(rssi))
    except Exception as exc:
        print(f"[GCS] RSSI progress update warning: {exc}")


def main():
    dpg.set_value = _safe_set_value
    gcs.dpg.set_value = _safe_set_value
    gcs.create_gui = _create_gui_with_rssi
    gcs.update_info_panel = _update_info_panel_with_rssi

    print("[GCS] V2.1.5 wlan1 long RSSI progress bar enabled")
    print("[GCS] RSSI colors: green >= -67, yellow -68..-75, red < -75 dBm")
    gcs.main()


if __name__ == "__main__":
    main()
