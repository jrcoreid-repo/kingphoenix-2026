#!/usr/bin/env python3
"""Kingphoenix GCS V2.1.4.

Keeps the frozen final operator layout, preserves the safe-set guard,
and shows wlan1 RSSI as a compact progress bar beside the connection message.
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
        dpg.add_progress_bar(
            default_value=0.0,
            tag="wifi_rssi_progress",
            parent="connection_strip",
            width=120,
            height=18,
            overlay="-- dBm",
        )
    except Exception as exc:
        print(f"[GCS] RSSI progress widget create warning: {exc}")


def _rssi_to_fraction(rssi):
    """Map Wi-Fi RSSI to an operator-friendly 0..1 link-quality bar."""
    if rssi is None:
        return 0.0

    rssi = float(rssi)
    # Practical field mapping:
    # -90 dBm or worse = 0%, -50 dBm or better = 100%.
    return max(0.0, min(1.0, (rssi + 90.0) / 40.0))


def _rssi_color(rssi):
    if rssi is None:
        return (95, 95, 100, 255)
    rssi = float(rssi)
    if rssi >= -67:
        return (55, 185, 85, 255)       # green
    if rssi >= -74:
        return (215, 165, 35, 255)      # yellow
    return (205, 60, 55, 255)           # red


def _apply_progress_color(color):
    """Create/reuse a tiny progress-bar theme for the current RSSI band."""
    theme_tag = "wifi_rssi_theme"
    if dpg.does_item_exist(theme_tag):
        try:
            dpg.delete_item(theme_tag)
        except Exception:
            pass

    with dpg.theme(tag=theme_tag):
        with dpg.theme_component(dpg.mvProgressBar):
            dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram, color)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (35, 40, 45, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 3)

    dpg.bind_item_theme("wifi_rssi_progress", theme_tag)


def _update_info_panel_with_rssi():
    _original_update_info_panel()

    try:
        meta = gcs.get_active_metadata()
        health = (meta or {}).get("pi_health") or {}
        rssi = health.get("wifi_rssi_dbm")

        if dpg.does_item_exist("wifi_rssi_progress"):
            fraction = _rssi_to_fraction(rssi)
            overlay = "-- dBm" if rssi is None else f"{float(rssi):.0f} dBm"
            _original_set_value("wifi_rssi_progress", fraction)
            dpg.configure_item("wifi_rssi_progress", overlay=overlay)
            _apply_progress_color(_rssi_color(rssi))
    except Exception as exc:
        print(f"[GCS] RSSI progress update warning: {exc}")


def main():
    dpg.set_value = _safe_set_value
    gcs.dpg.set_value = _safe_set_value
    gcs.create_gui = _create_gui_with_rssi
    gcs.update_info_panel = _update_info_panel_with_rssi

    print("[GCS] V2.1.4 wlan1 RSSI progress bar enabled")
    print("[GCS] RSSI colors: green >= -67, yellow >= -74, red < -74 dBm")
    gcs.main()


if __name__ == "__main__":
    main()
