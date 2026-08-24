#!/usr/bin/env python3
"""Kingphoenix GCS V2.1.7.

Keeps the frozen final operator layout, preserves the safe-set guard,
shows wlan1 RSSI as a long color-coded progress bar, keeps explicit
GCS<->Pi link lifecycle status, and binds selective read-only MAVLink
telemetry from the Raspberry Pi into the existing FLIGHT / MAVLINK panel.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs


_original_set_value = dpg.set_value
_original_create_gui = gcs.create_gui
_original_update_info_panel = gcs.update_info_panel
_original_shutdown = gcs.shutdown


def _safe_set_value(item, value):
    try:
        if item in (None, 0, ""):
            return None
        if isinstance(item, str) and not dpg.does_item_exist(item):
            return None
    except Exception:
        return None
    try:
        return _original_set_value(item, value)
    except Exception:
        return None


def _create_rssi_themes():
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


def _format_fix_type(fix_type):
    try:
        fix_type = int(fix_type)
    except Exception:
        return "--"
    names = {
        0: "NO GPS",
        1: "NO FIX",
        2: "2D",
        3: "3D",
        4: "DGPS",
        5: "RTK FLOAT",
        6: "RTK FIX",
    }
    return names.get(fix_type, str(fix_type))


def _update_readonly_mavlink(meta):
    telemetry = (meta or {}).get("telemetry") or {}
    connected = bool(telemetry.get("connected", False))

    if not connected:
        _safe_set_value("mav_link", "NOT CONNECTED")
        _safe_set_value("mav_mode", "--")
        _safe_set_value("mav_armed", "--")
        _safe_set_value("mav_battery", "--")
        _safe_set_value("mav_altitude", "--")
        _safe_set_value("mav_speed", "--")
        _safe_set_value("mav_gps", "--")
        return

    hb_age = telemetry.get("heartbeat_age_s")
    link_text = "CONNECTED"
    if hb_age is not None:
        link_text = f"CONNECTED / HB {float(hb_age):.2f}s"
    _safe_set_value("mav_link", link_text)

    _safe_set_value("mav_mode", str(telemetry.get("mode") or "UNKNOWN"))
    _safe_set_value("mav_armed", "ARMED" if telemetry.get("armed") else "DISARMED")

    batt_v = telemetry.get("battery_v")
    batt_pct = telemetry.get("battery_remaining")
    batt_parts = []
    if batt_v is not None:
        batt_parts.append(f"{float(batt_v):.2f} V")
    if batt_pct is not None:
        batt_parts.append(f"{int(batt_pct)}%")
    _safe_set_value("mav_battery", " / ".join(batt_parts) if batt_parts else "--")

    alt = telemetry.get("relative_alt_m")
    _safe_set_value("mav_altitude", "--" if alt is None else f"{float(alt):.2f} m")

    speed = telemetry.get("groundspeed_mps")
    _safe_set_value("mav_speed", "--" if speed is None else f"{float(speed):.2f} m/s")

    sats = telemetry.get("gps_satellites")
    fix_type = telemetry.get("gps_fix_type")
    if sats is None and fix_type is None:
        gps_text = "--"
    else:
        gps_text = f"{_format_fix_type(fix_type)} / {sats if sats is not None else '--'} SAT"
    _safe_set_value("mav_gps", gps_text)

    lidar = telemetry.get("rangefinder_m")
    if lidar is not None:
        _safe_set_value("value_rangefinder", f"{float(lidar):.2f} m")


def _update_info_panel_with_rssi_and_mavlink():
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

        _update_readonly_mavlink(meta)
    except Exception as exc:
        print(f"[GCS] telemetry update warning: {exc}")


def _shutdown_with_link_status():
    if getattr(gcs, "is_connected", False):
        print("[LINK] GCS close requested -> sending DISCONNECT to Raspberry Pi")
    else:
        print("[LINK] GCS close requested -> Pi link already disconnected")

    try:
        _original_shutdown()
    finally:
        print("[LINK] GCS network session CLOSED locally")
        print("[LINK] Raspberry Pi should report GRACEFUL CLOSE; heartbeat watchdog is fallback")


def main():
    dpg.set_value = _safe_set_value
    gcs.dpg.set_value = _safe_set_value
    gcs.create_gui = _create_gui_with_rssi
    gcs.update_info_panel = _update_info_panel_with_rssi_and_mavlink
    gcs.shutdown = _shutdown_with_link_status

    print("[GCS] V2.1.7 RSSI + supervised link + read-only MAVLink telemetry enabled")
    print("[GCS] MAVLink fields: link/mode/armed/battery/altitude/speed/GPS/LiDAR")
    print("[GCS] CONTROL TX remains DISABLED")
    gcs.main()


if __name__ == "__main__":
    main()
