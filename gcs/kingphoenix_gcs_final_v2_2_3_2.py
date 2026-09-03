#!/usr/bin/env python3
"""Kingphoenix GCS V2.2.3.2 - visible current vehicle location.

Extends V2.2.3.1 without changing the frozen operator structure. Current
Location prefers the FC navigation/EKF GLOBAL_POSITION_INT position and shows
source/rate freshness. GPS_RAW_INT is retained only as a fallback, but no
coordinate is presented as valid until ArduPilot reports at least a 3D fix.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final_v2_2_3 as v223
import kingphoenix_gcs_final_v2_2_3_1 as v2231


_original_create_gui_v223 = v223._create_gui_v223
_original_bind_existing_final_tags = v2231._bind_existing_final_tags


def _create_gui_v2232():
    _original_create_gui_v223()

    try:
        if not dpg.does_item_exist("waypoint_current_location"):
            dpg.add_spacer(height=3, parent="waypoint_control_card")
            with dpg.table(
                parent="waypoint_control_card",
                header_row=False,
                policy=dpg.mvTable_SizingStretchProp,
                borders_innerH=False,
                borders_outerH=False,
                borders_innerV=False,
                borders_outerV=False,
            ):
                dpg.add_table_column(init_width_or_weight=0.42)
                dpg.add_table_column(init_width_or_weight=0.58)
                with dpg.table_row():
                    dpg.add_text("Current Location")
                    dpg.add_text("--", tag="waypoint_current_location")
    except Exception as exc:
        print(f"[GCS] V2.2.3.2 location widget warning: {exc}")


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def _safe_int(value):
    try:
        return int(value)
    except Exception:
        return None


def _fix_label(fix_type):
    return {
        0: "NO GPS",
        1: "NO FIX",
        2: "2D FIX",
        3: "3D FIX",
        4: "DGPS",
        5: "RTK FLOAT",
        6: "RTK FIX",
    }.get(fix_type, "UNKNOWN FIX")


def _bind_existing_final_tags_v2232(meta):
    diag = _original_bind_existing_final_tags(meta)
    telemetry = (meta or {}).get("telemetry") or {}
    rates = telemetry.get("rx_rate_hz") or {}

    fix_type = _safe_int(telemetry.get("gps_fix_type"))
    sats = _safe_int(telemetry.get("gps_satellites"))
    gps_valid = fix_type is not None and fix_type >= 3

    # Do not let a fresh MAVLink packet make an invalid/stale receiver
    # coordinate look usable. GPS_RAW_INT can keep streaming at 10 Hz even
    # while the receiver has NO GPS / NO FIX / only 2D fix.
    if not gps_valid:
        fix_text = _fix_label(fix_type)
        sats_text = "--" if sats is None else str(sats)
        try:
            if dpg.does_item_exist("waypoint_current_location"):
                dpg.set_value(
                    "waypoint_current_location",
                    f"{fix_text} / {sats_text} SAT | waiting valid position",
                )
        except Exception:
            pass
        return diag

    # Primary source = navigation/EKF vehicle position used by ArduPilot.
    lat = _safe_float(telemetry.get("global_lat_deg"))
    lon = _safe_float(telemetry.get("global_lon_deg"))
    age = _safe_float(telemetry.get("global_position_age_s"))
    rate = _safe_float(rates.get("GLOBAL_POSITION_INT")) or 0.0
    source = "EKF"
    fresh = bool(telemetry.get("global_position_fresh", False))

    # Fallback to raw GPS only after a valid 3D-or-better fix exists.
    if lat is None or lon is None or not fresh:
        raw_lat = _safe_float(telemetry.get("gps_lat_deg"))
        raw_lon = _safe_float(telemetry.get("gps_lon_deg"))
        raw_age = _safe_float(telemetry.get("gps_raw_age_s"))
        raw_fresh = bool(telemetry.get("gps_raw_fresh", False))
        if raw_lat is not None and raw_lon is not None and raw_fresh:
            lat, lon, age = raw_lat, raw_lon, raw_age
            rate = _safe_float(rates.get("GPS_RAW_INT")) or 0.0
            source = "GPS"
            fresh = True

    try:
        if dpg.does_item_exist("waypoint_current_location"):
            fix_text = _fix_label(fix_type)
            if lat is None or lon is None:
                dpg.set_value(
                    "waypoint_current_location",
                    f"{fix_text} | POSITION unavailable",
                )
            elif fresh:
                dpg.set_value(
                    "waypoint_current_location",
                    f"{lat:.7f}, {lon:.7f} | {source} {rate:.1f}Hz | {fix_text}",
                )
            else:
                age_text = "--" if age is None else f"{age:.1f}s"
                dpg.set_value(
                    "waypoint_current_location",
                    f"STALE {age_text} | {lat:.7f}, {lon:.7f} | {fix_text}",
                )
    except Exception:
        pass

    return diag


def main():
    # v2231.main() eventually calls v223.main(); patch these hook points before
    # that chain installs the final DearPyGUI callbacks.
    v223._create_gui_v223 = _create_gui_v2232
    v2231._bind_existing_final_tags = _bind_existing_final_tags_v2232

    print("[GCS] V2.2.3.2 LIVE CURRENT LOCATION ENABLED")
    print("[GCS] Current Location requires GPS 3D fix or better")
    print("[GCS] Current Location prefers GLOBAL_POSITION_INT / EKF")
    print("[GCS] RAW GPS is fallback only after valid fix")
    print("[GCS] CONTROL TX remains DISABLED")
    v2231.main()


if __name__ == "__main__":
    main()
