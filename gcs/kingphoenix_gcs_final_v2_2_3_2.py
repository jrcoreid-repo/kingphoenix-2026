#!/usr/bin/env python3
"""Kingphoenix GCS V2.2.3.2 - visible current GPS location.

Extends V2.2.3.1 without changing the frozen operator structure. Adds one
compact Current Location row to the existing SETUP / WAYPOINT card and binds
it to live FC GPS latitude/longitude already present in V2.2.3 metadata.
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
        # Append inside the already-visible SETUP / WAYPOINT card. This keeps
        # the rest of the final operator layout untouched.
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


def _bind_existing_final_tags_v2232(meta):
    diag = _original_bind_existing_final_tags(meta)
    telemetry = (meta or {}).get("telemetry") or {}

    lat = telemetry.get("gps_lat_deg")
    lon = telemetry.get("gps_lon_deg")

    try:
        if dpg.does_item_exist("waypoint_current_location"):
            if lat is None or lon is None:
                dpg.set_value("waypoint_current_location", "--")
            else:
                dpg.set_value(
                    "waypoint_current_location",
                    f"{float(lat):.7f}, {float(lon):.7f}",
                )
    except Exception:
        pass

    return diag


def main():
    # v2231.main() eventually calls v223.main(); patch these hook points before
    # that chain installs the final DearPyGUI callbacks.
    v223._create_gui_v223 = _create_gui_v2232
    v2231._bind_existing_final_tags = _bind_existing_final_tags_v2232

    print("[GCS] V2.2.3.2 CURRENT GPS LOCATION ENABLED")
    print("[GCS] SETUP / WAYPOINT now shows live RAW GPS latitude, longitude")
    print("[GCS] CONTROL TX remains DISABLED")
    v2231.main()


if __name__ == "__main__":
    main()
