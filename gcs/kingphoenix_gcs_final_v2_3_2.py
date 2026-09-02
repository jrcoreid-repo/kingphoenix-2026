#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.2 - latest FC STATUSTEXT debug display.

Extends V2.3.1 and keeps the guarded setup-mode ARM/DISARM/mode controls.
Adds one compact FC MESSAGE string in the top connection strip. The message is
read-only telemetry from ArduPilot STATUSTEXT and is color-coded by MAVLink
severity for fast bench debugging.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_2_3_1 as v2231
import kingphoenix_gcs_final_v2_3_1 as v231


_original_create_gui_v231 = v231._create_gui_v231
_original_bind_existing_final_tags = v2231._bind_existing_final_tags


def _severity_color(severity):
    try:
        sev = int(severity)
    except Exception:
        return (150, 160, 170, 255)

    # MAV_SEVERITY: lower numeric value = more severe.
    if sev <= 3:   # emergency / alert / critical / error
        return (245, 80, 80, 255)
    if sev == 4:   # warning
        return (245, 190, 70, 255)
    if sev == 5:   # notice
        return (110, 190, 255, 255)
    return (175, 185, 195, 255)  # info / debug


def _create_gui_v232():
    _original_create_gui_v231()

    try:
        if not dpg.does_item_exist("fc_message_label"):
            dpg.add_spacer(width=12, parent="connection_strip")
            dpg.add_text(
                "FC MESSAGE:",
                parent="connection_strip",
                color=(145, 180, 220, 255),
            )
            dpg.add_text(
                "--",
                tag="fc_message_label",
                parent="connection_strip",
                color=(150, 160, 170, 255),
            )
    except Exception as exc:
        print(f"[GCS] V2.3.2 FC message widget warning: {exc}")


def _bind_existing_final_tags_v232(meta):
    diag = _original_bind_existing_final_tags(meta)
    telemetry = (meta or {}).get("telemetry") or {}

    text = telemetry.get("fc_message")
    severity = telemetry.get("fc_message_severity")
    age_s = telemetry.get("fc_message_age_s")

    try:
        if dpg.does_item_exist("fc_message_label"):
            if not text:
                dpg.set_value("fc_message_label", "--")
                dpg.configure_item(
                    "fc_message_label",
                    color=(150, 160, 170, 255),
                )
            else:
                # Keep the top strip compact; retain enough text for common
                # PreArm/Arm/failsafe diagnostics without building a log pane.
                compact = str(text).replace("\r", " ").replace("\n", " ").strip()
                if len(compact) > 92:
                    compact = compact[:89] + "..."

                suffix = ""
                try:
                    if age_s is not None and float(age_s) < 10.0:
                        suffix = f"  ({float(age_s):.1f}s)"
                except Exception:
                    pass

                dpg.set_value("fc_message_label", compact + suffix)
                dpg.configure_item(
                    "fc_message_label",
                    color=_severity_color(severity),
                )
    except Exception:
        pass

    return diag


def main():
    # Patch the existing wrapper chain before it installs the frozen GCS hooks.
    v231._create_gui_v231 = _create_gui_v232
    v2231._bind_existing_final_tags = _bind_existing_final_tags_v232

    print("[GCS] V2.3.2 FC STATUSTEXT DEBUG BAR")
    print("[GCS] Top strip shows latest ArduPilot FC message + MAV severity color")
    print("[GCS] Useful for PreArm / Arm / failsafe / sensor warning diagnosis")
    print("[GCS] Command authority unchanged from guarded V2.3.1")

    v231.main()


if __name__ == "__main__":
    main()
