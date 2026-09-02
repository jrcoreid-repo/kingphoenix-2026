#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.3 - prominent FC message warning banner.

Extends V2.3.2. Replaces the compact FC STATUSTEXT label with a long,
read-only yellow textbox in the top connection strip so flight-controller
warnings and PreArm/failsafe messages are hard to miss during bench/field use.
Command authority is unchanged.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final_v2_3_2 as v232


_original_create_gui_v232 = v232._create_gui_v232


def _create_fc_message_banner_theme():
    if dpg.does_item_exist("theme_fc_message_banner"):
        return

    with dpg.theme(tag="theme_fc_message_banner"):
        with dpg.theme_component(dpg.mvInputText):
            # Deliberately high-visibility for outdoor/bench operation.
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (255, 220, 45, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (255, 225, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (255, 225, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (10, 10, 10, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Border, (175, 135, 0, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize, 1)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 8, 5)


def _create_gui_v233():
    _original_create_gui_v232()

    try:
        _create_fc_message_banner_theme()

        # V2.3.2 creates fc_message_label as a normal text item. Replace only
        # that value field; keep the existing "FC MESSAGE:" caption and frozen
        # operator layout unchanged.
        if dpg.does_item_exist("fc_message_label"):
            dpg.delete_item("fc_message_label")

        dpg.add_input_text(
            tag="fc_message_label",
            parent="connection_strip",
            default_value="Waiting for FC message...",
            readonly=True,
            multiline=False,
            width=700,
            height=30,
        )
        dpg.bind_item_theme("fc_message_label", "theme_fc_message_banner")

    except Exception as exc:
        print(f"[GCS] V2.3.3 FC message banner warning: {exc}")


def main():
    # Let the complete V2.3.2 wrapper chain run, changing only its GUI creation
    # hook so STATUSTEXT transport/binding remains exactly the validated path.
    v232._create_gui_v232 = _create_gui_v233

    print("[GCS] V2.3.3 HIGH-VISIBILITY FC MESSAGE BANNER")
    print("[GCS] Latest ArduPilot STATUSTEXT shown in long yellow read-only textbox")
    print("[GCS] ARM/DISARM/mode authority unchanged; movement TX remains guarded")

    v232.main()


if __name__ == "__main__":
    main()
