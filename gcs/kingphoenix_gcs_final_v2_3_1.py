#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.1 - visual setup-mode interlock for FC bench controls.

Extends V2.3.0. ARM/DISARM/mode controls remain functionally guarded by
SETUP MODE, and now also mirror the existing servo/waypoint visual language:
locked controls are black/dim; entering SETUP MODE makes the manual FC bench
controls visibly active.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_3_0 as v230


_original_create_gui_v230 = v230._create_gui_v230
_original_set_operator_mode_v230 = v230._set_operator_mode_v230


def _create_fc_interlock_themes():
    """Create explicit locked/active themes for the new FC bench controls."""
    if not dpg.does_item_exist("theme_fc_bench_locked_button"):
        with dpg.theme(tag="theme_fc_bench_locked_button"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (5, 5, 5, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (5, 5, 5, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (5, 5, 5, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (105, 105, 105, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (85, 85, 85, 255))

    if not dpg.does_item_exist("theme_fc_bench_active_button"):
        with dpg.theme(tag="theme_fc_bench_active_button"):
            with dpg.theme_component(dpg.mvButton):
                # Same visual family as the manual servo/setup controls.
                dpg.add_theme_color(dpg.mvThemeCol_Button, (35, 105, 155, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (45, 145, 205, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (70, 175, 225, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))

    if not dpg.does_item_exist("theme_fc_bench_locked_combo"):
        with dpg.theme(tag="theme_fc_bench_locked_combo"):
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (5, 5, 5, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (5, 5, 5, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (5, 5, 5, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (105, 105, 105, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (85, 85, 85, 255))

    if not dpg.does_item_exist("theme_fc_bench_active_combo"):
        with dpg.theme(tag="theme_fc_bench_active_combo"):
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (20, 55, 80, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (30, 85, 120, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (40, 110, 150, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))


def _apply_fc_bench_visual_state(setup_mode):
    active = bool(setup_mode)
    button_theme = (
        "theme_fc_bench_active_button"
        if active
        else "theme_fc_bench_locked_button"
    )
    combo_theme = (
        "theme_fc_bench_active_combo"
        if active
        else "theme_fc_bench_locked_combo"
    )

    for tag in ("fc_arm_button", "fc_disarm_button", "fc_set_mode_button"):
        try:
            if dpg.does_item_exist(tag):
                dpg.bind_item_theme(tag, button_theme)
        except Exception:
            pass

    try:
        if dpg.does_item_exist("fc_mode_combo"):
            dpg.bind_item_theme("fc_mode_combo", combo_theme)
    except Exception:
        pass

    try:
        if dpg.does_item_exist("fc_command_state"):
            dpg.configure_item(
                "fc_command_state",
                color=(105, 105, 105, 255) if not active else (95, 230, 120, 255),
            )
    except Exception:
        pass


def _set_operator_mode_v231(setup_mode):
    # Keep all V2.3.0 functional enable/disable guards first.
    _original_set_operator_mode_v230(setup_mode)
    _apply_fc_bench_visual_state(setup_mode)


def _create_gui_v231():
    _original_create_gui_v230()
    try:
        _create_fc_interlock_themes()
        _apply_fc_bench_visual_state(getattr(gcs, "edit_mode_unlocked", False))
    except Exception as exc:
        print(f"[GCS] V2.3.1 FC visual interlock warning: {exc}")


def main():
    # Patch V2.3.0 hook points before its wrapper installs them into the frozen
    # final GCS chain.
    v230._create_gui_v230 = _create_gui_v231
    v230._set_operator_mode_v230 = _set_operator_mode_v231

    print("[GCS] V2.3.1 VISUAL SETUP INTERLOCK")
    print("[GCS] FC bench controls BLACK/DIM while SETUP MODE is locked")
    print("[GCS] FC bench controls become ACTIVE-COLORED only in SETUP MODE")
    print("[GCS] Functional ARM/DISARM/mode safety guards remain unchanged")

    v230.main()


if __name__ == "__main__":
    main()
