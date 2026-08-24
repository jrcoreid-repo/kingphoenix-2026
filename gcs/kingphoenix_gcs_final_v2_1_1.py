#!/usr/bin/env python3
"""Kingphoenix GCS V2.1.1 compatibility hotfix.

This wrapper keeps the frozen FINAL layout from kingphoenix_gcs_final.py but
prevents legacy/hidden GUI aliases that are not present in the final layout
from aborting update_info_panel().

Root cause fixed here:
DearPyGUI resolves a missing string alias to item id 0. The old info-panel
updater still writes several legacy tags (for example value_ai_task and
value_ai_model). That raises Error 1005 before the function reaches the live
Pi Health fields, so Pi Health/CPU Temp/Power remain blank even though metadata
already contains pi_health.

This wrapper intentionally changes no layout and no flight/control behavior.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs


_original_set_value = dpg.set_value


def _safe_set_value(item, value):
    """Ignore stale legacy aliases while preserving normal DearPyGUI updates."""
    if isinstance(item, str):
        try:
            if not dpg.does_item_exist(item):
                return None
        except Exception:
            pass
    return _original_set_value(item, value)


def main():
    # Patch the same DearPyGUI module object used by kingphoenix_gcs_final.
    # No visual/layout changes are made.
    dpg.set_value = _safe_set_value
    gcs.dpg.set_value = _safe_set_value

    print("[GCS] V2.1.1 Pi Health GUI compatibility hotfix active")
    print("[GCS] Missing legacy aliases will no longer abort info-panel refresh")
    gcs.main()


if __name__ == "__main__":
    main()
