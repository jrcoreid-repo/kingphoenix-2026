#!/usr/bin/env python3
"""Kingphoenix GCS V2.1.2 compatibility hotfix.

Keeps the frozen FINAL layout from kingphoenix_gcs_final.py while preventing
legacy/removed DearPyGUI aliases from aborting the operator info-panel refresh.

DearPyGUI may surface a missing alias as item id 0. The previous V2.1.1 guard
only filtered missing string aliases, so an already-resolved integer 0 could
still reach dpg.set_value() and raise Error 1005. This revision rejects any
invalid/non-existent item before forwarding the write.

No layout, flight-control, stream, or mission behavior is changed.
"""

from __future__ import annotations

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs


_original_set_value = dpg.set_value


def _safe_set_value(item, value):
    """Ignore invalid/stale GUI item references without aborting refresh."""
    # DearPyGUI uses 0 as the unresolved/missing item sentinel in this case.
    if item is None or item == 0:
        return None

    try:
        if not dpg.does_item_exist(item):
            return None
    except Exception:
        # If DearPyGUI cannot even resolve/check the item, skip the legacy write.
        return None

    try:
        return _original_set_value(item, value)
    except Exception:
        # Compatibility layer only: a removed legacy field must never prevent
        # live Pi Health / telemetry fields from updating.
        return None


def main():
    # Patch the same DearPyGUI module object imported by the frozen final GUI.
    dpg.set_value = _safe_set_value
    gcs.dpg.set_value = _safe_set_value

    print("[GCS] V2.1.2 Pi Health GUI compatibility hotfix active")
    print("[GCS] Invalid legacy GUI item writes are isolated")
    gcs.main()


if __name__ == "__main__":
    main()
