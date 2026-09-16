#!/usr/bin/env python3
"""Stable Windows application entrypoint for Kingphoenix GCS.

This file is intentionally version-agnostic for operators and packaging.
Update only the FINAL_GCS import when a newer frozen GUI replaces V2.3.6.
"""

from __future__ import annotations

from kingphoenix_gcs_final_v2_3_6 import main


if __name__ == "__main__":
    main()
