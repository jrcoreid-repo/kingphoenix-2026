#!/usr/bin/env python3
"""Loss-triggered pass-through preset for Kingphoenix gate Batch-2.

This wrapper preserves the field-smooth Batch-1/Batch-2 controller and changes
only when pass-through timing effectively starts:

  TRACK -> NEAR_ARMED -> detector disappears at close range -> COMMIT ->
  blind forward for --pass-duration -> ZERO -> LAND

The direct area-based COMMIT threshold is raised to 0.60 by default, so normal
field behavior is driven by expected close-range detector loss rather than by
an early bbox-area trigger. Loss before NEAR_ARMED remains a safety ZERO.

All options from run_vision_follow_gate_commit_headless.py remain available.
User-supplied arguments are appended after these presets, so they override the
preset values when the same option is specified again.
"""

from __future__ import annotations

import runpy
import sys


PRESET_ARGS = [
    # Keep NEAR arming at the already-tested close-range threshold.
    "--near-area", "0.12",
    "--commit-error", "0.12",
    "--near-frames", "2",

    # Effectively defer hard area COMMIT until extremely close. In normal use
    # the detector should disappear first, producing EXPECTED_CLOSE_LOSS.
    "--commit-area", "0.60",
    "--commit-frames", "3",
    "--commit-loss-grace", "0.50",

    # Once close-range detector loss is confirmed by the existing Batch-2
    # state machine, continue straight through the gate before landing.
    "--pass-speed", "2.0",
    "--pass-duration", "0.80",
]


def main() -> None:
    user_args = sys.argv[1:]
    sys.argv = [
        "tools/run_vision_follow_gate_commit_headless.py",
        *PRESET_ARGS,
        *user_args,
    ]

    print("============================================================")
    print(" KINGPHOENIX GATE BATCH-2B - LOSS-TRIGGERED PASS")
    print("============================================================")
    print("[FLOW] TRACK -> NEAR_ARMED -> CLOSE LOSS -> BLIND PASS -> LAND")
    print("[SAFE] loss before NEAR_ARMED still commands ZERO")
    print("[PRESET] hard area COMMIT=0.60 (normally detector loss happens first)")
    print("[PRESET] close-loss grace=0.50s | blind pass=2.0m/s x 0.80s")
    print("[NOTE] CLI options supplied here override these preset values")
    print("------------------------------------------------------------")

    runpy.run_module(
        "tools.run_vision_follow_gate_commit_headless",
        run_name="__main__",
    )


if __name__ == "__main__":
    main()
