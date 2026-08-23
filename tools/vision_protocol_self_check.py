#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from streaming.barcode import (
    BARCODE_TOTAL_BITS,
    BARCODE_PREAMBLE,
    barcode_bits,
    frame_id_checksum,
)
from streaming.metadata_streamer import build_camera_metadata


def check(name, cond):
    if not cond:
        raise AssertionError(name)
    print("[PASS]", name)


def bits_to_int(bits):
    value = 0
    for bit in bits:
        value = (value << 1) | (1 if bit else 0)
    return value


def main():
    print("======================================================")
    print(" KINGPHOENIX V1.9.3.1 VISION PROTOCOL SELF-CHECK")
    print("======================================================")

    test_ids = [0, 1, 0x12345678, 0xFFFFFFFF]

    for fid in test_ids:
        bits = barcode_bits(fid)
        check(f"barcode length fid={fid}", len(bits) == 48)
        check(f"barcode preamble fid={fid}", bits_to_int(bits[:8]) == BARCODE_PREAMBLE)
        check(f"barcode frame fid={fid}", bits_to_int(bits[8:40]) == (fid & 0xFFFFFFFF))
        check(
            f"barcode checksum fid={fid}",
            bits_to_int(bits[40:48]) == frame_id_checksum(fid),
        )

    meta = build_camera_metadata(
        camera="front",
        frame_id=123,
        timestamp_ns=456,
        ai_state="ACTIVE",
        ai_task="GATE_NAV",
    )

    check("metadata camera", meta["camera"] == "front")
    check("metadata frame_id", meta["frame_id"] == 123)
    check("metadata ai_state", meta["ai_state"] == "ACTIVE")
    check("metadata ai_task", meta["ai_task"] == "GATE_NAV")

    print("======================================================")
    print(" VISION PROTOCOL SELF-CHECK: PASS")
    print("======================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
