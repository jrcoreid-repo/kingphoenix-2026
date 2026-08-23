from __future__ import annotations

BARCODE_PREAMBLE = 0b10101100
BARCODE_PREAMBLE_BITS = 8
BARCODE_FRAME_BITS = 32
BARCODE_CHECKSUM_BITS = 8
BARCODE_TOTAL_BITS = 48

BARCODE_CELL_W = 8
BARCODE_CELL_H = 12
BARCODE_X = 8
BARCODE_BORDER = 2


def frame_id_checksum(frame_id):
    frame_id = int(frame_id) & 0xFFFFFFFF
    b0 = frame_id & 0xFF
    b1 = (frame_id >> 8) & 0xFF
    b2 = (frame_id >> 16) & 0xFF
    b3 = (frame_id >> 24) & 0xFF
    return (b0 ^ b1 ^ b2 ^ b3 ^ 0xA5) & 0xFF


def _bits(value, width):
    return [1 if (int(value) >> shift) & 1 else 0 for shift in range(width - 1, -1, -1)]


def barcode_bits(frame_id):
    return (
        _bits(BARCODE_PREAMBLE, BARCODE_PREAMBLE_BITS)
        + _bits(int(frame_id) & 0xFFFFFFFF, BARCODE_FRAME_BITS)
        + _bits(frame_id_checksum(frame_id), BARCODE_CHECKSUM_BITS)
    )


def draw_frame_id_barcode(frame, frame_id):
    """Draw the exact 48-cell barcode expected by the existing Windows GCS."""
    import cv2

    if frame is None:
        return frame

    h, w = frame.shape[:2]
    y = h - BARCODE_CELL_H - 4
    total_w = BARCODE_TOTAL_BITS * BARCODE_CELL_W

    # black border / quiet background
    x0 = max(0, BARCODE_X - BARCODE_BORDER)
    y0 = max(0, y - BARCODE_BORDER)
    x1 = min(w, BARCODE_X + total_w + BARCODE_BORDER)
    y1 = min(h, y + BARCODE_CELL_H + BARCODE_BORDER)
    cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), (0, 0, 0), -1)

    for i, bit in enumerate(barcode_bits(frame_id)):
        xa = BARCODE_X + i * BARCODE_CELL_W
        xb = xa + BARCODE_CELL_W - 1
        value = 255 if bit else 0
        cv2.rectangle(
            frame,
            (xa, y),
            (xb, y + BARCODE_CELL_H - 1),
            (value, value, value),
            -1,
        )

    return frame
