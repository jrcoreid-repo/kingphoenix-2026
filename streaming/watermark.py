from __future__ import annotations


def watermark_text(run_mode_cfg, profile):
    watermark = (run_mode_cfg or {}).get("watermark", {})
    profile = str(profile or "").upper()

    if profile == "FULL_MISSION" and watermark.get(
        "hide_in_full_mission",
        True,
    ):
        return ""

    if not watermark.get("enabled_in_test", True):
        return ""

    return str(
        watermark.get("text_by_profile", {}).get(profile, "")
    ).strip()


def draw_test_watermark(frame, text):
    """Small unobtrusive test label centered at the top of the transmitted frame."""
    if frame is None or not text:
        return frame

    import cv2

    text = str(text)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.38
    thickness = 1
    pad_x = 7
    pad_y = 5

    (tw, th), baseline = cv2.getTextSize(
        text,
        font,
        scale,
        thickness,
    )

    box_w = tw + pad_x * 2
    frame_w = int(frame.shape[1])
    x1 = max(4, (frame_w - box_w) // 2)
    y1 = 8
    x2 = x1 + box_w
    y2 = y1 + th + baseline + pad_y * 2

    overlay = frame.copy()
    cv2.rectangle(
        overlay,
        (x1, y1),
        (x2, y2),
        (0, 0, 0),
        -1,
    )
    cv2.addWeighted(
        overlay,
        0.62,
        frame,
        0.38,
        0,
        frame,
    )
    cv2.putText(
        frame,
        text,
        (x1 + pad_x, y2 - baseline - pad_y),
        font,
        scale,
        (245, 245, 245),
        thickness,
        cv2.LINE_AA,
    )
    return frame
