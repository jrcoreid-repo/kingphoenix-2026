#!/usr/bin/env python3

# ============================================================
# KINGPHOENIX DRONE VISION SERVER 2026 V4
#
# Raspberry Pi 5 + Hailo-8
#
# PRIMARY:
#   YOLOv8 Pose, 4 keypoints
#
# FALLBACK:
#   YOLO26 detector
#
# VIDEO:
#   H264 / MPEG-TS / UDP -> 14560
#
# METADATA:
#   JSON UDP -> 14561
#
# CONTROL:
#   JSON UDP <- 14562
#
# V4 FRAME SYNCHRONIZATION:
#
#   Frame ID is EMBEDDED INTO VIDEO PIXELS.
#
#   BARCODE:
#
#       PREAMBLE   : 8 bit
#       FRAME ID   : 32 bit
#       CHECKSUM   : 8 bit
#
#       TOTAL      : 48 cells
#
#   PC will:
#
#       decoded H264 frame
#             ↓
#       decode barcode
#             ↓
#       VIDEO FRAME ID
#             ↓
#       metadata_buffer[frame_id]
#             ↓
#       DRAW
#
# PTS is still included only for diagnostic/latency.
#
# ============================================================


import cv2
import json
import math
import time
import socket
import threading

from pathlib import Path
from contextlib import ExitStack
from fractions import Fraction

import numpy as np


# ============================================================
# GSTREAMER
# ============================================================

import gi

gi.require_version(
    "Gst",
    "1.0"
)

from gi.repository import Gst


# ============================================================
# HAILO
# ============================================================

from hailo_platform import (
    VDevice,
    HEF,
    ConfigureParams,
    InputVStreamParams,
    OutputVStreamParams,
    InferVStreams,
    HailoStreamInterface,
    FormatType,
)


Gst.init(None)


# ============================================================
# PATH
# ============================================================

FRONT_VIDEO_PATH = (
    "/home/jrcore-labs/riset-26krti/Gate6.mp4"
)

BOTTOM_VIDEO_PATH = (
    "/home/jrcore-labs/riset-26krti/Gate6.mp4"
)

POSE_HEF_PATH = (
    "/home/jrcore-labs/riset-26krti/yolov8n-posev1.hef"
)

YOLO26_HEF_PATH = (
    "/home/jrcore-labs/riset-26krti/yolo26detectFinal640.hef"
)


# ============================================================
# NETWORK
# ============================================================

FRONT_VIDEO_PORT = 14560
FRONT_METADATA_PORT = 14561
CONTROL_PORT = 14562
BOTTOM_VIDEO_PORT = 14563
BOTTOM_METADATA_PORT = 14564


# ============================================================
# VIDEO
# ============================================================

FRAME_W = 640
FRAME_H = 640
MODEL_SIZE = 640
FORCE_FPS = 0
LOOP_VIDEO = True
FRONT_BITRATE_KBPS = 3000
BOTTOM_BITRATE_KBPS = 2500
KEYFRAME_INTERVAL = 30


# ============================================================
# FRAME ID BARCODE
# ============================================================

BARCODE_ENABLED = True

# Keep this visible until synchronization is proven.
DEBUG_TEXT_ENABLED = False

# ------------------------------------------------------------
# Barcode format:
#
# 8 bit preamble
# 32 bit frame ID
# 8 bit checksum
#
# 48 cells total
# ------------------------------------------------------------

BARCODE_PREAMBLE = 0b10101100

BARCODE_PREAMBLE_BITS = 8

BARCODE_FRAME_BITS = 32

BARCODE_CHECKSUM_BITS = 8

BARCODE_TOTAL_BITS = (
    BARCODE_PREAMBLE_BITS
    +
    BARCODE_FRAME_BITS
    +
    BARCODE_CHECKSUM_BITS
)

# 48 * 8 = 384 px
BARCODE_CELL_W = 8

BARCODE_CELL_H = 12

BARCODE_X = 8

BARCODE_Y = (
    FRAME_H
    -
    BARCODE_CELL_H
    -
    4
)

BARCODE_WIDTH = (
    BARCODE_TOTAL_BITS
    *
    BARCODE_CELL_W
)

BARCODE_WHITE = 255

BARCODE_BLACK = 0

# Border around barcode
BARCODE_BORDER = 2


# ============================================================
# DEBUG TEXT
# ============================================================

DEBUG_TEXT_X = 12

DEBUG_TEXT_Y = 28

DEBUG_TERMINAL_EVERY_N = 30


# ============================================================
# POSE
# ============================================================

NUM_KEYPOINTS = 4

POSE_CONF_THRESHOLD = 0.25

KEYPOINT_CONF_THRESHOLD = 0.20

POSE_IOU_THRESHOLD = 0.60

REG_MAX = 15


# ============================================================
# FALLBACK LOGIC
# ============================================================

POSE_LOST_TIMEOUT_MS = 200

POSE_PROBE_INTERVAL_MS = 100

FALLBACK_BBOX_MAX_AGE_MS = 250


# ============================================================
# YOLO26
# ============================================================

DET_CONF_THRESHOLD = 0.25

DET_IOU_THRESHOLD = 0.45

GATE_CLASS_ID = 1


# ============================================================
# GATE
# ============================================================

GATE_WIDTH_M = 2.0

GATE_HEIGHT_M = 2.0


# ============================================================
# CAMERA
#
# Temporary estimate.
# Drawing coordinates do NOT depend on this.
# ============================================================

CAMERA_HFOV_DEG = 90.0

DIST_COEFFS = np.zeros(
    (
        5,
        1
    ),
    dtype=np.float64
)


# ============================================================
# CPU TEMP
# ============================================================

CPU_TEMP_PATH = (
    "/sys/class/thermal/thermal_zone0/temp"
)

CPU_TEMP_INTERVAL_SEC = 1.0


# ============================================================
# GLOBAL NETWORK STATE
# ============================================================

running = True
client_lock = threading.Lock()
client_connected = False
client_ip = None
client_front_video_port = FRONT_VIDEO_PORT
client_front_metadata_port = FRONT_METADATA_PORT
client_bottom_video_port = BOTTOM_VIDEO_PORT
client_bottom_metadata_port = BOTTOM_METADATA_PORT
last_heartbeat_time = 0.0


# ============================================================
# MATH
# ============================================================

def sigmoid(x):

    x = np.clip(
        x,
        -50.0,
        50.0
    )

    return (
        1.0
        /
        (
            1.0
            +
            np.exp(-x)
        )
    )


def softmax(x):

    x = (
        x
        -
        np.max(
            x,
            axis=-1,
            keepdims=True
        )
    )

    exp_x = np.exp(
        x
    )

    return (
        exp_x
        /
        np.sum(
            exp_x,
            axis=-1,
            keepdims=True
        )
    )


# ============================================================
# NETWORK INFO
# ============================================================

def get_local_ip():

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    try:

        sock.connect(
            (
                "8.8.8.8",
                80
            )
        )

        ip = (
            sock.getsockname()[0]
        )

    except Exception:

        try:

            ip = socket.gethostbyname(
                socket.gethostname()
            )

        except Exception:

            ip = "Unknown"

    finally:

        sock.close()

    return ip


# ============================================================
# CPU TEMP
# ============================================================

def read_cpu_temperature():

    try:

        with open(
            CPU_TEMP_PATH,
            "r",
            encoding="utf-8"
        ) as file:

            raw = float(
                file.read().strip()
            )

        return (
            raw
            /
            1000.0
        )

    except Exception:

        return None


# ============================================================
# FRAME ID CHECKSUM
#
# Simple XOR of four frame-ID bytes.
#
# PC will calculate the same checksum.
# ============================================================

def frame_id_checksum(
    frame_id
):

    frame_id = int(
        frame_id
    ) & 0xFFFFFFFF

    b0 = (
        frame_id
        &
        0xFF
    )

    b1 = (
        frame_id
        >>
        8
    ) & 0xFF

    b2 = (
        frame_id
        >>
        16
    ) & 0xFF

    b3 = (
        frame_id
        >>
        24
    ) & 0xFF

    checksum = (
        b0
        ^
        b1
        ^
        b2
        ^
        b3
        ^
        0xA5
    )

    return (
        checksum
        &
        0xFF
    )


# ============================================================
# INTEGER -> BITS
#
# MSB first.
# ============================================================

def integer_to_bits(
    value,
    bit_count
):

    result = []

    for bit_index in range(
        bit_count - 1,
        -1,
        -1
    ):

        bit = (
            (
                value
                >>
                bit_index
            )
            &
            1
        )

        result.append(
            bit
        )

    return result


# ============================================================
# CREATE BARCODE BIT ARRAY
# ============================================================

def build_frame_barcode_bits(
    frame_id
):

    frame_id = (
        int(
            frame_id
        )
        &
        0xFFFFFFFF
    )

    checksum = (
        frame_id_checksum(
            frame_id
        )
    )

    bits = []

    bits.extend(
        integer_to_bits(
            BARCODE_PREAMBLE,
            BARCODE_PREAMBLE_BITS
        )
    )

    bits.extend(
        integer_to_bits(
            frame_id,
            BARCODE_FRAME_BITS
        )
    )

    bits.extend(
        integer_to_bits(
            checksum,
            BARCODE_CHECKSUM_BITS
        )
    )

    if (
        len(bits)
        !=
        BARCODE_TOTAL_BITS
    ):

        raise RuntimeError(
            "Barcode bit count mismatch"
        )

    return (
        bits,
        checksum
    )


# ============================================================
# EMBED FRAME-ID BARCODE
#
# IMPORTANT:
#
# Called AFTER inference.
#
# Therefore barcode never enters YOLO input.
#
# Every bit is represented by an entire 8x12 block.
# PC reads mean luminance, making this resistant to H264.
# ============================================================

def embed_frame_id_barcode(
    frame,
    frame_id
):

    if not BARCODE_ENABLED:

        return None

    bits, checksum = (
        build_frame_barcode_bits(
            frame_id
        )
    )

    # --------------------------------------------------------
    # Clear background + border
    # --------------------------------------------------------

    x1 = (
        BARCODE_X
        -
        BARCODE_BORDER
    )

    y1 = (
        BARCODE_Y
        -
        BARCODE_BORDER
    )

    x2 = (
        BARCODE_X
        +
        BARCODE_WIDTH
        +
        BARCODE_BORDER
    )

    y2 = (
        BARCODE_Y
        +
        BARCODE_CELL_H
        +
        BARCODE_BORDER
    )

    cv2.rectangle(
        frame,
        (
            x1,
            y1
        ),
        (
            x2,
            y2
        ),
        (
            80,
            80,
            80
        ),
        -1
    )

    # --------------------------------------------------------
    # Draw each cell
    # --------------------------------------------------------

    for index, bit in enumerate(
        bits
    ):

        cell_x1 = (
            BARCODE_X
            +
            index
            *
            BARCODE_CELL_W
        )

        cell_x2 = (
            cell_x1
            +
            BARCODE_CELL_W
            -
            1
        )

        cell_y1 = (
            BARCODE_Y
        )

        cell_y2 = (
            BARCODE_Y
            +
            BARCODE_CELL_H
            -
            1
        )

        if bit:

            value = (
                BARCODE_WHITE
            )

        else:

            value = (
                BARCODE_BLACK
            )

        cv2.rectangle(
            frame,
            (
                cell_x1,
                cell_y1
            ),
            (
                cell_x2,
                cell_y2
            ),
            (
                value,
                value,
                value
            ),
            -1
        )

    return checksum


# ============================================================
# DEBUG HUMAN-READABLE FRAME ID
# ============================================================

def burn_debug_frame_id(
    frame,
    frame_id
):

    if not DEBUG_TEXT_ENABLED:

        return

    text = (
        f"VIDEO F:{frame_id:06d}"
    )

    cv2.rectangle(
        frame,
        (
            DEBUG_TEXT_X - 5,
            DEBUG_TEXT_Y - 21
        ),
        (
            DEBUG_TEXT_X + 170,
            DEBUG_TEXT_Y + 7
        ),
        (
            0,
            0,
            0
        ),
        -1
    )

    cv2.putText(
        frame,
        text,
        (
            DEBUG_TEXT_X,
            DEBUG_TEXT_Y
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (
            255,
            255,
            255
        ),
        2,
        cv2.LINE_AA
    )


# ============================================================
# PREPROCESS
#
# Current source/model/stream all 640x640.
# ============================================================

def preprocess_frame(
    frame
):

    if (
        frame.shape[1]
        !=
        MODEL_SIZE
        or
        frame.shape[0]
        !=
        MODEL_SIZE
    ):

        frame = cv2.resize(
            frame,
            (
                MODEL_SIZE,
                MODEL_SIZE
            ),
            interpolation=
            cv2.INTER_LINEAR
        )

    rgb = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2RGB
    )

    tensor = np.expand_dims(
        rgb,
        axis=0
    ).astype(
        np.uint8
    )

    return tensor


# ============================================================
# NMS
# ============================================================

def nms_xyxy(
    boxes,
    scores,
    threshold
):

    if len(boxes) == 0:

        return []

    boxes = np.asarray(
        boxes,
        dtype=np.float32
    )

    scores = np.asarray(
        scores,
        dtype=np.float32
    )

    x1 = boxes[:, 0]

    y1 = boxes[:, 1]

    x2 = boxes[:, 2]

    y2 = boxes[:, 3]

    areas = (
        np.maximum(
            0,
            x2 - x1
        )
        *
        np.maximum(
            0,
            y2 - y1
        )
    )

    order = (
        scores
        .argsort()[::-1]
    )

    keep = []

    while (
        order.size
        >
        0
    ):

        i = int(
            order[0]
        )

        keep.append(
            i
        )

        if (
            order.size
            ==
            1
        ):

            break

        xx1 = np.maximum(
            x1[i],
            x1[
                order[1:]
            ]
        )

        yy1 = np.maximum(
            y1[i],
            y1[
                order[1:]
            ]
        )

        xx2 = np.minimum(
            x2[i],
            x2[
                order[1:]
            ]
        )

        yy2 = np.minimum(
            y2[i],
            y2[
                order[1:]
            ]
        )

        inter_w = np.maximum(
            0,
            xx2 - xx1
        )

        inter_h = np.maximum(
            0,
            yy2 - yy1
        )

        intersection = (
            inter_w
            *
            inter_h
        )

        union = (
            areas[i]
            +
            areas[
                order[1:]
            ]
            -
            intersection
            +
            1e-9
        )

        iou = (
            intersection
            /
            union
        )

        remaining = np.where(
            iou
            <=
            threshold
        )[0]

        order = order[
            remaining
            +
            1
        ]

    return keep


# ============================================================
# FIND HAILO OUTPUT
# ============================================================

def find_output_by_shape(
    outputs,
    grid,
    channels
):

    expected = (
        1,
        grid,
        grid,
        channels
    )

    for _, tensor in (
        outputs.items()
    ):

        if (
            tensor.shape
            ==
            expected
        ):

            return tensor

    print()
    print(
        "[HAILO] Required output:",
        expected
    )

    print(
        "[HAILO] Available:"
    )

    for name, tensor in (
        outputs.items()
    ):

        print(
            " ",
            name,
            tensor.shape
        )

    raise RuntimeError(
        f"Missing tensor "
        f"{expected}"
    )


# ============================================================
# YOLOv8 POSE BBOX
# ============================================================

def decode_pose_boxes(
    tensor,
    stride
):

    batch, h, w, channels = (
        tensor.shape
    )

    tensor = tensor.reshape(
        batch,
        h * w,
        4,
        REG_MAX + 1
    )

    probability = (
        softmax(
            tensor
        )
    )

    bins = np.arange(
        REG_MAX + 1,
        dtype=np.float32
    )

    distance = np.sum(
        probability
        *
        bins,
        axis=-1
    )

    distance *= (
        stride
    )

    grid_y, grid_x = (
        np.meshgrid(
            np.arange(h),
            np.arange(w),
            indexing="ij"
        )
    )

    center_x = (
        grid_x.reshape(-1)
        +
        0.5
    ) * stride

    center_y = (
        grid_y.reshape(-1)
        +
        0.5
    ) * stride

    left = (
        distance[
            0,
            :,
            0
        ]
    )

    top = (
        distance[
            0,
            :,
            1
        ]
    )

    right = (
        distance[
            0,
            :,
            2
        ]
    )

    bottom = (
        distance[
            0,
            :,
            3
        ]
    )

    return np.stack(
        [
            center_x - left,
            center_y - top,
            center_x + right,
            center_y + bottom
        ],
        axis=1
    )


# ============================================================
# YOLOv8 POSE KEYPOINT
# ============================================================

def decode_pose_keypoints(
    tensor,
    stride
):

    batch, h, w, channels = (
        tensor.shape
    )

    expected = (
        NUM_KEYPOINTS
        *
        3
    )

    if (
        channels
        !=
        expected
    ):

        raise RuntimeError(
            f"Expected "
            f"{expected} "
            f"keypoint channels, "
            f"got {channels}"
        )

    raw = tensor.reshape(
        batch,
        h * w,
        NUM_KEYPOINTS,
        3
    )

    grid_y, grid_x = (
        np.meshgrid(
            np.arange(h),
            np.arange(w),
            indexing="ij"
        )
    )

    center_x = (
        (
            grid_x.reshape(
                1,
                h * w,
                1
            )
            +
            0.5
        )
        *
        stride
    )

    center_y = (
        (
            grid_y.reshape(
                1,
                h * w,
                1
            )
            +
            0.5
        )
        *
        stride
    )

    raw_x = (
        raw[..., 0]
        *
        2.0
    )

    raw_y = (
        raw[..., 1]
        *
        2.0
    )

    x = (
        stride
        *
        (
            raw_x
            -
            0.5
        )
        +
        center_x
    )

    y = (
        stride
        *
        (
            raw_y
            -
            0.5
        )
        +
        center_y
    )

    confidence = sigmoid(
        raw[..., 2]
    )

    decoded = np.stack(
        [
            x,
            y,
            confidence
        ],
        axis=-1
    )

    return decoded[0]


# ============================================================
# POSE POSTPROCESS
# ============================================================

def decode_pose(
    outputs
):

    all_boxes = []

    all_scores = []

    all_keypoints = []

    for grid, stride in [

        (80, 8),

        (40, 16),

        (20, 32),

    ]:

        bbox_tensor = (
            find_output_by_shape(
                outputs,
                grid,
                64
            )
        )

        score_tensor = (
            find_output_by_shape(
                outputs,
                grid,
                1
            )
        )

        keypoint_tensor = (
            find_output_by_shape(
                outputs,
                grid,
                12
            )
        )

        boxes = (
            decode_pose_boxes(
                bbox_tensor,
                stride
            )
        )

        scores = (
            score_tensor
            .reshape(-1)
        )

        if (
            np.min(
                scores
            )
            <
            0.0
            or
            np.max(
                scores
            )
            >
            1.0
        ):

            scores = sigmoid(
                scores
            )

        keypoints = (
            decode_pose_keypoints(
                keypoint_tensor,
                stride
            )
        )

        all_boxes.append(
            boxes
        )

        all_scores.append(
            scores
        )

        all_keypoints.append(
            keypoints
        )

    boxes = np.concatenate(
        all_boxes,
        axis=0
    )

    scores = np.concatenate(
        all_scores,
        axis=0
    )

    keypoints = np.concatenate(
        all_keypoints,
        axis=0
    )

    mask = (
        scores
        >=
        POSE_CONF_THRESHOLD
    )

    boxes = (
        boxes[
            mask
        ]
    )

    scores = (
        scores[
            mask
        ]
    )

    keypoints = (
        keypoints[
            mask
        ]
    )

    if (
        len(
            boxes
        )
        ==
        0
    ):

        return None

    boxes[:, [0, 2]] = np.clip(
        boxes[:, [0, 2]],
        0,
        FRAME_W - 1
    )

    boxes[:, [1, 3]] = np.clip(
        boxes[:, [1, 3]],
        0,
        FRAME_H - 1
    )

    keypoints[:, :, 0] = np.clip(
        keypoints[:, :, 0],
        0,
        FRAME_W - 1
    )

    keypoints[:, :, 1] = np.clip(
        keypoints[:, :, 1],
        0,
        FRAME_H - 1
    )

    valid = (
        (
            boxes[:, 2]
            >
            boxes[:, 0]
        )
        &
        (
            boxes[:, 3]
            >
            boxes[:, 1]
        )
    )

    boxes = (
        boxes[
            valid
        ]
    )

    scores = (
        scores[
            valid
        ]
    )

    keypoints = (
        keypoints[
            valid
        ]
    )

    if (
        len(
            boxes
        )
        ==
        0
    ):

        return None

    keep = (
        nms_xyxy(
            boxes,
            scores,
            POSE_IOU_THRESHOLD
        )
    )

    if not keep:

        return None

    index = (
        keep[0]
    )

    return {

        "conf":
            float(
                scores[
                    index
                ]
            ),

        "box":
            boxes[
                index
            ].copy(),

        "keypoints":
            keypoints[
                index
            ].copy(),

    }


# ============================================================
# VALIDATE POSE
# ============================================================

def validate_pose(
    pose
):

    if pose is None:

        return False

    keypoints = (
        pose[
            "keypoints"
        ]
    )

    if (
        keypoints.shape
        !=
        (
            NUM_KEYPOINTS,
            3
        )
    ):

        return False

    if np.any(
        keypoints[:, 2]
        <
        KEYPOINT_CONF_THRESHOLD
    ):

        return False

    return True


# ============================================================
# YOLO26 OUTPUT MAP
# ============================================================

def map_yolo26_outputs(
    outputs
):

    mapped = {}

    for _, tensor in (
        outputs.items()
    ):

        shape = (
            tensor.shape
        )

        if (
            len(
                shape
            )
            !=
            4
        ):

            continue

        if (
            shape[0]
            !=
            1
        ):

            continue

        grid_h = (
            shape[1]
        )

        grid_w = (
            shape[2]
        )

        channels = (
            shape[3]
        )

        if (
            grid_h
            !=
            grid_w
        ):

            continue

        if (
            grid_h
            not in
            (
                80,
                40,
                20
            )
        ):

            continue

        if (
            channels
            ==
            4
        ):

            mapped[
                f"reg_{grid_h}"
            ] = (
                tensor[0]
            )

        else:

            mapped[
                f"cls_{grid_h}"
            ] = (
                tensor[0]
            )

    required = [

        "reg_80",
        "reg_40",
        "reg_20",

        "cls_80",
        "cls_40",
        "cls_20",

    ]

    missing = [

        key

        for key in required

        if key
        not in
        mapped

    ]

    if missing:

        print()
        print(
            "[YOLO26] Available outputs:"
        )

        for name, tensor in (
            outputs.items()
        ):

            print(
                " ",
                name,
                tensor.shape
            )

        raise RuntimeError(
            f"YOLO26 missing: "
            f"{missing}"
        )

    return mapped


# ============================================================
# YOLO26 DECODER
# ============================================================

def decode_yolo26(
    outputs
):

    mapped = (
        map_yolo26_outputs(
            outputs
        )
    )

    boxes_all = []

    scores_all = []

    classes_all = []

    for grid, stride in [

        (80, 8),

        (40, 16),

        (20, 32),

    ]:

        cls_data = (
            mapped[
                f"cls_{grid}"
            ]
        )

        reg_data = (
            mapped[
                f"reg_{grid}"
            ]
        )

        if (
            np.min(
                cls_data
            )
            <
            0.0
            or
            np.max(
                cls_data
            )
            >
            1.0
        ):

            cls_prob = (
                sigmoid(
                    cls_data
                )
            )

        else:

            cls_prob = (
                cls_data
            )

        scores = (
            cls_prob
            .max(
                axis=2
            )
        )

        classes = (
            cls_prob
            .argmax(
                axis=2
            )
        )

        mask = (
            scores
            >=
            DET_CONF_THRESHOLD
        )

        if not np.any(
            mask
        ):

            continue

        rows, cols = (
            np.where(
                mask
            )
        )

        regression = (
            reg_data[
                rows,
                cols
            ]
        )

        selected_scores = (
            scores[
                rows,
                cols
            ]
        )

        selected_classes = (
            classes[
                rows,
                cols
            ]
        )

        left = (
            regression[:, 0]
        )

        top = (
            regression[:, 1]
        )

        right = (
            regression[:, 2]
        )

        bottom = (
            regression[:, 3]
        )

        x1 = (
            cols
            +
            0.5
            -
            left
        ) * stride

        y1 = (
            rows
            +
            0.5
            -
            top
        ) * stride

        x2 = (
            cols
            +
            0.5
            +
            right
        ) * stride

        y2 = (
            rows
            +
            0.5
            +
            bottom
        ) * stride

        boxes = np.stack(
            [
                x1,
                y1,
                x2,
                y2
            ],
            axis=1
        )

        boxes_all.append(
            boxes
        )

        scores_all.append(
            selected_scores
        )

        classes_all.append(
            selected_classes
        )

    if not boxes_all:

        return []

    boxes = np.concatenate(
        boxes_all,
        axis=0
    )

    scores = np.concatenate(
        scores_all,
        axis=0
    )

    classes = np.concatenate(
        classes_all,
        axis=0
    )

    boxes[:, [0, 2]] = np.clip(
        boxes[:, [0, 2]],
        0,
        FRAME_W - 1
    )

    boxes[:, [1, 3]] = np.clip(
        boxes[:, [1, 3]],
        0,
        FRAME_H - 1
    )

    keep = nms_xyxy(
        boxes,
        scores,
        DET_IOU_THRESHOLD
    )

    detections = []

    for index in keep:

        detections.append({

            "x1":
                float(
                    boxes[
                        index,
                        0
                    ]
                ),

            "y1":
                float(
                    boxes[
                        index,
                        1
                    ]
                ),

            "x2":
                float(
                    boxes[
                        index,
                        2
                    ]
                ),

            "y2":
                float(
                    boxes[
                        index,
                        3
                    ]
                ),

            "conf":
                float(
                    scores[
                        index
                    ]
                ),

            "cls_id":
                int(
                    classes[
                        index
                    ]
                ),

        })

    return detections


# ============================================================
# SELECT GATE
# ============================================================

def select_gate(
    detections
):

    candidates = [

        detection

        for detection
        in detections

        if (
            detection[
                "cls_id"
            ]
            ==
            GATE_CLASS_ID
        )

    ]

    if not candidates:

        return None

    return max(
        candidates,
        key=lambda detection:
        detection[
            "conf"
        ]
    )


# ============================================================
# CENTERS
# ============================================================

def pose_center(
    pose
):

    if pose is None:

        return None

    center = np.mean(
        pose[
            "keypoints"
        ][
            :,
            :2
        ],
        axis=0
    )

    return (

        float(
            center[0]
        ),

        float(
            center[1]
        ),

    )


def bbox_center(
    detection
):

    if detection is None:

        return None

    return (

        (
            detection["x1"]
            +
            detection["x2"]
        )
        /
        2.0,

        (
            detection["y1"]
            +
            detection["y2"]
        )
        /
        2.0,

    )


# ============================================================
# CAMERA MATRIX
# ============================================================

def create_camera_matrix():

    hfov = math.radians(
        CAMERA_HFOV_DEG
    )

    fx = (
        FRAME_W
        /
        (
            2.0
            *
            math.tan(
                hfov
                /
                2.0
            )
        )
    )

    fy = fx

    cx = (
        FRAME_W
        /
        2.0
    )

    cy = (
        FRAME_H
        /
        2.0
    )

    return np.array(
        [

            [
                fx,
                0.0,
                cx
            ],

            [
                0.0,
                fy,
                cy
            ],

            [
                0.0,
                0.0,
                1.0
            ],

        ],
        dtype=np.float64
    )


# ============================================================
# SOLVE PNP
# ============================================================

def solve_gate_pose(
    pose,
    camera_matrix
):

    if not validate_pose(
        pose
    ):

        return None

    half_w = (
        GATE_WIDTH_M
        /
        2.0
    )

    half_h = (
        GATE_HEIGHT_M
        /
        2.0
    )

    object_points = np.array(
        [

            [
                -half_w,
                -half_h,
                0.0
            ],

            [
                half_w,
                -half_h,
                0.0
            ],

            [
                half_w,
                half_h,
                0.0
            ],

            [
                -half_w,
                half_h,
                0.0
            ],

        ],
        dtype=np.float64
    )

    image_points = (
        pose[
            "keypoints"
        ][
            :,
            :2
        ]
        .astype(
            np.float64
        )
    )

    success, rvec, tvec = (
        cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            DIST_COEFFS,
            flags=
            cv2.SOLVEPNP_IPPE
        )
    )

    if not success:

        return None

    rotation_matrix, _ = (
        cv2.Rodrigues(
            rvec
        )
    )

    euler, _, _, _, _, _ = (
        cv2.RQDecomp3x3(
            rotation_matrix
        )
    )

    return {

        "x":
            float(
                tvec[
                    0,
                    0
                ]
            ),

        "y":
            float(
                tvec[
                    1,
                    0
                ]
            ),

        "z":
            float(
                tvec[
                    2,
                    0
                ]
            ),

        "roll":
            float(
                euler[0]
            ),

        "pitch":
            float(
                euler[1]
            ),

        "yaw":
            float(
                euler[2]
            ),

    }


# ============================================================
# GSTREAMER VIDEO STREAMER
# ============================================================

class GstVideoStreamer:

    def __init__(
        self,
        camera_name,
        target_ip,
        target_port,
        fps,
        bitrate_kbps
    ):

        self.camera_name = str(camera_name)
        self.target_ip = target_ip
        self.target_port = int(target_port)
        self.fps = float(fps)
        self.bitrate_kbps = int(bitrate_kbps)

        fraction = Fraction(self.fps).limit_denominator(1001)
        self.fps_num = fraction.numerator
        self.fps_den = fraction.denominator

        self.pipeline = None
        self.appsrc = None
        self.started = False
        self.stream_index = 0
        self.session_id = time.time_ns()

        self._build()

    def _build(self):

        source_name = f"source_{self.camera_name}"

        pipeline_text = (
            "appsrc "
            f"name={source_name} "
            "is-live=true "
            "block=true "
            "format=time "
            "do-timestamp=false "
            f"caps=video/x-raw,format=BGR,width={FRAME_W},height={FRAME_H},"
            f"framerate={self.fps_num}/{self.fps_den} "
            "! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! videoconvert "
            "! video/x-raw,format=I420 "
            "! x264enc "
            f"bitrate={self.bitrate_kbps} "
            "speed-preset=ultrafast "
            "tune=zerolatency "
            f"key-int-max={KEYFRAME_INTERVAL} "
            "bframes=0 "
            "byte-stream=true "
            "! h264parse config-interval=-1 "
            "! mpegtsmux alignment=7 "
            "! udpsink "
            f"host={self.target_ip} "
            f"port={self.target_port} "
            "sync=false async=false"
        )

        print()
        print(f"[GST:{self.camera_name}] Pipeline:")
        print(pipeline_text)

        self.pipeline = Gst.parse_launch(pipeline_text)
        self.appsrc = self.pipeline.get_by_name(source_name)

        if self.appsrc is None:
            raise RuntimeError(
                f"Cannot obtain appsrc for {self.camera_name}"
            )

    def start(self):

        if self.started:
            return

        result = self.pipeline.set_state(Gst.State.PLAYING)

        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(
                f"GStreamer PLAYING failed for {self.camera_name}"
            )

        self.started = True
        self.stream_index = 0

        print()
        print(
            f"[GST:{self.camera_name}] STREAM STARTED -> "
            f"{self.target_ip}:{self.target_port}"
        )
        print(
            f"[GST:{self.camera_name}] Session ID: {self.session_id}"
        )

    def pts_for_index(self, index):

        return int(
            (
                index
                * Gst.SECOND
                * self.fps_den
            )
            / self.fps_num
        )

    @staticmethod
    def ns_to_90k(pts_ns):

        return int(
            round(
                (pts_ns * 90000)
                / Gst.SECOND
            )
        )

    def push(self, frame):

        if not self.started:
            self.start()

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        index = self.stream_index
        pts_ns = self.pts_for_index(index)
        next_pts_ns = self.pts_for_index(index + 1)
        duration_ns = next_pts_ns - pts_ns
        payload = frame.tobytes()

        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        buffer.pts = pts_ns
        buffer.dts = pts_ns
        buffer.duration = duration_ns

        result = self.appsrc.emit("push-buffer", buffer)

        if result != Gst.FlowReturn.OK:
            raise RuntimeError(
                f"{self.camera_name} push-buffer failed: {result}"
            )

        self.stream_index += 1

        return {
            "session_id": int(self.session_id),
            "stream_index": int(index),
            "pts_ns": int(pts_ns),
            "pts_90k": int(self.ns_to_90k(pts_ns)),
            "duration_ns": int(duration_ns),
        }

    def stop(self):

        if self.pipeline is None:
            return

        try:
            if self.appsrc is not None:
                self.appsrc.emit("end-of-stream")
        except Exception:
            pass

        self.pipeline.set_state(Gst.State.NULL)
        self.started = False

        print()
        print(f"[GST:{self.camera_name}] STREAM STOPPED")


# ============================================================
# HAILO MODEL SWITCHER
# ============================================================

class HailoModelSwitcher:

    def __init__(
        self,
        pose_group,
        detector_group
    ):

        self.pose_group = (
            pose_group
        )

        self.detector_group = (
            detector_group
        )

        self.stack = (
            ExitStack()
        )

        self.active = None


    def switch(
        self,
        model
    ):

        if (
            self.active
            ==
            model
        ):

            return

        self.stack.close()

        self.stack = (
            ExitStack()
        )

        if model == "POSE":

            self.stack.enter_context(
                self.pose_group.activate()
            )

        elif model == "YOLO26":

            self.stack.enter_context(
                self.detector_group.activate()
            )

        else:

            raise ValueError(
                model
            )

        self.active = (
            model
        )


    def close(
        self
    ):

        self.stack.close()

        self.active = None


# ============================================================
# CONTROL SERVER
# ============================================================

def control_server():

    global running
    global client_connected
    global client_ip
    global client_front_video_port
    global client_front_metadata_port
    global client_bottom_video_port
    global client_bottom_metadata_port
    global last_heartbeat_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CONTROL_PORT))
    sock.settimeout(0.5)

    print(f"[CONTROL] Listening UDP :{CONTROL_PORT}")

    try:
        while running:

            try:
                data, address = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                continue

            try:
                message = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            msg_type = str(message.get("type", "")).upper()

            if msg_type == "HELLO":

                with client_lock:
                    client_ip = address[0]

                    # New dual-camera fields.
                    # Old single-camera names are accepted for front
                    # so an older GCS can still connect during testing.
                    client_front_video_port = int(
                        message.get(
                            "front_video_port",
                            message.get("video_port", FRONT_VIDEO_PORT)
                        )
                    )

                    client_front_metadata_port = int(
                        message.get(
                            "front_metadata_port",
                            message.get("metadata_port", FRONT_METADATA_PORT)
                        )
                    )

                    client_bottom_video_port = int(
                        message.get(
                            "bottom_video_port",
                            BOTTOM_VIDEO_PORT
                        )
                    )

                    client_bottom_metadata_port = int(
                        message.get(
                            "bottom_metadata_port",
                            BOTTOM_METADATA_PORT
                        )
                    )

                    client_connected = True
                    last_heartbeat_time = time.monotonic()

                print()
                print("==============================================")
                print("[CONTROL] GCS CONNECTED")
                print(f"GCS IP          : {client_ip}")
                print(
                    f"FRONT VIDEO     : {client_front_video_port}"
                )
                print(
                    f"FRONT META      : {client_front_metadata_port}"
                )
                print(
                    f"BOTTOM VIDEO    : {client_bottom_video_port}"
                )
                print(
                    f"BOTTOM META     : {client_bottom_metadata_port}"
                )
                print("AI CAMERA       : front")
                print("==============================================")

                response = {
                    "type": "HELLO_ACK",
                    "status": "OK",
                    "drone_ip": get_local_ip(),
                    "video_transport": "MPEGTS_H264_UDP_DUAL",
                    "sync": "PIXEL_FRAME_ID_V1",
                    "barcode_bits": BARCODE_TOTAL_BITS,
                    "barcode_cell_w": BARCODE_CELL_W,
                    "barcode_cell_h": BARCODE_CELL_H,
                    "barcode_x": BARCODE_X,
                    "barcode_y": BARCODE_Y,
                    "front_video_port": FRONT_VIDEO_PORT,
                    "front_metadata_port": FRONT_METADATA_PORT,
                    "bottom_video_port": BOTTOM_VIDEO_PORT,
                    "bottom_metadata_port": BOTTOM_METADATA_PORT,
                    "control_port": CONTROL_PORT,
                    "active_ai_camera": "front",
                }

                try:
                    sock.sendto(
                        json.dumps(response).encode("utf-8"),
                        address
                    )
                except Exception:
                    pass

            elif msg_type == "HEARTBEAT":

                with client_lock:
                    last_heartbeat_time = time.monotonic()

            elif msg_type == "DISCONNECT":

                with client_lock:
                    client_connected = False
                    client_ip = None

                print()
                print("[CONTROL] GCS DISCONNECTED")

    finally:
        sock.close()


# ============================================================
# JSON HELPERS
# ============================================================

def keypoints_to_json(
    pose
):

    if pose is None:

        return None

    result = []

    for point in (
        pose[
            "keypoints"
        ]
    ):

        result.append(
            [

                round(
                    float(
                        point[0]
                    ),
                    2
                ),

                round(
                    float(
                        point[1]
                    ),
                    2
                ),

                round(
                    float(
                        point[2]
                    ),
                    4
                ),

            ]
        )

    return result


def bbox_to_json(
    detection
):

    if detection is None:

        return None

    return [

        round(
            float(
                detection[
                    "x1"
                ]
            ),
            2
        ),

        round(
            float(
                detection[
                    "y1"
                ]
            ),
            2
        ),

        round(
            float(
                detection[
                    "x2"
                ]
            ),
            2
        ),

        round(
            float(
                detection[
                    "y2"
                ]
            ),
            2
        ),

    ]


# ============================================================
# BUILD METADATA
# ============================================================

def build_metadata(
    camera,
    camera_id,
    frame_id,
    barcode_checksum,
    timing,
    mode,
    pose,
    detection,
    target,
    pnp,
    cpu_temp,
    model_ms,
    model_fps,
    pipeline_fps,
    ai_state,
    ai_task,
    ai_model
):

    error = None

    if target is not None:
        error = [
            int(round(target[0] - FRAME_W / 2.0)),
            int(round(target[1] - FRAME_H / 2.0)),
        ]

    confidence = None

    if mode == "POSE" and pose is not None:
        confidence = pose["conf"]
    elif mode == "YOLO26" and detection is not None:
        confidence = detection["conf"]

    xyz = None
    rpy = None

    if pnp is not None:
        xyz = [
            round(pnp["x"], 4),
            round(pnp["y"], 4),
            round(pnp["z"], 4),
        ]

        rpy = [
            round(pnp["roll"], 3),
            round(pnp["pitch"], 3),
            round(pnp["yaw"], 3),
        ]

    return {
        "type": "VISION",
        "camera": str(camera),
        "camera_id": int(camera_id),
        "video_state": "LIVE",
        "ai_state": str(ai_state),
        "ai_task": str(ai_task),
        "ai_model": str(ai_model),

        # Primary frame identity.
        "frame_id": int(frame_id),
        "frame_sync": "PIXEL_ID_V1",
        "barcode_checksum": int(barcode_checksum),
        "barcode_preamble": int(BARCODE_PREAMBLE),

        # Secondary timing diagnostics.
        "session_id": timing["session_id"],
        "stream_index": timing["stream_index"],
        "video_pts_ns": timing["pts_ns"],
        "video_pts_90k": timing["pts_90k"],

        "video_width": FRAME_W,
        "video_height": FRAME_H,

        # Vision. Keep legacy 'mode' field for PC compatibility.
        "mode": mode,
        "vision_mode": mode,
        "conf": (
            round(float(confidence), 4)
            if confidence is not None
            else None
        ),
        "confidence": (
            round(float(confidence), 4)
            if confidence is not None
            else None
        ),
        "keypoints": keypoints_to_json(pose),
        "bbox": bbox_to_json(detection),
        "target": (
            [
                round(float(target[0]), 2),
                round(float(target[1]), 2),
            ]
            if target is not None
            else None
        ),
        "error": error,
        "xyz": xyz,
        "rpy": rpy,

        # System.
        "cpu_temp": (
            round(float(cpu_temp), 2)
            if cpu_temp is not None
            else None
        ),
        "model_ms": round(float(model_ms), 2),
        "model_fps": round(float(model_fps), 2),
        "pipeline_fps": round(float(pipeline_fps), 2),
    }


# ============================================================
# VIDEO SOURCE FPS
# ============================================================

def get_video_fps(cap):

    source_fps = cap.get(
        cv2.CAP_PROP_FPS
    )

    if source_fps <= 0:
        source_fps = 30.0

    if FORCE_FPS > 0:
        return float(FORCE_FPS)

    return float(source_fps)


# ============================================================
# BOTTOM CAMERA WORKER
#
# For this version the bottom camera is always LIVE but AI is
# STANDBY. It has its own source, streamer, frame counter,
# barcode and metadata buffer identity.
# ============================================================

def bottom_camera_worker():

    global running

    cap = cv2.VideoCapture(
        BOTTOM_VIDEO_PATH
    )

    if not cap.isOpened():
        print(
            f"[BOTTOM] Cannot open video: {BOTTOM_VIDEO_PATH}"
        )
        running = False
        return

    stream_fps = get_video_fps(
        cap
    )

    frame_period = 1.0 / stream_fps
    next_frame_time = time.perf_counter()

    metadata_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    streamer = None
    streamer_target = None
    frame_id = 0

    cpu_temp = None
    last_temp_read = 0.0

    print(
        f"[BOTTOM] Source FPS: {stream_fps:.3f}"
    )

    try:
        while running:

            with client_lock:
                connected = client_connected
                target_ip = client_ip
                target_video_port = client_bottom_video_port
                target_metadata_port = client_bottom_metadata_port

            if (
                not connected
                or target_ip is None
            ):

                if streamer is not None:
                    streamer.stop()
                    streamer = None
                    streamer_target = None

                time.sleep(0.05)
                continue

            target = (
                target_ip,
                target_video_port
            )

            if (
                streamer is None
                or streamer_target != target
            ):

                if streamer is not None:
                    streamer.stop()

                streamer = GstVideoStreamer(
                    camera_name="bottom",
                    target_ip=target_ip,
                    target_port=target_video_port,
                    fps=stream_fps,
                    bitrate_kbps=BOTTOM_BITRATE_KBPS
                )

                streamer.start()
                streamer_target = target

                cap.set(
                    cv2.CAP_PROP_POS_FRAMES,
                    0
                )

                frame_id = 0
                next_frame_time = time.perf_counter()

                print(
                    "[BOTTOM] STREAM ACTIVE / AI STANDBY"
                )

            pipeline_start = time.perf_counter()

            ret, frame = cap.read()

            if not ret:

                if LOOP_VIDEO:
                    cap.set(
                        cv2.CAP_PROP_POS_FRAMES,
                        0
                    )
                    continue

                time.sleep(0.05)
                continue

            if (
                frame.shape[1] != FRAME_W
                or frame.shape[0] != FRAME_H
            ):

                frame = cv2.resize(
                    frame,
                    (
                        FRAME_W,
                        FRAME_H
                    ),
                    interpolation=cv2.INTER_LINEAR
                )

            frame_id += 1

            now_mono = time.monotonic()

            if (
                now_mono - last_temp_read
                >= CPU_TEMP_INTERVAL_SEC
            ):
                cpu_temp = read_cpu_temperature()
                last_temp_read = now_mono

            # Barcode is added directly to the stream copy.
            # There is no AI on bottom in this version.
            stream_frame = frame.copy()

            barcode_checksum = embed_frame_id_barcode(
                stream_frame,
                frame_id
            )

            if barcode_checksum is None:
                barcode_checksum = frame_id_checksum(
                    frame_id
                )

            burn_debug_frame_id(
                stream_frame,
                frame_id
            )

            timing = streamer.push(
                stream_frame
            )

            pipeline_sec = (
                time.perf_counter()
                - pipeline_start
            )

            pipeline_fps = 1.0 / max(
                pipeline_sec,
                1e-9
            )

            metadata = build_metadata(
                camera="bottom",
                camera_id=2,
                frame_id=frame_id,
                barcode_checksum=barcode_checksum,
                timing=timing,
                mode="IDLE",
                pose=None,
                detection=None,
                target=None,
                pnp=None,
                cpu_temp=cpu_temp,
                model_ms=0.0,
                model_fps=0.0,
                pipeline_fps=pipeline_fps,
                ai_state="STANDBY",
                ai_task="NONE",
                ai_model="NONE"
            )

            try:
                payload = json.dumps(
                    metadata,
                    separators=(",", ":")
                ).encode("utf-8")

                metadata_socket.sendto(
                    payload,
                    (
                        target_ip,
                        target_metadata_port
                    )
                )

            except Exception as error:
                print(
                    "[BOTTOM META ERROR]",
                    error
                )

            if (
                frame_id
                % DEBUG_TERMINAL_EVERY_N
                == 0
            ):
                print(
                    f"\rBOTTOM F:{frame_id:06d} | "
                    f"BAR:{barcode_checksum:02X} | "
                    f"STANDBY | "
                    f"PIPE {pipeline_fps:5.1f}fps",
                    end="",
                    flush=True
                )

            next_frame_time += frame_period

            sleep_time = (
                next_frame_time
                - time.perf_counter()
            )

            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.5:
                next_frame_time = time.perf_counter()

    finally:

        cap.release()

        if streamer is not None:
            streamer.stop()

        metadata_socket.close()


# ============================================================
# MAIN
# ============================================================

def main():

    global running

    # ========================================================
    # FILE CHECK
    # ========================================================

    for file_path in [
        FRONT_VIDEO_PATH,
        BOTTOM_VIDEO_PATH,
        POSE_HEF_PATH,
        YOLO26_HEF_PATH,
    ]:

        if not Path(file_path).exists():
            raise FileNotFoundError(
                file_path
            )

    # ========================================================
    # FRONT VIDEO SOURCE
    # ========================================================

    cap = cv2.VideoCapture(
        FRONT_VIDEO_PATH
    )

    if not cap.isOpened():
        raise RuntimeError(
            "Cannot open FRONT video"
        )

    stream_fps = get_video_fps(
        cap
    )

    # ========================================================
    # HEADER
    # ========================================================

    print()
    print(
        "======================================================"
    )
    print(
        " KINGPHOENIX DRONE VISION SERVER 2026 V5 DUAL"
    )
    print(
        " MPEG-TS + PIXEL BARCODE / FRONT AI ONLY"
    )
    print(
        "======================================================"
    )
    print(
        f"Drone IP          : {get_local_ip()}"
    )
    print(
        f"FRONT Video UDP   : {FRONT_VIDEO_PORT}"
    )
    print(
        f"FRONT Metadata    : {FRONT_METADATA_PORT}"
    )
    print(
        f"Control UDP       : {CONTROL_PORT}"
    )
    print(
        f"BOTTOM Video UDP  : {BOTTOM_VIDEO_PORT}"
    )
    print(
        f"BOTTOM Metadata   : {BOTTOM_METADATA_PORT}"
    )
    print(
        f"FRONT Source      : {FRONT_VIDEO_PATH}"
    )
    print(
        f"BOTTOM Source     : {BOTTOM_VIDEO_PATH}"
    )
    print(
        f"FRONT FPS         : {stream_fps:.3f}"
    )
    print(
        f"Barcode sync      : {BARCODE_ENABLED}"
    )
    print(
        f"Barcode bits      : {BARCODE_TOTAL_BITS}"
    )
    print(
        f"Barcode size      : {BARCODE_WIDTH}x{BARCODE_CELL_H}"
    )
    print(
        f"Barcode XY        : {BARCODE_X},{BARCODE_Y}"
    )
    print(
        "AI active camera  : FRONT"
    )
    print(
        "BOTTOM AI         : STANDBY"
    )
    print(
        "======================================================"
    )

    # ========================================================
    # CAMERA MATRIX
    # ========================================================

    camera_matrix = create_camera_matrix()

    # ========================================================
    # HEF
    # ========================================================

    pose_hef = HEF(
        POSE_HEF_PATH
    )

    detector_hef = HEF(
        YOLO26_HEF_PATH
    )

    # ========================================================
    # HAILO
    # ========================================================

    device = VDevice()

    pose_config = (
        ConfigureParams
        .create_from_hef(
            pose_hef,
            interface=HailoStreamInterface.PCIe
        )
    )

    pose_group = device.configure(
        pose_hef,
        pose_config
    )[0]

    pose_input = InputVStreamParams.make(
        pose_group
    )

    pose_output = OutputVStreamParams.make(
        pose_group,
        format_type=FormatType.FLOAT32
    )

    detector_config = (
        ConfigureParams
        .create_from_hef(
            detector_hef,
            interface=HailoStreamInterface.PCIe
        )
    )

    detector_group = device.configure(
        detector_hef,
        detector_config
    )[0]

    detector_input = InputVStreamParams.make(
        detector_group
    )

    detector_output = OutputVStreamParams.make(
        detector_group,
        format_type=FormatType.FLOAT32
    )

    # ========================================================
    # CONTROL SERVER
    # ========================================================

    control_thread = threading.Thread(
        target=control_server,
        daemon=True
    )
    control_thread.start()

    # ========================================================
    # BOTTOM CAMERA THREAD
    # ========================================================

    bottom_thread = threading.Thread(
        target=bottom_camera_worker,
        daemon=True
    )
    bottom_thread.start()

    # ========================================================
    # FRONT METADATA SOCKET
    # ========================================================

    metadata_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    # ========================================================
    # FRONT STATE
    # ========================================================

    streamer = None
    streamer_target = None

    frame_id = 0

    state = "POSE"
    pose_lost_since = None
    last_pose_probe = 0.0
    last_detection = None
    last_detection_time = 0.0

    cpu_temp = None
    last_temp_read = 0.0

    frame_period = 1.0 / stream_fps
    next_frame_time = time.perf_counter()

    # ========================================================
    # FRONT AI PIPELINES
    # ========================================================

    with InferVStreams(
        pose_group,
        pose_input,
        pose_output
    ) as pose_pipeline:

        with InferVStreams(
            detector_group,
            detector_input,
            detector_output
        ) as detector_pipeline:

            switcher = HailoModelSwitcher(
                pose_group,
                detector_group
            )

            switcher.switch(
                "POSE"
            )

            print()
            print(
                "[SYSTEM] Waiting for Kingphoenix GCS..."
            )

            try:
                while running:

                    # =========================================
                    # GCS STATE
                    # =========================================

                    with client_lock:
                        connected = client_connected
                        target_ip = client_ip
                        target_video_port = (
                            client_front_video_port
                        )
                        target_metadata_port = (
                            client_front_metadata_port
                        )

                    # =========================================
                    # NO CLIENT
                    # =========================================

                    if (
                        not connected
                        or target_ip is None
                    ):

                        if streamer is not None:
                            streamer.stop()
                            streamer = None
                            streamer_target = None

                        time.sleep(0.05)
                        continue

                    # =========================================
                    # START / RESTART FRONT STREAM
                    # =========================================

                    target = (
                        target_ip,
                        target_video_port
                    )

                    if (
                        streamer is None
                        or streamer_target != target
                    ):

                        if streamer is not None:
                            streamer.stop()

                        streamer = GstVideoStreamer(
                            camera_name="front",
                            target_ip=target_ip,
                            target_port=target_video_port,
                            fps=stream_fps,
                            bitrate_kbps=FRONT_BITRATE_KBPS
                        )

                        streamer.start()
                        streamer_target = target

                        cap.set(
                            cv2.CAP_PROP_POS_FRAMES,
                            0
                        )

                        frame_id = 0
                        state = "POSE"
                        pose_lost_since = None
                        last_pose_probe = 0.0
                        last_detection = None
                        last_detection_time = 0.0
                        next_frame_time = time.perf_counter()

                        switcher.switch(
                            "POSE"
                        )

                        print()
                        print(
                            "[FRONT] STREAM ACTIVE / AI ACTIVE"
                        )

                    # =========================================
                    # FRONT FRAME
                    # =========================================

                    pipeline_start = time.perf_counter()

                    ret, frame = cap.read()

                    if not ret:

                        if LOOP_VIDEO:
                            cap.set(
                                cv2.CAP_PROP_POS_FRAMES,
                                0
                            )
                            continue

                        break

                    if (
                        frame.shape[1] != FRAME_W
                        or frame.shape[0] != FRAME_H
                    ):

                        frame = cv2.resize(
                            frame,
                            (
                                FRAME_W,
                                FRAME_H
                            ),
                            interpolation=cv2.INTER_LINEAR
                        )

                    frame_id += 1

                    # =========================================
                    # AI INPUT BEFORE BARCODE
                    # =========================================

                    input_tensor = preprocess_frame(
                        frame
                    )

                    # =========================================
                    # CPU TEMP
                    # =========================================

                    now_mono = time.monotonic()

                    if (
                        now_mono - last_temp_read
                        >= CPU_TEMP_INTERVAL_SEC
                    ):
                        cpu_temp = read_cpu_temperature()
                        last_temp_read = now_mono

                    # =========================================
                    # MODEL SELECT
                    # =========================================

                    pose_probe = False

                    if state == "POSE":
                        active_model = "POSE"
                    else:

                        elapsed_probe_ms = (
                            (
                                now_mono
                                - last_pose_probe
                            )
                            * 1000.0
                        )

                        if (
                            elapsed_probe_ms
                            >= POSE_PROBE_INTERVAL_MS
                        ):
                            active_model = "POSE"
                            pose_probe = True
                        else:
                            active_model = "YOLO26"

                    # =========================================
                    # RESULT DEFAULTS
                    # =========================================

                    pose = None
                    detection = None
                    target_center = None
                    pnp = None
                    mode = "SEARCH"
                    model_ms = 0.0
                    model_fps = 0.0
                    ai_model = active_model

                    # =========================================
                    # YOLOv8 POSE
                    # =========================================

                    if active_model == "POSE":

                        switcher.switch(
                            "POSE"
                        )

                        infer_start = time.perf_counter()

                        outputs = pose_pipeline.infer(
                            input_tensor
                        )

                        infer_sec = (
                            time.perf_counter()
                            - infer_start
                        )

                        model_ms = infer_sec * 1000.0
                        model_fps = 1.0 / max(
                            infer_sec,
                            1e-9
                        )

                        candidate = decode_pose(
                            outputs
                        )

                        if validate_pose(candidate):
                            pose = candidate

                        if pose is not None:

                            state = "POSE"
                            pose_lost_since = None
                            last_detection = None
                            mode = "POSE"
                            ai_model = "YOLO8_POSE"

                            target_center = pose_center(
                                pose
                            )

                            pnp = solve_gate_pose(
                                pose,
                                camera_matrix
                            )

                        else:

                            if pose_probe:

                                state = "FALLBACK"
                                last_pose_probe = now_mono

                                detection_age_ms = (
                                    (
                                        now_mono
                                        - last_detection_time
                                    )
                                    * 1000.0
                                )

                                if (
                                    last_detection is not None
                                    and detection_age_ms
                                    <= FALLBACK_BBOX_MAX_AGE_MS
                                ):
                                    detection = last_detection
                                    target_center = bbox_center(
                                        detection
                                    )
                                    mode = "YOLO26"
                                    ai_model = "YOLO26"

                            else:

                                if pose_lost_since is None:
                                    pose_lost_since = now_mono

                                lost_ms = (
                                    (
                                        now_mono
                                        - pose_lost_since
                                    )
                                    * 1000.0
                                )

                                if (
                                    lost_ms
                                    >= POSE_LOST_TIMEOUT_MS
                                ):
                                    state = "FALLBACK"
                                    last_pose_probe = now_mono

                    # =========================================
                    # YOLO26 FALLBACK
                    # =========================================

                    else:

                        switcher.switch(
                            "YOLO26"
                        )

                        infer_start = time.perf_counter()

                        outputs = detector_pipeline.infer(
                            input_tensor
                        )

                        infer_sec = (
                            time.perf_counter()
                            - infer_start
                        )

                        model_ms = infer_sec * 1000.0
                        model_fps = 1.0 / max(
                            infer_sec,
                            1e-9
                        )

                        detections = decode_yolo26(
                            outputs
                        )

                        detection = select_gate(
                            detections
                        )

                        ai_model = "YOLO26"

                        if detection is not None:

                            last_detection = detection
                            last_detection_time = now_mono
                            mode = "YOLO26"
                            target_center = bbox_center(
                                detection
                            )

                        else:

                            last_detection = None
                            mode = "SEARCH"

                    # =========================================
                    # BARCODE AFTER AI
                    # =========================================

                    stream_frame = frame.copy()

                    barcode_checksum = (
                        embed_frame_id_barcode(
                            stream_frame,
                            frame_id
                        )
                    )

                    if barcode_checksum is None:
                        barcode_checksum = frame_id_checksum(
                            frame_id
                        )

                    burn_debug_frame_id(
                        stream_frame,
                        frame_id
                    )

                    # =========================================
                    # PUSH FRONT VIDEO
                    # =========================================

                    timing = streamer.push(
                        stream_frame
                    )

                    pipeline_sec = (
                        time.perf_counter()
                        - pipeline_start
                    )

                    pipeline_fps = 1.0 / max(
                        pipeline_sec,
                        1e-9
                    )

                    # =========================================
                    # FRONT METADATA
                    # =========================================

                    metadata = build_metadata(
                        camera="front",
                        camera_id=1,
                        frame_id=frame_id,
                        barcode_checksum=barcode_checksum,
                        timing=timing,
                        mode=mode,
                        pose=pose,
                        detection=detection,
                        target=target_center,
                        pnp=pnp,
                        cpu_temp=cpu_temp,
                        model_ms=model_ms,
                        model_fps=model_fps,
                        pipeline_fps=pipeline_fps,
                        ai_state="ACTIVE",
                        ai_task="GATE_NAV",
                        ai_model=ai_model
                    )

                    try:
                        payload = json.dumps(
                            metadata,
                            separators=(",", ":")
                        ).encode("utf-8")

                        metadata_socket.sendto(
                            payload,
                            (
                                target_ip,
                                target_metadata_port
                            )
                        )

                    except Exception as error:
                        print()
                        print(
                            "[FRONT META ERROR]",
                            error
                        )

                    # =========================================
                    # PACING
                    # =========================================

                    next_frame_time += frame_period

                    sleep_time = (
                        next_frame_time
                        - time.perf_counter()
                    )

                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    elif sleep_time < -0.5:
                        next_frame_time = time.perf_counter()

                    # =========================================
                    # TERMINAL
                    # =========================================

                    if (
                        frame_id
                        % DEBUG_TERMINAL_EVERY_N
                        == 0
                    ):

                        if cpu_temp is not None:
                            temp_string = f"{cpu_temp:.1f}C"
                        else:
                            temp_string = "--.-C"

                        print(
                            f"\rFRONT F:{frame_id:06d} | "
                            f"BAR:{barcode_checksum:02X} | "
                            f"{mode:7s} | "
                            f"AI {model_ms:5.1f}ms | "
                            f"{model_fps:5.1f}fps | "
                            f"CPU {temp_string}",
                            end="",
                            flush=True
                        )

            except KeyboardInterrupt:

                print()
                print()
                print(
                    "[SYSTEM] CTRL+C"
                )

            finally:

                running = False
                switcher.close()
                cap.release()

                if streamer is not None:
                    streamer.stop()

                metadata_socket.close()

    print()
    print(
        "[SYSTEM] DONE"
    )


# ============================================================
# ENTRY
# ============================================================

if __name__ == "__main__":
    main()
