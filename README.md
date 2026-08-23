# KINGPHOENIX V2.1 - PI HEALTH CHECKPOINT

# KINGPHOENIX V2.0.1 — Lightweight YOLO26 Architecture

V2 carries forward the stable V1 foundation (direct UART MAVLink, dedicated RX,
VehicleState/freshness, VCT, dual camera streaming, barcode sync and GCS) while
starting the selected lightweight flight architecture.

```text
AR0234 full-wide sensor mode 1920x1200
runtime target 30 fps
    +-> network/GCS: native 640x400 @30
    +-> active camera only:
          640x400 resize
          + 120px top/bottom letterbox -> YOLO 640x640
          YOLO26 ONLY on Hailo
          bbox inverse-map -> 1920x1200
          bbox estimator -> global SharedState
```

Normal V2 runtime never loads the pose HEF. solvePnP/calibration tools remain
available only for research/debug and are not a flight dependency.

The bbox estimator publishes `ex`, `ey`, bbox area/size ratios, phase, and a
bounded velocity intent. V2.0 does **not** transmit that intent over MAVLink.

Front gate: `ex -> lateral/yaw`, bbox area -> approach/commit, LiDAR -> altitude.
Bottom drop: `ex -> right/left`, `ey -> forward/back`, LiDAR -> altitude.

The 1920x1200 sensor mode supports 60 fps, but V2 requests 30 fps by default to
preserve thermal headroom. The GCS uses native 640x400 video, so the existing
3000/2500 kbps bitrate is spent on useful pixels instead of black padding.

Run:
```bash
python tools/v2_architecture_self_check.py
python tools/camera_test.py
python tools/vision_stream_test.py
```

Windows GCS:
```bash
python gcs/kingphoenix_gcs_v2_lightweight_wide.py
```


## V2.0.1 Hailo activation hotfix

V2.0 configured the YOLO26 network group and created InferVStreams but did not
keep the network group activation context open. HailoRT therefore reported
`HAILO_NETWORK_GROUP_NOT_ACTIVATED(69)` when inference started.

V2.0.1:
- activates the YOLO26 network group once at engine open,
- keeps it active for the complete detector runtime,
- cleanly deactivates it during close,
- serializes inference across FRONT/BOTTOM during AI-router transitions,
- keeps the pose HEF unloaded,
- fixes metadata labels to `DISPLAY_640x400_WIDE` and `CAMERA_1920x1200`.


See `PI_HEALTH_INTEGRATION.md` for V2.1 changes.
