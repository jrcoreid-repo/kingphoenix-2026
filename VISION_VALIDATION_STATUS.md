# Kingphoenix V1.9 Vision Validation

## Bench validated before V1.9.1

- dual front/bottom video stream
- MPEG-TS/H264 UDP
- 48-cell barcode frame_id
- UDP metadata
- exact frame_id video/metadata sync
- Windows GCS live display
- HELLO/HELLO_ACK handshake

## V1.9.1 implementation target

- front Hailo AI active
- YOLOv8 Pose primary
- YOLO26 gate detector fallback
- exact AI metadata tied to the same video frame_id
- GCS pose/bbox overlay
- bottom AI remains standby

## Explicitly disabled

- vision-driven MAVLink movement
- real solvePnP xyz/rpy until front camera calibration is valid
- bottom ArUco AI (later V1.9.x milestone)


## V1.9.2 architecture additions

- dynamic front/bottom AI route
- selectable OFF / POSE_ONLY / YOLO26_ONLY / HYBRID
- detector class selectable per mission route
- bench timer switch front class-1 -> bottom class-2 every 5 s
- global SharedState vision snapshot for future movement consumer
- test-mode watermark; hidden for FULL_MISSION
