# Kingphoenix Version Map

This convention is intentionally simple so the development track is memorable.

## Frozen baseline

### V1.7.1 — Validated MAVLink/VCT checkpoint
Hardware-validated mode VCT and expected ARM rejection handling.

## Reserved physical-flight branch

### V1.8.x — Flight Physical / MAVLink VCT
Reserved for work that needs the actual complete aircraft:

- successful ARM/DISARM
- TAKEOFF
- LAND
- physical altitude verification
- AUTO/GUIDED transitions under flight
- Guided motion watchdog

V1.8.x is intentionally deferred until the aircraft is physically ready.

## Bench vision branch

### V1.9.x — Vision / Streaming
Work that can be developed in the lab without flying:

- camera ingestion
- front/bottom live stream
- barcode + metadata synchronization
- GCS handshake
- AI router
- Hailo gate detection
- OpenCV gate geometry / pose
- bottom ArUco pose
- YOLO26 class-2 drop fallback

No vision-driven aircraft movement is allowed in V1.9.x.

## Integration milestone

### V2.0
Merge the hardware-validated V1.8.x flight branch and the validated V1.9.x
vision branch into the complete Kingphoenix runtime.
