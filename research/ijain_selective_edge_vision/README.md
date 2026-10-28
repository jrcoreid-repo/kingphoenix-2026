# IJAIN Selective Edge Vision — research reference implementation

**Status:** This is newly reconstructed reference code based on the IJAIN manuscript, NOT an archival copy of historical flight-experiment software. It is **not flight validated**. It transmits no MAVLink commands, does not operate motors, and cannot substantiate the manuscript's reported numerical results.

Manuscript: Look Only When Needed: Selective Edge Vision for Unmanned Aerial Vehicle Guidance.

## Why this package is separate

The existing KingPhoenix main branch uses YOLO26/Hailo with a bbox-based estimator. Its V2 README explicitly notes that solvePnP is not a normal-flight dependency and that vision-driven MAVLink movement is disabled. Older vision/gate_pose.py provides a separate research/debug IPPE solver. Altering these production paths to imply an experimentally demonstrated closed-loop system would be misleading and potentially unsafe.

## Mapping to the methods

- Method 2.1 and 2.4 / Eq. (5): two-level activation (mission + refresh), continuous, fixed-rate k=2,5,10, confidence, geometry, confidence–geometry, and full reliability policies in scheduler.py.
- Refresh conditions: invalid physical corners; latest detector confidence < 0.50; normalized projection residual > 0.01; normalized causal temporal residual > 0.03; detector age >= 0.50 s. Simulated busy requests are coalesced.
- Detector age uses the last detection's source capture timestamp; it does not reset on a geometric update.
- Method 2.3: contour-derived physical corner candidates inside an expanded detector ROI and planar IPPE PnP with real intrinsics, positive depth and unresolved-ambiguity rejection are provided in geometry.py. Detector bounding-box corners are not used as PnP correspondences.
- Method 2.5: measurement acceptance separately checks four corners, positive depth, resolved ambiguity, <=3 px reprojection RMSE, and <=150 ms geometric feedback age.
- Method 2.6: offline trace replay compares policy execution counts, coverage, RTK-paired localization RMSE and the 0.20 m high-error acceptance threshold.

## Run from repository root

Python 3.10+; scheduler and CSV replay use only the standard library.

    python -m unittest research.ijain_selective_edge_vision.test_research -v
    python -m research.ijain_selective_edge_vision.replay --input /path/to/recorded_trace.csv --out-dir /tmp/ijain-replay --policy all --inference-ms 43

Run optional image geometry with opencv-python and numpy, measured gate dimensions and calibrated camera intrinsics at the native capture resolution.
See SCHEMA.md for the trace input. The evaluator outputs events_POLICY.csv and summary.json. Its assumed inference duration is not a measurement; replace 43 ms with the real log.

## Provenance and missing material

- Existing KingPhoenix repository: https://github.com/jrcoreid-repo/kingphoenix-2026
- Dataset supplied by authors: https://platform.ultralytics.com/jr-core/datasets/04-fullclass (public accessibility not yet confirmed).
- Not contained here: paper-specific model weights and HEF, camera extrinsic calibration, synchronized original video/RTK/power traces, 50 trial-level flight outcomes or original event logs.
- Current main-branch config/calibration/front_camera.json is marked valid=false. Do not fake a camera calibration.
- Offline replay is a TRACE-LEVEL model, not reproduction of physical inference/ROI reacquisition/closed-loop dynamics. It must not be represented as the actual software run in manuscript experiments.

## Independent verification required before publication

Recover the original experiment code revision, validate dataset split and model conversion, archive raw trial logs, and correlate detector execution timestamps, RTK quality, motion control and battery-line voltage/current with tables and figures. Tests here only verify reference algorithm properties, not the claimed flight results.

No field-control integration is intentionally provided. Safety testing and calibration must precede any UAV control use.
