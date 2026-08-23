# Kingphoenix Camera Calibration Hotfix V1.9.5.1

The calibration tools use the same real Picamera2 CameraManager configuration
as the Kingphoenix vision stream.

For the user's 16 x 11 square checkerboard:
- squares = 16 x 11
- INNER corners = 15 x 10

Example FRONT:

    python tools/camera_calibration_capture.py --camera front --cols 15 --rows 10 --count 25

Then measure one printed square accurately. Example if one square is 20 mm:

    python tools/camera_calibrate.py --camera front --cols 15 --rows 10 --square-mm 20

Validate:

    python tools/camera_calibration_validate.py --camera front

Repeat with --camera bottom.

Output is written directly to the project paths consumed by vision:

    config/calibration/front_camera.json
    config/calibration/bottom_camera.json

The generated file sets valid=true and contains top-level camera_matrix and
dist_coeffs compatible with GatePoseSolver.


## V1.9.5.2 Calibration Analyzer

For the existing BOTTOM dataset:

```bash
python tools/camera_calibration_analyze.py \
  --camera bottom \
  --cols 8 \
  --rows 6 \
  --square-mm 25
```

This ranks every capture by reprojection error, rejects only outliers, and
compares FULL_5_COEFF, FIX_K3, ZERO_TANGENT, and FIX_K3_ZERO_TANGENT.

After reviewing the result, write the best model into the project:

```bash
python tools/camera_calibration_analyze.py \
  --camera bottom \
  --cols 8 \
  --rows 6 \
  --square-mm 25 \
  --write-best
```

Source captures are never deleted.


## Important V1.9.5.3 note
Do not judge V1.9.5.2 `mean`/`max` reprojection values; they were scaled
incorrectly. Re-run `camera_calibration_analyze.py` with V1.9.5.3, first
without `--write-best`, review the result, then write the selected calibration.


## V1.9.6
Re-capture FRONT and BOTTOM calibration at 1280x720 full-FOV. Old 640x640 calibration is obsolete.


## V1.9.6.1 full-sensor calibration

Calibration must now be captured at 1920x1200, the same full-sensor geometry
used by solvePnP.

Example:

```bash
python tools/camera_calibration_capture.py \
  --camera front --cols 8 --rows 6 --count 30

python tools/camera_calibrate.py \
  --camera front --cols 8 --rows 6 --square-mm 25

python tools/camera_calibration_validate.py --camera front
```

Repeat for bottom. Any old 640x640 or 1280x720 calibration JSON is obsolete.
