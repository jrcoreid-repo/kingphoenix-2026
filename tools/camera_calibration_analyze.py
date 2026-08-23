#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
import argparse, json, math
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def load_dataset(camera, cols, rows, square_mm):
    capdir = ROOT / "calibration" / "captures" / camera
    paths = sorted(list(capdir.glob("*.jpg")) + list(capdir.glob("*.png")))
    if not paths:
        raise SystemExit(f"No calibration images in {capdir}")

    obj = np.zeros((rows*cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= float(square_mm)

    op, ip, used = [], [], []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)

    for p in paths:
        im = cv2.imread(str(p))
        if im is None:
            continue
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        ok, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if not ok:
            print("[REJECT DETECT]", p.name)
            continue
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), crit)
        op.append(obj.copy())
        ip.append(corners)
        used.append(p)

    if len(used) < 8:
        raise SystemExit(f"Only {len(used)} valid images; need at least 8.")
    return paths, used, op, ip, size

def calibrate(op, ip, size, indices, flags=0):
    op2 = [op[i] for i in indices]
    ip2 = [ip[i] for i in indices]
    rms, K, D, rv, tv = cv2.calibrateCamera(op2, ip2, size, None, None, flags=flags)
    errs = []
    for j, i in enumerate(indices):
        proj, _ = cv2.projectPoints(op[i], rv[j], tv[j], K, D)
        # Per-image reprojection RMSE in pixels.
        # cv2.norm() returns sqrt(sum(error^2)); divide by sqrt(N),
        # not N. The previous V1.9.5.2 formula under-reported per-image
        # errors by a factor of sqrt(number_of_corners).
        n_points = max(len(proj), 1)
        e = cv2.norm(ip[i], proj, cv2.NORM_L2) / np.sqrt(n_points)
        errs.append(float(e))
    return {
        "rms": float(rms), "K": K, "D": D.reshape(-1),
        "errors": errs, "indices": list(indices),
        "mean": float(np.mean(errs)), "max": float(np.max(errs))
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", choices=["front","bottom"], required=True)
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--square-mm", type=float, required=True)
    ap.add_argument("--sigma", type=float, default=1.6)
    ap.add_argument("--hard-px", type=float, default=2.00)
    ap.add_argument("--min-keep", type=int, default=18)
    ap.add_argument("--write-best", action="store_true")
    a = ap.parse_args()

    all_paths, used, op, ip, size = load_dataset(
        a.camera, a.cols, a.rows, a.square_mm
    )

    base = calibrate(op, ip, size, list(range(len(used))), 0)

    print("="*78)
    print(f" KINGPHOENIX V1.9.5.3 - {a.camera.upper()} CALIBRATION ANALYZER")
    print("="*78)
    print(f"Detected images : {len(used)}/{len(all_paths)}")
    print(f"Resolution      : {size[0]}x{size[1]}")
    print(f"Baseline RMS    : {base['rms']:.4f} px")
    print(f"Baseline mean   : {base['mean']:.4f} px")
    print(f"Baseline max    : {base['max']:.4f} px")

    ranked = sorted(
        zip(base["indices"], base["errors"]),
        key=lambda x: x[1], reverse=True
    )

    print("\n[PER-IMAGE ERROR - WORST FIRST]")
    print("RANK   ERROR(px)   IMAGE")
    for rank, (idx, err) in enumerate(ranked, 1):
        print(f"{rank:4d}   {err:9.4f}   {used[idx].name}")

    arr = np.array(base["errors"], float)
    threshold = min(
        a.hard_px,
        float(arr.mean() + a.sigma * arr.std())
    )

    keep = [
        idx for idx, err in zip(base["indices"], base["errors"])
        if err <= threshold
    ]

    if len(keep) < a.min_keep:
        keep = [idx for idx, _ in sorted(
            zip(base["indices"], base["errors"]),
            key=lambda x: x[1]
        )[:a.min_keep]]

    removed = sorted(set(base["indices"]) - set(keep))

    print("\n[OUTLIER REJECTION]")
    print(f"Threshold      : {threshold:.4f} px")
    print(f"Keep / remove  : {len(keep)} / {len(removed)}")
    for idx in removed:
        print(" REMOVE        :", used[idx].name)

    models = [
        ("FULL_5_COEFF", 0),
        ("FIX_K3", cv2.CALIB_FIX_K3),
        ("ZERO_TANGENT", cv2.CALIB_ZERO_TANGENT_DIST),
        ("FIX_K3_ZERO_TANGENT",
         cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST),
    ]

    results = {}
    print("\n[MODEL COMPARISON]")
    for name, flags in models:
        r = calibrate(op, ip, size, keep, flags)
        results[name] = r
        print(
            f"{name:22s} RMS={r['rms']:.4f}px "
            f"mean={r['mean']:.4f}px max={r['max']:.4f}px "
            f"N={len(r['indices'])}"
        )

    penalty = {
        "FULL_5_COEFF": 0.02,
        "FIX_K3": 0.0,
        "ZERO_TANGENT": 0.01,
        "FIX_K3_ZERO_TANGENT": 0.005,
    }
    best_name = min(
        results,
        key=lambda n: (results[n]["rms"] + penalty[n], results[n]["mean"])
    )
    best = results[best_name]
    K, D = best["K"], best["D"]
    w, h = size
    fx, fy = float(K[0,0]), float(K[1,1])
    hfov = math.degrees(2*math.atan(w/(2*fx)))
    vfov = math.degrees(2*math.atan(h/(2*fy)))

    print("\n[BEST MODEL]")
    print("Model          :", best_name)
    print(f"RMS / mean/max : {best['rms']:.4f} / {best['mean']:.4f} / {best['max']:.4f} px")
    print(f"fx / fy        : {fx:.3f} / {fy:.3f}")
    print(f"cx / cy        : {K[0,2]:.3f} / {K[1,2]:.3f}")
    print(f"HFOV / VFOV    : {hfov:.2f} / {vfov:.2f} deg")
    print("D              :", D.tolist())

    report = {
        "camera": a.camera,
        "board": {
            "inner_corners": [a.cols, a.rows],
            "square_size_mm": a.square_mm,
        },
        "baseline": {
            "rms": base["rms"], "mean": base["mean"], "max": base["max"],
            "valid_images": len(used),
        },
        "outlier_policy": {
            "sigma": a.sigma, "hard_px": a.hard_px,
            "threshold_px": threshold,
            "removed_files": [used[i].name for i in removed],
        },
        "best_model": best_name,
        "best": {
            "rms": best["rms"], "mean": best["mean"], "max": best["max"],
            "camera_matrix": K.tolist(),
            "dist_coeffs": D.tolist(),
            "hfov_deg": hfov, "vfov_deg": vfov,
            "used_files": [used[i].name for i in best["indices"]],
        }
    }
    report_path = ROOT / "calibration" / f"{a.camera}_calibration_analysis.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print("Analysis JSON  :", report_path)

    if a.write_best:
        target = ROOT / "config" / "calibration" / f"{a.camera}_camera.json"
        existing = {}
        if target.exists():
            try:
                existing = json.loads(target.read_text())
            except Exception:
                pass

        data = {
            "schema": "kingphoenix.camera_calibration.v1",
            "camera_name": a.camera,
            "camera": a.camera,
            "valid": True,
            "image_width": w,
            "image_height": h,
            "camera_matrix": K.tolist(),
            "dist_coeffs": D.tolist(),
            "intrinsics": {
                "resolution": [w,h],
                "camera_matrix": K.tolist(),
                "dist_coeffs": D.tolist(),
                "distortion_model": "opencv_plumb_bob",
            },
            "camera_to_body": existing.get(
                "camera_to_body",
                {"position_m":[0,0,0], "rpy_deg":[0,0,0]}
            ),
            "extrinsics": existing.get(
                "extrinsics",
                {"position_body_m":[0,0,0], "rotation_body_deg":[0,0,0]}
            ),
            "calibration_quality": {
                "model": best_name,
                "rms_reprojection_error_px": best["rms"],
                "mean_reprojection_error_px": best["mean"],
                "max_per_image_error_px": best["max"],
                "valid_images": len(best["indices"]),
                "removed_outliers": len(removed),
            },
            "hfov_deg": hfov,
            "vfov_deg": vfov,
            "board": {
                "type": "checkerboard",
                "inner_corners": [a.cols,a.rows],
                "square_size_mm": a.square_mm,
            },
            "runtime_contract": {
                "resolution": [w,h],
                "color_order": "BGR",
            },
            "used_images": [used[i].name for i in best["indices"]],
        }
        target.write_text(json.dumps(data, indent=2) + "\n")
        print("[WRITE] Best calibration ->", target)

    if best["rms"] <= 0.5:
        print("[QUALITY] EXCELLENT")
    elif best["rms"] <= 0.8:
        print("[QUALITY] GOOD")
    elif best["rms"] <= 1.0:
        print("[QUALITY] USABLE / REVIEW")
    else:
        print("[QUALITY] STILL HIGH")

if __name__ == "__main__":
    main()
