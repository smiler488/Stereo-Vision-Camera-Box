"""
Stereo calibration for the camera box.

Reads matching left/right chessboard images, calibrates each camera,
then the stereo pair, and writes everything the capture GUI needs to a
JSON file.

The board in ``calibration/`` is the calib.io 9 x 6 squares, 15 mm board,
which has 8 x 5 *inner* corners. That is what ``--pattern`` refers to.

Example
-------
    python stereo_calibrate.py --left "calibration/images/left/*.png" \
        --right "calibration/images/right/*.png" --pattern 8 5 --square 15

Author: Liangchao Deng (邓良超)
License: MIT
"""

import argparse
import glob
import json
import re

import cv2
import numpy as np


def natural_key(path):
    """Sort 'left2' before 'left10'."""
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", path)]


def find_corners(gray, pattern):
    ok, corners = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def calibrate(left_files, right_files, pattern, square_mm):
    if len(left_files) != len(right_files):
        raise SystemExit(f"{len(left_files)} left vs {len(right_files)} right images")

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_mm

    obj_pts, pts_l, pts_r, size = [], [], [], None
    for fl, fr in zip(left_files, right_files):
        gl = cv2.imread(fl, cv2.IMREAD_GRAYSCALE)
        gr = cv2.imread(fr, cv2.IMREAD_GRAYSCALE)
        if gl is None or gr is None:
            print(f"skip {fl} / {fr}: cannot read")
            continue
        size = gl.shape[::-1]
        cl, cr = find_corners(gl, pattern), find_corners(gr, pattern)
        if cl is None or cr is None:
            print(f"skip {fl} / {fr}: board not found in both views")
            continue
        obj_pts.append(objp)
        pts_l.append(cl)
        pts_r.append(cr)

    if len(obj_pts) < 5:
        raise SystemExit(f"only {len(obj_pts)} usable pairs; take at least 10-15")

    rms_l, K1, D1, _, _ = cv2.calibrateCamera(obj_pts, pts_l, size, None, None)
    rms_r, K2, D2, _, _ = cv2.calibrateCamera(obj_pts, pts_r, size, None, None)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(
        obj_pts, pts_l, pts_r, K1, D1, K2, D2, size,
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)

    print(f"pairs used: {len(obj_pts)}")
    print(f"RMS reprojection error  left {rms_l:.3f}px  right {rms_r:.3f}px  stereo {rms:.3f}px")
    print(f"baseline: {np.linalg.norm(T):.2f} mm")

    return {
        "image_size": list(size),
        "pattern_inner_corners": list(pattern),
        "square_size_mm": square_mm,
        "pairs_used": len(obj_pts),
        "rms_stereo_px": rms,
        "left_camera_matrix": K1.tolist(),
        "left_distortion": D1.ravel().tolist(),
        "right_camera_matrix": K2.tolist(),
        "right_distortion": D2.ravel().tolist(),
        "R": R.tolist(),
        "T_mm": T.ravel().tolist(),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="Calibrate the stereo camera box.")
    ap.add_argument("--left", default="calibration/images/left/*.png", help="glob for left images")
    ap.add_argument("--right", default="calibration/images/right/*.png", help="glob for right images")
    ap.add_argument("--pattern", type=int, nargs=2, default=(8, 5), metavar=("COLS", "ROWS"),
                    help="inner corners per row and column (default: 8 5)")
    ap.add_argument("--square", type=float, default=15.0, help="square size in mm (default: 15)")
    ap.add_argument("-o", "--output", default="stereo_params.json")
    args = ap.parse_args(argv)

    left = sorted(glob.glob(args.left), key=natural_key)
    right = sorted(glob.glob(args.right), key=natural_key)
    params = calibrate(left, right, tuple(args.pattern), args.square)
    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump(params, fh, indent=2)
    print(f"written to {args.output}")


if __name__ == "__main__":
    main()
