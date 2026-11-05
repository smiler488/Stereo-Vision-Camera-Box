"""Calibrate on the bundled images and run the disparity pipeline offline."""
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import stereo_capture_box_gui as gui  # noqa: E402


def test_calibration_on_sample_images(tmp_path):
    out = tmp_path / "params.json"
    subprocess.run([sys.executable, str(ROOT / "stereo_calibrate.py"),
                    "--left", str(ROOT / "calibration/images/left/*.png"),
                    "--right", str(ROOT / "calibration/images/right/*.png"),
                    "-o", str(out)], check=True, capture_output=True)
    p = json.loads(out.read_text())
    assert p["pairs_used"] == 20
    assert p["rms_stereo_px"] < 0.3
    assert 50 < np.linalg.norm(p["T_mm"]) < 70


def test_rig_reprojects_sample_pair(tmp_path):
    params, _ = gui.load_params(ROOT / "calibration/stereo_params_from_sample_images.json")
    rig = gui.StereoRig(params)
    left = cv2.imread(str(ROOT / "calibration/images/left/left5.png"))
    right = cv2.imread(str(ROOT / "calibration/images/right/right5.png"))
    l, r, disp = rig.process(np.hstack([left, right]))
    assert l.shape == left.shape and disp.shape == left.shape[:2]
    xyz = cv2.reprojectImageTo3D(disp, rig.Q, handleMissingValues=True)
    mask = (disp > 0) & (xyz[..., 2] > 0) & (xyz[..., 2] < 2000)
    assert mask.mean() > 0.5
    assert 300 < np.median(xyz[..., 2][mask]) < 700     # board inside the box, ~0.5 m away
    ply = tmp_path / "t.ply"
    gui.write_ply(ply, xyz[mask][:100], l[mask][:100])
    assert "element vertex 100" in ply.read_text().splitlines()[2]


def test_missing_calibration_falls_back(tmp_path):
    params, source = gui.load_params(tmp_path / "nope.json")
    assert source == "built-in" and params["image_size"] == [640, 480]
