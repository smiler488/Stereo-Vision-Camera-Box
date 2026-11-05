# -*- coding: utf-8 -*-
"""
Capture GUI for the stereo camera box.

Reads the side-by-side stream of a USB stereo module, rectifies both views,
computes a disparity map (SGBM, optionally refined with a WLS filter) and
saves, per sample: rectified left/right images, a colour disparity image,
the raw disparity as .npy, a coloured point cloud (.ply, millimetres) and a
small JSON metadata file. Clicking the disparity view prints the distance of
that pixel.

Calibration is read from ``stereo_params.json`` (written by
``stereo_calibrate.py``). Without that file the built-in values below are
used; they belong to my own box and will be wrong for any other unit.

Keys: ``s`` saves the current frame, ``n`` starts the next sample.

Author: Liangchao Deng (邓良超)
License: MIT
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

try:  # the GUI needs Tk; StereoRig and the helpers work without it
    import tkinter as tk
    from tkinter import messagebox, ttk
    from PIL import ImageTk
except ImportError:  # pragma: no cover - depends on the Python build
    tk = None

# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

# Built-in parameters of my box (640 x 480 per view, baseline about 120 mm).
DEFAULT_PARAMS = {
    "image_size": [640, 480],
    "left_camera_matrix": [[416.841180253704, 0.0, 338.485167779639],
                           [0.0, 416.465934495134, 230.419201769346],
                           [0.0, 0.0, 1.0]],
    "left_distortion": [-0.0170280933781798, 0.0643596519467521,
                        -0.00161785356900972, -0.00330684695473645, 0.0],
    "right_camera_matrix": [[417.765094485395, 0.0, 315.061245379892],
                            [0.0, 417.845058291483, 238.181766936442],
                            [0.0, 0.0, 1.0]],
    "right_distortion": [-0.0394089328586398, 0.131112076868352,
                         -0.00133793245429668, -0.00188957913931929, 0.0],
    "R": [[0.999962872853149, 0.00187779299260463, -0.00840992323112715],
          [-0.0018408858041373, 0.999988651353238, 0.00439412154902114],
          [0.00841807904053251, -0.00437847669953504, 0.999954981430194]],
    "T_mm": [-120.326603502087, 0.199732192805711, -0.203594457929446],
}

PREVIEW_SCALE = 0.6      # size of the preview images relative to the camera frame
FRAME_INTERVAL_MS = 30   # GUI refresh period


def load_params(path):
    """Read calibration from JSON, falling back to the built-in values."""
    if path and Path(path).is_file():
        with open(path, encoding="utf-8") as fh:
            params = json.load(fh)
        print(f"Calibration loaded from {path}")
        return params, str(path)
    print("No calibration file found, using built-in parameters.")
    return DEFAULT_PARAMS, "built-in"


class StereoRig:
    """Rectification maps, disparity matcher and reprojection matrix."""

    def __init__(self, params, num_disp=160, block_size=5, use_wls=True):
        self.size = tuple(params["image_size"])
        K1 = np.array(params["left_camera_matrix"], dtype=np.float64)
        D1 = np.array(params["left_distortion"], dtype=np.float64)
        K2 = np.array(params["right_camera_matrix"], dtype=np.float64)
        D2 = np.array(params["right_distortion"], dtype=np.float64)
        R = np.array(params["R"], dtype=np.float64)
        self.T = np.array(params["T_mm"], dtype=np.float64).reshape(3, 1)

        R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, self.size, R, self.T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.focal_px = P1[0, 0]
        self.baseline_mm = float(np.linalg.norm(self.T))
        self.map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, self.size, cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, self.size, cv2.CV_16SC2)

        self.num_disp = num_disp  # must be a multiple of 16
        self.block_size = block_size
        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,
            P2=32 * 3 * block_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=12,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        # WLS refinement needs opencv-contrib-python
        self.use_wls = False
        if use_wls and hasattr(cv2, "ximgproc"):
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
            self.wls = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
            self.wls.setLambda(8000.0)
            self.wls.setSigmaColor(1.5)
            self.use_wls = True

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def split(self, frame):
        """Split a side-by-side frame into raw left and right images."""
        w, h = self.size
        return frame[:h, :w], frame[:h, w:2 * w]

    def process(self, frame):
        """Return rectified left, rectified right and float disparity (pixels)."""
        left_raw, right_raw = self.split(frame)
        left = cv2.remap(left_raw, *self.map_l, cv2.INTER_LINEAR)
        right = cv2.remap(right_raw, *self.map_r, cv2.INTER_LINEAR)
        gl = self.clahe.apply(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY))
        gr = self.clahe.apply(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY))

        # SGBM returns fixed-point disparity (int16, x16)
        disp_l = self.left_matcher.compute(gl, gr)
        if self.use_wls:
            # The WLS filter expects the raw x16 maps, not values already divided by 16
            disp_r = self.right_matcher.compute(gr, gl)
            disp_l = self.wls.filter(disp_l, left, disparity_map_right=disp_r)
        disp = disp_l.astype(np.float32) / 16.0
        return left, right, disp

    def depth_at(self, disp_value):
        """Distance along the optical axis in metres for one disparity value."""
        return self.focal_px * self.baseline_mm / 1000.0 / disp_value


def disparity_to_color(disp):
    """Colour-map a disparity image, ignoring invalid (<= 0) pixels."""
    valid = disp > 0
    out = np.zeros(disp.shape + (3,), np.uint8)
    if valid.any():
        lo, hi = np.percentile(disp[valid], [1, 99])
        scaled = np.clip((disp - lo) / max(hi - lo, 1e-6), 0, 1)
        out = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_JET)
        out[~valid] = 0
    return out


def write_ply(path, points, colors_bgr):
    """Write an ASCII PLY with xyz (float) and rgb (uchar)."""
    rgb = colors_bgr[:, ::-1].astype(np.uint8)
    header = (
        "ply\nformat ascii 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    with open(path, "w", encoding="ascii") as fh:
        fh.write(header)
        data = np.hstack([points.astype(np.float32), rgb.astype(np.float32)])
        np.savetxt(fh, data, fmt="%.4f %.4f %.4f %d %d %d")


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class StereoApp:
    def __init__(self, root, rig, cam_id=0, out_dir="data", calib_source="built-in",
                 max_depth_mm=2000.0):
        self.root = root
        self.rig = rig
        self.cam_id = cam_id
        self.out_dir = Path(out_dir)
        self.calib_source = calib_source
        self.max_depth_mm = max_depth_mm
        root.title("Stereo capture box")

        self.sample_id = tk.StringVar()
        self.status = tk.StringVar(value="Enter a sample ID and press Start preview.")
        self.running = False
        self.last = (None, None, None)  # rectified left, rectified right, disparity

        ctrl = ttk.Frame(root)
        ctrl.pack(fill="x", padx=10, pady=8)
        ttk.Label(ctrl, text="Sample ID:").pack(side="left")
        self.entry = ttk.Entry(ctrl, textvariable=self.sample_id, width=20)
        self.entry.pack(side="left", padx=6)
        ttk.Button(ctrl, text="Start preview", command=self.start_preview).pack(side="left", padx=4)
        ttk.Button(ctrl, text="Save (s)", command=self.save_current).pack(side="left", padx=4)
        ttk.Button(ctrl, text="Next sample (n)", command=self.next_sample).pack(side="left", padx=4)
        ttk.Button(ctrl, text="Quit", command=self.on_quit).pack(side="right")

        view = ttk.Frame(root)
        view.pack(fill="both", expand=True, padx=10, pady=4)
        self.lbl_left = ttk.Label(view, text="Left")
        self.lbl_right = ttk.Label(view, text="Right")
        self.lbl_disp = ttk.Label(view, text="Disparity (click to measure)")
        self.lbl_left.grid(row=0, column=0, padx=5, pady=5)
        self.lbl_right.grid(row=0, column=1, padx=5, pady=5)
        self.lbl_disp.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        self.lbl_disp.bind("<Button-1>", self.on_disp_click)

        ttk.Label(root, textvariable=self.status, anchor="w").pack(fill="x", padx=10, pady=(0, 8))

        root.bind("<KeyPress-s>", lambda e: self.save_current() if e.widget is not self.entry else None)
        root.bind("<KeyPress-n>", lambda e: self.next_sample() if e.widget is not self.entry else None)
        root.protocol("WM_DELETE_WINDOW", self.on_quit)

        w, h = rig.size
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w * 2)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self.cap.isOpened():
            messagebox.showerror("Camera", f"Cannot open camera {cam_id}.")
            root.after(200, root.destroy)

    # Frames are grabbed on the Tk main loop via after(); Tk widgets must not
    # be touched from a worker thread (that crashed on macOS before).
    def start_preview(self):
        if self.running:
            return
        if not self.sample_id.get().strip():
            messagebox.showwarning("Sample ID", "Enter a sample ID first.")
            return
        self.running = True
        self.status.set(f"Previewing. Calibration: {self.calib_source}, "
                        f"baseline {self.rig.baseline_mm:.1f} mm, WLS {'on' if self.rig.use_wls else 'off'}.")
        self.update_frame()

    def update_frame(self):
        if not self.running:
            return
        ok, frame = self.cap.read()
        if ok and frame.shape[1] >= 2 * self.rig.size[0]:
            left, right, disp = self.rig.process(frame)
            self.last = (left, right, disp)
            self.show(self.lbl_left, left)
            self.show(self.lbl_right, right)
            self.show(self.lbl_disp, disparity_to_color(disp))
        self.root.after(FRAME_INTERVAL_MS, self.update_frame)

    def show(self, label, bgr):
        w, h = self.rig.size
        im = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        im = im.resize((int(w * PREVIEW_SCALE), int(h * PREVIEW_SCALE)))
        label.imgtk = ImageTk.PhotoImage(image=im)
        label.configure(image=label.imgtk)

    def save_current(self):
        sample = self.sample_id.get().strip()
        left, right, disp = self.last
        if not sample:
            messagebox.showwarning("Sample ID", "Enter a sample ID first.")
            return
        if left is None:
            messagebox.showwarning("Save", "No frame yet. Start the preview first.")
            return

        ts = time.strftime("%Y%m%d_%H%M%S")
        base = self.out_dir / sample
        for sub in ("left", "right", "depth", "pointcloud"):
            (base / sub).mkdir(parents=True, exist_ok=True)
        stem = f"{sample}_{ts}"

        cv2.imwrite(str(base / "left" / f"{stem}_left.png"), left)
        cv2.imwrite(str(base / "right" / f"{stem}_right.png"), right)
        cv2.imwrite(str(base / "depth" / f"{stem}_disparity.png"), disparity_to_color(disp))
        np.save(base / "depth" / f"{stem}_disparity.npy", disp.astype(np.float32))

        # Point cloud in millimetres (same unit as the calibration baseline)
        xyz = cv2.reprojectImageTo3D(disp, self.rig.Q, handleMissingValues=True)
        mask = (disp > 0) & np.isfinite(xyz[..., 2]) & (xyz[..., 2] > 0) & (xyz[..., 2] < self.max_depth_mm)
        write_ply(base / "pointcloud" / f"{stem}.ply", xyz[mask], left[mask])

        meta = {
            "sample_id": sample,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "camera": {
                "cam_id": self.cam_id,
                "image_size": {"width": self.rig.size[0], "height": self.rig.size[1]},
                "exposure": self.cap.get(cv2.CAP_PROP_EXPOSURE),
                "gain": self.cap.get(cv2.CAP_PROP_GAIN),
            },
            "calibration": self.calib_source,
            "baseline_mm": self.rig.baseline_mm,
            "stereo": {"num_disp": self.rig.num_disp, "block_size": self.rig.block_size,
                       "use_wls": self.rig.use_wls},
            "pointcloud_unit": "mm",
            "points_saved": int(mask.sum()),
        }
        with open(base / f"{stem}_metadata.json", "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)

        self.status.set(f"Saved {stem} to {base} ({int(mask.sum())} points).")

    def next_sample(self):
        self.sample_id.set("")
        self.entry.focus_set()
        self.status.set("Enter the next sample ID.")

    def on_disp_click(self, event):
        disp = self.last[2]
        if disp is None:
            return
        x, y = int(event.x / PREVIEW_SCALE), int(event.y / PREVIEW_SCALE)
        if not (0 <= x < disp.shape[1] and 0 <= y < disp.shape[0]):
            return
        d = float(disp[y, x])
        if d <= 0 or not np.isfinite(d):
            self.status.set(f"Pixel ({x}, {y}): no valid disparity.")
            return
        self.status.set(f"Pixel ({x}, {y}): disparity {d:.2f}px, distance about {self.rig.depth_at(d):.3f} m.")

    def on_quit(self):
        self.running = False
        if self.cap is not None:
            self.cap.release()
        self.root.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Stereo camera box capture GUI.")
    ap.add_argument("--camera", type=int, default=0, help="video device index (default: 0)")
    ap.add_argument("--calib", default="stereo_params.json",
                    help="calibration JSON from stereo_calibrate.py (default: stereo_params.json)")
    ap.add_argument("--out", default="data", help="output folder (default: data)")
    ap.add_argument("--num-disp", type=int, default=160, help="SGBM disparity range, multiple of 16")
    ap.add_argument("--max-depth", type=float, default=2000.0,
                    help="drop points farther than this from the camera, in mm (default: 2000)")
    ap.add_argument("--no-wls", action="store_true", help="disable the WLS disparity filter")
    args = ap.parse_args(argv)

    if tk is None:
        sys.exit("Tkinter is not available. On Debian/Ubuntu: sudo apt install python3-tk")
    params, source = load_params(args.calib)
    rig = StereoRig(params, num_disp=args.num_disp, use_wls=not args.no_wls)

    root = tk.Tk()
    StereoApp(root, rig, cam_id=args.camera, out_dir=args.out, calib_source=source,
              max_depth_mm=args.max_depth)
    root.mainloop()


if __name__ == "__main__":
    main()
