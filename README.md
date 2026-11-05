# Stereo-Vision-Camera-Box

English | [中文](README.zh-CN.md)

[![CI](https://github.com/smiler488/Stereo-Vision-Camera-Box/actions/workflows/test.yml/badge.svg)](https://github.com/smiler488/Stereo-Vision-Camera-Box/actions/workflows/test.yml) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE) ![Python 3.9+](https://img.shields.io/badge/Python_3.9%2B-informational)
A closed, evenly lit box with a USB stereo camera in the lid, plus a small Python GUI that captures rectified image pairs, disparity maps and coloured point clouds sample by sample.

I built it to image detached cotton leaves and other small organs indoors. In the field or under room light the stereo matching kept failing on shiny or shaded leaf surfaces. Inside the box the light is the same for every sample, so the disparity maps are much cleaner.

![box outside](https://github.com/user-attachments/assets/ba0acbbc-21d0-40e3-bfcb-cd144da1db0b)
![box inside](https://github.com/user-attachments/assets/c44c0a7f-2479-4f5a-84cb-f4ad422afb9f)
![capture](https://github.com/user-attachments/assets/9b747aff-a842-46be-9373-a77018882c85)

## Hardware

- **Camera**: USB stereo module that outputs both views side by side (1280 × 480, i.e. 640 × 480 per view). Lenses are M12 3.0 mm, F2.4, about 65° horizontal FOV, on 1/4" OV9732 sensors. The lens drawing is in [`hardware/lens_M12_3mm_OV9732_spec.pdf`](hardware/lens_M12_3mm_OV9732_spec.pdf).
- **Box**: camera fixed at the centre of the lid, LED strip around it, reflective film on the side walls, white diffuse sheet on the floor to soften shadows.
- **Calibration board**: [`calibration/checkerboard_9x6_15mm_calib.io.pdf`](calibration/checkerboard_9x6_15mm_calib.io.pdf), 9 × 6 squares of 15 mm (8 × 5 inner corners). Print it at 100 % and measure a square before calibrating. More boards are on my [target generator page](https://smiler488.github.io/app/targets/).

## Installation

```bash
git clone https://github.com/smiler488/Stereo-Vision-Camera-Box.git
cd Stereo-Vision-Camera-Box
pip install -r requirements.txt
```

Install `opencv-contrib-python` only, not together with `opencv-python`. The two packages overwrite each other. Tkinter comes with Python; on Debian/Ubuntu install `python3-tk` if it is missing.

## 1. Calibrate

Take 15 to 25 pairs of the board at different positions and tilts inside the box, saved as `left/leftN.png` and `right/rightN.png`. Then:

```bash
python stereo_calibrate.py --left "calibration/images/left/*.png" \
    --right "calibration/images/right/*.png" --pattern 8 5 --square 15
```

This writes `stereo_params.json` and prints the RMS reprojection error and the baseline. Below 0.3 px is good. The baseline scales linearly with `--square`, so if it does not match your camera module, the printed square size is the first thing to check.

The 20 pairs in `calibration/images/` are from my box. They give 0.14 px RMS. The result is in `calibration/stereo_params_from_sample_images.json` for reference.

## 2. Capture

```bash
python stereo_capture_box_gui.py --camera 0
```

1. Type a sample ID and press **Start preview**.
2. Press **Save** (or `s`) to store the current frame.
3. Click on the disparity view to read the distance of that pixel in the status bar.
4. **Next sample** (or `n`) clears the ID for the next one.

Options: `--calib` (calibration file, default `stereo_params.json`), `--out` (output folder), `--num-disp` (SGBM disparity range), `--max-depth` (crop the point cloud, mm), `--no-wls`.

If `stereo_params.json` is missing, the GUI falls back to the parameters of my own unit that are hard-coded in the script. Those will not fit another camera.

## Output

```
data/<sample_id>/
├── left/        <id>_<time>_left.png         rectified left view
├── right/       <id>_<time>_right.png        rectified right view
├── depth/       <id>_<time>_disparity.png    colour disparity (1-99 % stretch)
│                <id>_<time>_disparity.npy    float32 disparity in pixels
├── pointcloud/  <id>_<time>.ply              xyz in mm + RGB from the left view
└── <id>_<time>_metadata.json                 camera, calibration and SGBM settings
```

Depth along the optical axis is `Z = f · B / d`, with `f` the rectified focal length in pixels, `B` the baseline and `d` the disparity.

## Changes in this revision

- The GUI updated Tk widgets from a worker thread, which is not allowed in Tk and crashed on macOS. Frames are now pulled on the Tk main loop.
- The WLS filter was given disparities already divided by 16. It now receives the raw fixed-point maps.
- Point clouds are written with NumPy instead of a Python loop (seconds instead of minutes for 300k points), and limited to a depth range.
- Added `stereo_calibrate.py`. Before, the README asked for calibration but the repository had no script for it.
- Calibration is read from JSON instead of being edited in the source.
- Files sorted into `calibration/` and `hardware/`, the lens drawing renamed, install instructions corrected (`tkinter` is not a pip package, the two OpenCV wheels conflict).

## License

MIT, see [LICENSE](LICENSE). Liangchao Deng (邓良超).
