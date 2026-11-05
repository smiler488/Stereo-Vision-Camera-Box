# Stereo-Vision-Camera-Box

[English](README.md) | 中文

[![CI](https://github.com/smiler488/Stereo-Vision-Camera-Box/actions/workflows/test.yml/badge.svg)](https://github.com/smiler488/Stereo-Vision-Camera-Box/actions/workflows/test.yml) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE) ![Python 3.9+](https://img.shields.io/badge/Python_3.9%2B-informational)
一个光照均匀的封闭箱体，盖子上装 USB 双目相机，配一个 Python 图形界面，按样品逐个采集校正后的左右图像、视差图和彩色点云。

我做这个箱子是为了在室内拍离体的棉花叶片等小器官。在田间或室内自然光下，叶片表面反光、有阴影，立体匹配经常失败。放进箱子以后，每个样品的光照条件一致，视差图干净了很多。

![箱体外观](https://github.com/user-attachments/assets/ba0acbbc-21d0-40e3-bfcb-cd144da1db0b)
![箱体内部](https://github.com/user-attachments/assets/c44c0a7f-2479-4f5a-84cb-f4ad422afb9f)
![采集界面](https://github.com/user-attachments/assets/9b747aff-a842-46be-9373-a77018882c85)

## 硬件

- **相机**：左右画面拼接输出的 USB 双目模组，分辨率 1280 × 480，即单目 640 × 480。镜头为 M12 接口、3.0 mm、F2.4，水平视场角约 65°，传感器为 1/4" OV9732。镜头图纸见 [`hardware/lens_M12_3mm_OV9732_spec.pdf`](hardware/lens_M12_3mm_OV9732_spec.pdf)。
- **箱体**：相机固定在盖子中央，周围一圈 LED 灯带，侧壁贴反光膜，底部铺白色漫反射板，用来减弱阴影。
- **标定板**：[`calibration/checkerboard_9x6_15mm_calib.io.pdf`](calibration/checkerboard_9x6_15mm_calib.io.pdf)，9 × 6 个方格，边长 15 mm（内角点 8 × 5）。打印时选 100% 比例，标定前用尺子量一下方格的实际边长。更多标定板可以在我的[标定板生成页面](https://smiler488.github.io/app/targets/)生成。

## 安装

```bash
git clone https://github.com/smiler488/Stereo-Vision-Camera-Box.git
cd Stereo-Vision-Camera-Box
pip install -r requirements.txt
```

只装 `opencv-contrib-python`，不要和 `opencv-python` 一起装，两个包会互相覆盖。Tkinter 是 Python 自带的；Debian/Ubuntu 上如果缺失，装一下 `python3-tk`。

## 1. 标定

在箱内以不同位置和倾角拍 15–25 对标定板图像，保存为 `left/leftN.png` 和 `right/rightN.png`，然后运行：

```bash
python stereo_calibrate.py --left "calibration/images/left/*.png" \
    --right "calibration/images/right/*.png" --pattern 8 5 --square 15
```

脚本会生成 `stereo_params.json`，并打印 RMS 重投影误差和基线长度，误差低于 0.3 px 就算好。基线长度和 `--square` 成正比，如果算出来和相机模组的标称值对不上，先检查打印出的方格尺寸。

`calibration/images/` 里的 20 对图来自我自己的箱子，RMS 为 0.14 px，标定结果放在 `calibration/stereo_params_from_sample_images.json`，供参考。

## 2. 采集

```bash
python stereo_capture_box_gui.py --camera 0
```

1. 输入样品编号，点 **Start preview**。
2. 点 **Save**（或按 `s`）保存当前帧。
3. 在视差图上点击，状态栏会显示该像素到相机的距离。
4. 点 **Next sample**（或按 `n`）清空编号，准备拍下一个样品。

可选参数：`--calib`（标定文件，默认 `stereo_params.json`）、`--out`（输出目录）、`--num-disp`（SGBM 视差范围）、`--max-depth`（点云最大深度，单位 mm）、`--no-wls`。

找不到 `stereo_params.json` 时，程序会改用脚本里写死的参数，那是我自己那台相机的，换一台相机就不适用了。

## 输出

```
data/<样品编号>/
├── left/        <编号>_<时间>_left.png         校正后左图
├── right/       <编号>_<时间>_right.png        校正后右图
├── depth/       <编号>_<时间>_disparity.png    彩色视差图（1–99% 拉伸）
│                <编号>_<时间>_disparity.npy    float32 视差，单位像素
├── pointcloud/  <编号>_<时间>.ply              xyz（mm）+ 左图颜色
└── <编号>_<时间>_metadata.json                 相机、标定和 SGBM 参数
```

沿光轴方向的深度 `Z = f · B / d`，其中 `f` 是校正后的焦距（像素），`B` 是基线，`d` 是视差。

## 本次修订

- 原来在子线程里直接刷新 Tk 控件，Tk 不允许这样做，在 macOS 上会崩溃。现在改为在 Tk 主循环里取帧。
- WLS 滤波原来接收的是已经除以 16 的视差，现在改为传入原始定点视差。
- 点云改用 NumPy 写出，30 万个点从几分钟缩短到几秒，并且可以按深度裁剪。
- 新增 `stereo_calibrate.py`。原 README 要求先标定，但仓库里一直没有标定脚本。
- 标定参数改为从 JSON 读取，不用再去改源码。
- 文件整理到 `calibration/` 和 `hardware/` 两个目录，镜头图纸重新命名；修正了安装说明（`tkinter` 不能用 pip 安装，两个 OpenCV 包会冲突）。

## 许可证

MIT，见 [LICENSE](LICENSE)。邓良超（Liangchao Deng）。
