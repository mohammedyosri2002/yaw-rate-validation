# Camera-Based Yaw-Rate Validation on a Single-Axis Testbed

This repository documents the experimental procedure, camera calibration, AprilTag yaw fusion, encoder reference logging, filtering, synchronization, equations, and reported results for the paper:

**From Timing-Limited to Optics-Limited: Synchronization-Aware Validation of Camera-Based Yaw-Rate Estimation on a Spacecraft Attitude Testbed**

## Important scope

This camera-ready release contains the acquisition code, calibration assets, the **30 archived experimental runs**, and the offline re-analysis used to answer the software-synchronization circularity question raised during review.

- Ten setpoints (`30` to `300 deg/s`) were tested, with three valid runs per setpoint: **30 runs total**.
- The reported study used software synchronization; no hardware trigger or LED synchronization was used.
- All 30 camera logs have `filter_enabled = 1`, so the acquisition-stage first-order IIR filter (`alpha = 0.20`) was active.
- The accepted paper's absolute result tables remain the headline results. The new `analysis/reanalysis.py` implements a **paired sensitivity re-analysis**: the same reconstructed chain is applied to all alignment strategies, and only within-run differences are used to quantify sensitivity to the alignment procedure.
- The reconstructed chain uses a 100 Hz common grid and a second-order Savitzky-Golay derivative with a 31-sample (0.31 s) window, consistent with the paper's documented "approximately 0.3 s" setting. Some exact historical offline implementation choices were not archived, so the rebuilt absolute RMS values are **not used to replace the accepted tables**.
- ESAT ADCS timing is treated as independent/uncertain. The raw ESAT timestamps resemble epoch time, but the archive does not establish where they are generated. No common-clock claim is made.

## Repository structure

```text
code/
  basler_checkerboard_calibration.py
  basler_apriltag_multitag_fusion.py
  encoder_reference_logger.ino

calibration/
  camera_calibration_results.npz
  intrinsics.md
  sample_images/

targets/
  checkerboard_A4_20mm_10x7.pdf
  tag36h11_id0-6_70mm.pdf

docs/figures/
  testbed_overview.jpg
  tagged_platform.jpg
  basler_camera_view.png

results/
  paper_results.csv
  paper_results.md
  reanalysis/
    table_A_B_per_run.csv
    per_run_derived.csv
    sweeps.json
    imu_lag.csv
    imu_aligned.csv

analysis/
  reanalysis.py
  imu_analysis.py
  run_all.py
  README.md

data/raw_runs/
  first_test_30deg-s/ ... third_test_300deg-s/
```

## Hardware

- Basler ace acA1920-40um monochrome camera with an 8 mm lens.
- Mono8 image acquisition.
- Fixed exposure: `5 ms`.
- Gain: `0 dB`.
- Seven `tag36h11` markers, each with a black-tag size of `70 mm`.
- 12 V metal gearmotor, approximately 130 RPM.
- BTS7960 motor driver.
- Magnetic quadrature encoder: `1496` decoded counts/revolution.
- Encoder logging rate: `100 Hz`.
- Arduino controller.
- Theia Space ESAT ADCS gyroscope-derived rotational-speed output as the supporting inertial reference. The ADCS integrates a six-axis accelerometer/gyroscope IMU plus a three-axis magnetometer; the manufacturer specifies gyroscope rotational-speed accuracy of `1 deg/s`.
- The approximately `1 Hz` value in this dataset is the ESAT telemetry logging rate used in the experiment, not the gyroscope bandwidth.

# Experimental procedure

## 1. Print the checkerboard

Use:

```text
targets/checkerboard_A4_20mm_10x7.pdf
```

Printing requirements:

- Print at 100% scale.
- Do not use “Fit to page”.
- Board size: `10 × 7` squares.
- Inner corners used by the code: `9 × 6`.
- Square size: `20 mm`.
- Verify the printed scale using a ruler.

## 2. Capture calibration images

Capture approximately 40 sharp images using the final lens, focus, resolution, and camera position.

Include:

- different tilts;
- different distances;
- the image center and corners;
- the checkerboard at several positions.

The reported study used 39 valid calibration images from 40 captures.

## 3. Run camera calibration

```bash
python3 code/basler_checkerboard_calibration.py
```

Reported calibration values:

```text
fx = 1443.2497 px
fy = 1443.7377 px
cx = 963.2151 px
cy = 580.3333 px

k1 = -0.1575447
k2 =  0.1619021
p1 = -0.0004467
p2 = -0.0002209
k3 = -0.1184079

RMS reprojection error ≈ 0.16 px
```

Recalibrate after changing the lens, focus, aperture, image resolution, or mounting geometry.

## 4. Mount the AprilTags

Use IDs 0–6 from:

```text
targets/tag36h11_id0-6_70mm.pdf
```

Configuration:

- One central marker.
- Six peripheral markers.
- Black-tag size passed to the pose solver: `70 mm`.
- All tags should be visible before motion starts because each tag's first valid observation defines its zero angle.

## 5. Configure the camera and AprilTag acquisition

```text
Pixel format       = Mono8
Exposure           = 5000 µs
Gain               = 0 dB
Auto exposure      = OFF
Auto gain          = OFF
Tag family         = tag36h11
Tag size           = 0.07 m
Outlier threshold  = 12°
Online IIR alpha   = 0.20
```

Run:

```bash
python3 code/basler_apriltag_multitag_fusion.py
```

The software allows the online IIR filter to be disabled, but it was enabled during all 30 acquisitions reported in the paper. The online finite-difference rate is a status display only. The final paper rates were calculated offline using the Savitzky-Golay differentiator described below.

## 6. Record each run

Test setpoints:

```text
30, 60, 90, 120, 150, 180, 210, 240, 270, 300 deg/s
```

Repeat each speed three times.

Each run follows:

```text
rest → ramp-up → constant-speed plateau → ramp-down → rest
```

Record time-stamped camera, encoder, and IMU logs for subsequent software synchronization.

# Equations and processing

## Encoder angle

```math
\theta_{\mathrm{enc}}(t)=\frac{360^\circ}{1496}n(t)
```

where `n(t)` is the decoded encoder count.

## Per-tag yaw

```math
\psi_i(t)=\operatorname{atan2}\!\left(R^{(i)}_{21}(t),R^{(i)}_{11}(t)\right)
```

The camera yaw sign is reversed for this rig so that its positive direction matches the encoder.

## Relative yaw

```math
\psi_{i,\mathrm{rel}}(t)=\psi_i(t)-\psi_i(t_0)
```

Each tag is unwrapped independently across ±180°.

## Outlier rejection

```math
V(t)=\left\{i:\left|\psi_i(t)-\operatorname{median}(\psi(t))\right|\le 12^\circ\right\}
```

## Decision-margin-weighted fusion

```math
\hat{\psi}(t)=
\frac{\sum_{i\in V(t)}w_i\psi_i(t)}
{\sum_{i\in V(t)}w_i},
\qquad
w_i=\max(\text{decision margin}_i,1)
```

Here, `w_i` is the AprilTag decision-margin weight. It is intentionally written as `w_i`, rather than `ω_i`, to avoid confusion with angular velocity.

## Stage 1 — Online first-order IIR low-pass filter

During all 30 reported acquisitions, the saved fused camera yaw was filtered using:

```math
y_k=0.20x_k+0.80y_{k-1}
```

where:

- `x_k` is the current fused yaw;
- `y_k` is the saved filtered yaw;
- `y_{k-1}` is the previous filtered yaw.

## Stage 2 — Offline Savitzky-Golay differentiator

The final rate estimate used a second-order Savitzky-Golay differentiator with a window of approximately `0.3 s`.

The same Savitzky-Golay differentiation operator was applied to:

- the filtered camera yaw; and
- the encoder angle.

The camera yaw had already passed through the online IIR filter during acquisition.

Median tag rejection is a robust spatial-fusion rule, not a temporal filter.

## Synchronization

No LED was used.

The camera and encoder were synchronized in software after acquisition. The ESAT rate telemetry was aligned separately to the encoder; its timestamp origin is not established by the archive:

1. Compute preliminary camera, encoder, and IMU rate signals.
2. Detect the rest-to-motion onset in each stream.
3. Apply an initial onset-based time offset.
4. Refine the offset by minimizing squared disagreement over the common motion profile.

```math
\Delta t^*=
\operatorname*{arg\,min}_{\Delta t}
\sum_k
\left[
\omega_{\mathrm{cam}}(t_k+\Delta t)
-
\omega_{\mathrm{enc}}(t_k)
\right]^2
```

where:

- `Δt*` is the estimated camera-to-encoder time offset;
- `Δt` is a trial time offset;
- `ω_cam` is the camera-derived yaw rate;
- `ω_enc` is the encoder-derived yaw rate;
- `t_k` is the `k`th encoder timestamp.

The ESAT rotational-speed telemetry is aligned to the encoder using the same onset-seed plus least-squares principle. The sparse ESAT stream is never upsampled in `analysis/imu_analysis.py`; camera and encoder rates are evaluated only at actual ESAT sample instants.

The synchronization window and evaluation window are different:

- synchronization uses the common motion profile;
- RMS, MAE, and bias use only the trimmed constant-speed plateau.

## Metric window

RMS, MAE, and bias are calculated over the constant-speed interval after trimming the first and last `1.5 s` to exclude acceleration and deceleration transients.

## Metrics

```math
e_{\mathrm{RMS}}=
\sqrt{\frac{1}{K}\sum_{k=1}^{K}e_k^2}
```

```math
e_{\mathrm{MAE}}=
\frac{1}{K}\sum_{k=1}^{K}|e_k|
```

```math
b=
\frac{1}{K}\sum_{k=1}^{K}e_k
```

where `K` is the number of samples in the selected evaluation window.

Error directions:

```text
Camera–Encoder = Camera − Encoder
Encoder–IMU    = Encoder − IMU
Camera–IMU     = Camera − IMU
```

The tables report mean ± standard deviation over three repeats.

# Results

The complete manuscript result tables are available in:

- `results/paper_results.csv` — machine-readable RMS, MAE, and bias values;
- `results/paper_results.md` — human-readable formatted tables, including the Camera–Encoder correlations.

Main interpretation:

- Camera–Encoder disagreement grows with speed.
- Camera–IMU disagreement also grows with speed.
- Encoder–IMU agreement remains approximately flat, ranging from about `1.3` to `2.0 deg/s`, with no systematic dependence on speed.
- Camera–Encoder correlation is `0.99` or higher across all tested setpoints.
- Camera–Encoder bias magnitude remains below `0.6 deg/s`.
- The camera-side increase is consistent with camera-side acquisition effects. With fixed `5 ms` exposure and host-side camera timestamps, the present dataset does not separate motion blur from residual frame-timestamp effects.

The camera encoder-independent rate-variation estimate used in the manuscript is a diagnostic based on the difference between lighter and heavier yaw-rate derivative estimates. It should be interpreted as a camera-side rate-variation proxy rather than as a direct measurement of fundamental sensor noise.

# Reproducing the camera-ready re-analysis

Create a Python environment and install dependencies:

```bash
pip install -r requirements.txt
```

Then run:

```bash
python analysis/run_all.py
```

The scripts read the 30 folders under `data/raw_runs/` and rewrite the files under `results/reanalysis/`. The key camera-encoder sensitivity outputs are `table_A_B_per_run.csv`, `per_run_derived.csv`, and `sweeps.json`; the supporting ESAT checks are `imu_lag.csv` and `imu_aligned.csv`.

The camera-ready alignment result is a paired comparison, not a replacement for the accepted absolute result tables. Holding the constant-speed plateau out of the offset fit changes plateau RMS by about `+0.005 deg/s` on average and by at most about `0.031 deg/s` in any run; the largest relative change is about `3.07%` and occurs in a different low-rate run.

# Data and code availability statement

> The camera-calibration script, AprilTag multi-marker fusion and acquisition script, encoder logger firmware, printable calibration and tag targets, recovered intrinsics, raw per-run camera, encoder and ESAT logs for all thirty runs, offline alignment-sensitivity analysis, and result tables are available in this repository.

# Files not to publish

Do not upload:

- WhatsApp archives;
- editable manuscript files;
- private conversations;
- `.venv/`;
- `__pycache__/`;
- temporary files;
- personal information.

# Citation

This repository accompanies ASET/STAE 2026 paper #416. Add the final proceedings citation and DOI when assigned.
