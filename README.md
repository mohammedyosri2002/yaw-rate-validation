# Camera-Based Yaw-Rate Validation on a Single-Axis Testbed

This repository documents the experimental procedure, camera calibration, AprilTag yaw fusion, encoder reference logging, filtering, synchronization, equations, and reported results for the paper:

**From Timing-Limited to Optics-Limited: Synchronization-Aware Validation of Camera-Based Yaw-Rate Estimation on a Spacecraft Attitude Testbed**

## Important scope

This repository contains the documentation and acquisition code prepared for the conference submission.

- The reported study used **motion-onset software synchronization**; no LED synchronization was used.
- The original exploratory offline-analysis script is not included in this release.
- The final processing equations, settings, synchronization procedure, and manuscript result tables are documented in this repository.
- All 30 camera logs have `filter_enabled = 1`; therefore, the first-order online IIR low-pass filter with `alpha = 0.20` was enabled during acquisition.
- The reported offline yaw-rate estimation then used a second-order Savitzky-Golay differentiator with a window of approximately `0.3 s`.
- Therefore, the camera yaw passed through two temporal-processing stages: the acquisition-stage IIR filter and the offline Savitzky-Golay differentiator.
- The raw 30-run dataset should be archived separately after submission.

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
```

## Hardware

- Basler monochrome camera with an 8 mm lens.
- Mono8 image acquisition.
- Fixed exposure: `5 ms`.
- Gain: `0 dB`.
- Seven `tag36h11` markers, each with a black-tag size of `70 mm`.
- 12 V metal gearmotor, approximately 130 RPM.
- BTS7960 motor driver.
- Magnetic quadrature encoder: `1496` decoded counts/revolution.
- Encoder logging rate: `100 Hz`.
- Arduino controller.
- Calibrated six-axis IMU integrated into the Theia Space Educational Satellite (ESAT) ADCS board.
- The approximately `1 Hz` IMU update rate used in this experiment was imposed by the telemetry logging path, not by the internal IMU bandwidth.

Add the exact camera, Arduino, motor, and IMU model numbers before making the repository public, when those model numbers are available.

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

The camera, encoder, and IMU operated on independent clocks and were synchronized in software after acquisition:

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

The IMU is aligned to the encoder using the same principle.

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
- The camera-side increase is consistent with motion blur during the fixed `5 ms` exposure.

The camera encoder-independent rate-variation estimate used in the manuscript is a diagnostic based on the difference between lighter and heavier yaw-rate derivative estimates. It should be interpreted as a camera-side rate-variation proxy rather than as a direct measurement of fundamental sensor noise.

# Data and code availability statement

> The camera-calibration code, AprilTag multi-marker fusion code, encoder logger, experimental targets, calibration parameters, acquisition settings, processing equations, synchronization method, and reported result tables are available in this repository. The study used motion-onset software synchronization; no LED synchronization was used.

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

Add the final conference citation and DOI after acceptance.
