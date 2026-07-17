# Camera-Based Yaw-Rate Validation on a Single-Axis Testbed

This repository documents the experimental procedure, camera calibration, AprilTag yaw fusion, encoder reference logging, filters, equations, and reported results for the paper:

**From Timing-Limited to Optics-Limited: Synchronization-Aware Validation of Camera-Based Yaw-Rate Estimation on a Spacecraft Attitude Testbed**

## Important scope

This is the documentation and acquisition-code repository prepared for the conference submission.

- The reported study used **motion-onset synchronization**, not an LED.
- The unavailable exploratory offline-analysis script is not included.
- The exact filtering equations, parameters, synchronization procedure, and manuscript result tables are documented below.
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
- Mono8 acquisition.
- Fixed exposure: 5 ms.
- Gain: 0 dB.
- Seven `tag36h11` markers, 70 mm.
- 12 V metal gearmotor, approximately 130 RPM.
- BTS7960 motor driver.
- Magnetic quadrature encoder: 1496 decoded counts/revolution.
- Encoder logging rate: 100 Hz.
- Arduino controller.
- Independent rate-output IMU at approximately 1 Hz.

Add the exact camera, Arduino, motor, and IMU model numbers before making the repository public if they are available.

# Experimental steps

## 1. Print the checkerboard

Use `targets/checkerboard_A4_20mm_10x7.pdf`.

- Print at 100% scale.
- Do not use “Fit to page”.
- Board: 10 × 7 squares.
- Inner corners used by the code: 9 × 6.
- Square size: 20 mm.
- Verify the printed scale using a ruler.

## 2. Capture calibration images

Capture approximately 40 sharp images using the final lens, focus, resolution, and camera position.

Include:

- different tilts;
- different distances;
- the image center and corners;
- the checkerboard at several positions.

The study used 39 valid images from 40.

## 3. Run calibration

```bash
python3 code/basler_checkerboard_calibration.py
```

Study calibration:

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

Recalibrate after changing the lens, focus, aperture, resolution, or mounting geometry.

## 4. Mount the AprilTags

Use IDs 0–6 from `targets/tag36h11_id0-6_70mm.pdf`.

- One central marker.
- Six peripheral markers.
- Black tag size passed to the pose solver: 70 mm.
- All tags should be visible before motion starts because the first valid observation defines each tag’s zero angle.

## 5. Configure the camera

```text
Pixel format     = Mono8
Exposure         = 5000 µs
Gain             = 0 dB
Auto exposure    = OFF
Auto gain        = OFF
Tag family       = tag36h11
Tag size         = 0.07 m
Outlier threshold= 12°
Online LPF alpha = 0.20 (optional display filter)
```

Run:

```bash
python3 code/basler_apriltag_multitag_fusion.py
```

The online finite-difference rate is a status display only. Final paper rates were calculated offline.

## 6. Record each run

Test:

```text
30, 60, 90, 120, 150, 180, 210, 240, 270, 300 deg/s
```

Repeat each speed three times.

Every run follows:

```text
rest → ramp-up → constant-speed plateau → ramp-down → rest
```

Record synchronized camera, encoder, and IMU logs.

# Equations and processing

## Encoder angle

```math
\theta_{enc}(t)=\frac{360^\circ}{1496}n(t)
```

## Per-tag yaw

```math
\psi_i(t)=\operatorname{atan2}(R^{(i)}_{21}(t),R^{(i)}_{11}(t))
```

The camera yaw sign is reversed for this rig so that its positive direction matches the encoder.

## Relative yaw

```math
\psi_{i,rel}(t)=\psi_i(t)-\psi_i(t_0)
```

Each tag is unwrapped independently across ±180°.

## Outlier rejection

```math
V(t)=\{i:|\psi_i(t)-\operatorname{median}(\psi(t))|\le 12^\circ\}
```

## Decision-margin-weighted fusion

```math
\hat\psi(t)=
\frac{\sum_{i\in V(t)}w_i\psi_i(t)}
{\sum_{i\in V(t)}w_i},
\qquad
w_i=\max(\text{decision margin}_i,1)
```

## Optional online LPF

```math
y_k=0.20x_k+0.80y_{k-1}
```

This filter is optional and intended for online display.

## Offline yaw-rate filter

The final rate estimate used a second-order Savitzky–Golay differentiator with a window of approximately 0.3 s. The same differentiation operator was applied to camera yaw and encoder angle to provide a common effective bandwidth.

## Synchronization

No LED was used.

1. Compute preliminary camera, encoder, and IMU rates.
2. Detect the rest-to-motion onset in each stream.
3. Apply the initial onset time offset.
4. Refine the offset by minimizing the squared error over the common motion profile.

```math
\Delta t^*=
\operatorname*{arg\,min}_{\Delta t}
\sum_k[
\omega_{cam}(t_k+\Delta t)-\omega_{enc}(t_k)
]^2
```

The IMU is aligned to the encoder using the same principle.

## Metric window

RMS, MAE, and bias are calculated over the constant-speed interval after trimming the first and last 1.5 s to avoid acceleration and deceleration transients.

## Metrics

```math
e_{RMS}=\sqrt{\frac{1}{K}\sum_{k=1}^{K}e_k^2}
```

```math
e_{MAE}=\frac{1}{K}\sum_{k=1}^{K}|e_k|
```

```math
b=\frac{1}{K}\sum_{k=1}^{K}e_k
```

Error directions:

```text
Camera–Encoder = Camera − Encoder
Encoder–IMU    = Encoder − IMU
Camera–IMU     = Camera − IMU
```

The tables report mean ± standard deviation over three repeats.

# Results

The complete manuscript tables are in:

- `results/paper_results.csv`
- `results/paper_results.md`

Main interpretation:

- Camera–Encoder disagreement grows with speed.
- Camera–IMU disagreement also grows with speed.
- Encoder–IMU agreement remains approximately flat at about 1.3–2.0 deg/s.
- The camera-side increase is consistent with motion blur during the fixed 5 ms exposure.

# Data and code availability statement for the paper

> The camera-calibration code, AprilTag multi-marker fusion code, encoder logger, experimental targets, calibration parameters, acquisition settings, processing equations, and reported result tables are available in the accompanying GitHub repository. The study used motion-onset synchronization; no LED synchronization was used.

# Files not to publish

Do not upload:

- WhatsApp archives;
- the editable manuscript;
- private conversations;
- `.venv/`;
- `__pycache__/`;
- temporary files;
- personal information.

# Citation

Add the final conference citation and DOI after acceptance.
