# Offline camera-ready re-analysis

The re-analysis was added after review to test whether the camera-to-encoder offset refinement artificially improves the steady-state camera accuracy.

## Why paired differences are reported

The accepted absolute result tables remain unchanged. Some exact choices from the historical offline pipeline were not archived (for example the exact resampling and segmentation implementation), so this directory uses one reconstructed processing chain for every alternative alignment and reports **paired within-run changes**. This makes the sensitivity comparison meaningful without replacing the accepted absolute RMS values.

## Scripts

- `reanalysis.py` — camera/encoder paired analysis: refined vs onset-only vs transition-only/held-out-plateau alignment; offset sensitivity sweeps; host-side frame-interval statistics; alignment-independent total-angle check.
- `imu_analysis.py` — supporting ESAT ADCS comparison. ESAT timing is treated as independent/uncertain; the ESAT stream is aligned to the encoder, and the sparse ~1 Hz samples are never upsampled.
- `run_all.py` — runs both scripts with repository-relative paths.
- `verify_camera_ready_numbers.py` — asserts the 30-run count and the headline paired-analysis numbers used in the camera-ready manuscript.

## Reconstructed processing constants

- encoder: raw count × 360/1496;
- common grid: 100 Hz;
- Savitzky-Golay derivative: 31 samples (0.31 s), order 2;
- plateau: longest encoder-rate region above 0.85 × its 90th-percentile level;
- evaluation window: plateau with 1.5 s trimmed at each end;
- onset threshold: 0.20 × plateau level;
- offset refinement search: ±0.5 s, 1 ms step;
- offset-sensitivity sweep: ±0.30 s, 5 ms step.

The 31-sample/100-Hz differentiator is consistent with the manuscript's documented second-order Savitzky-Golay window of approximately 0.3 s, but should not be interpreted as proof that this exact sample count was used in the historical offline script.

## Sign normalization

For the same commanded rotation in the archive, camera fused yaw is negative-going, encoder count is positive-going, and ESAT rate telemetry is negative. The public scripts normalize:

```text
camera = -fused_yaw_deg
encoder = +count * 360/1496
ESAT rate = -ADCS_ROTATIONAL_SPEED
```

This convention reproduces the sign of the accepted Camera-Encoder bias.

## Main reviewer-facing result

Across all 30 runs, estimating the camera/encoder alignment from transitions only, with the constant-speed plateau completely held out from the fit, changes plateau RMS by about +0.005 deg/s on average and by at most about 0.031 deg/s in any run. The largest relative RMS change is about 3.07% and occurs in a different 30 deg/s run. Onset-only alignment changes RMS by about -0.018 deg/s on average; the full refinement lowers plateau RMS in 9/30 runs and raises it in 21/30.

See `../results/reanalysis/` for the machine-readable outputs.
