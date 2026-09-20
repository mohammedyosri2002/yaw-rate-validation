# Re-analysis result summary

This directory contains machine-readable outputs from the camera-ready paired re-analysis.

## Alignment sensitivity (30 runs)

- Transition-only fit, plateau held out: mean ΔRMS = **+0.005 deg/s**.
- Maximum absolute |ΔRMS| = **0.031 deg/s**.
- Maximum relative |ΔRMS|/RMS_ref = **3.07%**; this occurs in a different run from the maximum absolute change.
- Onset-only alignment: mean ΔRMS = **-0.018 deg/s**.
- Full refinement lowers plateau RMS in **9/30** runs and raises it in **21/30** relative to onset-only.
- Maximum refined-vs-onset offset difference = **83 ms**.
- Maximum refined-vs-transition-only offset difference = **20 ms**.

The accepted paper's absolute result tables are not replaced by these rebuilt absolute values. The reviewer-facing claim is based on the paired changes computed with one identical reconstructed chain.

## Supporting checks

- Camera/encoder total swept-angle mean absolute difference = **0.055%**; maximum absolute difference = **0.145%**.
- Largest measured angular span in the archive is about **22.4 revolutions**.
- Mean of the 30 run-wise median camera frame intervals = **29.87 ms** (~33.5 fps).
- Mean run-wise frame-interval SD = **4.17 ms**; mean SD after excluding >1.5×-median gaps = **2.23 ms**.
- Long intervals: **237** of **39083** (0.61%), with ~**337** skipped-frame equivalents.
- Supporting ESAT rebuild: encoder–ESAT RMS slope vs commanded rate ≈ **0.0020**, camera–ESAT RMS slope ≈ **0.0197** (deg/s error per deg/s setpoint).
- Raw ESAT/camera onset timestamp difference averages **6.17 s**, but the archive does not establish whether this is telemetry latency, a distinct timestamp origin, batching, or a combination. No common-clock interpretation is made.

## Files

- `table_A_B_per_run.csv`: compact 30-row refined/onset/transition comparison.
- `per_run_derived.csv`: detailed camera/encoder derived quantities and frame-timing statistics.
- `sweeps.json`: plateau-RMS-versus-offset sweep for every run.
- `imu_lag.csv`: raw camera/ESAT onset/offset timing observations with limited interpretation.
- `imu_aligned.csv`: ESAT-aligned supporting encoder–ESAT and camera–ESAT plateau metrics.
