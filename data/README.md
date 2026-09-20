# Raw 30-run dataset

`raw_runs/` contains the 30 valid runs used for the camera-ready re-analysis: ten commanded setpoints from 30 to 300 deg/s, with three repeats per setpoint.

Each run folder contains:

- `apriltag_multitag_fused_*.xlsx` — camera/AprilTag log, including host `wall_time_ms`, fused yaw, per-tag yaw and decision margin, filter flag, and frame brightness;
- `encoderdata.txt` — encoder microsecond timestamp and decoded count (plus acquisition/controller fields);
- `imu*.txt` — Theia Space ESAT ADCS gyroscope-derived rotational-speed telemetry log.

The archived `first_test_30deg-s` is a replacement re-run. An earlier 30 deg/s recording became merged/corrupted during acquisition and is not present in this release and was not used in the analysis.

The folder names preserve the original acquisition spelling (`seconed_test_*`) so that the public archive matches the source dataset exactly.
