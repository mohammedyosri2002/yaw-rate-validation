#!/usr/bin/env python3
"""Supporting ESAT ADCS gyroscope checks for the 30-run dataset.

The ESAT rate is the gyroscope-derived rotational-speed telemetry from the
Theia Space ESAT ADCS subsystem, logged at about 1 Hz.  The archive does not
establish where the ESAT timestamp is generated.  Camera and ESAT timing are
therefore treated as independent/uncertain; no common-clock claim is made.

The ESAT stream is aligned to the encoder by onset seed plus least-squares
refinement.  The sparse ESAT stream is never upsampled: camera and encoder
rates are evaluated only at actual ESAT sample instants.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
from pathlib import Path

import numpy as np
import pandas as pd
import reanalysis as R


def discover_runs(root: Path):
    runs = [p for p in sorted(glob.glob(str(root / "*_test_*deg-s"))) if os.path.isdir(p)]
    if len(runs) != 30:
        raise RuntimeError(f"Expected 30 run folders under {root}, found {len(runs)}")
    return runs


def prep(folder):
    tc, yc, _ = R.load_camera(folder)
    te, ye = R.load_encoder(folder)
    ti, wi = R.load_imu(folder)
    gc, yy = R.to_grid(tc - tc[0], yc); wc = R.sgdiff(yy)
    ge, ee = R.to_grid(te - te[0], ye); we = R.sgdiff(ee)
    seg = R.segment(we, ge)
    lvl = np.percentile(wc[np.isfinite(wc)], 90)
    seed = R.onset_time(np.nan_to_num(wc), gc, lvl) - seg["t_on"]
    ok = np.isfinite(we)
    idxf = np.arange(seg["i_on"], seg["i_off"]); idxf = idxf[ok[idxf]]
    dt_cam = R.refine(seed, ge, we, gc, wc, idxf)
    return dict(tc0=tc[0], gc=gc, wc=wc, ge=ge, we=we, seg=seg, ti=ti, wi=wi, dt_cam=dt_cam, lvl=lvl)


def measure_lag(run_dirs):
    """Raw timestamp/onset comparison. Interpretation is intentionally limited."""
    out = []
    for f in run_dirs:
        sp = int(re.search(r"_(\d+)deg-s", f).group(1))
        p = prep(f)
        m = np.nan_to_num(p["wc"]) > 0.2 * p["lvl"]
        c_on = p["gc"][np.argmax(m)]
        c_off = p["gc"][len(m) - np.argmax(m[::-1]) - 1]
        tr = p["ti"] - p["tc0"]
        ilv = np.percentile(np.abs(p["wi"]), 90)
        im = np.abs(p["wi"]) > 0.2 * max(ilv, 1)
        i_on = tr[np.argmax(im)]
        i_off = tr[len(im) - np.argmax(im[::-1]) - 1]
        out.append(dict(run=os.path.basename(f), sp=sp, cam_on=c_on, imu_on=i_on, lag_on=i_on-c_on,
                        cam_off=c_off, imu_off=i_off, lag_off=i_off-c_off,
                        imu_plateau_mean=float(p["wi"][im].mean()), imu_plateau_sd=float(p["wi"][im].std()),
                        imu_n=int(im.sum())))
    return pd.DataFrame(out)


def imu_metrics(run_dirs):
    out = []
    for f in run_dirs:
        sp = int(re.search(r"_(\d+)deg-s", f).group(1))
        p = prep(f)
        # Express the ESAT timestamp stream relative to the camera-log epoch, then
        # fit an independent ESAT-to-encoder shift. This does NOT assume a common clock.
        tr = p["ti"] - p["tc0"] - p["dt_cam"]
        ilv = np.percentile(np.abs(p["wi"]), 90)
        im = np.abs(p["wi"]) > 0.2 * max(ilv, 1)
        seed = p["seg"]["t_on"] - tr[np.argmax(im)]
        best = (np.inf, seed)
        for g in np.arange(seed - 2.0, seed + 2.0, 0.01):
            v = np.interp(tr + g, p["ge"], p["we"], left=np.nan, right=np.nan)
            d = v - p["wi"]
            d = d[np.isfinite(d)]
            if len(d) > 5 and np.sum(d**2) < best[0]:
                best = (float(np.sum(d**2)), float(g))
        dt_imu = best[1]
        tI = tr + dt_imu
        sel = (tI >= p["ge"][p["seg"]["e0"]]) & (tI <= p["ge"][p["seg"]["e1"]])
        enc_at = np.interp(tI[sel], p["ge"], p["we"], left=np.nan, right=np.nan)
        cam_at = np.interp(tI[sel] + p["dt_cam"], p["gc"], p["wc"], left=np.nan, right=np.nan)
        m = np.isfinite(enc_at) & np.isfinite(cam_at)
        dei = enc_at[m] - p["wi"][sel][m]
        dci = cam_at[m] - p["wi"][sel][m]
        out.append(dict(run=os.path.basename(f), sp=sp, n=int(m.sum()),
                        ei_rms=float(np.sqrt(np.mean(dei**2))), ei_mae=float(np.mean(np.abs(dei))), ei_bias=float(np.mean(dei)),
                        ci_rms=float(np.sqrt(np.mean(dci**2))), ci_mae=float(np.mean(np.abs(dci))), ci_bias=float(np.mean(dci))))
    return pd.DataFrame(out)


def main():
    here = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", type=Path, default=here / "data" / "raw_runs")
    ap.add_argument("--output-dir", type=Path, default=here / "results" / "reanalysis")
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_dirs = discover_runs(args.data_root)
    lag = measure_lag(run_dirs)
    met = imu_metrics(run_dirs)
    lag.to_csv(args.output_dir / "imu_lag.csv", index=False)
    met.to_csv(args.output_dir / "imu_aligned.csv", index=False)
    print(f"runs analysed: {len(run_dirs)}")
    print(f"mean onset timestamp difference (raw streams): {lag['lag_on'].mean():.3f} s")
    print(f"encoder-ESAT RMS range (run level): {met['ei_rms'].min():.3f} to {met['ei_rms'].max():.3f} deg/s")


if __name__ == "__main__":
    main()
