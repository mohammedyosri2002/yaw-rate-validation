#!/usr/bin/env python3
"""Offline paired re-analysis of the 30-run yaw-rate dataset.

This script quantifies how the plateau-error metrics change when the
camera-to-encoder time offset is estimated in three different ways:

  refined     onset seed + least-squares refinement over the full motion profile
  onset       onset seed only, with no least-squares refinement
  transition  onset seed + least-squares refinement using rising/falling edges only
              (the constant-speed plateau is completely held out of the fit)

Important scope: this is an Option-1 paired sensitivity analysis.  It uses one
reconstructed processing chain for every condition and reports within-run
DIFFERENCES between alignment strategies.  The absolute RMS values produced by
this rebuilt chain are not intended to replace the accepted paper's absolute
Tables IV/V, because some exact historical offline-processing choices were not
archived (for example the exact resampling/segmentation implementation).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_coeffs

COUNTS_PER_REV = 1496.0
FS = 100.0
SG_WIN = 31
SG_POLY = 2
TRIM_S = 1.5
PLATEAU_FRAC = 0.85
ONSET_FRAC = 0.20
SEARCH_S = 0.50
SEARCH_STEP = 0.001
SWEEP_S = 0.30
SWEEP_STEP = 0.005

# Sign convention that reproduces the accepted paper's Camera-Encoder bias sign.
S_CAM, S_ENC, S_IMU = -1.0, +1.0, -1.0

_sg = savgol_coeffs(SG_WIN, SG_POLY, deriv=1, delta=1.0 / FS, use="dot")
SG_GAIN = float(np.sqrt(np.sum(_sg**2)))


def sgdiff(y: np.ndarray) -> np.ndarray:
    m = SG_WIN // 2
    out = np.full(len(y), np.nan)
    for i in range(m, len(y) - m):
        out[i] = np.dot(_sg, y[i - m : i + m + 1])
    return out


def to_grid(t: np.ndarray, y: np.ndarray, fs: float = FS):
    g = np.arange(t[0], t[-1], 1.0 / fs)
    return g, np.interp(g, t, y)


def load_camera(folder: str):
    files = glob.glob(os.path.join(folder, "apriltag_multitag_fused_*.xlsx"))
    if len(files) != 1:
        raise RuntimeError(f"Expected one camera xlsx in {folder}, found {len(files)}")
    f = files[0]
    d = pd.read_excel(f).dropna(subset=["fused_yaw_deg"])
    t = d["wall_time_ms"].to_numpy(float) / 1000.0
    y = S_CAM * d["fused_yaw_deg"].to_numpy(float)
    o = np.argsort(t)
    return t[o], y[o], os.path.basename(f)


def load_encoder(folder: str):
    f = os.path.join(folder, "encoderdata.txt")
    d = pd.read_csv(
        f,
        comment="#",
        header=None,
        on_bad_lines="skip",
        names=["t_us", "count", "angle", "rpm", "pwm", "target"],
    )
    d = d.apply(pd.to_numeric, errors="coerce").dropna(subset=["t_us", "count"])
    t = d["t_us"].to_numpy(float) / 1e6
    y = S_ENC * d["count"].to_numpy(float) * 360.0 / COUNTS_PER_REV
    o = np.argsort(t)
    t, y = t[o], y[o]
    keep = np.concatenate(([True], np.diff(t) > 0))
    return t[keep], y[keep]


def load_imu(folder: str):
    files = [p for p in glob.glob(os.path.join(folder, "*.txt")) if "imu" in os.path.basename(p).lower()]
    if len(files) != 1:
        raise RuntimeError(f"Expected one IMU text file in {folder}, found {len(files)}")
    rows = []
    with open(files[0], errors="ignore") as fh:
        for line in fh:
            parts = line.strip().split("\t")
            if len(parts) >= 2:
                try:
                    rows.append((float(parts[0]), float(parts[1])))
                except ValueError:
                    pass
    if not rows:
        raise RuntimeError(f"No numeric IMU samples in {files[0]}")
    a = np.asarray(rows, dtype=float)
    return a[:, 0], S_IMU * a[:, 1]


def segment(rate: np.ndarray, t: np.ndarray):
    v = rate[np.isfinite(rate)]
    level = np.percentile(v, 90)
    mask = rate > PLATEAU_FRAC * level
    best, cur = (0, 0), None
    for i, flag in enumerate(mask):
        if flag and cur is None:
            cur = i
        elif not flag and cur is not None:
            if i - cur > best[1] - best[0]:
                best = (cur, i)
            cur = None
    if cur is not None and len(mask) - cur > best[1] - best[0]:
        best = (cur, len(mask))
    p0, p1 = best
    on = rate > ONSET_FRAC * level
    i_on = int(np.argmax(on))
    i_off = len(on) - int(np.argmax(on[::-1])) - 1
    ntrim = int(round(TRIM_S * FS))
    if p1 - p0 <= 2 * ntrim:
        raise RuntimeError("Detected plateau is too short after trimming")
    return dict(
        level=level,
        p0=p0,
        p1=p1,
        e0=p0 + ntrim,
        e1=p1 - ntrim,
        i_on=i_on,
        i_off=i_off,
        t_on=t[i_on],
        t_off=t[i_off],
        t_p0=t[p0],
        t_p1=t[p1],
    )


def onset_time(rate: np.ndarray, t: np.ndarray, level: float) -> float:
    on = rate > ONSET_FRAC * level
    return float(t[int(np.argmax(on))])


def sse(dt_off, te, we, tc, wc, idx):
    wi = np.interp(te[idx] + dt_off, tc, wc, left=np.nan, right=np.nan)
    d = wi - we[idx]
    d = d[np.isfinite(d)]
    return np.inf if len(d) < 10 else float(np.sum(d**2))


def refine(seed, te, we, tc, wc, idx):
    grid = np.arange(seed - SEARCH_S, seed + SEARCH_S + SEARCH_STEP, SEARCH_STEP)
    vals = np.asarray([sse(g, te, we, tc, wc, idx) for g in grid])
    return float(grid[int(np.nanargmin(vals))])


def metrics(dt_off, te, we, tc, wc, idx):
    wi = np.interp(te[idx] + dt_off, tc, wc, left=np.nan, right=np.nan)
    d = wi - we[idx]
    d = d[np.isfinite(d)]
    if len(d) < 10:
        return dict(rms=np.nan, mae=np.nan, bias=np.nan, n=len(d))
    return dict(
        rms=float(np.sqrt(np.mean(d**2))),
        mae=float(np.mean(np.abs(d))),
        bias=float(np.mean(d)),
        n=int(len(d)),
    )


def analyse(folder: str):
    sp = int(re.search(r"_(\d+)deg-s", folder).group(1))
    rep = re.match(r"(\w+)_test_", os.path.basename(folder)).group(1)

    tc_raw, yc_raw, camfile = load_camera(folder)
    te_raw, ye_raw = load_encoder(folder)

    dti = np.diff(tc_raw) * 1000.0
    med = float(np.median(dti))
    gaps = dti[dti > 1.5 * med]
    core = dti[dti <= 1.5 * med]

    gc, yc = to_grid(tc_raw - tc_raw[0], yc_raw)
    ge, ye = to_grid(te_raw - te_raw[0], ye_raw)
    wc, we = sgdiff(yc), sgdiff(ye)

    seg = segment(we, ge)
    cam_level = np.percentile(wc[np.isfinite(wc)], 90)
    dt_onset = onset_time(np.nan_to_num(wc), gc, cam_level) - seg["t_on"]

    ok = np.isfinite(we)
    idx_eval = np.arange(seg["e0"], seg["e1"]); idx_eval = idx_eval[ok[idx_eval]]
    idx_full = np.arange(seg["i_on"], seg["i_off"]); idx_full = idx_full[ok[idx_full]]
    idx_rise = np.arange(seg["i_on"], seg["p0"])
    idx_fall = np.arange(seg["p1"], seg["i_off"])
    idx_tran = np.concatenate([idx_rise, idx_fall]); idx_tran = idx_tran[ok[idx_tran]]

    dt_ref = refine(dt_onset, ge, we, gc, wc, idx_full)
    dt_tra = refine(dt_onset, ge, we, gc, wc, idx_tran)

    m_on = metrics(dt_onset, ge, we, gc, wc, idx_eval)
    m_rf = metrics(dt_ref, ge, we, gc, wc, idx_eval)
    m_tr = metrics(dt_tra, ge, we, gc, wc, idx_eval)

    wref = np.interp(ge[idx_full] + dt_ref, gc, wc, left=np.nan, right=np.nan)
    good = np.isfinite(wref)
    corr = float(np.corrcoef(wref[good], we[idx_full][good])[0, 1])

    sweep = [
        (float(s), metrics(dt_ref + s, ge, we, gc, wc, idx_eval)["rms"])
        for s in np.arange(-SWEEP_S, SWEEP_S + SWEEP_STEP, SWEEP_STEP)
    ]

    return dict(
        run=os.path.basename(folder), sp=sp, rep=rep, camfile=camfile,
        plateau_level=float(seg["level"]), plateau_s=float(seg["t_p1"] - seg["t_p0"]), eval_n=int(len(idx_eval)),
        dt_onset=dt_onset, dt_refined=dt_ref, dt_transition=dt_tra,
        d_ref_onset=dt_ref - dt_onset, d_ref_tran=dt_ref - dt_tra,
        rms_onset=m_on["rms"], rms_refined=m_rf["rms"], rms_transition=m_tr["rms"],
        mae_onset=m_on["mae"], mae_refined=m_rf["mae"], mae_transition=m_tr["mae"],
        bias_onset=m_on["bias"], bias_refined=m_rf["bias"], bias_transition=m_tr["bias"],
        corr_refined=corr,
        cam_total_deg=float(yc_raw[-1] - yc_raw[0]), enc_total_deg=float(ye_raw[-1] - ye_raw[0]),
        cam_span_deg=float(np.max(yc_raw) - np.min(yc_raw)), enc_span_deg=float(np.max(ye_raw) - np.min(ye_raw)),
        dt_med_ms=med, dt_sd_ms=float(dti.std()), dt_core_sd_ms=float(core.std()),
        dt_mad_ms=float(np.median(np.abs(dti - med))), dt_p99_ms=float(np.percentile(dti, 99)),
        dt_max_ms=float(dti.max()), n_gaps=int(len(gaps)), n_int=int(len(dti)),
        est_dropped=int(np.sum(np.round(gaps / med) - 1)) if len(gaps) else 0,
        sweep=sweep,
    )


def discover_runs(root: Path):
    runs = [Path(p) for p in sorted(glob.glob(str(root / "*_test_*deg-s"))) if os.path.isdir(p)]
    if len(runs) != 30:
        raise RuntimeError(f"Expected 30 run folders under {root}, found {len(runs)}")
    return runs


def build_tables(df: pd.DataFrame):
    order = {"first": 1, "seconed": 2, "third": 3}
    df["rep_n"] = df["rep"].map(order)
    df = df.sort_values(["sp", "rep_n"]).reset_index(drop=True)
    for which in ["onset", "transition"]:
        df[f"dRMS_{which}"] = df[f"rms_{which}"] - df["rms_refined"]
        df[f"pct_{which}"] = 100 * df[f"dRMS_{which}"] / df["rms_refined"]
        df[f"dMAE_{which}"] = df[f"mae_{which}"] - df["mae_refined"]
        df[f"dBIAS_{which}"] = df[f"bias_{which}"] - df["bias_refined"]
    df["angle_pct_diff"] = 100 * (df["cam_total_deg"] - df["enc_total_deg"]) / df["enc_total_deg"]

    ab = pd.DataFrame({
        "setpoint": df["sp"], "repeat": df["rep_n"],
        "dt_onset_s": df["dt_onset"], "dt_refined_s": df["dt_refined"], "dt_transition_s": df["dt_transition"],
        "RMS_refined": df["rms_refined"], "RMS_onset": df["rms_onset"], "RMS_transition": df["rms_transition"],
        "dRMS_onset": df["dRMS_onset"], "pct_onset": df["pct_onset"],
        "dRMS_transition": df["dRMS_transition"], "pct_transition": df["pct_transition"],
        "MAE_refined": df["mae_refined"], "bias_refined": df["bias_refined"],
        "cam_total_deg": df["cam_total_deg"], "enc_total_deg": df["enc_total_deg"], "angle_pct_diff": df["angle_pct_diff"],
    })
    return df, ab


def main():
    here = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", type=Path, default=here / "data" / "raw_runs")
    ap.add_argument("--output-dir", type=Path, default=here / "results" / "reanalysis")
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    res = [analyse(str(p)) for p in discover_runs(args.data_root)]
    sweeps = {r["run"]: r.pop("sweep") for r in res}
    df = pd.DataFrame(res)
    df, ab = build_tables(df)

    df.to_csv(args.output_dir / "per_run_derived.csv", index=False)
    ab.to_csv(args.output_dir / "table_A_B_per_run.csv", index=False)
    with open(args.output_dir / "sweeps.json", "w") as fh:
        json.dump(sweeps, fh)

    print(f"runs analysed: {len(df)}")
    print(f"SG derivative: window={SG_WIN} samples ({SG_WIN/FS:.2f} s), polyorder={SG_POLY}, grid={FS:.0f} Hz")
    print(f"transition-only mean dRMS: {df['dRMS_transition'].mean():+.6f} deg/s")
    print(f"transition-only max |dRMS|: {df['dRMS_transition'].abs().max():.6f} deg/s")
    print(f"transition-only max |relative dRMS|: {df['pct_transition'].abs().max():.4f}%")
    print(f"onset-only mean dRMS: {df['dRMS_onset'].mean():+.6f} deg/s")


if __name__ == "__main__":
    main()
