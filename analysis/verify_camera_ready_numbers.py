#!/usr/bin/env python3
"""Sanity checks for the camera-ready paired re-analysis outputs."""
from pathlib import Path
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / 'results' / 'reanalysis'
df = pd.read_csv(R / 'per_run_derived.csv')

assert len(df) == 30, len(df)
assert sorted(df['sp'].unique().tolist()) == list(range(30, 301, 30))
assert all(df.groupby('sp').size() == 3)

mean_trans = df['dRMS_transition'].mean()
max_abs_trans = df['dRMS_transition'].abs().max()
max_rel_trans = df['pct_transition'].abs().max()
mean_onset = df['dRMS_onset'].mean()

assert abs(mean_trans - 0.005114651247066076) < 1e-9
assert abs(max_abs_trans - 0.0311717926234589) < 1e-9
assert abs(max_rel_trans - 3.068481010328709) < 1e-9
assert abs(mean_onset - (-0.01806612890735972)) < 1e-9
assert int((df['dRMS_onset'] > 0).sum()) == 9
assert int((df['dRMS_onset'] < 0).sum()) == 21
assert abs(df['d_ref_onset'].abs().max() - 0.083) < 1e-9
assert abs(df['d_ref_tran'].abs().max() - 0.020) < 1e-9

print('PASS: 30 runs and all camera-ready paired-analysis headline values match.')
