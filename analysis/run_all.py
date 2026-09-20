#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DATA = ROOT / 'data' / 'raw_runs'
OUT = ROOT / 'results' / 'reanalysis'

for script in ['reanalysis.py', 'imu_analysis.py']:
    subprocess.run([sys.executable, str(HERE / script), '--data-root', str(DATA), '--output-dir', str(OUT)], check=True)
print(f'All analysis outputs written to {OUT}')
