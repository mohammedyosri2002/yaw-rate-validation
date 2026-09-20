# Camera-ready repository release

This release adds the material needed to reproduce the reviewer-facing synchronization sensitivity analysis without changing the accepted paper's original experimental tables.

Added:

- all 30 valid raw run folders (camera, encoder, ESAT logs);
- portable `analysis/reanalysis.py`;
- portable `analysis/imu_analysis.py`;
- `analysis/run_all.py` and a numerical verification script;
- machine-readable re-analysis outputs under `results/reanalysis/`;
- explicit documentation of the Option-1 paired-analysis scope and timing limitations.

The accepted absolute camera/encoder and reference-comparison tables remain in `results/paper_results.*` and are not replaced by the rebuilt absolute values.
