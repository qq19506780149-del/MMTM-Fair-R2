# MMTM Fair Recheck (Reviewer #1, Comment 3)

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=qq19506780149-del/MMTM-Fair-R2&file=RUN_ALL_FAIR_R2.m)

This repository contains six MATLAB scripts for a fresh 30-run **Map A / Map B × SSA / NGO / GWO** fair-recheck of the path-planning experiments. The scripts share the same random-seed rule and candidate-attempt schedule within each stage, retain all runs, remove targeted post-hoc variance/median/run-indexed candidate adjustments, and record actual stage-level and cumulative objective-function evaluations (FE).

## Run in MATLAB Online

1. Click **Open in MATLAB Online** above, sign in to your MathWorks account, and accept cloning the repository.
2. In the MATLAB command window, set the current folder to the cloned repository if it is not already selected.
3. Execute:

   ```matlab
   RUN_ALL_FAIR_R2
   ```

   To run just one experiment (recommended as a smoke test):

   ```matlab
   FAIR_R2_MAPA_SSA
   ```

4. Each experiment writes its own date-stamped `FAIR_R2_*` output directory under the current folder. Keep all 30 runs and raw CSVs. Do not manually choose subsets or alter summary statistics.

## Scripts

- `RUN_ALL_FAIR_R2.m`: sequentially runs all six experiments.
- `FAIR_R2_MAPA_{SSA,NGO,GWO}.m`: three algorithms on Map A.
- `FAIR_R2_MAPB_{SSA,NGO,GWO}.m`: three algorithms on Map B.

The scripts were statically inspected but **have not been executed in MATLAB in this environment**. Their numerical results will be determined by the MATLAB run and may differ from previously published tables. Equal candidate counts across optimizers do not guarantee identical FE; use the recorded `StageObjectiveFE` and `CumulativeObjectiveFE` when analyzing computational fairness.

For transparency, this is a fresh recheck protocol, so the resulting Tables 4–5 and Figures 15–16 should be updated from actual outputs before claiming a recomputation in the response letter.