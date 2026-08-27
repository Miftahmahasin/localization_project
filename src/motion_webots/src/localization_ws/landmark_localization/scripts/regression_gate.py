#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP C1 — PASS/FAIL gate for the 8b/8c regression suite.

Reuses ``landmark_eval.summarize`` (single source of truth for RMSE / converge /
mirror-side) to reduce each run CSV to scalars, then asserts a scenario against a
baseline so any change in BAGIAN B that quietly regresses the mature results
(8c 0.208 m, 8b 0.230 m, mirror 0%, flips 0) is caught before it is stacked on.

Design note — robust to the ONE out-of-scope physics/vision-death outlier (~1/5):
  * Position is checked on the MEDIAN across runs of each run's own MEDIAN error
    (steady-state accuracy). Full-run RMSE was mis-specified for the 8b re-entry
    scenario — the robot STARTS kidnapped and must re-converge, so the convergence
    transient inflates full-run RMSE even for a perfect run (e.g. kr_run1: full-run
    RMSE 1.88 m but per-run median 0.085 m, mirror 0%, flips 0 — a clean run).
  * Mirror / flips are checked as a COUNT of runs exceeding tolerance, allowing up to
    --outlier-max (default 1, consistent with conv-min 4/5). The live baseline itself
    produces ~1/5 vision-death runaway at the kidnap window; a max-across-ALL-runs
    check would fail on that inherent physics variance with NO code change, making the
    gate a coin flip rather than a regression detector. A real localization regression
    shows up in SEVERAL runs (2+), which still fails. Tolerating 1 outlier in the GATE
    does not fix the underlying hole — C2 (ref-blend anti-garbage) + C3 (ZUPT velocity
    pin) do; the gate just stays a reliable detector across the baseline's own noise.
Falls / vision-death are gait/perception issues, not a localization-code regression.

Exit code 0 = PASS, 1 = FAIL (so it can gate CI / a shell &&-chain).

Usage:
  python3 regression_gate.py --scenario "8c" --rmse-th 0.25 --conv-min 4 \
      --mirror-tol 1.0 --flip-th 1  c_run*.csv
"""
import argparse
import csv
import importlib.util
import os
import statistics
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    'landmark_eval', os.path.join(_HERE, 'landmark_eval.py'))
_le = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_le)


def _metrics(path, conv_m, conv_hold, deadband):
    rows = list(csv.DictReader(open(path)))
    return _le.summarize(rows, conv_m, conv_hold,
                         label=os.path.basename(path), center_deadband=deadband)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--scenario', default='scenario')
    ap.add_argument('--rmse-th', type=float, required=True,
                    help='max allowed MEDIAN-across-runs of each run MEDIAN error [m] '
                         '(steady-state accuracy; robust to re-entry transient)')
    ap.add_argument('--mirror-tol', type=float, default=1.0,
                    help='a run is a mirror-outlier if its mirror%% exceeds this')
    ap.add_argument('--flip-th', type=int, default=1,
                    help='a run is a flip-outlier if its side flips exceed this')
    ap.add_argument('--outlier-max', type=int, default=1,
                    help='max runs allowed to be a mirror/flip outlier (physics/'
                         'vision-death noise; a real regression hits 2+ runs)')
    ap.add_argument('--conv-min', type=int, default=4,
                    help='min runs that must converge (of N)')
    ap.add_argument('--conv-m', type=float, default=0.30)
    ap.add_argument('--conv-hold', type=float, default=3.0)
    ap.add_argument('--deadband', type=float, default=0.5)
    args = ap.parse_args()

    runs = [_metrics(f, args.conv_m, args.conv_hold, args.deadband)
            for f in args.files]
    runs = [r for r in runs if r]
    n = len(runs)
    if n == 0:
        print('!! no readable runs'); sys.exit(1)

    # Steady-state accuracy: median across runs of each run's OWN median error, so a
    # re-entry convergence transient (or one runaway run) cannot fail the position gate.
    med_of_medians = statistics.median(sorted(r['median'] for r in runs))
    # Mirror / flips: COUNT runs that are outliers (tolerate up to --outlier-max).
    mirror_bad = sum(1 for r in runs if r['mirror_pct'] > args.mirror_tol)
    flip_bad = sum(1 for r in runs if r['flips'] > args.flip_th)
    converged = sum(1 for r in runs if r['conv'] is not None)

    checks = [
        ('pos median-of-med', med_of_medians, '<=', args.rmse_th, 'm'),
        ('mirror-outliers',   mirror_bad, '<=', args.outlier_max, 'runs'),
        ('flip-outliers',     flip_bad, '<=', args.outlier_max, 'runs'),
        ('converged',         converged, '>=', args.conv_min, '/%d' % n),
    ]
    ok = (med_of_medians <= args.rmse_th and mirror_bad <= args.outlier_max
          and flip_bad <= args.outlier_max and converged >= args.conv_min)

    print('=' * 60)
    print('REGRESSION GATE — %s   (n=%d runs)' % (args.scenario, n))
    print('-' * 60)
    for name, val, op, th, unit in checks:
        good = (val <= th) if op == '<=' else (val >= th)
        print('  [%s] %-16s %7.3f %s %.3g %s'
              % ('PASS' if good else 'FAIL', name, val, op, th, unit))
    print('-' * 60)
    print('  VERDICT: %s' % ('PASS' if ok else 'FAIL'))
    print('=' * 60)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
