#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 8 — aggregate ≥5 landmark_eval CSVs into a per-scenario DISTRIBUTION.

The plan (TAHAP 8) requires multi-run results reported as a distribution
(mean + spread), not a single number, with predictions committed BEFORE the data
is seen. This tool reuses ``landmark_eval.summarize`` (the single source of truth
for the RMSE / time-to-converge / mirror-side logic) to reduce each run's CSV to
scalars, then reports mean / std / min / max across the runs of a scenario.

Usage:
  # one scenario (e.g. tracking) from 5 runs:
  python3 landmark_multirun.py --label "8c tracking" c_run*.csv

  # several scenarios in one table (repeat --label then its files):
  python3 landmark_multirun.py \
      --label "8a global-reloc" a_run*.csv \
      --label "8b kidnap"       b_run*.csv \
      --label "8c tracking"     c_run*.csv

Metrics per run: pos RMSE (m), yaw RMSE (deg), median (m), p95 (m),
time-to-converge (s; runs that never converge are reported separately, not as 0),
mirror% and flips (TAHAP 5 side integrity).
"""
import argparse
import csv
import importlib.util
import math
import os
import sys

# import summarize() from the sibling landmark_eval.py (single source of truth)
_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    'landmark_eval', os.path.join(_HERE, 'landmark_eval.py'))
_le = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_le)


def _run_metrics(path, conv_m, conv_hold, deadband):
    rows = list(csv.DictReader(open(path)))
    # summarize prints per-run detail (kept — useful) and returns the dict
    return _le.summarize(rows, conv_m, conv_hold,
                         label=os.path.basename(path),
                         center_deadband=deadband)


def _stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    n = len(vals)
    m = sum(vals) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / n) if n > 1 else 0.0
    return dict(n=n, mean=m, std=sd, mn=min(vals), mx=max(vals))


def _fmt(s, unit=''):
    if s is None:
        return '        —'
    return '%.3f±%.3f' % (s['mean'], s['std'])


def _report(label, metrics):
    print('\n' + '=' * 74)
    print('SCENARIO: %s   (n_runs=%d)' % (label, len(metrics)))
    print('=' * 74)
    keys = [('rmse', 'pos RMSE [m]'), ('yrmse', 'yaw RMSE [deg]'),
            ('median', 'median [m]'), ('p95', 'p95 [m]')]
    print('  %-16s  %-15s  %8s  %8s' % ('metric', 'mean±std', 'min', 'max'))
    print('  ' + '-' * 54)
    for k, name in keys:
        s = _stats([m[k] for m in metrics if m])
        if s:
            print('  %-16s  %-15s  %8.3f  %8.3f'
                  % (name, '%.3f±%.3f' % (s['mean'], s['std']), s['mn'], s['mx']))
    # time-to-converge: report converged runs' distribution + how many reached it
    convs = [m['conv'] for m in metrics if m and m['conv'] is not None]
    n_conv = len(convs)
    s = _stats(convs)
    if s:
        print('  %-16s  %-15s  %8.3f  %8.3f   [%d/%d runs converged]'
              % ('converge [s]', '%.1f±%.1f' % (s['mean'], s['std']),
                 s['mn'], s['mx'], n_conv, len(metrics)))
    else:
        print('  %-16s  %-15s          converged in 0/%d runs'
              % ('converge [s]', '—', len(metrics)))
    # mirror-side integrity (TAHAP 5)
    mir = _stats([m['mirror_pct'] for m in metrics if m and m['mirror_pct'] is not None])
    flips = _stats([m['flips'] for m in metrics if m and m['flips'] is not None])
    if mir:
        print('  %-16s  %-15s  %8.3f  %8.3f'
              % ('mirror [%]', '%.1f±%.1f' % (mir['mean'], mir['std']),
                 mir['mn'], mir['mx']))
    if flips:
        print('  %-16s  %-15s  %8.0f  %8.0f'
              % ('side flips [#]', '%.1f±%.1f' % (flips['mean'], flips['std']),
                 flips['mn'], flips['mx']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', action='append', default=[],
                    help='scenario label; the CSVs after it belong to it')
    ap.add_argument('--conv_m', type=float, default=0.30)
    ap.add_argument('--conv_hold', type=float, default=3.0)
    ap.add_argument('--deadband', type=float, default=0.5)
    ap.add_argument('files', nargs='*', help='CSVs (one unlabelled scenario)')
    args, extra = ap.parse_known_args()

    # Build scenario -> files. argparse can't interleave; re-parse argv so that
    # files following each --label attach to it.
    scenarios = []            # list of (label, [files])
    cur_label = None
    cur_files = []
    argv = sys.argv[1:]
    i = 0
    skip = {'--conv_m', '--conv_hold', '--deadband'}
    while i < len(argv):
        a = argv[i]
        if a == '--label':
            if cur_label is not None or cur_files:
                scenarios.append((cur_label or 'scenario', cur_files))
            cur_label = argv[i + 1]
            cur_files = []
            i += 2
        elif a in skip:
            i += 2
        elif a.endswith('.csv'):
            cur_files.append(a)
            i += 1
        else:
            i += 1
    if cur_label is not None or cur_files:
        scenarios.append((cur_label or 'scenario', cur_files))

    if not scenarios:
        ap.error('no CSV files given')

    for label, files in scenarios:
        files = [f for f in files if os.path.exists(f)]
        if not files:
            print('\n[%s] no CSVs found' % label)
            continue
        print('\n' + '#' * 74 + '\n# %s  (%d runs)\n' % (label, len(files))
              + '#' * 74)
        metrics = [_run_metrics(f, args.conv_m, args.conv_hold, args.deadband)
                   for f in files]
        _report(label, [m for m in metrics if m])


if __name__ == '__main__':
    main()
