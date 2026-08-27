#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP H — split-metric analyzer for the degradation sweep (S1/S5).

The whole point (directive H2): the insurance value of line-heading is NOT in the
frames that already have a landmark fix — C6-live measured exactly that and found
nothing (0.772 vs 0.774 deg). It is in the frames where the landmark fix is ABSENT
(junctions degraded away), where line-heading (image-based, junction-independent)
keeps the EKF yaw observed instead of drifting. So this tool reports THREE things
separately, never conflated:

  fix-rate     — fraction of frames with a FRESH geometric fix (a new /landmark_pose,
                 detected by fix value change). This is the DETECTOR-degradation curve
                 (S5, line-heading-independent) — it should match between A and B under
                 the same degradation; report it as the sweep-point's break-point input.
  (a) yaw@scarce — ekf_yaw_err in SCARCE windows (>= --scarce consecutive frames with no
                 fresh fix). A (line-heading ON) vs B (OFF). THIS is where line-heading
                 insurance lives. Also the whole-run p95 tail, which the scarce windows
                 drive.
  (b) err@fix  — pos/yaw error on FRESH-FIX frames, A vs B (the C6-live cut; expect ~0).

Usage:
  harness_analyze.py --a lh_on*.csv --b lh_off*.csv [--scarce 5] [--eps 1e-4]
"""
import argparse
import csv
import statistics as st


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _load(path):
    rows = list(csv.DictReader(open(path)))
    out = []
    prev = None
    for r in rows:
        fx, fy = _f(r.get('fix_x')), _f(r.get('fix_y'))
        fresh = False
        if fx is not None and fy is not None:
            cur = (round(fx, 4), round(fy, 4))
            fresh = (prev is None) or (cur != prev)
            prev = cur
        out.append({
            't': _f(r.get('t')),
            'ekf_err': _f(r.get('ekf_err')),
            'ekf_yaw_err': _f(r.get('ekf_yaw_err')),
            'fresh': fresh,
            'has_fix': fx is not None,
        })
    return out


def _scarce_mask(rows, scarce):
    """True on frames inside a run of >= `scarce` consecutive non-fresh frames."""
    n = len(rows)
    mask = [False] * n
    i = 0
    while i < n:
        if rows[i]['fresh']:
            i += 1
            continue
        j = i
        while j < n and not rows[j]['fresh']:
            j += 1
        if j - i >= scarce:
            for k in range(i, j):
                mask[k] = True
        i = j
    return mask


def _stat(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    s = sorted(vals)
    p95 = s[min(len(s) - 1, int(0.95 * len(s)))]
    return (st.median(s), p95, len(s))


def _agg(paths, scarce):
    fresh_rate, yaw_scarce, yaw_all, err_fresh, yaw_fresh = [], [], [], [], []
    for p in paths:
        rows = _load(p)
        if not rows:
            continue
        n = len(rows)
        nf = sum(1 for r in rows if r['fresh'])
        fresh_rate.append(100.0 * nf / n)
        mask = _scarce_mask(rows, scarce)
        yaw_scarce += [rows[k]['ekf_yaw_err'] for k in range(n) if mask[k]]
        yaw_all += [r['ekf_yaw_err'] for r in rows]
        err_fresh += [r['ekf_err'] for r in rows if r['fresh']]
        yaw_fresh += [r['ekf_yaw_err'] for r in rows if r['fresh']]
    return {
        'n_runs': len(fresh_rate),
        'fresh_rate': (st.mean(fresh_rate) if fresh_rate else None),
        'scarce_frac': (100.0 * len(yaw_scarce)
                        / max(1, len(yaw_all))),
        'yaw_scarce': _stat(yaw_scarce),
        'yaw_all': _stat(yaw_all),
        'err_fresh': _stat(err_fresh),
        'yaw_fresh': _stat(yaw_fresh),
    }


def _line(lbl, s):
    if s is None:
        return '  %-22s   (no samples)' % lbl
    return '  %-22s median=%7.3f  p95=%7.3f  n=%d' % (lbl, s[0], s[1], s[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a', nargs='+', required=True,
                    help='condition-A CSVs (line-heading ON)')
    ap.add_argument('--b', nargs='+', required=True,
                    help='condition-B CSVs (line-heading OFF)')
    ap.add_argument('--scarce', type=int, default=5,
                    help='consecutive non-fresh frames = a scarce window')
    ap.add_argument('--label', default='degradation point')
    args = ap.parse_args()

    A = _agg(args.a, args.scarce)
    B = _agg(args.b, args.scarce)

    print('=' * 68)
    print('HARNESS SPLIT-METRIC — %s' % args.label)
    print('  A = line-heading ON (%d run)   B = OFF (%d run)   scarce>=%d frames'
          % (A['n_runs'], B['n_runs'], args.scarce))
    print('-' * 68)
    print('DETECTOR degradation (S5; line-heading-independent, should ~match):')
    print('  fresh fix-rate   A=%.1f%%   B=%.1f%%   (scarce frames A=%.1f%% B=%.1f%%)'
          % (A['fresh_rate'] or 0, B['fresh_rate'] or 0,
             A['scarce_frac'], B['scarce_frac']))
    print('-' * 68)
    print('(a) YAW @ SCARCE windows  [deg] — WHERE LINE-HEADING INSURANCE LIVES:')
    print(_line('A · line-heading ON', A['yaw_scarce']))
    print(_line('B · line-heading OFF', B['yaw_scarce']))
    print('     whole-run yaw tail:')
    print(_line('A · all frames', A['yaw_all']))
    print(_line('B · all frames', B['yaw_all']))
    print('-' * 68)
    print('(b) ERROR @ FRESH-FIX frames — the C6-live cut (expect A ~= B):')
    print(_line('A pos [m]', A['err_fresh']))
    print(_line('B pos [m]', B['err_fresh']))
    print(_line('A yaw [deg]', A['yaw_fresh']))
    print(_line('B yaw [deg]', B['yaw_fresh']))
    print('=' * 68)


if __name__ == '__main__':
    main()
