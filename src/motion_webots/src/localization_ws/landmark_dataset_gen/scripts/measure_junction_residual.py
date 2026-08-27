#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 1 (numeric) — PERPENDICULAR (vertical) residual of the projected junction
vs the rendered painted line, per distance bucket.

The 2D "distance to nearest line pixel" metric conflates longitudinal grass-gap
distance with true cross-line error on thin, broken far lines. The residual that
actually matters for a label is the VERTICAL offset between the projected junction
row and the painted-line row at that column — the thing that makes a box look
"above/below" the line. We take the junction pixel straight from the sidecar
(pixel_uv, i.e. the generator's own projection) and search a vertical window for
the achromatic-bright ridge peak (robust to domain randomisation), sub-pixel.

signed offset = v_ridge - v_projected   (>0: line is BELOW the projected point;
                                         <0: line is ABOVE it)

USAGE:
  python3 measure_junction_residual.py \
      --data-dir /media/miftah/backup/landmark_dataset/_smoke --win 26
"""
import argparse
import json
import math
import os

import numpy as np
import cv2

GROUND = {'L', 'T', 'X'}
RIDGE_MIN = 90
RIDGE_SAT = 80


def load_meta(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def ridge_row(img, u, v0, win):
    """Sub-pixel row of the achromatic-bright ridge nearest v0 in column u."""
    H, W = img.shape[:2]
    u = int(round(u))
    if not (0 <= u < W):
        return None
    r0 = max(0, int(v0) - win)
    r1 = min(H, int(v0) + win + 1)
    if r1 - r0 < 3:
        return None
    col = img[r0:r1, u, :].astype(np.int16)
    mn = np.minimum(np.minimum(col[:, 0], col[:, 1]), col[:, 2])
    mx = np.maximum(np.maximum(col[:, 0], col[:, 1]), col[:, 2])
    score = np.where((mn >= RIDGE_MIN) & ((mx - mn) <= RIDGE_SAT), mn, 0)
    if score.max() <= 0:
        return None
    # peak nearest v0: weight score by proximity to the projected row
    rows = np.arange(r0, r1)
    prox = 1.0 / (1.0 + np.abs(rows - v0) / 6.0)
    k = int(np.argmax(score * prox))
    if score[k] <= 0:
        return None
    # parabolic sub-pixel refine on the raw min-RGB score
    if 0 < k < len(score) - 1:
        a, b, c = float(score[k - 1]), float(score[k]), float(score[k + 1])
        denom = (a - 2 * b + c)
        d = 0.5 * (a - c) / denom if abs(denom) > 1e-6 else 0.0
    else:
        d = 0.0
    return r0 + k + d


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--n', type=int, default=0, help='frames (0=all)')
    ap.add_argument('--win', type=int, default=26, help='vertical search half-window px')
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    recs = load_meta(os.path.join(data_dir, 'gt_metadata.jsonl'))
    if args.n:
        recs = recs[:args.n]

    signed = {}     # bucket -> list of signed offsets
    absned = {}
    n_used = 0
    n_junc = 0
    for rec in recs:
        img = cv2.imread(os.path.join(data_dir, 'images', rec['image']))
        if img is None:
            continue
        H, W = img.shape[:2]
        for lm in rec.get('landmarks', []):
            if lm['class'] not in GROUND:
                continue
            u, v = lm['pixel_uv']
            if not (0 <= u < W and 0 <= v < H):
                continue
            n_junc += 1
            vr = ridge_row(img, u, v, args.win)
            if vr is None:
                continue
            off = vr - v
            b = int(lm['distance_m'])
            signed.setdefault(b, []).append(off)
            absned.setdefault(b, []).append(abs(off))
            n_used += 1

    print('=' * 72)
    print('TAHAP 1 junction VERTICAL residual (signed = line - projected) — %s'
          % data_dir)
    print('  frames %d   in-frame junctions %d   ridge-matched %d (%.0f%%)   win=%dpx'
          % (len(recs), n_junc, n_used, 100.0 * n_used / max(n_junc, 1), args.win))
    print('-' * 72)
    print(' dist[m)   n    signed_median  signed_p50|..|  |off|_median  |off|_p95')
    alls, alla = [], []
    for b in sorted(signed):
        s = signed[b]
        a = absned[b]
        alls += s
        alla += a
        print('  %2d-%-2d  %4d     %+7.2f          %5.2f         %5.2f       %5.2f'
              % (b, b + 1, len(s), pct(s, 50), pct([abs(x) for x in s], 50),
                 pct(a, 50), pct(a, 95)))
    print('-' * 72)
    if alls:
        print('  ALL: signed median %+.2f   |off| median %.2f   |off| p95 %.2f   n=%d'
              % (pct(alls, 50), pct(alla, 50), pct(alla, 95), len(alls)))
    print('=' * 72)
    if alla and pct(alla, 50) <= 2.0:
        print('VERDICT GATE 1: junctions sit on the line — |vertical offset| median '
              '<= 2 px. Camera model CORRECT: keep chain, freeze pitch_bias, SKIP '
              'TAHAP 2.')
    elif alla:
        print('VERDICT GATE 1: |vertical offset| median %.2f px (>2). Check signed '
              'trend vs distance before deciding TAHAP 2.' % pct(alla, 50))


if __name__ == '__main__':
    main()
