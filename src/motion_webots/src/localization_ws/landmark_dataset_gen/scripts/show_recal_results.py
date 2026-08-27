#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Show the APPLIED labels on real frames AND quantify PER-FRAME residual.

The global calibration (pitch_bias/base_z) is a single constant for the whole
dataset. This tool tests whether that is enough: for each frame it measures the
median vertical residual of its OWN ground landmarks (label-box center vs the
real painted-line row detected in that image), so we can see the spread frame to
frame — not just the dataset average. It then renders result frames spanning the
residual distribution (best / typical / worst) with the applied labels drawn, so
the per-frame quality is visible, not asserted.

USAGE:
  python3 show_recal_results.py --data-dir /media/miftah/backup/landmark_dataset/train \
      --scan 600 --out /tmp/recal_results
"""
import argparse
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("need opencv")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from measure_pitch_residual import measure_line_row               # noqa: E402

CLS_COLOR = {0: (0, 220, 0), 1: (0, 160, 255), 2: (255, 0, 200),
             3: (255, 220, 0), 4: (0, 255, 255)}
CLS_NAME = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}
GROUND = {0, 1, 2}


def read_labels(path, W, H):
    out = []
    if not os.path.isfile(path):
        return out
    for ln in open(path):
        p = ln.split()
        if len(p) == 5:
            cid = int(p[0]); xc, yc, w, h = (float(v) for v in p[1:])
            out.append((cid, xc * W, yc * H, w * W, h * H))
    return out


def frame_residual(chanmin, boxes, search, hw, thr, dmax_px=None):
    """Median residual (line_row - box_center_row) over a frame's ground boxes."""
    res = []
    for cid, cx, cy, bw, bh in boxes:
        if cid not in GROUND:
            continue
        row = measure_line_row(chanmin, cx, cy, search, hw, thr)
        if row is not None:
            res.append(row - cy)
    return (float(np.median(res)), len(res)) if res else (None, 0)


def draw(img, boxes, resid=None):
    vis = img.copy()
    for cid, cx, cy, bw, bh in boxes:
        x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
        x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
        cv2.rectangle(vis, (x1, y1), (x2, y2), CLS_COLOR.get(cid, (255, 255, 255)), 1)
    tag = 'per-frame residual: %+.1f px' % resid if resid is not None else 'n/a'
    cv2.putText(vis, tag, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--labels', default='labels')
    ap.add_argument('--scan', type=int, default=600, help='frames to measure')
    ap.add_argument('--search', type=int, default=16)
    ap.add_argument('--hw', type=int, default=6)
    ap.add_argument('--min-line', type=float, default=115.0)
    ap.add_argument('--out', default=None)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--random', type=int, default=0,
                    help='if >0, render this many RANDOM frames (+montage) '
                         'instead of the best/worst representative picks')
    ap.add_argument('--cols', type=int, default=4)
    args = ap.parse_args()

    dd = args.data_dir
    img_dir = os.path.join(dd, 'images')
    lbl_dir = os.path.join(dd, args.labels)
    out_dir = args.out or os.path.join(dd, 'recal_results')
    os.makedirs(out_dir, exist_ok=True)

    stems = sorted(f[:-4] for f in os.listdir(img_dir) if f.endswith('.png'))
    rng = np.random.default_rng(args.seed)
    rng.shuffle(stems)
    stems = stems[:args.scan]

    h0, w0 = cv2.imread(os.path.join(img_dir, stems[0] + '.png')).shape[:2]
    per_frame = []      # (stem, resid, n)
    for st in stems:
        img = cv2.imread(os.path.join(img_dir, st + '.png'))
        boxes = read_labels(os.path.join(lbl_dir, st + '.txt'), w0, h0)
        chanmin = img.min(axis=2)
        r, n = frame_residual(chanmin, boxes, args.search, args.hw, args.min_line)
        if r is not None and n >= 3:
            per_frame.append((st, r, n))

    arr = np.array([r for _, r, _ in per_frame])
    print("per-frame residual (median of each frame's ground boxes), n=%d frames"
          % len(arr))
    print("  distribution: median=%+.2f  mean=%+.2f  std=%.2f px" %
          (np.median(arr), arr.mean(), arr.std()))
    for p in (5, 25, 50, 75, 95):
        print("    p%-2d = %+.2f px" % (p, np.percentile(arr, p)))
    print("  |per-frame median| >5px: %d frames (%.1f%%)  >8px: %d (%.1f%%)" %
          ((np.abs(arr) > 5).sum(), 100 * (np.abs(arr) > 5).mean(),
           (np.abs(arr) > 8).sum(), 100 * (np.abs(arr) > 8).mean()))
    # how much of the spread is measurement noise? per-frame std of within-frame
    # residuals gives the noise floor; compare to across-frame std.
    print("  (across-frame std %.2f px vs typical within-frame scatter — if the "
          "former is much larger, error is genuinely per-frame)" % arr.std())

    if args.random > 0:
        pf = {s: (r, n) for s, r, n in per_frame}
        chosen = [s for s in stems if s in pf][:args.random]
        tiles = []
        for st in chosen:
            img = cv2.imread(os.path.join(img_dir, st + '.png'))
            boxes = read_labels(os.path.join(lbl_dir, st + '.txt'), w0, h0)
            r, n = pf[st]
            vis = draw(img, boxes, r)
            cv2.putText(vis, st, (8, h0 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(out_dir, 'rand_%s.png' % st), vis)
            tiles.append(vis)
        cols = args.cols
        rows = int(math.ceil(len(tiles) / cols))
        th, tw = h0 // 2, w0 // 2
        canvas = np.zeros((rows * th, cols * tw, 3), dtype=np.uint8)
        for i, t in enumerate(tiles):
            rr, cc = divmod(i, cols)
            canvas[rr * th:(rr + 1) * th, cc * tw:(cc + 1) * tw] = \
                cv2.resize(t, (tw, th))
        mp = os.path.join(out_dir, 'recal_random_montage.png')
        cv2.imwrite(mp, canvas)
        print("rendered %d random frames + montage -> %s" % (len(tiles), mp))
        return

    # pick representative frames across the residual distribution to render
    order = sorted(per_frame, key=lambda t: t[1])
    picks = []
    labels = ['most-negative', 'p25', 'median', 'p75', 'most-positive',
              'best(|min|)']
    idxs = [0, len(order) // 4, len(order) // 2, 3 * len(order) // 4,
            len(order) - 1, int(np.argmin(np.abs([t[1] for t in order])))]
    tiles = []
    for lab, i in zip(labels, idxs):
        st, r, n = order[i]
        img = cv2.imread(os.path.join(img_dir, st + '.png'))
        boxes = read_labels(os.path.join(lbl_dir, st + '.txt'), w0, h0)
        vis = draw(img, boxes, r)
        cv2.putText(vis, '%s  %s' % (lab, st), (8, h0 - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(out_dir, 'res_%s_%s.png' % (lab, st)), vis)
        tiles.append(vis)
        print("  rendered %-14s %s  residual=%+.1f px (n=%d)" % (lab, st, r, n))

    # montage 3x2
    th, tw = h0 // 2, w0 // 2
    canvas = np.zeros((2 * th, 3 * tw, 3), dtype=np.uint8)
    for i, t in enumerate(tiles):
        rr, cc = divmod(i, 3)
        canvas[rr * th:(rr + 1) * th, cc * tw:(cc + 1) * tw] = cv2.resize(t, (tw, th))
    mp = os.path.join(out_dir, 'recal_results_montage.png')
    cv2.imwrite(mp, canvas)
    print("montage -> %s" % mp)


if __name__ == '__main__':
    main()
