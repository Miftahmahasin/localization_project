#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 5 self-check — run after EVERY collection, no human eyes needed.

For each written junction/mark label (L/T/X), crop the box from the rendered
image and count achromatic-bright (painted-line) pixels inside it. A junction box
MUST contain part of a line; a box sitting on empty grass is a wrong label (the
colleague's "2/69 boxes with no line pixels"). Reports the empty-box rate per
class and exits non-zero if it exceeds --max-empty-frac, so a bad collection is
caught mechanically. Goalpost/center_circle boxes are reported but not gated
(they are not line-junction features).

USAGE:
  python3 selfcheck_labels.py --data-dir /media/miftah/backup/landmark_dataset/_smoke
"""
import argparse
import math
import os
import sys

import numpy as np
import cv2

CLASS_NAMES = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}
GROUND_IDS = {0, 1, 2}
RIDGE_MIN = 95
RIDGE_SAT = 80
MIN_RIDGE_FRAC = 0.008   # <0.8% line pixels in the box => "empty" (grass only)


def ridge_frac(crop):
    if crop.size == 0:
        return 0.0
    b, g, r = crop[:, :, 0], crop[:, :, 1], crop[:, :, 2]
    mn = np.minimum(np.minimum(b, g), r).astype(np.int16)
    mx = np.maximum(np.maximum(b, g), r).astype(np.int16)
    mask = (mn >= RIDGE_MIN) & ((mx - mn) <= RIDGE_SAT)
    return float(mask.mean())


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
    ap.add_argument('--max-empty-frac', type=float, default=0.02,
                    help='fail if >this fraction of junction boxes are empty')
    ap.add_argument('--list', type=int, default=15, help='max empties to list')
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    img_dir = os.path.join(data_dir, 'images')
    lbl_dir = os.path.join(data_dir, 'labels')
    stems = sorted(os.path.splitext(f)[0] for f in os.listdir(lbl_dir)
                   if f.endswith('.txt'))

    n = {c: 0 for c in CLASS_NAMES}
    empty = {c: 0 for c in CLASS_NAMES}
    fracs = {c: [] for c in CLASS_NAMES}
    empties = []
    for stem in stems:
        img = cv2.imread(os.path.join(img_dir, stem + '.png'))
        if img is None:
            continue
        H, W = img.shape[:2]
        with open(os.path.join(lbl_dir, stem + '.txt')) as f:
            for line in f:
                p = line.split()
                if len(p) != 5:
                    continue
                cid = int(float(p[0]))
                xc, yc, bw, bh = (float(p[1]) * W, float(p[2]) * H,
                                  float(p[3]) * W, float(p[4]) * H)
                x1, y1 = max(0, int(xc - bw / 2)), max(0, int(yc - bh / 2))
                x2, y2 = min(W, int(xc + bw / 2)), min(H, int(yc + bh / 2))
                fr = ridge_frac(img[y1:y2, x1:x2])
                n[cid] += 1
                fracs[cid].append(fr)
                if cid in GROUND_IDS and fr < MIN_RIDGE_FRAC:
                    empty[cid] += 1
                    empties.append((stem, CLASS_NAMES[cid], round(fr, 4),
                                    int(xc), int(yc)))

    ng = sum(n[c] for c in GROUND_IDS)
    ne = sum(empty[c] for c in GROUND_IDS)
    print('=' * 62)
    print('TAHAP 5 self-check — %s' % data_dir)
    print('  frames: %d   junction/mark labels: %d' % (len(stems), ng))
    print('-' * 62)
    for c in sorted(CLASS_NAMES):
        if n[c] == 0:
            continue
        tag = ('  empty %d (%.1f%%)' % (empty[c], 100.0 * empty[c] / n[c])
               if c in GROUND_IDS else '  (not gated)')
        print('  %-13s n=%-4d  ridge_frac med %.3f p5 %.3f%s'
              % (CLASS_NAMES[c], n[c], pct(fracs[c], 50), pct(fracs[c], 5), tag))
    print('-' * 62)
    ef = (ne / ng) if ng else 0.0
    print('  EMPTY junction boxes: %d / %d  (%.2f%%)   threshold %.2f%%'
          % (ne, ng, 100.0 * ef, 100.0 * args.max_empty_frac))
    for e in empties[:args.list]:
        print('     %s  %s  frac=%.4f  uv=(%d,%d)' % e)
    print('=' * 62)
    if ef > args.max_empty_frac:
        print('SELF-CHECK FAIL: empty-box rate %.2f%% > %.2f%%.'
              % (100.0 * ef, 100.0 * args.max_empty_frac))
        sys.exit(1)
    print('SELF-CHECK PASS: empty-box rate within threshold.')


if __name__ == '__main__':
    main()
