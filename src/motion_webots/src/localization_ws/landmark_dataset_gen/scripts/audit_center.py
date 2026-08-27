#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 0 (offline) — audit whether each junction/mark box is CENTERED on the
projected junction point, straight from the stored sidecar. No re-render.

WHY: the "labels droop below the line" report needs to be split into two causes
that look identical to the eye but are fixed in different places:
  (a) the box center is NOT the junction point  -> box-construction bug (TAHAP 4)
  (b) the box center IS the junction, but the box is a long thin rectangle or was
      clipped at the frame edge -> shape / clipping (TAHAP 4/5), NOT a center bug.

This tool reads gt_metadata.jsonl and, for every L/T/X label, compares:
    box center  = (bbox_norm center) * (W, H)         [what YOLO stores]
    junction    = pixel_uv                            [pure projected junction]
It reports the offset distribution for boxes whose junction is INSIDE the frame
and whose box does NOT touch a frame edge (there the center MUST equal the
junction), and separately counts the edge-clipped / off-frame cases (where a
shift is expected and legitimate).

USAGE:
  python3 audit_center.py --data-dir ~/landmark_dataset/meta_check
  python3 audit_center.py --data-dir ~/landmark_dataset/meta_check --width 1280 --height 720
"""
import argparse
import json
import math
import os
import sys

GROUND_CLASSES = {'L', 'T', 'X'}   # junction/mark boxes are built centered-on-point
EDGE_TOL_PX = 0.75                 # box edge within this of the frame border = clipped
CENTER_TOL_PX = 0.5                # DoD: |center - junction| must be <= this


def load_meta(path):
    recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    return recs


def image_size(data_dir, image_name, fallback):
    """Return (W, H) for an image, cheaply, without a full decode if possible."""
    path = os.path.join(data_dir, 'images', image_name)
    if not os.path.isfile(path):
        return fallback
    try:
        from PIL import Image
        with Image.open(path) as im:
            return im.size  # (W, H)
    except Exception:
        try:
            import cv2
            im = cv2.imread(path)
            if im is not None:
                return im.shape[1], im.shape[0]
        except Exception:
            pass
    return fallback


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * (p / 100.0)
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True,
                    help='dataset split dir containing gt_metadata.jsonl + images/')
    ap.add_argument('--width', type=int, default=0,
                    help='force image width (else auto-detect per image)')
    ap.add_argument('--height', type=int, default=0,
                    help='force image height (else auto-detect per image)')
    ap.add_argument('--fallback-width', type=int, default=1280)
    ap.add_argument('--fallback-height', type=int, default=720)
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    meta_path = os.path.join(data_dir, 'gt_metadata.jsonl')
    if not os.path.isfile(meta_path):
        sys.exit('no gt_metadata.jsonl in %s' % data_dir)
    recs = load_meta(meta_path)

    forced = (args.width and args.height)
    fallback = (args.fallback_width, args.fallback_height)

    clean = []              # (offset, class) for in-frame, non-clipped boxes
    per_class = {}          # class -> list of clean offsets
    n_total = 0
    n_offframe = 0          # junction projected outside the frame
    n_clipped = 0           # box touches a frame edge
    clipped_off = []        # offsets of clipped boxes (expected large)
    violators = []          # clean boxes with offset > CENTER_TOL_PX

    for rec in recs:
        if forced:
            W, H = args.width, args.height
        else:
            W, H = image_size(data_dir, rec['image'], fallback)
        for lm in rec.get('landmarks', []):
            if lm['class'] not in GROUND_CLASSES:
                continue
            n_total += 1
            bx, by, bw, bh = lm['bbox_norm']
            cx, cy = bx * W, by * H
            hw, hh = bw * W * 0.5, bh * H * 0.5
            x1, y1, x2, y2 = cx - hw, cy - hh, cx + hw, cy + hh
            ju, jv = lm['pixel_uv']
            offset = math.hypot(cx - ju, cy - jv)

            junction_in = (0.0 <= ju < W) and (0.0 <= jv < H)
            # project_junction clamps boxes to [0, W-1]x[0, H-1]; the far edge of
            # a clipped box sits at W-1 / H-1, so test against that, not W / H.
            edge_clipped = (x1 <= EDGE_TOL_PX or y1 <= EDGE_TOL_PX or
                            x2 >= (W - 1) - EDGE_TOL_PX or
                            y2 >= (H - 1) - EDGE_TOL_PX)

            if not junction_in:
                n_offframe += 1
                clipped_off.append(offset)
                continue
            if edge_clipped:
                n_clipped += 1
                clipped_off.append(offset)
                continue
            clean.append(offset)
            per_class.setdefault(lm['class'], []).append(offset)
            if offset > CENTER_TOL_PX:
                violators.append((rec['image'], lm['class'], lm['label'],
                                  round(offset, 3), round(ju, 1), round(jv, 1)))

    print('=' * 68)
    print('TAHAP 0 offline center audit  —  %s' % data_dir)
    print('  frames: %d   junction/mark labels: %d' % (len(recs), n_total))
    print('  resolution: %s' %
          ('forced %dx%d' % (args.width, args.height) if forced
           else 'auto-detected per image (fallback %dx%d)' % fallback))
    print('-' * 68)
    print('CLEAN boxes (junction in-frame AND box not edge-clipped): %d' % len(clean))
    print('  -> for these, box center MUST equal the junction point.')
    if clean:
        print('  |center - junction| px:  median %.3f   p95 %.3f   max %.3f'
              % (pct(clean, 50), pct(clean, 95), max(clean)))
        for c in sorted(per_class):
            v = per_class[c]
            print('     %-2s n=%-3d  median %.3f  p95 %.3f  max %.3f'
                  % (c, len(v), pct(v, 50), pct(v, 95), max(v)))
        print('  violators (> %.1f px): %d' % (CENTER_TOL_PX, len(violators)))
        for row in violators[:20]:
            print('     %s  %s/%s  off=%.3f  ju=(%.1f,%.1f)' %
                  (row[0], row[1], row[2], row[3], row[4], row[5]))
    print('-' * 68)
    print('CLIPPED / OFF-FRAME boxes (shift is expected, NOT a center bug):')
    print('  junction off-frame: %d    box edge-clipped: %d' % (n_offframe, n_clipped))
    if clipped_off:
        print('  their |center - junction| px: median %.2f  p95 %.2f  max %.2f'
              % (pct(clipped_off, 50), pct(clipped_off, 95), max(clipped_off)))
    print('=' * 68)
    # GATE 0 verdict
    if clean and not violators:
        print('VERDICT: construction is centered — every clean box has '
              'center==junction (<= %.1f px). Remaining visual shift is shape/'
              'clipping (TAHAP 4/5), not a center bug.' % CENTER_TOL_PX)
    elif clean:
        print('VERDICT: %d/%d clean boxes are OFF-CENTER (> %.1f px) — a real '
              'box-construction bug remains (TAHAP 4).'
              % (len(violators), len(clean), CENTER_TOL_PX))
    else:
        print('VERDICT: no clean boxes to judge (all clipped/off-frame).')


if __name__ == '__main__':
    main()
