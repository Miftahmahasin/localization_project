#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 0/4 (offline, pose-agnostic) — drive the CURRENT Projector directly on
real stored poses and measure box geometry WITHOUT Webots or a renderer.

WHY: the stored gt_metadata.jsonl may have been written by an older projection.py.
To judge the code as it stands NOW we re-run project_junction ourselves on the
real poses (poses.csv) and the real field model, then report:
  1. center offset  |box_center - projected_junction|  (TAHAP 0: should be ~0 for
     non-clipped boxes if construction is centered)
  2. aspect ratio  w/h  and box height distribution     (TAHAP 4: independent
     half_w/half_h + a min-size floor make far boxes wide-and-short)

Center offset is pose/K-agnostic: if the construction is symmetric about the
junction it is 0 for every non-clipped box regardless of the exact camera pose.

USAGE:
  python3 audit_box_geometry.py --data-dir ~/landmark_dataset/meta_check \
      --width 1280 --height 720 --fov 1.3613 --pitch-bias -5.0
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
from landmark_dataset_gen.projection import Projector          # noqa: E402
from landmark_dataset_gen import field_landmarks as FL          # noqa: E402

EDGE_TOL_PX = 0.75
GROUND = {0, 1, 2}   # yolo ids L,T,X


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def load_poses(path):
    """poses.csv: stem,x,y,yaw,head_pan,head_tilt (radians for angles)."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            p = line.split(',')
            if len(p) < 6:
                continue
            try:
                out.append((p[0], float(p[1]), float(p[2]),
                            float(p[3]), float(p[4]), float(p[5])))
            except ValueError:
                continue   # header
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fov', type=float, default=1.3613, help='horizontal FOV rad')
    ap.add_argument('--base-z', type=float, default=0.28,
                    help='nominal robot base height (does NOT affect centering)')
    ap.add_argument('--pitch-bias', type=float, default=-5.0)
    ap.add_argument('--ground-max-range', type=float, default=5.0)
    ap.add_argument('--min-emit', type=float, default=18.0)
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    poses = load_poses(os.path.join(data_dir, 'poses.csv'))
    if not poses:
        sys.exit('no usable poses.csv in %s' % data_dir)

    W, H = args.width, args.height
    fx = W / (2.0 * math.tan(args.fov * 0.5))
    K = np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])
    P = Projector(K, W, H, max_range_m=9.0, min_emit_px=args.min_emit,
                  pitch_bias_deg=args.pitch_bias,
                  ground_max_range_m=args.ground_max_range)

    junctions = FL.build_line_intersections()

    center_off = []      # non-clipped only
    aspects = []         # non-clipped only  (w/h)
    heights = []
    widths = []
    n_total = 0
    n_clipped = 0
    at_floor_h = 0
    floor = args.min_emit

    for (_stem, x, y, yaw, hpan, htilt) in poses:
        P.set_pose(x, y, args.base_z, yaw, hpan, htilt)
        for j in junctions:
            det = P.project_junction(j)
            if det is None:
                continue
            n_total += 1
            cx = 0.5 * (det.x1 + det.x2)
            cy = 0.5 * (det.y1 + det.y2)
            w = det.x2 - det.x1
            h = det.y2 - det.y1
            # recover the pure junction pixel to compare
            uv, valid = P._project(np.array([[j.x, j.y, 0.0]]),
                                   range_limit=P.ground_max_range)
            ju, jv = float(uv[0, 0]), float(uv[0, 1])
            # project_junction clamps to [0, W-1]x[0, H-1], so the far edge sits
            # at W-1 / H-1 when clipped — test against that, not W / H.
            clipped = (det.x1 <= EDGE_TOL_PX or det.y1 <= EDGE_TOL_PX or
                       det.x2 >= (W - 1) - EDGE_TOL_PX or
                       det.y2 >= (H - 1) - EDGE_TOL_PX)
            if clipped:
                n_clipped += 1
                continue
            center_off.append(math.hypot(cx - ju, cy - jv))
            aspects.append(w / h if h > 0 else float('nan'))
            heights.append(h)
            widths.append(w)
            if h <= floor + 0.5:
                at_floor_h += 1

    print('=' * 68)
    print('TAHAP 0/4 box-geometry audit (CURRENT code) — %s' % data_dir)
    print('  poses: %d   junction boxes emitted: %d   (non-clipped: %d)'
          % (len(poses), n_total, len(center_off)))
    print('  K: fx=%.1f cx=%.1f cy=%.1f  min_emit=%.0f  gmr=%.1f  pitch_bias=%.2f'
          % (fx, W / 2.0, H / 2.0, args.min_emit, args.ground_max_range,
             args.pitch_bias))
    print('-' * 68)
    print('TAHAP 0  center offset |box_center - junction| px (non-clipped):')
    if center_off:
        print('  median %.4f   p95 %.4f   max %.4f   (>0.5px: %d)'
              % (pct(center_off, 50), pct(center_off, 95), max(center_off),
                 sum(1 for v in center_off if v > 0.5)))
    print('-' * 68)
    print('TAHAP 4  aspect ratio w/h (non-clipped):')
    if aspects:
        print('  median %.2f   p5 %.2f   p95 %.2f   min %.2f   max %.2f'
              % (pct(aspects, 50), pct(aspects, 5), pct(aspects, 95),
                 min(aspects), max(aspects)))
        print('  box height px:  median %.1f  p5 %.1f  p95 %.1f   at floor(<=%.0f): '
              '%d/%d' % (pct(heights, 50), pct(heights, 5), pct(heights, 95),
                         floor, at_floor_h, len(heights)))
        print('  box width  px:  median %.1f  p5 %.1f  p95 %.1f'
              % (pct(widths, 50), pct(widths, 5), pct(widths, 95)))
    print('=' * 68)
    if center_off and sum(1 for v in center_off if v > 0.5) == 0:
        print('VERDICT TAHAP 0: CURRENT construction is centered — every '
              'non-clipped box has center==junction (<=0.5px).')
    elif center_off:
        print('VERDICT TAHAP 0: %d non-clipped boxes still off-center — real bug.'
              % sum(1 for v in center_off if v > 0.5))
    if aspects and (pct(aspects, 95) > 2.0 or pct(aspects, 5) < 0.5):
        print('VERDICT TAHAP 4: aspect ratio varies widely (needs square boxes).')
    elif aspects:
        print('VERDICT TAHAP 4: aspect already near-square.')


if __name__ == '__main__':
    main()
