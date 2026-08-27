#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 1 — full line-model overlay + model-vs-ridge residual per distance bucket.

The single strongest diagnostic: rebuild each frame's EXACT camera from its
sidecar (cam_world_pos + cam_world_quat), project the whole painted-line model
(field_lines.build_field_line_points, 2 cm), and (a) draw it as red dots over the
render, (b) measure the perpendicular distance from every projected line point to
the nearest actually-painted pixel via a distance transform on an achromatic-ridge
mask (lines are white/achromatic, grass is green). Reports median & p95 residual
per 1 m distance bucket — GATE 1: overlay must stick <=2 px out to 9 m.

USAGE:
  python3 overlay_lines.py --data-dir /media/miftah/backup/landmark_dataset/_smoke \
      --n 80 --save 8 --out-dir /tmp/tahap1
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
from landmark_dataset_gen.projection import Projector          # noqa: E402
from landmark_dataset_gen.field_lines import build_field_line_points  # noqa: E402

RIDGE_MIN = 90       # min(R,G,B) above this = bright (candidate line pixel)
RIDGE_SAT = 80       # max(R,G,B)-min(R,G,B) below this = achromatic (white line)
CLOSE_K = 5          # morphological close kernel — bridge grass gaps within a line
MATCH_CAP_PX = 12.0  # a projected point farther than this from any ridge = unmatched


def load_meta(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[r['image']] = r
    return out


def quat2mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def ridge_dist_map(img):
    """Distance (px) from every pixel to the nearest achromatic bright (line) pixel."""
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    mn = np.minimum(np.minimum(b, g), r).astype(np.int16)
    mx = np.maximum(np.maximum(b, g), r).astype(np.int16)
    mask = ((mn >= RIDGE_MIN) & ((mx - mn) <= RIDGE_SAT)).astype(np.uint8)
    # bridge grass-texture gaps WITHIN a line so the distance transform measures
    # distance to the line band, not to the nearest surviving speckle.
    if CLOSE_K > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    if mask.sum() == 0:
        return None
    # distance to nearest mask (line) pixel: transform on the INVERSE
    inv = (1 - mask).astype(np.uint8)
    return cv2.distanceTransform(inv, cv2.DIST_L2, 3)


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--n', type=int, default=80, help='frames to use for stats')
    ap.add_argument('--save', type=int, default=8, help='overlay PNGs to write')
    ap.add_argument('--out-dir', default='/tmp/tahap1')
    ap.add_argument('--fov', type=float, default=1.3613)
    ap.add_argument('--max-range', type=float, default=10.0)
    ap.add_argument('--pitch-bias', type=float, default=-5.0,
                    help='only used if rebuilding via set_pose; sidecar pose is exact')
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    meta = load_meta(os.path.join(data_dir, 'gt_metadata.jsonl'))
    os.makedirs(args.out_dir, exist_ok=True)
    names = sorted(meta.keys())[:args.n]

    line_pts = build_field_line_points(spacing=0.02)   # Nx3 world, z=0
    # residuals bucketed by ground distance (1 m bins)
    buckets = {}     # bin_idx -> list of matched residual px
    seen = {}        # bin_idx -> total projected-in-frame points
    P = None

    for i, name in enumerate(names):
        rec = meta[name]
        img = cv2.imread(os.path.join(data_dir, 'images', name))
        if img is None:
            continue
        H, W = img.shape[:2]
        if P is None:
            fx = W / (2.0 * math.tan(args.fov * 0.5))
            K = np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])
            P = Projector(K, W, H, max_range_m=args.max_range)
        cam = rec['camera']
        pos = np.array(cam['cam_world_pos'], dtype=np.float64)
        qw, qx, qy, qz = cam['cam_world_quat']
        T = np.eye(4)
        T[:3, :3] = quat2mat(qw, qx, qy, qz)
        T[:3, 3] = pos
        P._T_map_cam = T
        P._T_cam_map = np.linalg.inv(T)
        P._cam_pos = pos.copy()

        uv, valid = P._project(line_pts, range_limit=args.max_range)
        dist = np.linalg.norm(line_pts - pos[None, :], axis=1)
        inframe = valid & (uv[:, 0] >= 0) & (uv[:, 0] < W) & \
            (uv[:, 1] >= 0) & (uv[:, 1] < H)

        dmap = ridge_dist_map(img)
        if dmap is not None:
            us = uv[inframe]
            ds = dist[inframe]
            for (u, v), gd in zip(us, ds):
                res = float(dmap[int(v), int(u)])
                b = int(gd)                     # 1 m bins: [b, b+1)
                seen[b] = seen.get(b, 0) + 1
                if res <= MATCH_CAP_PX:
                    buckets.setdefault(b, []).append(res)

        if i < args.save:
            ov = img.copy()
            us = uv[inframe].astype(int)
            for u, v in us:
                ov[v, u] = (0, 0, 255)          # red model dot
            cv2.imwrite(os.path.join(args.out_dir, name.replace('.png',
                        '') + '_overlay.png'), ov)

    print('=' * 70)
    print('TAHAP 1 line-model overlay residual — %s' % data_dir)
    print('  frames used: %d   overlays saved to: %s' % (len(names), args.out_dir))
    print('  ridge mask: min(RGB)>=%d & sat<=%d ;  match cap %.0f px' %
          (RIDGE_MIN, RIDGE_SAT, MATCH_CAP_PX))
    print('-' * 70)
    print(' dist[m)  n_pts  matched%%  median_px  p95_px   (matched only)')
    all_res = []
    for b in sorted(seen):
        res = buckets.get(b, [])
        n = seen[b]
        mrate = 100.0 * len(res) / n if n else 0.0
        med = pct(res, 50) if res else float('nan')
        p95 = pct(res, 95) if res else float('nan')
        all_res += res
        print('  %2d-%-2d  %6d   %5.1f    %7.2f  %7.2f'
              % (b, b + 1, n, mrate, med, p95))
    print('-' * 70)
    if all_res:
        print('  ALL matched: median %.2f  p95 %.2f  n=%d'
              % (pct(all_res, 50), pct(all_res, 95), len(all_res)))
    print('=' * 70)
    # GATE 1 verdict on the near/mid range where lines are unambiguous
    near = [r for b in buckets for r in buckets[b] if b < 9]
    if near and pct(near, 50) <= 2.0:
        print('VERDICT GATE 1: line model sticks (median <= 2 px to 9 m) — camera '
              'model is CORRECT. Keep chain, freeze pitch_bias, SKIP TAHAP 2.')
    elif near:
        print('VERDICT GATE 1: median %.2f px (>2) — extrinsic error; TAHAP 2 '
              '(ground-truth pose) is warranted.' % pct(near, 50))


if __name__ == '__main__':
    main()
