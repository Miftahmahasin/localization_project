#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 1 (numeric, authoritative) — PERPENDICULAR residual of the projected line
model vs the rendered painted line, per distance bucket.

This avoids the two artifacts that inflate the other metrics:
  * 2D "distance to nearest line pixel" adds longitudinal grass-gap distance;
  * a purely VERTICAL search over-reads near-horizontal (far) lines by 1/cos(angle).
Here, for each field segment we know its LOCAL image direction (from projected
consecutive points), so we search strictly along the PERPENDICULAR for the
achromatic-bright ridge peak (sub-pixel, bilinear) — the true cross-line error,
the same quantity a perpendicular ridge measurement uses. Camera is rebuilt
exactly from each frame's sidecar (cam_world_pos + quat).

USAGE:
  python3 measure_line_perp.py --data-dir /media/miftah/backup/landmark_dataset/_smoke --n 120
"""
import argparse
import json
import math
import os

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
import sys
sys.path.insert(0, os.path.join(_HERE, '..'))
from landmark_dataset_gen.projection import Projector           # noqa: E402
from landmark_dataset_gen.field_lines import build_field_segments  # noqa: E402
from landmark_dataset_gen.field_landmarks import CENTER_CIRCLE_R    # noqa: E402

RIDGE_MIN = 90
RIDGE_SAT = 80
WIN = 14.0        # perpendicular search half-length (px)
STEP = 0.5        # perpendicular sample step (px)
# (RIDGE_MIN / WIN overridable via CLI for the noise-vs-signal check)


def load_meta(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def quat2mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def world_segment_points(spacing=0.02):
    """Yield (Nx3 points, ) for each straight segment AND circle arcs, so local
    direction is well-defined within each."""
    segs = []
    for (a, b) in build_field_segments():
        (x1, y1), (x2, y2) = a, b
        L = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(math.ceil(L / spacing)) + 1)
        t = np.linspace(0, 1, n)
        segs.append(np.stack([x1 + (x2 - x1) * t, y1 + (y2 - y1) * t,
                              np.zeros(n)], axis=1))
    # circle as one closed polyline
    n = max(120, int(math.ceil(2 * math.pi * CENTER_CIRCLE_R / spacing)))
    ang = np.linspace(0, 2 * math.pi, n, endpoint=True)
    segs.append(np.stack([CENTER_CIRCLE_R * np.cos(ang),
                          CENTER_CIRCLE_R * np.sin(ang), np.zeros(n)], axis=1))
    return segs


def sample_min_rgb(img, x, y):
    """Bilinear min(R,G,B) and achromatic flag at float (x,y)."""
    H, W = img.shape[:2]
    if x < 0 or y < 0 or x >= W - 1 or y >= H - 1:
        return -1.0
    x0, y0 = int(x), int(y)
    fx, fy = x - x0, y - y0
    patch = img[y0:y0 + 2, x0:x0 + 2, :].astype(np.float32)
    wts = np.array([[(1 - fx) * (1 - fy), fx * (1 - fy)],
                    [(1 - fx) * fy, fx * fy]])
    px = (patch * wts[:, :, None]).sum(axis=(0, 1))
    mn = float(min(px)); mx = float(max(px))
    if mn >= RIDGE_MIN and (mx - mn) <= RIDGE_SAT:
        return mn
    return -1.0


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
    ap.add_argument('--n', type=int, default=120)
    ap.add_argument('--fov', type=float, default=1.3613)
    global RIDGE_MIN
    ap.add_argument('--max-range', type=float, default=9.0)
    ap.add_argument('--win', type=float, default=14.0)
    ap.add_argument('--ridge-min', type=int, default=90)
    args = ap.parse_args()
    RIDGE_MIN = args.ridge_min

    data_dir = os.path.expanduser(args.data_dir)
    recs = load_meta(os.path.join(data_dir, 'gt_metadata.jsonl'))[:args.n]
    segs = world_segment_points(0.02)

    signed = {}
    absd = {}
    P = None
    ts = np.arange(-args.win, args.win + 1e-6, STEP)

    for rec in recs:
        img = cv2.imread(os.path.join(data_dir, 'images', rec['image']))
        if img is None:
            continue
        H, W = img.shape[:2]
        if P is None:
            fx = W / (2.0 * math.tan(args.fov * 0.5))
            K = np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])
            P = Projector(K, W, H, max_range_m=args.max_range)
        cam = rec['camera']
        pos = np.array(cam['cam_world_pos'], float)
        qw, qx, qy, qz = cam['cam_world_quat']
        T = np.eye(4); T[:3, :3] = quat2mat(qw, qx, qy, qz); T[:3, 3] = pos
        P._T_map_cam = T; P._T_cam_map = np.linalg.inv(T); P._cam_pos = pos.copy()

        for seg in segs:
            uv, valid = P._project(seg, range_limit=args.max_range)
            dist = np.linalg.norm(seg - pos[None, :], axis=1)
            for i in range(1, len(seg) - 1):
                if not (valid[i] and valid[i - 1] and valid[i + 1]):
                    continue
                u, v = uv[i]
                if not (2 <= u < W - 2 and 2 <= v < H - 2):
                    continue
                # local image direction from neighbours; perpendicular
                d = uv[i + 1] - uv[i - 1]
                dn = math.hypot(d[0], d[1])
                if dn < 1e-3:
                    continue
                perp = np.array([-d[1], d[0]]) / dn
                # scan perpendicular for the achromatic ridge peak nearest t=0
                best_s, best_t = 0.0, None
                for t in ts:
                    s = sample_min_rgb(img, u + t * perp[0], v + t * perp[1])
                    if s > 0:
                        w = s * (1.0 / (1.0 + abs(t) / 4.0))
                        if w > best_s:
                            best_s, best_t = w, t
                if best_t is None:
                    continue
                b = int(dist[i])
                signed.setdefault(b, []).append(best_t)
                absd.setdefault(b, []).append(abs(best_t))

    print('=' * 70)
    print('TAHAP 1 line PERPENDICULAR residual — %s' % data_dir)
    print('  frames %d   win=%.0fpx  (signed>0 = line is to +perp side of model)'
          % (len(recs), WIN))
    print('-' * 70)
    print(' dist[m)    n     signed_median   |perp|_median   |perp|_p95')
    alls, alla = [], []
    for b in sorted(signed):
        s = signed[b]; a = absd[b]; alls += s; alla += a
        print('  %2d-%-2d  %6d     %+7.2f          %6.2f        %6.2f'
              % (b, b + 1, len(s), pct(s, 50), pct(a, 50), pct(a, 95)))
    print('-' * 70)
    if alla:
        print('  ALL: signed median %+.2f   |perp| median %.2f   p95 %.2f   n=%d'
              % (pct(alls, 50), pct(alla, 50), pct(alla, 95), len(alls)))
    print('=' * 70)
    if alla and pct(alla, 50) <= 2.0:
        print('VERDICT GATE 1: PASS — perpendicular |residual| median <= 2 px. '
              'Camera model correct: keep chain, freeze pitch_bias, SKIP TAHAP 2.')
    elif alla:
        print('VERDICT GATE 1: |perp| median %.2f px (>2) — real extrinsic error; '
              'TAHAP 2 (ground-truth pose) warranted.' % pct(alla, 50))


if __name__ == '__main__':
    main()
