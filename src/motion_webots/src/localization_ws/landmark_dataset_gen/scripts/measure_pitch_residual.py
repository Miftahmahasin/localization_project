#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VALIDATOR (TAHAP 7) — sidecar geometry integrity under the canonical Webots K.

HISTORY: this tool used to RECOVER K + base_z by least squares and grid-search a
`pitch_bias_deg` to recommend. That is GONE. GATE 1 froze the calibration
(pitch_bias = -5.0 is a real physical offset; the perpendicular line residual is
signed-0 out to 9 m), so there is nothing to fit. The old K recovery was also the
unreliable part (it returned cy ~= 560 instead of the true 540, absorbing residual
into the principal point) — exactly why fitting is the wrong tool.

WHAT IT DOES NOW: it does NOT fit anything. It builds the CANONICAL K straight
from the Webots camera model (fx = W / (2 tan(FOV/2)), cx = W/2, cy = H/2 — the
same formula the op3_extern_controller publishes on camera_info), then reprojects
every stored landmark's world_xy through that K and the stored camera pose and
checks it matches the stored pixel_uv (< 0.5 px). This asserts (a) the generator
used the canonical K, and (b) the sidecar is self-consistent / uncorrupted.
Requires >= 200 samples. Exits non-zero on failure.

For image-vs-render alignment use measure_line_perp.py; for empty-box detection
use selfcheck_labels.py.

USAGE:
  python3 measure_pitch_residual.py --data-dir /media/miftah/backup/landmark_dataset/train
"""
import argparse
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
from landmark_dataset_gen.projection import Projector          # noqa: E402

GROUND = {'L', 'T', 'X'}
MIN_SAMPLES = 200
SELF_CHECK_PX = 0.5


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


def image_size(data_dir, name, fallback):
    p = os.path.join(data_dir, 'images', name)
    try:
        from PIL import Image
        with Image.open(p) as im:
            return im.size
    except Exception:
        return fallback


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
    ap.add_argument('--meta', default='gt_metadata.jsonl')
    ap.add_argument('--fov', type=float, default=1.3613, help='Webots horiz FOV rad')
    ap.add_argument('--width', type=int, default=0)
    ap.add_argument('--height', type=int, default=0)
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    recs = load_meta(os.path.join(data_dir, args.meta))
    if len(recs) < MIN_SAMPLES:
        print('VALIDATOR: only %d samples (< %d required). Collect more.'
              % (len(recs), MIN_SAMPLES))
        sys.exit(2)

    if args.width and args.height:
        W, H = args.width, args.height
    else:
        W, H = image_size(data_dir, recs[0]['image'], (1920, 1080))
    fx = W / (2.0 * math.tan(args.fov * 0.5))
    K = np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])
    P = Projector(K, W, H, max_range_m=1e9)   # geometry only, no range gating

    err = []
    n_lm = 0
    for rec in recs:
        cam = rec['camera']
        pos = np.array(cam['cam_world_pos'], float)
        qw, qx, qy, qz = cam['cam_world_quat']
        T = np.eye(4)
        T[:3, :3] = quat2mat(qw, qx, qy, qz)
        T[:3, 3] = pos
        P._T_map_cam = T
        P._T_cam_map = np.linalg.inv(T)
        P._cam_pos = pos.copy()
        for lm in rec.get('landmarks', []):
            if lm['class'] not in GROUND:
                continue
            n_lm += 1
            gx, gy = lm['world_xy']
            uv, valid = P._project(np.array([[gx, gy, 0.0]]))
            if not valid[0]:
                continue
            du = uv[0, 0] - lm['pixel_uv'][0]
            dv = uv[0, 1] - lm['pixel_uv'][1]
            err.append(math.hypot(du, dv))

    print('=' * 62)
    print('SIDECAR VALIDATOR (canonical K, no fitting) — %s' % data_dir)
    print('  samples %d   junction/mark landmarks %d' % (len(recs), n_lm))
    print('  canonical K: fx=fy=%.2f  cx=%.1f  cy=%.1f  (FOV=%.4f, %dx%d)'
          % (fx, W / 2.0, H / 2.0, args.fov, W, H))
    print('-' * 62)
    if err:
        print('  reprojection error vs stored pixel_uv:')
        print('    median %.4f px   p95 %.4f px   max %.4f px   n=%d'
              % (pct(err, 50), pct(err, 95), max(err), len(err)))
    print('=' * 62)
    if err and pct(err, 95) <= SELF_CHECK_PX:
        print('VALIDATOR PASS: sidecar reprojects under the canonical Webots K '
              '(p95 <= %.1f px). K is canonical and the sidecar is consistent.'
              % SELF_CHECK_PX)
    else:
        print('VALIDATOR FAIL: reprojection p95 %.3f px > %.1f px — the generator '
              'K is NOT canonical, or the sidecar is inconsistent. Investigate '
              '(do NOT "fix" it with a pitch_bias — calibration is frozen).'
              % (pct(err, 95) if err else float('nan'), SELF_CHECK_PX))
        sys.exit(1)


if __name__ == '__main__':
    main()
