#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 1 — backend sensitivity to detector quality, from the GT sidecar.

Two offline studies, no Webots, deterministic:

  A. PROJECTION ACCURACY (TAHAP 2.2 preview): reproject each labelled landmark
     pixel through the SINGLE shared camera model at the sidecar's exact stored
     camera pose and compare the ground hit to the stored world_xy. Per class,
     per 1 m distance bucket: median / p95 / n. This is the "keypoint world
     error" the covariance model will be built on.

  B. BACKEND SENSITIVITY: for each frame build clean landmark observations in
     the robot frame from ground truth, DEGRADE them (recall dropout, false
     positives, L/T/X class confusion, ground noise), run the geometric MHL
     localizer (landmark_localization.mhl) with NO prior, and score the recovered
     pose against GT (and its 180-deg mirror). Sweeping each degradation axis
     answers: what recall / precision / class accuracy does the backend actually
     need to converge? -> a CURVE, per prompt, that sets the training target.

Usage:
  python3 sweep_sensitivity.py --sidecar /media/miftah/backup/landmark_dataset/val \
        --n 1500 --out /home/miftah/basbot/fase_gy1_plots
"""
import argparse
import json
import math
import os
import random

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from landmark_geometry.projection import Projector  # noqa: E402
from landmark_localization.mhl import (  # noqa: E402
    GeometricLocalizer, Obs, CLS_L, CLS_T, CLS_X)

_NAME2ID = {'L': 0, 'T': 1, 'X': 2, 'goalpost': 3, 'center_circle': 4}
_GROUND = (0, 1, 2, 4)
_LTX = (CLS_L, CLS_T, CLS_X)
_SUCC_M = 0.5     # position error (m) counted as a converged relocalization


def _quat_to_R(q, order):
    if order == 'wxyz':
        w, x, y, z = q
    else:
        x, y, z, w = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _T_from(pos, quat, order):
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(quat, order)
    T[:3, 3] = pos
    return T


def load(sidecar_dir, n, seed=0):
    path = os.path.join(sidecar_dir, 'gt_metadata.jsonl')
    rows = []
    with open(path) as f:
        for line in f:
            rows.append(json.loads(line))
    random.Random(seed).shuffle(rows)
    return rows[:n] if n and n < len(rows) else rows


def detect_quat_order(rows):
    """Pick the quaternion component order that reprojects landmarks best."""
    errs = {'wxyz': [], 'xyzw': []}
    for r in rows[:60]:
        cam = r['camera']
        K = np.array([[cam['K_fx_fy_cx_cy'][0], 0, cam['K_fx_fy_cx_cy'][2]],
                      [0, cam['K_fx_fy_cx_cy'][1], cam['K_fx_fy_cx_cy'][3]],
                      [0, 0, 1]])
        W, H = cam['image_wh']
        for order in ('wxyz', 'xyzw'):
            pr = Projector(K, W, H, max_range_m=30, ground_max_range_m=30)
            pr.set_pose_matrix(_T_from(cam['cam_world_pos'],
                                       cam['cam_world_quat'], order))
            for lm in r['landmarks']:
                if _NAME2ID[lm['class']] not in _GROUND:
                    continue
                w = pr.unproject_to_ground(*lm['pixel_uv'])
                if w is None:
                    errs[order].append(9.9)
                    continue
                errs[order].append(math.hypot(w[0] - lm['world_xy'][0],
                                              w[1] - lm['world_xy'][1]))
    med = {k: (np.median(v) if v else 9.9) for k, v in errs.items()}
    return 'wxyz' if med['wxyz'] <= med['xyzw'] else 'xyzw', med


def study_projection(rows, order):
    """Per-class, per-1m-bucket reprojection error (ground classes + goalpost)."""
    buckets = {}
    for r in rows:
        cam = r['camera']
        K = np.array([[cam['K_fx_fy_cx_cy'][0], 0, cam['K_fx_fy_cx_cy'][2]],
                      [0, cam['K_fx_fy_cx_cy'][1], cam['K_fx_fy_cx_cy'][3]],
                      [0, 0, 1]])
        W, H = cam['image_wh']
        pr = Projector(K, W, H, max_range_m=30, ground_max_range_m=30)
        pr.set_pose_matrix(_T_from(cam['cam_world_pos'],
                                   cam['cam_world_quat'], order))
        for lm in r['landmarks']:
            cid = _NAME2ID[lm['class']]
            if cid not in _GROUND:      # goalpost pixel_uv is not the ground pt
                continue
            w = pr.unproject_to_ground(*lm['pixel_uv'])
            if w is None:
                continue
            e = math.hypot(w[0] - lm['world_xy'][0], w[1] - lm['world_xy'][1])
            d = int(lm['distance_m'])
            buckets.setdefault((cid, d), []).append(e)
    return buckets


def clean_obs(r):
    """Landmark observations in the robot base frame, from ground truth."""
    gp = r['gt_robot_pose']
    yaw = math.radians(gp['yaw_deg'])
    c, s = math.cos(-yaw), math.sin(-yaw)
    R = np.array([[c, -s], [s, c]])
    p = np.array([gp['x'], gp['y']])
    obs = []
    for lm in r['landmarks']:
        w = np.array(lm['world_xy'])
        b = R @ (w - p)
        obs.append((_NAME2ID[lm['class']], b, float(lm['distance_m'])))
    return obs, gp


def degrade(obs, recall, fp, confuse, noise, rng, max_lm=7):
    out = []
    for cid, b, d in obs:
        if rng.random() > recall:
            continue
        c = cid
        if cid in _LTX and rng.random() < confuse:
            c = rng.choice([x for x in _LTX if x != cid])
        bb = b + rng.normal(0.0, noise, size=2)
        out.append(Obs(c, bb))
    for _ in range(fp):
        rng_bear = rng.uniform(-0.6, 0.6)
        rng_rng = rng.uniform(1.0, 6.0)
        b = np.array([rng_rng * math.cos(rng_bear), rng_rng * math.sin(rng_bear)])
        out.append(Obs(int(rng.integers(0, 5)), b))
    # keep the >=nearest max_lm (ARTEMIS compute cap)
    out.sort(key=lambda o: float(np.hypot(*o.b)))
    return out[:max_lm]


def score(rows, loc, recall, fp, confuse, noise, seed=0):
    rng = np.random.default_rng(seed)
    n_ok2 = solved = succ_c = succ_e = 0
    errs = []
    for r in rows:
        obs, gp = clean_obs(r)
        obs = degrade(obs, recall, fp, confuse, noise, rng)
        if len(obs) < 2:
            continue
        n_ok2 += 1
        res = loc.localize(obs)
        if res is None:
            continue
        pose, votes, nh = res
        solved += 1
        egt = math.hypot(pose.x - gp['x'], pose.y - gp['y'])
        emir = math.hypot(pose.x + gp['x'], pose.y + gp['y'])
        errs.append(egt)
        if egt < _SUCC_M:
            succ_c += 1
        if min(egt, emir) < _SUCC_M:
            succ_e += 1
    denom = max(n_ok2, 1)
    return dict(n=n_ok2, solved=100.0 * solved / denom,
                correct=100.0 * succ_c / denom, either=100.0 * succ_e / denom,
                med_err=float(np.median(errs)) if errs else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sidecar',
                    default='/media/miftah/backup/landmark_dataset/val')
    ap.add_argument('--n', type=int, default=1500)
    ap.add_argument('--out', default='/home/miftah/basbot/fase_gy1_plots')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = load(args.sidecar, args.n)
    print('loaded %d frames from %s' % (len(rows), args.sidecar))
    order, med = detect_quat_order(rows)
    print('quat order = %s  (median reproj err wxyz=%.4f xyzw=%.4f m)'
          % (order, med['wxyz'], med['xyzw']))

    # ── A. projection accuracy ────────────────────────────────────────────────
    buckets = study_projection(rows, order)
    print('\n== A. PROJECTION ACCURACY (ground pixel -> world) ==')
    print('%-14s %5s %8s %8s %6s' % ('class', 'd[m]', 'med[m]', 'p95[m]', 'n'))
    names = {0: 'L', 1: 'T', 2: 'X', 4: 'center_circle'}
    for (cid, d) in sorted(buckets):
        e = np.array(buckets[(cid, d)])
        print('%-14s %5d %8.4f %8.4f %6d'
              % (names[cid], d, np.median(e), np.percentile(e, 95), e.size))

    # ── B. sensitivity sweeps ─────────────────────────────────────────────────
    loc = GeometricLocalizer()
    base = dict(recall=1.0, fp=0, confuse=0.0, noise=0.03)
    print('\n== B. BACKEND SENSITIVITY (MHL, no prior; success = err<%.2fm) =='
          % _SUCC_M)

    def run(axis, vals, key):
        print('\n-- sweep %s --' % axis)
        print('%8s %6s %8s %9s %8s %9s' %
              (axis, 'n', 'solved%', 'correct%', 'either%', 'med_err'))
        rowsout = []
        for v in vals:
            kw = dict(base)
            kw[key] = v
            s = score(rows, loc, kw['recall'], kw['fp'], kw['confuse'],
                      kw['noise'])
            print('%8s %6d %8.1f %9.1f %8.1f %9.3f'
                  % (str(v), s['n'], s['solved'], s['correct'], s['either'],
                     s['med_err']))
            rowsout.append((v, s))
        return rowsout

    sweeps = {
        'recall': run('recall', [1.0, 0.8, 0.6, 0.5, 0.4, 0.3], 'recall'),
        'fp/frame': run('fp/frame', [0, 1, 2, 3, 5], 'fp'),
        'confuse': run('confuse', [0.0, 0.1, 0.2, 0.3, 0.5], 'confuse'),
    }

    # ── plots + csv ───────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    csv = os.path.join(args.out, 'sensitivity.csv')
    with open(csv, 'w') as f:
        f.write('axis,value,n,solved_pct,correct_pct,either_pct,med_err\n')
        for ax, (name, data) in zip(axes, sweeps.items()):
            xs = [v for v, _ in data]
            ax.plot(xs, [s['correct'] for _, s in data], 'o-', label='correct')
            ax.plot(xs, [s['either'] for _, s in data], 's--',
                    label='either(±180°)')
            ax.set_xlabel(name)
            ax.set_ylabel('relocalize success %')
            ax.set_ylim(0, 105)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
            ax.set_title('vs %s' % name)
            for v, s in data:
                f.write('%s,%s,%d,%.1f,%.1f,%.1f,%.3f\n'
                        % (name, v, s['n'], s['solved'], s['correct'],
                           s['either'], s['med_err']))
    fig.tight_layout()
    png = os.path.join(args.out, 'sensitivity.png')
    fig.savefig(png, dpi=110)
    print('\nsaved %s and %s' % (csv, png))


if __name__ == '__main__':
    main()
