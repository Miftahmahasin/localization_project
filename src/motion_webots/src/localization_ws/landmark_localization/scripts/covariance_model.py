#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 2.2/2.3 — world-position uncertainty of a projected landmark vs distance.

The reprojection itself is exact (TAHAP 1 Part A: 0.0000 m against GT), so the
real ground-position error comes from propagating the INPUT uncertainties through
the pinhole+ground-intersection geometry:

    inputs:  pixel center (sigma_px), head tilt & pan (sigma_ang),
             base/camera height (sigma_h)
    output:  ground (x, y) covariance, which blows up toward the horizon
             (range error ~ d^2, cross-range ~ d)

Method: at each labelled landmark, build the camera pose from the sidecar and
compute the 2x5 Jacobian d(ground_xy)/d(u,v,tilt,pan,h) by finite differences,
then C = J diag(sigma^2) J^T. Rotate C into radial (range) / tangential
(cross-range) axes relative to the camera and bin by 1 m distance, per class.
This yields sigma_range(d), sigma_cross(d) per class -> the measurement
covariance model, and a proposed max_range per class (where range sigma exceeds
a usable bound). Goalpost uses its BASE point (world x,+-1.3 forward-projected),
matching the runtime node (box ymax = post base).

The default input sigmas are stated assumptions (detector px localization, joint
encoder/calibration, stance height); pass measured values to refine. The model
STRUCTURE (distance scaling) is geometry-driven and robust to their exact values.
"""
import argparse
import json
import math
import os

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from landmark_geometry.projection import Projector  # noqa: E402

_NAME2ID = {'L': 0, 'T': 1, 'X': 2, 'goalpost': 3, 'center_circle': 4}
_NAMES = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}
_USABLE_SIGMA = 0.50   # range sigma (m) beyond which a class is not a position fix


def _quat_to_R(q):  # sidecar order = wxyz (auto-detected in TAHAP 1)
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _T(pos, quat):
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(quat)
    T[:3, 3] = pos
    return T


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def _ground(pr, T, u, v):
    pr.set_pose_matrix(T)
    return pr.unproject_to_ground(u, v)


def jac_cov(pr, T, u, v, s_px, s_tilt, s_pan, s_h):
    """2x2 world-xy covariance of the ground point via finite-difference Jacobian."""
    g0 = _ground(pr, T, u, v)
    if g0 is None:
        return None, None
    g0 = g0[:2]
    cols = []
    # pixel u, v
    for du, dv in ((1.0, 0.0), (0.0, 1.0)):
        g = _ground(pr, T, u + du, v + dv)
        if g is None:
            return None, None
        cols.append((g[:2] - g0) / 1.0)
    # tilt (optical X), pan (optical Y): rotate camera in optical frame
    eps = 1e-3
    for Rax in (_rot_x, _rot_y):
        g = _ground(pr, T @ Rax(eps), u, v)
        if g is None:
            return None, None
        cols.append((g[:2] - g0) / eps)
    # height
    Th = T.copy(); Th[2, 3] += eps
    g = _ground(pr, Th, u, v)
    if g is None:
        return None, None
    cols.append((g[:2] - g0) / eps)
    J = np.stack(cols, axis=1)                       # 2x5
    var = np.array([s_px**2, s_px**2, s_tilt**2, s_pan**2, s_h**2])
    C = J @ np.diag(var) @ J.T
    return g0, C


def radial_sigmas(g0, cam_xy, C):
    d = math.hypot(g0[0] - cam_xy[0], g0[1] - cam_xy[1])
    phi = math.atan2(g0[1] - cam_xy[1], g0[0] - cam_xy[0])
    c, s = math.cos(-phi), math.sin(-phi)
    R = np.array([[c, -s], [s, c]])
    Crb = R @ C @ R.T
    return d, math.sqrt(max(Crb[0, 0], 0.0)), math.sqrt(max(Crb[1, 1], 0.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sidecar',
                    default='/media/miftah/backup/landmark_dataset/val')
    ap.add_argument('--n', type=int, default=500)
    ap.add_argument('--out', default='/home/miftah/basbot/fase_gy1_plots')
    ap.add_argument('--sigma_px', type=float, default=3.0)
    ap.add_argument('--sigma_ang_deg', type=float, default=0.5)
    ap.add_argument('--sigma_h_m', type=float, default=0.02)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    s_ang = math.radians(args.sigma_ang_deg)

    rows = []
    with open(os.path.join(args.sidecar, 'gt_metadata.jsonl')) as f:
        for line in f:
            rows.append(json.loads(line))
    rows = rows[:args.n]
    print('frames=%d  input sigma: px=%.1f ang=%.2fdeg h=%.3fm'
          % (len(rows), args.sigma_px, args.sigma_ang_deg, args.sigma_h_m))

    # bucket[(cid, d_int)] -> list of (sigma_range, sigma_cross)
    buckets = {}
    for r in rows:
        cam = r['camera']
        K = np.array([[cam['K_fx_fy_cx_cy'][0], 0, cam['K_fx_fy_cx_cy'][2]],
                      [0, cam['K_fx_fy_cx_cy'][1], cam['K_fx_fy_cx_cy'][3]],
                      [0, 0, 1]])
        W, H = cam['image_wh']
        pr = Projector(K, W, H, max_range_m=30, ground_max_range_m=30)
        T = _T(cam['cam_world_pos'], cam['cam_world_quat'])
        cam_xy = np.array(cam['cam_world_pos'][:2])
        for lm in r['landmarks']:
            cid = _NAME2ID[lm['class']]
            if cid == 3:  # goalpost: use BASE point (forward-project world x,+-1.3)
                pr.set_pose_matrix(T)
                uv, valid = pr._project(np.array([[lm['world_xy'][0],
                                                   lm['world_xy'][1], 0.0]]))
                if not valid[0]:
                    continue
                u, v = float(uv[0, 0]), float(uv[0, 1])
            else:
                u, v = lm['pixel_uv']
            g0, C = jac_cov(pr, T, u, v, args.sigma_px, s_ang, s_ang,
                            args.sigma_h_m)
            if C is None:
                continue
            d, sr, sc = radial_sigmas(g0, cam_xy, C)
            buckets.setdefault((cid, int(d)), []).append((sr, sc))

    # report + fit sigma_range(d) ~ a + b d + c d^2 per class
    print('\n%-14s %4s %9s %9s %6s' %
          ('class', 'd[m]', 'sig_rng', 'sig_crs', 'n'))
    per_class = {}
    for (cid, d) in sorted(buckets):
        arr = np.array(buckets[(cid, d)])
        sr, sc = np.median(arr[:, 0]), np.median(arr[:, 1])
        print('%-14s %4d %9.4f %9.4f %6d' % (_NAMES[cid], d, sr, sc, len(arr)))
        per_class.setdefault(cid, []).append((d, sr, sc))

    print('\n== proposed max_range per class (sigma_range <= %.2f m) ==' %
          _USABLE_SIGMA)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    csv = os.path.join(args.out, 'covariance_model.csv')
    fout = open(csv, 'w')
    fout.write('class,d,sigma_range,sigma_cross,n\n')
    model = {}
    for cid in sorted(per_class):
        pts = np.array(per_class[cid])
        d, sr, sc = pts[:, 0], pts[:, 1], pts[:, 2]
        # polynomial fit sigma_range(d) (deg2), guard for few points
        deg = 2 if len(d) >= 3 else 1
        cr = np.polyfit(d, sr, deg)
        cc = np.polyfit(d, sc, deg)
        # max_range = smallest bucket-d where sigma_range exceeds bound
        over = d[sr > _USABLE_SIGMA]
        mr = float(over.min()) if over.size else float(d.max() + 1)
        model[cid] = dict(range_poly=cr.tolist(), cross_poly=cc.tolist(),
                          max_range=mr)
        print('  %-14s max_range=%.1f m  (sig_range@far=%.3f)'
              % (_NAMES[cid], mr, sr[-1]))
        ax[0].plot(d, sr, 'o-', label=_NAMES[cid])
        ax[1].plot(d, sc, 'o-', label=_NAMES[cid])
        for row in per_class[cid]:
            fout.write('%s,%d,%.4f,%.4f,%d\n'
                       % (_NAMES[cid], int(row[0]), row[1], row[2],
                          len(buckets[(cid, int(row[0]))])))
    fout.close()
    for a, t in zip(ax, ('sigma_range (radial) [m]', 'sigma_cross (tangential) [m]')):
        a.axhline(_USABLE_SIGMA, ls='--', c='k', alpha=0.4)
        a.set_xlabel('distance [m]'); a.set_ylabel(t); a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    png = os.path.join(args.out, 'covariance_model.png')
    fig.savefig(png, dpi=110)
    with open(os.path.join(args.out, 'covariance_model.json'), 'w') as f:
        json.dump({'input_sigma': dict(px=args.sigma_px,
                                       ang_deg=args.sigma_ang_deg,
                                       h_m=args.sigma_h_m),
                   'per_class': {_NAMES[k]: v for k, v in model.items()}},
                  f, indent=1)
    print('\nsaved %s, %s, covariance_model.json' % (csv, png))


if __name__ == '__main__':
    main()
