#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 4.4 — VALID fix rate as a function of gaze (head_pan, head_tilt).

GATE 4's fix-rate numbers were all measured on the frozen-head dataset
(head_pan=0, head_tilt=-15deg for every frame) — i.e. the robot staring at the
horizon, the most landmark-rich gaze. In gameplay the neck tracks the ball: tilt
pitches down, pan swings, and most frames hold only nearby grass + a line or two.
The whole "gap between fixes is small at 15 fps" argument rests on how the fix
rate holds up under THAT gaze, which the dataset never sampled.

This is a *geometric visibility* question, so it needs no Webots and no image
rendering: the ONE shared camera model (landmark_geometry.Projector) tells us
exactly which landmarks fall in the FOV / range / occlusion for any (base pose,
head_pan, head_tilt) — the same projection that made the labels. We sweep a grid
of gaze angles, and for each, over a field-uniform sample of base poses, run the
same VALID-fix test as GATE 4 (CLAP solve -> WLS -> conditioning + accuracy up to
the 180deg mirror). The result is a heatmap: valid fix rate vs where the head looks.

Output feeds two decisions:
  * TAHAP 4 — is the no-odom per-frame-fix architecture still viable once the head
    looks down to track the ball, or must line-scan / active-vision carry the gap?
  * TAHAP 6 — the map tells the neck FSM which gaze directions actually buy a fix,
    so re-localization gaze is chosen, not guessed.
"""
import argparse
import json
import math
import os

import numpy as np

from landmark_geometry import error_model
from landmark_geometry.projection import Projector
from landmark_geometry.field_landmarks import (
    build_line_intersections, build_goalposts, build_center_circle)
from landmark_localization.mhl import GeometricLocalizer, Obs, build_map
from landmark_localization.pose_solve import solve_pose_wls, mirror_pose

_NAME2ID = {'L': 0, 'T': 1, 'X': 2, 'goalpost': 3, 'center_circle': 4}


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def calibrate_base_z(sidecar_dir):
    """Find base_z so the shared Projector reproduces the sidecar camera height.

    cam_z is affine in base_z (slope 1), so one matched frame pins it exactly.
    """
    with open(os.path.join(sidecar_dir, 'gt_metadata.jsonl')) as f:
        r = json.loads(f.readline())
    cam = r['camera']
    K = np.array([[cam['K_fx_fy_cx_cy'][0], 0, cam['K_fx_fy_cx_cy'][2]],
                  [0, cam['K_fx_fy_cx_cy'][1], cam['K_fx_fy_cx_cy'][3]],
                  [0, 0, 1.0]])
    w, h = cam['image_wh']
    gp = r['gt_robot_pose']
    prj = Projector(K, w, h)
    prj.set_pose(gp['x'], gp['y'], 0.0, math.radians(gp['yaw_deg']),
                 math.radians(cam['head_pan_deg']),
                 math.radians(cam['head_tilt_deg']))
    cam_z0 = float(prj._cam_pos[2])
    base_z = float(cam['cam_world_pos'][2]) - cam_z0
    return K, int(w), int(h), base_z


def build_obs(prj, junc, posts, circ, bx, by, yaw):
    """Visible landmarks -> base-frame observations (class_id, b) via projection."""
    dets = prj.project_all(junc, posts, circ)
    Rt = _R(-yaw)
    obs = []
    for d in dets:
        w = _LABEL_XY.get(d.label)
        if w is None:
            continue
        b = Rt @ (np.array(w) - np.array([bx, by]))
        obs.append(Obs(d.class_id, b))
    return obs


_LABEL_XY = {}


def valid_fix(loc, map_w, map_cls, obs, gt_pose,
              pos_thr, yaw_thr, cond_max):
    """Return (solved, valid) for one frame's observations (mirror-aware)."""
    if len(obs) < 2:
        return False, False
    out = loc.localize(obs)
    if out is None:
        return False, False
    phyp, _v, _n = out
    seed = np.array([phyp.x, phyp.y, phyp.yaw])
    px, py, sy = seed
    Rinv = _R(-sy)
    bs, ws, covs = [], [], []
    for o in obs:
        same = np.nonzero(map_cls == o.class_id)[0]
        if same.size == 0:
            continue
        pred = (map_w[same] - np.array([px, py])) @ Rinv.T
        j = int(np.argmin(np.linalg.norm(pred - o.b, axis=1)))
        if np.linalg.norm(pred[j] - o.b) <= 0.60:
            bs.append(o.b); ws.append(map_w[same[j]])
            covs.append(np.array(error_model.cov_2x2(float(o.b[0]),
                                 float(o.b[1]))).reshape(2, 2))
    fit = solve_pose_wls(bs, covs, ws, seed)
    if fit is None:
        return False, False
    rx, ry, gyaw = gt_pose
    gm = mirror_pose(np.array([rx, ry, gyaw]))
    ep = min(float(np.hypot(fit.pose[0]-rx, fit.pose[1]-ry)),
             float(np.hypot(fit.pose[0]-gm[0], fit.pose[1]-gm[1])))
    if float(np.hypot(fit.pose[0]-rx, fit.pose[1]-ry)) <= \
            float(np.hypot(fit.pose[0]-gm[0], fit.pose[1]-gm[1])):
        ey = abs(_wrap(fit.pose[2]-gyaw))
    else:
        ey = abs(_wrap(fit.pose[2]-gm[2]))
    ok = (fit.cond <= cond_max) and (ep <= pos_thr) and (ey <= yaw_thr)
    return True, ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sidecar', default='/media/miftah/backup/landmark_dataset/val')
    ap.add_argument('--out', default='/home/miftah/basbot/fase_gy1_plots')
    ap.add_argument('--n_pose', type=int, default=250)
    ap.add_argument('--pan_steps', type=int, default=9)
    ap.add_argument('--tilt_steps', type=int, default=10)
    ap.add_argument('--pos_thr_m', type=float, default=0.30)
    ap.add_argument('--yaw_thr_deg', type=float, default=10.0)
    ap.add_argument('--cond_max', type=float, default=1.0e4)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    K, W, H, base_z = calibrate_base_z(args.sidecar)
    print('calibrated base_z=%.4f m' % base_z)

    junc = build_line_intersections()
    posts = build_goalposts()
    circ = build_center_circle()
    for j in junc:
        _LABEL_XY[j.label] = (j.x, j.y)
    for p in posts:
        _LABEL_XY[p.label] = (p.x, p.y)
    for c in circ:
        _LABEL_XY[c.label] = (c.cx, c.cy)

    fmap = build_map()
    map_w = np.array([m.w for m in fmap])
    map_cls = np.array([m.class_id for m in fmap])
    loc = GeometricLocalizer(field_map=fmap)
    prj = Projector(K, W, H, max_range_m=9.0, ground_max_range_m=6.0)

    rng = np.random.default_rng(args.seed)
    # field-uniform base poses (match dataset extents), shared across all gazes
    poses = np.column_stack([
        rng.uniform(-4.3, 4.3, args.n_pose),
        rng.uniform(-2.8, 2.8, args.n_pose),
        rng.uniform(-math.pi, math.pi, args.n_pose)])

    pans = np.linspace(-60, 60, args.pan_steps)
    tilts = np.linspace(-60, 15, args.tilt_steps)   # down-look .. slightly up
    yaw_thr = math.radians(args.yaw_thr_deg)

    valid_grid = np.zeros((len(tilts), len(pans)))
    solved_grid = np.zeros((len(tilts), len(pans)))
    for ti, tilt in enumerate(tilts):
        for pi, pan in enumerate(pans):
            nv = ns = 0
            for (bx, by, yaw) in poses:
                prj.set_pose(bx, by, base_z, yaw,
                             math.radians(pan), math.radians(tilt))
                obs = build_obs(prj, junc, posts, circ, bx, by, yaw)
                solved, ok = valid_fix(loc, map_w, map_cls, obs,
                                       (bx, by, yaw), args.pos_thr_m,
                                       yaw_thr, args.cond_max)
                ns += solved; nv += ok
            valid_grid[ti, pi] = 100.0 * nv / args.n_pose
            solved_grid[ti, pi] = 100.0 * ns / args.n_pose
        print('tilt=%+5.1f deg : valid-fix %%= %s' %
              (tilt, ' '.join('%4.0f' % v for v in valid_grid[ti])))

    np.savez(os.path.join(args.out, 'gaze_fix_map.npz'),
             pans=pans, tilts=tilts, valid=valid_grid, solved=solved_grid)

    # representative gazes for the report
    def at(tilt_deg):
        ti = int(np.argmin(np.abs(tilts - tilt_deg)))
        return float(np.mean(valid_grid[ti]))
    print('\n-- valid-fix rate averaged over pan, by tilt --')
    for t in (-15, -30, -45, -55):
        print('  tilt=%+d deg (%s): %.1f%%' %
              (t, 'horizon/current dataset' if t == -15 else 'ball-track gaze',
               at(t)))

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 6))
        im = ax.imshow(valid_grid, origin='lower', aspect='auto', cmap='viridis',
                       extent=[pans[0], pans[-1], tilts[0], tilts[-1]],
                       vmin=0, vmax=100)
        ax.set_xlabel('head_pan (deg)'); ax.set_ylabel('head_tilt (deg)')
        ax.set_title('VALID per-frame fix rate vs gaze (clean detection)\n'
                     'dataset only sampled the dashed line (tilt=-15, pan=0)')
        ax.axhline(-15, ls='--', c='w', lw=1); ax.axvline(0, ls='--', c='w', lw=1)
        ax.axhspan(-60, -35, color='red', alpha=0.12)
        ax.text(0, -50, 'ball-tracking\ngaze band', color='w', ha='center',
                fontsize=9, weight='bold')
        cb = fig.colorbar(im); cb.set_label('% frames with a valid fix')
        plt.tight_layout()
        plt.savefig(os.path.join(args.out, 'gaze_fix_map.png'), dpi=110)
        print('saved %s/gaze_fix_map.png' % args.out)
    except Exception as e:
        print('plot skipped: %r' % e)


if __name__ == '__main__':
    main()
