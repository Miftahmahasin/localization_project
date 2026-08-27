#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 4.3 (GATE 4) — VALID per-frame fix rate against the GT (no odom).

The architecture of TAHAP 4 stands on one claim: a *prior-free* per-frame
geometric fix (CLAP Eq.5) is available often enough that body odometry is not
needed to bridge glimpses. The plan is explicit that the number that matters is
NOT "how many frames have >=2 landmarks" but the **valid fix rate**: the fraction
of frames that yield a pose which is

  * solved      — CLAP finds a consensus from the observations alone,
  * unique       — its 180-degree field-symmetry twin does NOT explain the frame
                   equally well (else the fix is two-fold ambiguous),
  * well-conditioned — the WLS information matrix is not near-singular (two
                   landmarks at nearly the same bearing "count as >=2" but pin
                   down nothing), and
  * accurate    — position & yaw error below threshold vs the sidecar GT pose.

Two landmarks at a narrow bearing separation, or a pose that its mirror explains
just as well, are the two ways a naive ">=2 landmark" count lies; both are caught
here. Everything runs offline off the GT sidecar (== poses.csv per-frame GT), no
Webots, no odom.

Detector degradation is CORRELATED, not independent (the plan's standing debt from
TAHAP 1): drop probability rises with range, whole classes can collapse in a frame
(junctions vanishing at imgsz 320), and a frame can be globally "bad". Independent
dropout is optimistic — real misses cluster.

Reported (the GATE 4 headline):
  * valid fix rate, and the breakdown of WHY frames fail (single/none, unsolved,
    ill-conditioned, ambiguous-mirror, inaccurate);
  * position/yaw error median & p95 over solved frames;
  * information-matrix condition-number distribution;
  * class composition of the valid fixes: what fraction still fix with goalposts
    + centre circle ALONE (i.e. survive junctions disappearing at 320).
"""
import argparse
import json
import math
import os
from collections import defaultdict

import numpy as np

from landmark_geometry import error_model
from landmark_localization.mhl import GeometricLocalizer, Obs, build_map
from landmark_localization.pose_solve import solve_pose_wls, mirror_pose

_NAME2ID = {'L': 0, 'T': 1, 'X': 2, 'goalpost': 3, 'center_circle': 4}
_NAMES = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}
_LTX = (0, 1, 2)
_JUNCTION = {0, 1, 2}
_NONJUNCTION = {3, 4}          # goalpost + centre circle: survive imgsz 320


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _associate_at(pose, obs_b, obs_cls, map_w, map_cls, radius=0.60):
    """Nearest same-class map landmark to each obs, predicted through ``pose``.

    Returns aligned lists (bs, covs_idx, ws, classes) of matched observations.
    Prior-free association for the WLS refine — pose comes from CLAP, not odom.
    """
    px, py, yaw = pose
    Rinv = _R(-yaw)
    bs, ws, cls, oi = [], [], [], []
    for k, (b, c) in enumerate(zip(obs_b, obs_cls)):
        same = np.nonzero(map_cls == c)[0]
        if same.size == 0:
            continue
        # predicted base-frame position of each same-class map landmark
        pred = (map_w[same] - np.array([px, py])) @ Rinv.T
        d = np.linalg.norm(pred - b, axis=1)
        j = int(np.argmin(d))
        if d[j] <= radius:
            bs.append(b)
            ws.append(map_w[same[j]])
            cls.append(c)
            oi.append(k)
    return bs, ws, cls, oi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sidecar', default='/media/miftah/backup/landmark_dataset/val')
    ap.add_argument('--n', type=int, default=1200)
    ap.add_argument('--out', default='/home/miftah/basbot/fase_gy1_plots')
    # detector degradation (correlated)
    ap.add_argument('--recall', type=float, default=1.0)
    ap.add_argument('--dist_corr', type=float, default=0.0,
                    help='0=uniform recall; >0 drops farther landmarks more')
    ap.add_argument('--class_collapse', type=float, default=0.0,
                    help='per-frame prob a whole class is dropped')
    ap.add_argument('--frame_bad_prob', type=float, default=0.0)
    ap.add_argument('--frame_bad_mult', type=float, default=0.4)
    ap.add_argument('--confuse', type=float, default=0.0)
    ap.add_argument('--fp_per_frame', type=int, default=0)
    ap.add_argument('--pixel_noise_px', type=float, default=0.0)
    # valid-fix thresholds
    ap.add_argument('--pos_thr_m', type=float, default=0.30)
    ap.add_argument('--yaw_thr_deg', type=float, default=10.0)
    ap.add_argument('--cond_max', type=float, default=1.0e4)
    ap.add_argument('--dual_ratio', type=float, default=1.5,
                    help='mirror is ambiguous if its resid <= ratio*primary resid')
    ap.add_argument('--dual_min_sep_m', type=float, default=1.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--a2', action='store_true',
                    help='TAHAP A2: class-composition regimes (clean labels) '
                         'through the FULL path so mirror-ambiguity is measured '
                         'for every regime, incl. no-junction')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = []
    with open(os.path.join(args.sidecar, 'gt_metadata.jsonl')) as f:
        for line in f:
            rows.append(json.loads(line))
    rows = rows[:args.n]

    fmap = build_map()
    map_w = np.array([m.w for m in fmap])
    map_cls = np.array([m.class_id for m in fmap])
    loc = GeometricLocalizer(field_map=fmap)

    def run(recall, dist_corr, class_collapse, frame_bad_prob, frame_bad_mult,
            confuse, fp, pixnoise, seed, keep_classes=None):
        rng = np.random.default_rng(seed)
        pos_thr = args.pos_thr_m
        yaw_thr = math.radians(args.yaw_thr_deg)
        m = dict(frames=0, ge2=0, solved=0, valid=0,
                 fail_single=0, fail_unsolved=0, fail_illcond=0,
                 fail_ambig=0, fail_inacc=0,
                 fix_needs_junction=0, fix_nonjunction_only=0)
        errs, yaws, conds = [], [], []
        for r in rows:
            gp = r['gt_robot_pose']
            rx, ry = float(gp['x']), float(gp['y'])
            gyaw = math.radians(gp['yaw_deg'])
            Rt = _R(-gyaw)
            frame_mult = frame_bad_mult if rng.random() < frame_bad_prob else 1.0
            dropped_class = set()
            for cid in set(_NAME2ID[l['class']] for l in r['landmarks']):
                if rng.random() < class_collapse:
                    dropped_class.add(cid)
            obs_b, obs_cls = [], []
            for lm in r['landmarks']:
                cid = _NAME2ID[lm['class']]
                if keep_classes is not None and cid not in keep_classes:
                    continue                       # A2 class-composition filter
                w = np.asarray(lm['world_xy'], float)
                b = Rt @ (w - np.array([rx, ry]))
                d = float(np.hypot(b[0], b[1]))
                if d > error_model.max_range(cid):
                    continue
                if cid in dropped_class:
                    continue
                p_keep = recall * (1.0 - dist_corr * min(d, 6.0) / 6.0) * frame_mult
                if rng.random() > max(0.03, p_keep):
                    continue
                ocls = cid
                if cid in _LTX and rng.random() < confuse:
                    ocls = int(rng.choice([x for x in _LTX if x != cid]))
                if pixnoise > 0:
                    sr, _sc = error_model.ground_sigmas(d)
                    b = b + rng.normal(0, sr * pixnoise / 3.0, 2)
                obs_b.append(b)
                obs_cls.append(ocls)
            for _ in range(fp):
                ang = rng.uniform(-0.9, 0.9)
                rr = rng.uniform(0.5, 6.0)
                obs_b.append(np.array([rr * math.cos(ang), rr * math.sin(ang)]))
                obs_cls.append(int(rng.integers(0, 5)))

            m['frames'] += 1
            if len(obs_b) >= 2:
                m['ge2'] += 1
            if len(obs_b) < 2:
                m['fail_single'] += 1
                continue

            out = loc.localize([Obs(c, b) for b, c in zip(obs_b, obs_cls)])
            if out is None:
                m['fail_unsolved'] += 1
                continue
            phyp, _votes, _nh = out
            seed_pose = np.array([phyp.x, phyp.y, phyp.yaw])

            bs, ws, cls, _oi = _associate_at(
                seed_pose, obs_b, obs_cls, map_w, map_cls)
            covs = [np.array(error_model.cov_2x2(float(b[0]), float(b[1]))
                             ).reshape(2, 2) for b in bs]
            fit = solve_pose_wls(bs, covs, ws, seed_pose)
            if fit is None:
                m['fail_unsolved'] += 1
                continue
            m['solved'] += 1
            conds.append(fit.cond)

            # mirror-ambiguity test: refine from the 180-deg twin, compare residual
            mp = mirror_pose(fit.pose)
            mbs, mws, _mc, _mi = _associate_at(mp, obs_b, obs_cls, map_w, map_cls)
            ambiguous = False
            if len(mbs) >= 2:
                mcovs = [np.array(error_model.cov_2x2(float(b[0]), float(b[1]))
                                  ).reshape(2, 2) for b in mbs]
                mfit = solve_pose_wls(mbs, mcovs, mws, mp)
                if mfit is not None:
                    sep = float(np.hypot(mfit.pose[0] - fit.pose[0],
                                         mfit.pose[1] - fit.pose[1]))
                    if (mfit.mean_resid_m <= args.dual_ratio * max(fit.mean_resid_m, 1e-4)
                            and sep >= args.dual_min_sep_m):
                        ambiguous = True

            # Score accuracy UP TO the 180-deg field symmetry: a prior-free frame
            # cannot pick between the true pose and its mirror, and it is not
            # TAHAP 4's job to (the EKF prior does, TAHAP 5). Error is to the
            # nearer of {true, mirror(true)}; the two-fold ambiguity is tracked
            # separately as the load TAHAP 5's prior must carry, not a fix failure.
            gm = mirror_pose(np.array([rx, ry, gyaw]))
            ep_t = float(np.hypot(fit.pose[0] - rx, fit.pose[1] - ry))
            ep_m = float(np.hypot(fit.pose[0] - gm[0], fit.pose[1] - gm[1]))
            if ep_t <= ep_m:
                ep, ey = ep_t, abs(_wrap(fit.pose[2] - gyaw))
            else:
                ep, ey = ep_m, abs(_wrap(fit.pose[2] - gm[2]))
            errs.append(ep)
            yaws.append(math.degrees(ey))
            if ambiguous:
                m['fail_ambig'] += 1          # diagnostic only (see note above)

            if fit.cond > args.cond_max:
                m['fail_illcond'] += 1
                continue
            if ep > pos_thr or ey > yaw_thr:
                m['fail_inacc'] += 1
                continue
            m['valid'] += 1
            if any(c in _JUNCTION for c in cls):
                m['fix_needs_junction'] += 1
            if sum(1 for c in cls if c in _NONJUNCTION) >= 2:
                m['fix_nonjunction_only'] += 1
        return m, errs, yaws, conds

    def pct(a, b):
        return 100.0 * a / max(b, 1)

    def report(tag, m, errs, yaws, conds):
        fr = max(m['frames'], 1)
        print('\n== %s ==' % tag)
        print('  frames=%d  >=2 obs=%.1f%%  solved=%.1f%%  VALID FIX=%.1f%%' %
              (m['frames'], pct(m['ge2'], fr), pct(m['solved'], fr),
               pct(m['valid'], fr)))
        print('  fail: single/none=%.1f%%  unsolved=%.1f%%  ill-cond=%.1f%%  '
              'inaccurate=%.1f%%' %
              (pct(m['fail_single'], fr), pct(m['fail_unsolved'], fr),
               pct(m['fail_illcond'], fr), pct(m['fail_inacc'], fr)))
        print('  [diag] 180-deg mirror-ambiguous=%.1f%% of frames '
              '(load TAHAP 5 prior must break, not a fix failure)' %
              pct(m['fail_ambig'], fr))
        if errs:
            errs = np.array(errs); yaws = np.array(yaws); conds = np.array(conds)
            print('  err_pos  median=%.3f m  p95=%.3f m' %
                  (np.median(errs), np.percentile(errs, 95)))
            print('  err_yaw  median=%.2f deg p95=%.2f deg' %
                  (np.median(yaws), np.percentile(yaws, 95)))
            print('  cond(info) median=%.0f  p95=%.0f' %
                  (np.median(conds), np.percentile(conds, 95)))
        vf = max(m['valid'], 1)
        print('  of VALID fixes: rely on >=1 junction=%.1f%%  '
              'doable with goalpost+circle alone=%.1f%%' %
              (pct(m['fix_needs_junction'], vf), pct(m['fix_nonjunction_only'], vf)))

    if args.a2:
        # TAHAP A2 — class-composition sufficiency, CLEAN labels (isolate geometry,
        # not detector recall). Note: "goalpost+circle only" IS "L/T/X removed"
        # (imgsz-320 junction collapse), so the plan's regimes 2 & 3 coincide.
        # Every regime runs the FULL path -> mirror-ambiguity (fail_ambig) is the
        # "does anti-mirror weaken without junctions?" measurement.
        regimes = [
            ('all classes',          {0, 1, 2, 3, 4}),
            ('goalpost+circle only', {3, 4}),
            ('junctions only',       {0, 1, 2}),
        ]
        csv = os.path.join(args.out, 'a2_class_composition.csv')
        with open(csv, 'w') as fout:
            fout.write('regime,ge2,solved,valid_fix,fail_single,fail_unsolved,'
                       'fail_illcond,fail_ambig,fail_inacc,err_pos_med,err_pos_p95,'
                       'err_yaw_med,cond_med,cond_p95,fix_needs_junction,'
                       'fix_nonjunction_only\n')
            for name, keep in regimes:
                res = run(1.0, 0.0, 0.0, 0.0, args.frame_bad_mult, 0.0, 0, 0.0,
                          args.seed, keep_classes=keep)
                report('A2: %s (clean labels)' % name, *res)
                m, errs, yaws, conds = res
                fr = max(m['frames'], 1); vf = max(m['valid'], 1)
                em = np.median(errs) if errs else float('nan')
                ep95 = np.percentile(errs, 95) if errs else float('nan')
                ym = np.median(yaws) if yaws else float('nan')
                cm = np.median(conds) if conds else float('nan')
                cp95 = np.percentile(conds, 95) if conds else float('nan')
                fout.write('%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,'
                           '%.3f,%.3f,%.2f,%.0f,%.0f,%.3f,%.3f\n' %
                           (name, m['ge2'] / fr, m['solved'] / fr, m['valid'] / fr,
                            m['fail_single'] / fr, m['fail_unsolved'] / fr,
                            m['fail_illcond'] / fr, m['fail_ambig'] / fr,
                            m['fail_inacc'] / fr, em, ep95, ym, cm, cp95,
                            m['fix_needs_junction'] / vf,
                            m['fix_nonjunction_only'] / vf))
        print('\nsaved %s' % csv)
        return

    if not args.sweep:
        res = run(args.recall, args.dist_corr, args.class_collapse,
                  args.frame_bad_prob, args.frame_bad_mult, args.confuse,
                  args.fp_per_frame, args.pixel_noise_px, args.seed)
        report('single (recall=%.2f dist_corr=%.2f collapse=%.2f)' %
               (args.recall, args.dist_corr, args.class_collapse), *res)
        return

    scen = [
        ('clean (GT labels)', dict(recall=1.0)),
        ('recall=0.7 indep', dict(recall=0.7)),
        ('recall=0.53 indep', dict(recall=0.53)),
        ('recall=0.53 +distcorr', dict(recall=0.53, dist_corr=0.5)),
        ('recall=0.53 +collapse.2', dict(recall=0.53, dist_corr=0.5,
                                         class_collapse=0.2)),
        ('recall=0.53 correlated', dict(recall=0.53, dist_corr=0.5,
                                        class_collapse=0.2, frame_bad_prob=0.2)),
        ('confuse=0.2', dict(recall=0.7, confuse=0.2)),
        ('fp=2/frame', dict(recall=0.7, fp_per_frame=2)),
        ('junctions gone (L/T/X=0)', dict(recall=0.0, dist_corr=0.0)),
    ]
    csv = os.path.join(args.out, 'fix_rate_eval.csv')
    with open(csv, 'w') as fout:
        fout.write('scenario,ge2,solved,valid_fix,fail_single,fail_unsolved,'
                   'fail_illcond,fail_ambig,fail_inacc,err_pos_med,err_pos_p95,'
                   'err_yaw_med,cond_med,fix_needs_junction,fix_nonjunction_only\n')
        for name, ov in scen:
            p = dict(recall=1.0, dist_corr=0.0, class_collapse=0.0,
                     frame_bad_prob=0.0, frame_bad_mult=args.frame_bad_mult,
                     confuse=0.0, fp_per_frame=0, pixel_noise_px=0.0)
            p.update(ov)
            # 'junctions gone' means junction classes never detected
            if name.startswith('junctions gone'):
                res = run_junctions_gone(rows, fmap, map_w, map_cls, loc, args)
            else:
                res = run(p['recall'], p['dist_corr'], p['class_collapse'],
                          p['frame_bad_prob'], p['frame_bad_mult'], p['confuse'],
                          p['fp_per_frame'], p['pixel_noise_px'], args.seed)
            report(name, *res)
            m, errs, yaws, conds = res
            fr = max(m['frames'], 1); vf = max(m['valid'], 1)
            em = np.median(errs) if errs else float('nan')
            ep95 = np.percentile(errs, 95) if errs else float('nan')
            ym = np.median(yaws) if yaws else float('nan')
            cm = np.median(conds) if conds else float('nan')
            fout.write('%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,'
                       '%.3f,%.3f,%.2f,%.0f,%.3f,%.3f\n' %
                       (name, m['ge2']/fr, m['solved']/fr, m['valid']/fr,
                        m['fail_single']/fr, m['fail_unsolved']/fr,
                        m['fail_illcond']/fr, m['fail_ambig']/fr,
                        m['fail_inacc']/fr, em, ep95, ym, cm,
                        m['fix_needs_junction']/vf, m['fix_nonjunction_only']/vf))
    print('\nsaved %s' % csv)


def run_junctions_gone(rows, fmap, map_w, map_cls, loc, args):
    """Degradation scenario the plan flags as REAL at imgsz 320: junction classes
    (L/T/X) never detected. Does the fix survive on goalposts + centre circle?"""
    import numpy as _np
    from landmark_localization.mhl import Obs as _Obs
    rng = _np.random.default_rng(args.seed)
    pos_thr = args.pos_thr_m
    yaw_thr = math.radians(args.yaw_thr_deg)
    m = dict(frames=0, ge2=0, solved=0, valid=0, fail_single=0, fail_unsolved=0,
             fail_illcond=0, fail_ambig=0, fail_inacc=0,
             fix_needs_junction=0, fix_nonjunction_only=0)
    errs, yaws, conds = [], [], []
    for r in rows:
        gp = r['gt_robot_pose']
        rx, ry = float(gp['x']), float(gp['y'])
        gyaw = math.radians(gp['yaw_deg'])
        Rt = _R(-gyaw)
        obs_b, obs_cls = [], []
        for lm in r['landmarks']:
            cid = _NAME2ID[lm['class']]
            if cid in _JUNCTION:            # junctions gone
                continue
            w = _np.asarray(lm['world_xy'], float)
            b = Rt @ (w - _np.array([rx, ry]))
            if float(_np.hypot(b[0], b[1])) > error_model.max_range(cid):
                continue
            obs_b.append(b); obs_cls.append(cid)
        m['frames'] += 1
        if len(obs_b) >= 2:
            m['ge2'] += 1
        if len(obs_b) < 2:
            m['fail_single'] += 1
            continue
        out = loc.localize([_Obs(c, b) for b, c in zip(obs_b, obs_cls)])
        if out is None:
            m['fail_unsolved'] += 1
            continue
        phyp, _v, _n = out
        seed_pose = _np.array([phyp.x, phyp.y, phyp.yaw])
        bs, ws, cls, _oi = _associate_at(seed_pose, obs_b, obs_cls, map_w, map_cls)
        covs = [_np.array(error_model.cov_2x2(float(b[0]), float(b[1]))
                          ).reshape(2, 2) for b in bs]
        fit = solve_pose_wls(bs, covs, ws, seed_pose)
        if fit is None:
            m['fail_unsolved'] += 1
            continue
        m['solved'] += 1
        conds.append(fit.cond)
        ep = float(_np.hypot(fit.pose[0] - rx, fit.pose[1] - ry))
        ey = abs(_wrap(fit.pose[2] - gyaw))
        errs.append(ep); yaws.append(math.degrees(ey))
        if fit.cond > args.cond_max:
            m['fail_illcond'] += 1; continue
        if ep > pos_thr or ey > yaw_thr:
            m['fail_inacc'] += 1; continue
        m['valid'] += 1
        m['fix_nonjunction_only'] += 1
    return m, errs, yaws, conds


if __name__ == '__main__':
    main()
