#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 3 (GATE 3) — measure data-association quality against the GT sidecar.

For every frame we KNOW, from the sidecar, both the true label of each landmark
(hence the true map index) and the GT robot pose. We can therefore score the
associator directly and answer the question the plan asks: **what is the
mis-association rate**, and how does it hold up as (a) the EKF prior drifts,
(b) the detector confuses classes / injects false positives / mislocalizes
pixels?

Pipeline per frame (no Webots, deterministic):
  1. Build the true base-frame observations from the sidecar: for each labelled
     landmark, b = R(-yaw_gt) (world_xy - robot_xy_gt). This is the exact
     projection (TAHAP 1 Part A = 0.0000 m), so association error here is the
     association logic's own, not the camera model's.
  2. Attach the TAHAP 2 anisotropic covariance (error_model.cov_2x2).
  3. Degrade: recall dropout, class confusion (L<->T<->X), false positives
     (random plausible ground points), pixel noise (-> metric noise on b).
  4. Feed a PERTURBED prior pose (GT + gaussian) with a matching 3x3 P, the way
     the real EKF prior would be wrong.
  5. Associate, then compare each real observation's assigned map landmark to the
     true one (map landmark nearest the sidecar world_xy — exact by construction).

Metrics (the GATE 3 report):
  * mis-assoc rate = wrong-map / associated-real          (the headline number)
  * recall(assoc)  = correctly-associated-real / real     (also gated-out reals)
  * FP acceptance  = false positives that got associated / injected FPs
  * gated-out rate = reals rejected by the gate / real
  * mode + RANSAC usage, per-class mis-assoc breakdown.
"""
import argparse
import json
import math
import os
from collections import defaultdict

import numpy as np

from landmark_geometry import error_model
from landmark_localization.association import DataAssociator, AObs
from landmark_localization.mhl import build_map

_NAME2ID = {'L': 0, 'T': 1, 'X': 2, 'goalpost': 3, 'center_circle': 4}
_NAMES = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}
_LTX = (0, 1, 2)


def _group(s):
    """'LTX'->{0,1,2}, 'LT'->{0,1} (X distinctive), 'none'->{} (pure by-type)."""
    m = {'L': 0, 'T': 1, 'X': 2}
    return frozenset(m[c] for c in s.upper() if c in m)


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _true_map_index(map_w, map_cls, world_xy, cid):
    """Index of the SAME-CLASS map landmark coincident with this sidecar landmark.

    Class matters: the center X-mark and the center_circle share the point (0,0),
    so a class-blind nearest search would mislabel the truth of the center mark.
    """
    same = np.nonzero(map_cls == cid)[0]
    if same.size == 0:
        return -1
    d = np.linalg.norm(map_w[same] - np.asarray(world_xy, float), axis=1)
    k = int(np.argmin(d))
    return int(same[k]) if d[k] < 0.05 else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sidecar',
                    default='/media/miftah/backup/landmark_dataset/val')
    ap.add_argument('--n', type=int, default=1000)
    ap.add_argument('--out', default='/home/miftah/basbot/fase_gy1_plots')
    # prior (EKF) error
    ap.add_argument('--prior_pos_sigma', type=float, default=0.30)
    ap.add_argument('--prior_yaw_sigma_deg', type=float, default=8.0)
    # detector degradation
    ap.add_argument('--recall', type=float, default=1.0)
    ap.add_argument('--confuse', type=float, default=0.0)
    ap.add_argument('--fp_per_frame', type=int, default=0)
    ap.add_argument('--pixel_noise_px', type=float, default=0.0)
    ap.add_argument('--gate_chi2', type=float, default=9.21)
    ap.add_argument('--max_range_m', type=float, default=6.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--agnostic_group', default='LTX',
                    help="classes that may cross-match in agnostic mode: "
                         "'LTX' (default) or 'LT' (X distinctive)")
    ap.add_argument('--sweep', action='store_true',
                    help='run the standard degradation sweep and write a table')
    ap.add_argument('--b3', action='store_true',
                    help='B3.2: compare X-agnostic vs X-distinctive across '
                         'class-confusion levels (good prior)')
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
    assoc = DataAssociator(field_map=fmap, gate_chi2=args.gate_chi2,
                           agnostic_group=_group(args.agnostic_group))

    def run(recall, confuse, fp, pixnoise, prior_pos, prior_yaw_deg, seed):
        rng = np.random.default_rng(seed)
        prior_yaw = math.radians(prior_yaw_deg)
        # px pixel noise -> metric noise on b: use ~ (d/focal)*px as a rough map;
        # simpler and geometry-honest: reuse the error model's range sigma scaled.
        m = dict(assoc_total=0, mis=0, correct=0, real_total=0, gated_out=0,
                 fp_total=0, fp_accepted=0, ransac=0, agnostic=0, frames=0)
        per_cls = defaultdict(lambda: [0, 0])   # cid -> [assoc, mis]
        for r in rows:
            gp = r['gt_robot_pose']
            rx, ry = float(gp['x']), float(gp['y'])
            yaw = math.radians(gp['yaw_deg'])
            Rt = _R(-yaw)
            obs, truth = [], []                 # truth[k] = true map idx or -2 (FP)
            for lm in r['landmarks']:
                cid = _NAME2ID[lm['class']]
                w = np.asarray(lm['world_xy'], float)
                b = Rt @ (w - np.array([rx, ry]))
                d = float(np.hypot(b[0], b[1]))
                if d > args.max_range_m or d > error_model.max_range(cid):
                    continue
                if rng.random() > recall:
                    continue
                tj = _true_map_index(map_w, map_cls, w, cid)
                ocls = cid
                if cid in _LTX and rng.random() < confuse:
                    ocls = int(rng.choice([x for x in _LTX if x != cid]))
                if pixnoise > 0:
                    # pixel noise mapped to metric via the range-sigma slope
                    sr, sc = error_model.ground_sigmas(d)
                    b = b + rng.normal(0, sr * pixnoise / 3.0, 2)
                cov = np.array(error_model.cov_2x2(float(b[0]), float(b[1]))
                               ).reshape(2, 2)
                obs.append(AObs(ocls, b, cov, 0.95))
                truth.append(tj)
            # false positives at random plausible ground points
            for _ in range(fp):
                ang = rng.uniform(-0.9, 0.9)
                rr = rng.uniform(0.5, args.max_range_m)
                b = np.array([rr * math.cos(ang), rr * math.sin(ang)])
                cid = int(rng.integers(0, 5))
                cov = np.array(error_model.cov_2x2(float(b[0]), float(b[1]))
                               ).reshape(2, 2)
                obs.append(AObs(cid, b, cov, 0.5))
                truth.append(-2)                # -2 marks a false positive
            if len(obs) < 1:
                continue
            m['frames'] += 1

            # perturbed EKF prior
            pose = (rx + rng.normal(0, prior_pos),
                    ry + rng.normal(0, prior_pos),
                    yaw + rng.normal(0, prior_yaw))
            P = np.diag([prior_pos**2, prior_pos**2, prior_yaw**2])

            res = assoc.associate(obs, pose, P)
            if res.used_ransac:
                m['ransac'] += 1
            if res.mode_agnostic:
                m['agnostic'] += 1
            for a in res.assocs:
                t = truth[a.obs_idx]
                if t == -2:                     # this obs is a false positive
                    m['fp_total'] += 1
                    if a.map_idx >= 0:
                        m['fp_accepted'] += 1
                    continue
                m['real_total'] += 1
                if a.map_idx < 0:
                    m['gated_out'] += 1
                    continue
                m['assoc_total'] += 1
                # a real obs mis-associated if it points to the wrong map landmark
                cid_true = int(fmap[t].class_id) if t >= 0 else -1
                if a.map_idx == t:
                    m['correct'] += 1
                    if t >= 0:
                        per_cls[cid_true][0] += 1
                else:
                    m['mis'] += 1
                    if t >= 0:
                        per_cls[cid_true][0] += 1
                        per_cls[cid_true][1] += 1
        return m, per_cls

    def report(tag, m, per_cls):
        at = max(m['assoc_total'], 1)
        rt = max(m['real_total'], 1)
        ft = max(m['fp_total'], 1)
        fr = max(m['frames'], 1)
        print('\n== %s ==' % tag)
        print('  frames=%d  real_obs=%d  assoc=%d' %
              (m['frames'], m['real_total'], m['assoc_total']))
        print('  MIS-ASSOC rate      : %.3f  (%d / %d associated real)' %
              (m['mis'] / at, m['mis'], m['assoc_total']))
        print('  assoc recall        : %.3f  (correct / real; gated-out=%d)' %
              (m['correct'] / rt, m['gated_out']))
        print('  gated-out rate      : %.3f' % (m['gated_out'] / rt))
        print('  FP acceptance rate  : %.3f  (%d / %d injected FP)' %
              (m['fp_accepted'] / ft, m['fp_accepted'], m['fp_total']))
        print('  mode agnostic / RANSAC per frame: %.2f / %.2f' %
              (m['agnostic'] / fr, m['ransac'] / fr))
        if per_cls:
            print('  per-class mis-assoc:')
            for cid in sorted(k for k in per_cls if k >= 0):
                a, mis = per_cls[cid]
                print('    %-13s %.3f  (%d/%d)' %
                      (_NAMES.get(cid, cid), mis / max(a, 1), mis, a))

    if args.b3:
        # B3.2 — X-agnostic vs X-distinctive across class-confusion, GOOD prior
        # (the tracking/re-entry regime where association matters). Trade-off:
        # X-distinctive should cut mis-assoc when the detector is reliable, but
        # HURT when the detector genuinely confuses X<->L/T (a mislabeled X can no
        # longer be recovered). The numbers decide, not intuition.
        confs = [0.0, 0.1, 0.2, 0.4]
        modes = [('X-agnostic  (LTX)', 'LTX'), ('X-distinctive (LT)', 'LT')]
        csv = os.path.join(args.out, 'b3_class_policy.csv')
        with open(csv, 'w') as fout:
            fout.write('mode,confuse,mis_assoc,assoc_recall,fp_accept,'
                       'agnostic_pf,assoc_n\n')
            for mname, g in modes:
                assoc = DataAssociator(field_map=fmap, gate_chi2=args.gate_chi2,
                                       agnostic_group=_group(g))
                for cf in confs:
                    m, pc = run(1.0, cf, 0, 0.0, args.prior_pos_sigma,
                                args.prior_yaw_sigma_deg, args.seed)
                    report('B3 %s confuse=%.2f' % (mname, cf), m, pc)
                    at = max(m['assoc_total'], 1); rt = max(m['real_total'], 1)
                    ft = max(m['fp_total'], 1); fr = max(m['frames'], 1)
                    fout.write('%s,%.2f,%.4f,%.4f,%.4f,%.3f,%d\n' %
                               (mname.strip(), cf, m['mis'] / at,
                                m['correct'] / rt, m['fp_accepted'] / ft,
                                m['agnostic'] / fr, m['assoc_total']))
        print('\nsaved %s' % csv)
        return

    if not args.sweep:
        m, pc = run(args.recall, args.confuse, args.fp_per_frame,
                    args.pixel_noise_px, args.prior_pos_sigma,
                    args.prior_yaw_sigma_deg, args.seed)
        report('single (recall=%.2f confuse=%.2f fp=%d prior=%.2fm/%.0fdeg)' %
               (args.recall, args.confuse, args.fp_per_frame,
                args.prior_pos_sigma, args.prior_yaw_sigma_deg), m, pc)
        return

    # standard sweep: isolate each degradation axis
    scen = [
        ('clean', dict()),
        ('prior_pos=0.6m', dict(prior_pos=0.60)),
        ('prior_pos=1.0m', dict(prior_pos=1.00)),
        ('prior_yaw=20deg', dict(prior_yaw_deg=20.0)),
        ('recall=0.5', dict(recall=0.5)),
        ('confuse=0.2', dict(confuse=0.2)),
        ('confuse=0.4', dict(confuse=0.4)),
        ('fp=2/frame', dict(fp=2)),
        ('fp=5/frame', dict(fp=5)),
        ('pixnoise=3px', dict(pixnoise=3.0)),
        ('kidnap prior=3m/60deg', dict(prior_pos=3.0, prior_yaw_deg=60.0, fp=1)),
    ]
    base = dict(recall=1.0, confuse=0.0, fp=0, pixnoise=0.0,
                prior_pos=args.prior_pos_sigma,
                prior_yaw_deg=args.prior_yaw_sigma_deg)
    csv = os.path.join(args.out, 'assoc_eval.csv')
    with open(csv, 'w') as fout:
        fout.write('scenario,mis_assoc,assoc_recall,gated_out,fp_accept,'
                   'ransac_pf,agnostic_pf,assoc_n\n')
        for name, ov in scen:
            p = dict(base); p.update(ov)
            m, pc = run(p['recall'], p['confuse'], p['fp'], p['pixnoise'],
                        p['prior_pos'], p['prior_yaw_deg'], args.seed)
            report(name, m, pc)
            at = max(m['assoc_total'], 1); rt = max(m['real_total'], 1)
            ft = max(m['fp_total'], 1); fr = max(m['frames'], 1)
            fout.write('%s,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d\n' %
                       (name, m['mis'] / at, m['correct'] / rt,
                        m['gated_out'] / rt, m['fp_accepted'] / ft,
                        m['ransac'] / fr, m['agnostic'] / fr, m['assoc_total']))
    print('\nsaved %s' % csv)


if __name__ == '__main__':
    main()
