#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-project the labels of an EXISTING dataset at a new calibration, in place,
without re-rendering (pitch_bias / base_z_offset are label-space corrections;
the rendered pixels are unchanged and already correct).

Faithful to the sampler:
  * boxes are rebuilt with the SAME Projector methods + gates + min_emit padding;
  * the dataset's CLEANING decisions are INHERITED — we only re-project the
    landmarks that survived cleaning, identified by matching each clean YOLO
    label to its metadata landmark (same class, near-exact bbox). We never
    re-introduce a culled box and never reconstruct the (lost) clean script.

Outputs are written NON-DESTRUCTIVELY by default:
  labels_recal/            new YOLO labels (survivors only, new boxes)
  gt_metadata_recal.jsonl  metadata re-projected (all landmarks, new uv/bbox)
Pass --apply to replace labels/ + gt_metadata.jsonl (originals moved to
labels_precalib/ + gt_metadata_precalib.jsonl first).

USAGE:
  python3 relabel_recalibrate.py --data-dir /media/miftah/backup/landmark_dataset/train
  python3 relabel_recalibrate.py --data-dir .../train --apply
"""
import argparse
import json
import math
import os
import shutil
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
from landmark_dataset_gen.projection import Projector              # noqa: E402
from landmark_dataset_gen.field_landmarks import (                 # noqa: E402
    build_line_intersections, build_center_circle, build_goalposts, CLASS_NAMES)

GEN_PITCH_BIAS_DEG = -5.0        # calibration the dataset was generated at
MAX_RANGE_M = 9.0
GROUND_MAX_RANGE_M = 7.0
MIN_BOX_PX = 6.0
NAME2ID = {v: k for k, v in CLASS_NAMES.items()}


def field_index():
    idx = {}
    for j in build_line_intersections():
        idx[j.label] = ('junction', j)
    for p in build_goalposts():
        idx[p.label] = ('post', p)
    for c in build_center_circle():
        idx[c.label] = ('circle', c)
    return idx


def recover_k_basez(recs, w, h):
    P = Projector(np.eye(3), w, h)
    P.pitch_bias = math.radians(GEN_PITCH_BIAS_DEG)
    bz = []
    for r in recs:
        c, g = r['camera'], r['gt_robot_pose']
        P.set_pose(g['x'], g['y'], 0.0, math.radians(g['yaw_deg']),
                   math.radians(c['head_pan_deg']), math.radians(c['head_tilt_deg']))
        bz.append(c['cam_world_pos'][2] - P._cam_pos[2])
    base_z = float(np.median(bz))
    xr, ur, yr, vr = [], [], [], []
    for r in recs:
        c, g = r['camera'], r['gt_robot_pose']
        P.set_pose(g['x'], g['y'], base_z, math.radians(g['yaw_deg']),
                   math.radians(c['head_pan_deg']), math.radians(c['head_tilt_deg']))
        for lm in r['landmarks']:
            wx, wy = lm['world_xy']
            cam = P._to_cam(np.array([[wx, wy, 0.0]]))[0]
            if cam[2] <= 1e-3:
                continue
            u, v = lm['pixel_uv']
            xr.append(cam[0] / cam[2]); ur.append(u)
            yr.append(cam[1] / cam[2]); vr.append(v)
    fx, cx = np.polyfit(np.asarray(xr), np.asarray(ur), 1)
    fy, cy = np.polyfit(np.asarray(yr), np.asarray(vr), 1)
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]), base_z


def project_obj(prj, kind, obj):
    if kind == 'junction':
        return prj.project_junction(obj)
    if kind == 'post':
        return prj.project_goalpost(obj)
    return prj.project_center_circle(obj)


def det_to_norm(d, w, h):
    return (0.5 * (d.x1 + d.x2) / w, 0.5 * (d.y1 + d.y2) / h,
            (d.x2 - d.x1) / w, (d.y2 - d.y1) / h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--meta', default='gt_metadata.jsonl')
    ap.add_argument('--labels', default='labels')
    ap.add_argument('--new', type=float, nargs=2, default=[-4.25, -0.035],
                    metavar=('PITCH', 'ZOFF'))
    ap.add_argument('--old', type=float, nargs=2, default=[-5.0, 0.0],
                    metavar=('PITCH', 'ZOFF'),
                    help='calibration the CURRENT boxes are at (default = the '
                         'generation -5/0; use the applied calib for a '
                         'padding-only pass)')
    ap.add_argument('--old-emit', type=float, default=12.0,
                    help='min_emit_px the current boxes were built at')
    ap.add_argument('--new-emit', type=float, default=12.0,
                    help='target min_emit_px (e.g. 8 to retighten far boxes)')
    ap.add_argument('--backup-tag', default='precalib',
                    help='suffix for backups on --apply (use a fresh tag per '
                         'pass, e.g. pretighten)')
    ap.add_argument('--match-px', type=float, default=3.0,
                    help='max center dist (px) to match a clean label to meta')
    ap.add_argument('--apply', action='store_true',
                    help='replace labels/ + metadata (originals backed up)')
    args = ap.parse_args()

    dd = args.data_dir
    img_probe = os.path.join(dd, 'images')
    meta_path = os.path.join(dd, args.meta)
    lbl_dir = os.path.join(dd, args.labels)
    for p in (img_probe, meta_path, lbl_dir):
        if not os.path.exists(p):
            sys.exit("missing %s" % p)

    # image size from any frame
    any_img = next(f for f in os.listdir(img_probe) if f.endswith('.png'))
    import cv2
    h0, w0 = cv2.imread(os.path.join(img_probe, any_img)).shape[:2]

    recs = [json.loads(l) for l in open(meta_path) if l.strip()]
    meta_by_stem = {r['image'][:-4]: r for r in recs}
    print("frames in metadata: %d | image %dx%d" % (len(recs), w0, h0))

    print("recovering K + base_z ...")
    K, base_z = recover_k_basez(recs, w0, h0)
    print("  fy=%.1f base_z=%.4f | old(%.2f,%+.3f,emit%.0f) -> "
          "new(%.2f,%+.3f,emit%.0f)" %
          (K[1, 1], base_z, args.old[0], args.old[1], args.old_emit,
           args.new[0], args.new[1], args.new_emit))

    idx = field_index()

    def mk(pitch, zoff, emit):
        return Projector(K, w0, h0, max_range_m=MAX_RANGE_M,
                         ground_max_range_m=GROUND_MAX_RANGE_M,
                         min_box_px=MIN_BOX_PX, min_emit_px=emit,
                         base_z_offset=zoff, pitch_bias_deg=pitch)
    # OLD + NEW projectors: we apply the DELTA (new-old) to each STORED box, so
    # only the intended change moves/reshapes it — the box's shape and any pose-
    # rounding artefact (common-mode) cancel out. Works for a calibration shift
    # (old->new pitch/zoff) OR a padding change (old-emit->new-emit) OR both.
    prj_old = mk(args.old[0], args.old[1], args.old_emit)
    prj = mk(args.new[0], args.new[1], args.new_emit)

    out_lbl = os.path.join(dd, 'labels_recal')
    out_meta = os.path.join(dd, 'gt_metadata_recal.jsonl')
    os.makedirs(out_lbl, exist_ok=True)
    if os.path.exists(out_meta):
        os.remove(out_meta)
    mf = open(out_meta, 'w')

    n_frames = n_surv = n_fallback = n_unmatched = n_reproj_none = 0
    shifts = []
    for stem, r in sorted(meta_by_stem.items()):
        g, c = r['gt_robot_pose'], r['camera']
        pose = (g['x'], g['y'], base_z, math.radians(g['yaw_deg']),
                math.radians(c['head_pan_deg']), math.radians(c['head_tilt_deg']))
        prj.set_pose(*pose)
        prj_old.set_pose(*pose)

        # For each metadata landmark: box_new = stored_box + (proj_new - proj_old)
        # (delta cancels box shape + pose-rounding; leaves only calibration).
        newbox = {}     # id(lm) -> (cid, xc,yc,w,h) shifted; None if reproj failed
        meta_lms = []
        for lm in r['landmarks']:
            kind, obj = idx[lm['label']]
            d_new = project_obj(prj, kind, obj)
            d_old = project_obj(prj_old, kind, obj)
            ob = lm['bbox_norm']
            if d_new is None or d_old is None:
                newbox[id(lm)] = None
                n_reproj_none += 1
                meta_lms.append(dict(lm))            # keep stored box
                continue
            nn = det_to_norm(d_new, w0, h0)
            oo = det_to_norm(d_old, w0, h0)
            cid = NAME2ID[lm['class']]
            xc = ob[0] + (nn[0] - oo[0]); yc = ob[1] + (nn[1] - oo[1])
            bw = ob[2] + (nn[2] - oo[2]); bh = ob[3] + (nn[3] - oo[3])
            newbox[id(lm)] = (cid, xc, yc, bw, bh)
            # pixel_uv: stored point + (new-old) projected-point delta
            pt = np.array([[lm['world_xy'][0], lm['world_xy'][1], 0.0]])
            uvn, _ = prj._project(pt, range_limit=MAX_RANGE_M)
            uvo, _ = prj_old._project(pt, range_limit=MAX_RANGE_M)
            su, sv = lm['pixel_uv']
            nlm = dict(lm)
            nlm['pixel_uv'] = [float(su + (uvn[0, 0] - uvo[0, 0])),
                               float(sv + (uvn[0, 1] - uvo[0, 1]))]
            nlm['bbox_norm'] = [round(xc, 6), round(yc, 6),
                                round(bw, 6), round(bh, 6)]
            meta_lms.append(nlm)

        # --- inherit cleaning: match each CLEAN label to a metadata landmark ---
        lbl_file = os.path.join(lbl_dir, stem + '.txt')
        clean = []
        if os.path.isfile(lbl_file):
            for ln in open(lbl_file):
                p = ln.split()
                if len(p) == 5:
                    clean.append((int(p[0]), float(p[1]), float(p[2])))
        out_lines = []
        used = set()
        for cid, cxc, cyc in clean:
            best, bestd = None, 1e9
            for lm in r['landmarks']:
                if id(lm) in used:
                    continue
                if NAME2ID[lm['class']] != cid:
                    continue
                ox, oy = lm['bbox_norm'][0], lm['bbox_norm'][1]
                dd2 = ((ox - cxc) * w0) ** 2 + ((oy - cyc) * h0) ** 2
                if dd2 < bestd:
                    bestd, best = dd2, lm
            if best is None or math.sqrt(bestd) > args.match_px:
                n_unmatched += 1
                # cannot identify -> keep the clean label unchanged (safe)
                out_lines.append('%d %.6f %.6f' % (cid, cxc, cyc))  # placeholder
                continue
            used.add(id(best))
            nb = newbox[id(best)]
            if nb is None:                     # reproj failed: keep old box
                ob = best['bbox_norm']
                out_lines.append('%d %.6f %.6f %.6f %.6f' %
                                 (cid, ob[0], ob[1], ob[2], ob[3]))
                n_fallback += 1
            else:
                _, xc, yc, bw, bh = nb
                out_lines.append('%d %.6f %.6f %.6f %.6f' % (cid, xc, yc, bw, bh))
                shifts.append((yc - best['bbox_norm'][1]) * h0)
            n_surv += 1

        with open(os.path.join(out_lbl, stem + '.txt'), 'w') as f:
            f.write('\n'.join(out_lines) + ('\n' if out_lines else ''))
        nr = dict(r)
        nr['landmarks'] = meta_lms
        mf.write(json.dumps(nr) + '\n')
        n_frames += 1
    mf.close()

    print("\nframes relabeled     : %d" % n_frames)
    print("survivor labels wrote: %d" % n_surv)
    print("reproj None (metadata): %d" % n_reproj_none)
    print("reproj None on survivor->kept old box: %d" % n_fallback)
    print("unmatched clean labels (kept as-is): %d" % n_unmatched)
    if shifts:
        sh = np.asarray(shifts)
        print("survivor box vertical shift old->new: median=%+.2f  "
              "mean=%+.2f px (n=%d)" % (np.median(sh), sh.mean(), len(sh)))
    print("\nwrote -> %s/  and  %s" % (out_lbl, out_meta))

    if args.apply:
        _swap(dd, args.labels, args.meta, out_lbl, out_meta, args.backup_tag)


def _swap(dd, labels, meta, out_lbl, out_meta, tag):
    bak_lbl = os.path.join(dd, 'labels_%s' % tag)
    bak_meta = os.path.join(dd, 'gt_metadata_%s.jsonl' % tag)
    if os.path.exists(bak_lbl) or os.path.exists(bak_meta):
        sys.exit("backup already exists (labels_%s / gt_metadata_%s.jsonl) — "
                 "use a fresh --backup-tag." % (tag, tag))
    shutil.move(os.path.join(dd, labels), bak_lbl)
    shutil.move(os.path.join(dd, meta), bak_meta)
    shutil.move(out_lbl, os.path.join(dd, labels))
    shutil.move(out_meta, os.path.join(dd, meta))
    print("APPLIED: originals -> labels_%s/, gt_metadata_%s.jsonl" % (tag, tag))


if __name__ == '__main__':
    main()
