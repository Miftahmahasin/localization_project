#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Preview the label change from a calibration tweak WITHOUT re-rendering.

Re-projects the canonical field landmarks onto EXISTING rendered frames using
the SAME Projector code path (and the same range/box gates + min_emit padding)
the sampler uses, at both the OLD and the NEW calibration, and draws them on the
image: OLD boxes in red, NEW boxes in green. This is a faithful stand-in for
"render 50 test frames" — it exercises the identical projection/gating logic, so
if the NEW green boxes hug the painted lines better (especially far ground
junctions near the horizon), the recalibration is confirmed before any full regen.

Sanity: at the OLD calibration the drawn boxes should match the stored labels
(the dataset was generated at OLD), which the script verifies numerically.

USAGE:
  python3 preview_recalibration.py \
      --data-dir /media/miftah/backup/landmark_dataset/train \
      --old -5.0 0.0  --new -4.25 -0.035  --n 16 \
      --out /media/miftah/backup/landmark_dataset/recalib_preview
"""
import argparse
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("need opencv (pip install opencv-python)")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))
from landmark_dataset_gen.projection import Projector          # noqa: E402
from landmark_dataset_gen.field_landmarks import (             # noqa: E402
    build_line_intersections, build_center_circle, build_goalposts)

GEN_PITCH_BIAS_DEG = -5.0        # calibration the dataset was generated at
# sampler Projector defaults (see landmark_dataset_sampler._ensure_projector)
MAX_RANGE_M = 9.0
GROUND_MAX_RANGE_M = 7.0
MIN_BOX_PX = 6.0


def load_meta(path):
    with open(path) as f:
        return [json.loads(l) for l in f if l.strip()]


def recover_k_basez(recs, w, h):
    """Same recovery as measure_pitch_residual: fit K + median base_z."""
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


def make_projector(K, w, h, pitch_deg, z_off):
    return Projector(K, w, h, max_range_m=MAX_RANGE_M,
                     ground_max_range_m=GROUND_MAX_RANGE_M,
                     min_box_px=MIN_BOX_PX,
                     base_z_offset=z_off, pitch_bias_deg=pitch_deg)


def project_frame(prj, base_z, g, c, field):
    prj.set_pose(g['x'], g['y'], base_z, math.radians(g['yaw_deg']),
                 math.radians(c['head_pan_deg']), math.radians(c['head_tilt_deg']))
    js, posts, circ = field
    return prj.project_all(js, posts, circ)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--meta', default='gt_metadata.jsonl')
    ap.add_argument('--old', type=float, nargs=2, default=[-5.0, 0.0],
                    metavar=('PITCH', 'ZOFF'))
    ap.add_argument('--new', type=float, nargs=2, default=[-4.25, -0.035],
                    metavar=('PITCH', 'ZOFF'))
    ap.add_argument('--n', type=int, default=16, help='frames in the montage')
    ap.add_argument('--cols', type=int, default=4)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    img_dir = os.path.join(args.data_dir, 'images')
    meta_path = os.path.join(args.data_dir, args.meta)
    out_dir = args.out or os.path.join(args.data_dir, 'recalib_preview')
    os.makedirs(out_dir, exist_ok=True)

    recs = load_meta(meta_path)
    on_disk = [r for r in recs
               if os.path.isfile(os.path.join(img_dir, r['image']))]
    if not on_disk:
        sys.exit("no images on disk")
    h0, w0 = cv2.imread(os.path.join(img_dir, on_disk[0]['image'])).shape[:2]

    print("recovering K + base_z ...")
    K, base_z = recover_k_basez(recs, w0, h0)
    print("  fy=%.1f base_z=%.4f | OLD=%s NEW=%s" %
          (K[1, 1], base_z, args.old, args.new))

    field = (build_line_intersections(), build_goalposts(),
             build_center_circle())
    prj_old = make_projector(K, w0, h0, args.old[0], args.old[1])
    prj_new = make_projector(K, w0, h0, args.new[0], args.new[1])

    # rank frames by how many FAR ground landmarks they show (best demo of the
    # horizon effect); fall back to any ground content.
    def far_ground(r):
        return sum(1 for lm in r['landmarks']
                   if lm['class'] in ('L', 'T', 'X') and
                   lm.get('distance_m', 0) >= 4.5)
    on_disk.sort(key=far_ground, reverse=True)
    picks = on_disk[:args.n]

    # numeric sanity: OLD boxes vs stored labels (mean center error, px)
    errs, shifts = [], []
    tiles = []
    for r in picks:
        img = cv2.imread(os.path.join(img_dir, r['image']))
        g, c = r['gt_robot_pose'], r['camera']
        dets_old = project_frame(prj_old, base_z, g, c, field)
        dets_new = project_frame(prj_new, base_z, g, c, field)

        # OLD-vs-stored center error (match by nearest center)
        stored = [(lm['bbox_norm'][0] * w0, lm['bbox_norm'][1] * h0)
                  for lm in r['landmarks']]
        for d in dets_old:
            cxd, cyd = 0.5 * (d.x1 + d.x2), 0.5 * (d.y1 + d.y2)
            if stored:
                dd = min((cxd - sx) ** 2 + (cyd - sy) ** 2 for sx, sy in stored)
                errs.append(math.sqrt(dd))

        # vertical shift OLD->NEW for ground boxes (by matching label set order)
        for do, dn in zip(dets_old, dets_new):
            if do.class_id in (0, 1, 2) and do.label == dn.label:
                shifts.append(0.5 * (dn.y1 + dn.y2) - 0.5 * (do.y1 + do.y2))

        vis = img.copy()
        for d in dets_old:
            cv2.rectangle(vis, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)),
                          (0, 0, 220), 1)                       # OLD = red
        for d in dets_new:
            cv2.rectangle(vis, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)),
                          (0, 220, 0), 1)                       # NEW = green
        cv2.putText(vis, r['image'], (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(out_dir, 'cmp_' + r['image']), vis)
        tiles.append(vis)

    # montage
    cols = args.cols
    rows = int(math.ceil(len(tiles) / cols))
    th, tw = h0 // 2, w0 // 2
    canvas = np.zeros((rows * th, cols * tw, 3), dtype=np.uint8)
    for i, t in enumerate(tiles):
        rr, cc = divmod(i, cols)
        canvas[rr * th:(rr + 1) * th, cc * tw:(cc + 1) * tw] = \
            cv2.resize(t, (tw, th))
    # legend
    cv2.putText(canvas, 'RED = old (%.2f/%.3f)   GREEN = new (%.2f/%.3f)' %
                (args.old[0], args.old[1], args.new[0], args.new[1]),
                (10, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 0), 2, cv2.LINE_AA)
    mpath = os.path.join(out_dir, 'recalib_montage.png')
    cv2.imwrite(mpath, canvas)

    print("\nsanity: OLD box center vs STORED label  mean=%.2f  p95=%.2f px "
          "(n=%d)  -> should be ~0 (confirms faithful reprojection)" %
          (np.mean(errs), np.percentile(errs, 95), len(errs)))
    sh = np.asarray(shifts)
    print("ground-box vertical shift OLD->NEW: median=%+.2f px  (n=%d)  "
          "[+ = moved down toward far lines]" % (np.median(sh), len(sh)))
    print("wrote %d overlays + montage -> %s" % (len(tiles), mpath))


if __name__ == '__main__':
    main()
