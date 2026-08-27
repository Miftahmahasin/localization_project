#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TAHAP 8 — regenerate YOLO labels from the sidecar ONLY (no Webots, no render).

Every image stored its exact camera in gt_metadata.jsonl (cam_world_pos + quat +
intrinsics). So any change to the projection/box/cull logic can be re-applied to
the WHOLE dataset by re-projecting the field model from those stored poses — no
8-hour re-collection. This is what makes label iteration cheap after this point.

It injects the stored T_world_camera straight into the Projector (so pitch_bias /
base_z are already baked into the pose and are NOT re-applied) and runs the same
project_all + culls a live collection would.

USAGE:
  # in place (overwrite labels/), keeping images:
  python3 relabel.py --data-dir /media/miftah/backup/landmark_dataset/train
  # to a separate dir, and redraw debug overlays for a diff:
  python3 relabel.py --data-dir .../train --out-dir /tmp/relabel --debug 30
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
from landmark_dataset_gen import field_landmarks as FL          # noqa: E402

CLASS_COLORS = {0: (0, 220, 0), 1: (0, 160, 255), 2: (255, 0, 200),
                3: (255, 220, 0), 4: (0, 255, 255)}


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--out-dir', default='', help='default: overwrite in place')
    ap.add_argument('--ground-max-range', type=float, default=0.0,
                    help='override; 0 = use sidecar value (fallback 7.0)')
    ap.add_argument('--min-emit', type=float, default=0.0,
                    help='override; 0 = use sidecar value (fallback 18.0)')
    ap.add_argument('--fov', type=float, default=1.3613, help='fallback K if sidecar has none')
    ap.add_argument('--debug', type=int, default=0, help='redraw N debug overlays')
    args = ap.parse_args()

    data_dir = os.path.expanduser(args.data_dir)
    out_dir = os.path.expanduser(args.out_dir) if args.out_dir else data_dir
    lbl_dir = os.path.join(out_dir, 'labels')
    dbg_dir = os.path.join(out_dir, 'debug')
    os.makedirs(lbl_dir, exist_ok=True)
    if args.debug:
        os.makedirs(dbg_dir, exist_ok=True)
    recs = load_meta(os.path.join(data_dir, 'gt_metadata.jsonl'))

    junctions = FL.build_line_intersections()
    posts = FL.build_goalposts()
    circles = FL.build_center_circle()

    n_lbl = 0
    n_det = 0
    for i, rec in enumerate(recs):
        cam = rec['camera']
        pos = np.array(cam['cam_world_pos'], float)
        qw, qx, qy, qz = cam['cam_world_quat']
        # intrinsics + size: prefer sidecar, else canonical from FOV + image size
        if 'image_wh' in cam and 'K_fx_fy_cx_cy' in cam:
            W, H = cam['image_wh']
            fx, fy, cx, cy = cam['K_fx_fy_cx_cy']
        else:
            import cv2
            im = cv2.imread(os.path.join(data_dir, 'images', rec['image']))
            H, W = im.shape[:2]
            fx = fy = W / (2.0 * math.tan(args.fov * 0.5))
            cx, cy = W / 2.0, H / 2.0
        pj = rec.get('projector', {})
        gmr = args.ground_max_range or pj.get('ground_max_range_m', 7.0)
        me = args.min_emit or pj.get('min_emit_px', 18.0)
        mr = pj.get('max_range_m', 9.0)

        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        P = Projector(K, W, H, max_range_m=mr, ground_max_range_m=gmr,
                      min_emit_px=me)
        T = np.eye(4)
        T[:3, :3] = quat2mat(qw, qx, qy, qz)
        T[:3, 3] = pos
        P._T_map_cam = T
        P._T_cam_map = np.linalg.inv(T)
        P._cam_pos = pos.copy()

        dets = P.project_all(junctions, posts, circles)
        stem = os.path.splitext(rec['image'])[0]
        lines = []
        for d in dets:
            xc = ((d.x1 + d.x2) * 0.5) / W
            yc = ((d.y1 + d.y2) * 0.5) / H
            bw = (d.x2 - d.x1) / W
            bh = (d.y2 - d.y1) / H
            lines.append('%d %.6f %.6f %.6f %.6f'
                         % (d.class_id, xc, yc, bw, bh))
        with open(os.path.join(lbl_dir, stem + '.txt'), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        n_lbl += 1
        n_det += len(dets)

        if args.debug and i < args.debug:
            import cv2
            im = cv2.imread(os.path.join(data_dir, 'images', rec['image']))
            if im is not None:
                for d in dets:
                    c = CLASS_COLORS.get(d.class_id, (255, 255, 255))
                    cv2.rectangle(im, (int(d.x1), int(d.y1)),
                                  (int(d.x2), int(d.y2)), c, 2)
                cv2.imwrite(os.path.join(dbg_dir, stem + '.png'), im)

    print('relabelled %d images  ->  %s' % (n_lbl, lbl_dir))
    print('  total labels: %d   (mean %.2f / image)'
          % (n_det, n_det / max(n_lbl, 1)))
    if args.debug:
        print('  redrew %d debug overlays -> %s' % (min(args.debug, n_lbl), dbg_dir))


if __name__ == '__main__':
    main()
