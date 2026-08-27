#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Repack the landmark dataset for cloud training (Colab/Kaggle).

Converts the ~16 GB of PNG frames to JPEG (default q90) — visually lossless for
detection — copies the YOLO labels verbatim, writes a portable ``landmark.yaml``
(relative paths), and tars it all. Result is ~2.5 GB, easy to upload.

    python package_for_cloud.py \
        --src /media/miftah/backup/landmark_dataset \
        --out /media/miftah/backup/landmark_yolo_cloud \
        --quality 90 --tar

Only images/ + labels/ are included; debug/, gt_metadata.jsonl, poses.csv and
reports are intentionally left out (not needed to train).
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

import cv2


def repack_split(src_split, dst_split, quality):
    img_src = os.path.join(src_split, 'images')
    lbl_src = os.path.join(src_split, 'labels')
    img_dst = os.path.join(dst_split, 'images')
    lbl_dst = os.path.join(dst_split, 'labels')
    os.makedirs(img_dst, exist_ok=True)
    os.makedirs(lbl_dst, exist_ok=True)

    pngs = sorted(glob.glob(os.path.join(img_src, '*.png')))
    n = 0
    for pth in pngs:
        stem = os.path.splitext(os.path.basename(pth))[0]
        im = cv2.imread(pth, cv2.IMREAD_COLOR)
        if im is None:
            print('  WARN unreadable: %s' % pth)
            continue
        cv2.imwrite(os.path.join(img_dst, stem + '.jpg'), im,
                    [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        lbl = os.path.join(lbl_src, stem + '.txt')
        if os.path.exists(lbl):
            shutil.copy2(lbl, os.path.join(lbl_dst, stem + '.txt'))
        else:  # keep an empty label so YOLO treats it as a negative frame
            open(os.path.join(lbl_dst, stem + '.txt'), 'w').close()
        n += 1
        if n % 500 == 0:
            print('  %s: %d/%d' % (os.path.basename(src_split), n, len(pngs)))
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='/media/miftah/backup/landmark_dataset')
    ap.add_argument('--out', default='/media/miftah/backup/landmark_yolo_cloud')
    ap.add_argument('--quality', type=int, default=90)
    ap.add_argument('--tar', action='store_true',
                    help='also create <out>.tar.gz')
    args = ap.parse_args()

    if not os.path.isdir(args.src):
        sys.exit('src not found: %s' % args.src)
    os.makedirs(args.out, exist_ok=True)

    total = 0
    for split in ('train', 'val'):
        s = os.path.join(args.src, split)
        if not os.path.isdir(s):
            sys.exit('missing split: %s' % s)
        print('repacking %s -> JPG q%d ...' % (split, args.quality))
        total += repack_split(s, os.path.join(args.out, split), args.quality)

    # portable data config (relative — resolves inside the extracted folder)
    with open(os.path.join(args.out, 'landmark.yaml'), 'w') as f:
        f.write('path: .\ntrain: train/images\nval: val/images\n')
        f.write("nc: 5\nnames: ['L', 'T', 'X', 'goalpost', 'center_circle']\n")

    print('done: %d images -> %s' % (total, args.out))
    if args.tar:
        tar = args.out.rstrip('/') + '.tar.gz'
        print('taring -> %s (this takes a while)...' % tar)
        subprocess.check_call(
            ['tar', 'czf', tar, '-C', os.path.dirname(args.out),
             os.path.basename(args.out)])
        sz = os.path.getsize(tar) / (1024 ** 3)
        print('tarball: %s  (%.2f GB)' % (tar, sz))


if __name__ == '__main__':
    main()
