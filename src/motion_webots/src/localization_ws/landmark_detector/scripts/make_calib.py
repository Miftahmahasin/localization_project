#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build a small INT8 calibration set from the val split.

TensorRT/OpenVINO INT8 PTQ only needs a few hundred representative frames. This
samples ~N val images (+ their labels) into a self-contained folder with a
``calib.yaml`` so INT8 export on the Orin does not need the full 16 GB dataset.

    python make_calib.py --src /media/miftah/backup/landmark_dataset \
                         --out ./calib --n 500 --quality 90
"""
import argparse
import glob
import os
import random
import shutil
import sys

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='/media/miftah/backup/landmark_dataset')
    ap.add_argument('--out', default='./calib')
    ap.add_argument('--n', type=int, default=500)
    ap.add_argument('--quality', type=int, default=90)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    val_img = os.path.join(args.src, 'val', 'images')
    val_lbl = os.path.join(args.src, 'val', 'labels')
    pngs = sorted(glob.glob(os.path.join(val_img, '*.png')))
    if not pngs:
        sys.exit('no val images at %s' % val_img)
    random.Random(args.seed).shuffle(pngs)
    pick = pngs[:min(args.n, len(pngs))]

    img_dst = os.path.join(args.out, 'val', 'images')
    lbl_dst = os.path.join(args.out, 'val', 'labels')
    os.makedirs(img_dst, exist_ok=True)
    os.makedirs(lbl_dst, exist_ok=True)
    for pth in pick:
        stem = os.path.splitext(os.path.basename(pth))[0]
        im = cv2.imread(pth, cv2.IMREAD_COLOR)
        cv2.imwrite(os.path.join(img_dst, stem + '.jpg'), im,
                    [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
        lbl = os.path.join(val_lbl, stem + '.txt')
        if os.path.exists(lbl):
            shutil.copy2(lbl, os.path.join(lbl_dst, stem + '.txt'))
        else:
            open(os.path.join(lbl_dst, stem + '.txt'), 'w').close()

    # ultralytics INT8 export reads `val:` for calibration images
    with open(os.path.join(args.out, 'calib.yaml'), 'w') as f:
        f.write('path: .\ntrain: val/images\nval: val/images\n')
        f.write("nc: 5\nnames: ['L', 'T', 'X', 'goalpost', 'center_circle']\n")
    print('calib set: %d images -> %s' % (len(pick), args.out))


if __name__ == '__main__':
    main()
