#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""On-device latency / FPS probe for a given model + imgsz.

Run this ON the target (NUC for the OpenVINO IR, Orin for the .engine). It loads
the model, warms up, then times `predict` over N frames (a real image if given,
else random) and reports mean/median/p95 ms and FPS. Sweep imgsz to choose the
smallest size that still meets your accuracy bar (see benchmark.py).

    # NUC
    python latency_probe.py --weights best_int8_openvino_model --imgsz 640 512 416
    # Orin
    python latency_probe.py --weights best_int8.engine --imgsz 640 512 416 --device 0
"""
import argparse
import time

import numpy as np
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--imgsz', type=int, nargs='+', default=[640, 512, 416])
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--warmup', type=int, default=15)
    ap.add_argument('--device', default='')
    ap.add_argument('--image', default='', help='optional real frame to time')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.5)
    args = ap.parse_args()

    if args.image:
        import cv2
        frame0 = cv2.imread(args.image)
    else:
        frame0 = None

    print('weights: %s' % args.weights)
    print('%-6s %8s %8s %8s %8s' % ('imgsz', 'mean_ms', 'med_ms', 'p95_ms', 'fps'))
    for isz in args.imgsz:
        model = YOLO(args.weights, task='detect')
        frame = frame0 if frame0 is not None else \
            np.random.randint(0, 255, (isz, isz, 3), dtype=np.uint8)
        kw = dict(imgsz=isz, conf=args.conf, iou=args.iou, verbose=False)
        if args.device:
            kw['device'] = args.device
        for _ in range(args.warmup):
            model.predict(frame, **kw)
        ts = []
        for _ in range(args.n):
            t0 = time.perf_counter()
            model.predict(frame, **kw)
            ts.append((time.perf_counter() - t0) * 1000.0)
        ts = np.array(ts)
        print('%-6d %8.2f %8.2f %8.2f %8.1f' %
              (isz, ts.mean(), np.median(ts), np.percentile(ts, 95),
               1000.0 / ts.mean()))


if __name__ == '__main__':
    main()
