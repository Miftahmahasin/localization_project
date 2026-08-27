#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Accuracy benchmark across imgsz (and optionally a second exported model).

Runs Ultralytics `val` for each requested imgsz on a given weights file and
prints a compact per-class table (mAP50, mAP50-95, recall). Point `--weights`
at best.pt for the FP32 reference, then at the INT8 IR / engine to read off the
quantization drop. Latency is measured separately, on-device, by
latency_probe.py (val here is about ACCURACY only).

    python benchmark.py --weights best.pt --data landmark.yaml --imgsz 640 512 416
    python benchmark.py --weights best_int8_openvino_model --data landmark.yaml --imgsz 640
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True)
    ap.add_argument('--data', default='landmark.yaml')
    ap.add_argument('--imgsz', type=int, nargs='+', default=[640, 512, 416])
    ap.add_argument('--conf', type=float, default=0.001)  # val = full PR sweep
    ap.add_argument('--iou', type=float, default=0.7)
    args = ap.parse_args()

    names = ['L', 'T', 'X', 'goalpost', 'center_circle']
    print('weights: %s' % args.weights)
    print('%-6s %-14s %8s %8s %8s' %
          ('imgsz', 'class', 'mAP50', 'mAP5095', 'recall'))
    for isz in args.imgsz:
        model = YOLO(args.weights, task='detect')
        m = model.val(data=args.data, imgsz=isz, conf=args.conf, iou=args.iou,
                      verbose=False)
        # overall
        print('%-6d %-14s %8.3f %8.3f %8.3f' %
              (isz, 'ALL', m.box.map50, m.box.map, m.box.mr))
        # per class (indices follow the data yaml order)
        for i, nm in enumerate(names):
            try:
                p, r, ap50, ap = m.box.class_result(i)
                print('%-6s %-14s %8.3f %8.3f %8.3f' % ('', nm, ap50, ap, r))
            except Exception:
                pass
        print('-' * 48)


if __name__ == '__main__':
    main()
