#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Export a trained .pt to ONNX + OpenVINO IR (INT8) for the Intel NUC.

Runs on any x86 host (cloud, or the NUC itself). INT8 uses NNCF post-training
quantization calibrated on the dataset's val split (`--data`). The resulting
OpenVINO IR directory is what detect_openvino.launch.py loads.

    python export_openvino.py --weights best.pt --data landmark.yaml \
                              --imgsz 640 --int8

Produces (next to the weights):
    best.onnx
    best_openvino_model/         (FP32/FP16)
    best_int8_openvino_model/    (INT8, when --int8)
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', required=True, help='path to best.pt')
    ap.add_argument('--data', default='landmark.yaml',
                    help='data yaml (val split used for INT8 calibration)')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--int8', action='store_true', help='INT8 PTQ (NNCF)')
    ap.add_argument('--half', action='store_true', help='FP16 IR (no --int8)')
    ap.add_argument('--opset', type=int, default=12)
    args = ap.parse_args()

    model = YOLO(args.weights, task='detect')

    # portable ONNX intermediate (also usable as a TensorRT source)
    onnx_path = model.export(format='onnx', imgsz=args.imgsz, opset=args.opset,
                             simplify=True, dynamic=False)
    print('ONNX  ->', onnx_path)

    ov_path = model.export(format='openvino', imgsz=args.imgsz,
                           int8=args.int8, half=args.half and not args.int8,
                           data=args.data)
    print('OpenVINO ->', ov_path,
          '(INT8)' if args.int8 else ('(FP16)' if args.half else '(FP32)'))


if __name__ == '__main__':
    main()
