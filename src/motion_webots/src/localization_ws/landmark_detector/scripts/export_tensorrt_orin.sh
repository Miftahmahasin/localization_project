#!/usr/bin/env bash
# Build a TensorRT engine ON the Jetson Orin Nano (engines are NOT portable
# across devices / TensorRT versions, so this must run on the target).
#
# Prereqs on the Orin (JetPack): ultralytics, torch (Jetson build), tensorrt.
# Copy best.pt and the calib/ folder (from make_calib.py) to the Orin first.
#
# Usage:
#   ./export_tensorrt_orin.sh best.pt calib/calib.yaml 640 int8
#   ./export_tensorrt_orin.sh best.pt calib/calib.yaml 640 fp16
set -euo pipefail

WEIGHTS="${1:?usage: export_tensorrt_orin.sh <best.pt> <calib.yaml> <imgsz> <int8|fp16>}"
DATA="${2:?need calib.yaml}"
IMGSZ="${3:-640}"
PREC="${4:-int8}"

if [[ "$PREC" == "int8" ]]; then
  echo ">> TensorRT INT8 engine (calibration: $DATA, imgsz=$IMGSZ)"
  yolo export model="$WEIGHTS" format=engine int8=True data="$DATA" \
       imgsz="$IMGSZ" device=0 workspace=4
elif [[ "$PREC" == "fp16" ]]; then
  echo ">> TensorRT FP16 engine (imgsz=$IMGSZ)"
  yolo export model="$WEIGHTS" format=engine half=True \
       imgsz="$IMGSZ" device=0 workspace=4
else
  echo "unknown precision: $PREC (use int8 or fp16)"; exit 2
fi
echo ">> engine written next to $WEIGHTS (rename to best_${PREC}.engine)"
