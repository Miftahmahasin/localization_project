#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
"""Backend-pluggable YOLOv8n landmark inference node.

One code path serves every deployment target because Ultralytics ``YOLO()``
loads a PyTorch ``.pt``, an ``.onnx``, an OpenVINO IR directory, or a TensorRT
``.engine`` transparently and picks the matching runtime:

    * dev / sim  : ``.pt``      (CPU or CUDA)
    * Intel NUC  : OpenVINO IR  (INT8)
    * Orin Nano  : ``.engine``  (TensorRT INT8)

Subscribes a camera ``Image`` and republishes ``soccer_msgs/BoundingBoxes``
(the same contract the existing ``soccer_object_detection`` node uses), so the
rest of the stack does not care which backend produced the boxes. Published
class ids are remapped via :mod:`landmark_detector.class_map`.
"""

import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from soccer_msgs.msg import BoundingBox, BoundingBoxes

from landmark_detector.class_map import parse_class_map, DATASET_NAMES

# per-class overlay colours (BGR) for the debug image, keyed by DATASET id
_COLORS = {0: (0, 220, 0), 1: (0, 160, 255), 2: (255, 0, 200),
           3: (255, 220, 0), 4: (0, 255, 255)}


class LandmarkDetectorNode(Node):
    def __init__(self):
        super().__init__('landmark_detector')
        p = self.declare_parameter

        self.model_path = os.path.expanduser(str(p('model_path', '').value))
        # 'backend' is informational only — YOLO() auto-detects from the path.
        self.backend = str(p('backend', 'auto').value)
        self.imgsz = int(p('imgsz', 640).value)
        self.conf = float(p('conf', 0.25).value)
        self.iou = float(p('iou', 0.5).value)
        self.half = bool(p('half', False).value)
        self.device = str(p('device', '').value)          # '', 'cpu', '0', ...
        self.image_topic = str(p('image_topic', '/camera/image_raw').value)
        self.class_map = parse_class_map(str(p('class_map', 'identity').value))
        self.publish_debug = bool(p('publish_debug', True).value)
        self.robot_name = self.get_namespace().strip('/') or 'robot1'

        if not self.model_path or not os.path.exists(self.model_path):
            self.get_logger().error(
                'model_path not found: %r — pass model_path:=<.pt/.onnx/'
                'openvino_model dir/.engine>' % self.model_path)
            raise SystemExit(2)

        # Import Ultralytics lazily so `--help`/param errors don't pay the cost.
        from ultralytics import YOLO
        self.get_logger().info('loading model: %s (backend=%s)'
                               % (self.model_path, self.backend))
        self.model = YOLO(self.model_path, task='detect')

        self.br = CvBridge()
        self.pub_bbox = self.create_publisher(
            BoundingBoxes, '%s/object_bounding_boxes' % self.robot_name, 10)
        self.pub_debug = self.create_publisher(
            Image, '%s/detection_image' % self.robot_name, 10) \
            if self.publish_debug else None

        self._warmup()

        self.create_subscription(Image, self.image_topic, self.callback,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            'landmark_detector ready: sub=%s pub=%s/object_bounding_boxes '
            'imgsz=%d conf=%.2f iou=%.2f half=%s map=%s'
            % (self.image_topic, self.robot_name, self.imgsz, self.conf,
               self.iou, self.half, self.class_map))

    def _predict_kwargs(self):
        kw = dict(imgsz=self.imgsz, conf=self.conf, iou=self.iou,
                  half=self.half, verbose=False)
        if self.device:
            kw['device'] = self.device
        return kw

    def _warmup(self):
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        try:
            self.model.predict(dummy, **self._predict_kwargs())
            self.get_logger().info('warmup ok')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('warmup failed: %s' % exc)

    def callback(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, 'bgr8')  # drops alpha if bgra8
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc,
                                   throttle_duration_sec=2.0)
            return

        t0 = time.time()
        try:
            res = self.model.predict(img, **self._predict_kwargs())[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return
        dt_ms = (time.time() - t0) * 1000.0

        bbs = BoundingBoxes()
        bbs.header = msg.header
        overlay = img.copy() if self.pub_debug else None
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clss = res.boxes.cls.cpu().numpy().astype(int)
            for i in range(len(clss)):
                ds_id = int(clss[i])
                out_id = self.class_map.get(ds_id, ds_id)
                if out_id < 0:
                    continue  # class explicitly dropped by the map
                x1, y1, x2, y2 = (float(v) for v in xyxy[i])
                bb = BoundingBox()
                bb.xmin, bb.ymin = int(round(x1)), int(round(y1))
                bb.xmax, bb.ymax = int(round(x2)), int(round(y2))
                bb.probability = float(confs[i])
                bb.id = i
                bb.data = str(out_id)
                bbs.bounding_boxes.append(bb)
                if overlay is not None:
                    c = _COLORS.get(ds_id, (255, 255, 255))
                    import cv2
                    cv2.rectangle(overlay, (bb.xmin, bb.ymin),
                                  (bb.xmax, bb.ymax), c, 2)
                    cv2.putText(overlay, DATASET_NAMES.get(ds_id, str(ds_id)),
                                (bb.xmin, max(0, bb.ymin - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)

        if bbs.bounding_boxes:
            self.pub_bbox.publish(bbs)
        if self.pub_debug is not None and overlay is not None:
            dbg = self.br.cv2_to_imgmsg(overlay, encoding='bgr8')
            dbg.header = msg.header
            self.pub_debug.publish(dbg)
        self.get_logger().debug('%d dets in %.1f ms'
                                % (len(bbs.bounding_boxes), dt_ms),
                                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LandmarkDetectorNode()
    except SystemExit as exc:
        rclpy.shutdown()
        raise SystemExit(exc.code)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
