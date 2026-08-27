#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 ROBOTIS / Bascorro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""YOLO-based soccer field landmark detector for ROBOTIS OP3.

Runs the field8n YOLO model (ONNX or OpenVINO) on the camera stream and
publishes detected soccer-field landmarks (goal posts, corners, penalty areas,
centre spot, penalty spot) as vision_msgs/Detection2DArray on `detections`.

Classes in field8n.onnx:
  0: Corner          1: Gawang          2: Kotak Penalti
  3: bola            4: left corner     5: right corner
  6: titik penalti   7: titik tengah

Compatible with both hardware camera (compressed) and Webots simulation
(/robotis_op3/camera/image_raw, raw BGRA8 image).

Default model: <op3_ball_detector share>/model/field8n.onnx
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (Detection2DArray, Detection2D,
                              ObjectHypothesisWithPose, BoundingBox2D)

from cv_bridge import CvBridge


class YoloLandmarkDetector(Node):
    """Detects soccer-field landmarks with a YOLO model."""

    def __init__(self):
        super().__init__('yolo_landmark_detector')

        # ----- parameters -------------------------------------------------
        self.model_path = str(self.declare_parameter('model_path', '').value)
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', False).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.4).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        self.imgsz = int(self.declare_parameter('imgsz', 512).value)
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        self._predict_precision = {'half': True} if self.half else {}
        self.max_detections = int(
            self.declare_parameter('max_detections', 20).value)
        # Empty list → detect all classes in the model.
        # ROS2 returns None when a list parameter is declared with [] default
        # and the YAML also supplies []; guard against that here.
        _tc = self.declare_parameter('target_classes', []).value
        self.target_classes = list(_tc) if _tc is not None else []
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 15.0).value)
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'cam_link').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 15 Hz')
            self.detection_rate = 15.0

        # ----- load model -------------------------------------------------
        self.model = self._load_model()

        # ----- ROS interface ---------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None
        self._processed_stamp = None

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        if self.publish_image:
            self.image_pub = self.create_publisher(Image, 'image_out', 1)
        else:
            self.image_pub = None

        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, 'image_in',
                self._compressed_image_callback, qos_profile_sensor_data)
        else:
            self.image_sub = self.create_subscription(
                Image, 'image_in',
                self._image_callback, qos_profile_sensor_data)

        self.enable_sub = self.create_subscription(
            Bool, 'enable', self._enable_callback, 1)

        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'YOLO landmark detector ready '
            f'(compressed={self.use_compressed}, conf={self.confidence_threshold}, '
            f'device={self.device}, rate={self.detection_rate:.1f}Hz, '
            f'enabled={self.enabled})')

    # ------------------------------------------------------------------
    # setup
    # ------------------------------------------------------------------
    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().fatal(
                'Failed to import ultralytics. Install it with '
                '"pip3 install ultralytics". Error: %s' % exc)
            raise

        share_dir = get_package_share_directory('op3_ball_detector')
        model_path = self.model_path.strip()
        if not model_path:
            model_path = os.path.join(share_dir, 'model', 'field8n.onnx')
        elif not os.path.isabs(model_path) and not os.path.exists(model_path):
            model_path = os.path.join(share_dir, model_path)

        if not os.path.exists(model_path):
            self.get_logger().fatal('Model path not found: %s' % model_path)
            raise FileNotFoundError(model_path)

        self.get_logger().info('Loading landmark model: %s' % model_path)
        model = (YOLO(model_path, task=self.model_task)
                 if self.model_task else YOLO(model_path))

        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_target_class_ids()
        self.get_logger().info(
            'Model classes: %s | targeting ids: %s'
            % (self.class_names, self.target_class_ids))

        try:
            import numpy as np
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            model.predict(dummy, imgsz=self.imgsz, device=self.device,
                          verbose=False, **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Model warm-up skipped: %s' % exc)

        return model

    def _resolve_target_class_ids(self):
        if not self.target_classes:
            return None  # detect all
        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        ids = []
        for wanted in self.target_classes:
            key = str(wanted).lower()
            if key in name_to_id:
                ids.append(name_to_id[key])
            else:
                self.get_logger().warn(
                    'target class "%s" not in model; ignoring' % wanted)
        if not ids:
            self.get_logger().warn(
                'No configured target class matched the model; detecting ALL.')
            return None
        return ids

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                'Detection %s' % ('ENABLED' if msg.data else 'DISABLED'))
        self.enabled = msg.data

    def _compressed_image_callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('compressed cv_bridge failed: %s' % exc)
            return
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    def _image_callback(self, msg: Image):
        try:
            # Webots publishes BGRA8; cv_bridge converts to bgr8 automatically.
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc)
            return
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    # ------------------------------------------------------------------
    # main detection loop
    # ------------------------------------------------------------------
    def _process_latest(self):
        with self._frame_lock:
            data = self._latest_image
            self._latest_image = None
        if data is None:
            return
        cv_img, header = data

        if not self.enabled:
            self._publish_raw_image(cv_img, header)
            return

        try:
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_detections,
                classes=self.target_class_ids,
                verbose=False,
                **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('YOLO inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return

        speed = getattr(results[0], 'speed', None) if results else None
        if speed:
            pre = speed.get('preprocess', 0.0)
            inf = speed.get('inference', 0.0)
            post = speed.get('postprocess', 0.0)
            self.get_logger().info(
                'latency: pre=%.1f infer=%.1f post=%.1f ms (total=%.1f ms)'
                % (pre, inf, post, pre + inf + post),
                throttle_duration_sec=5.0)

        height, width = cv_img.shape[:2]
        self._publish_detections(results, width, height, header)
        self._publish_annotated(results, cv_img, header)

    def _publish_detections(self, results, width, height, header):
        arr_msg = Detection2DArray()
        arr_msg.header.stamp = header.stamp
        arr_msg.header.frame_id = self.detection_frame_id

        if results:
            result = results[0]
            boxes = getattr(result, 'boxes', None)
            if boxes is not None and boxes.xyxy is not None:
                xyxy = boxes.xyxy.cpu().numpy()
                confs = (boxes.conf.cpu().numpy()
                         if boxes.conf is not None else None)
                cls_ids = (boxes.cls.cpu().numpy().astype(int)
                           if boxes.cls is not None else None)

                for idx, (x1, y1, x2, y2) in enumerate(xyxy):
                    cx = (x1 + x2) * 0.5
                    cy = (y1 + y2) * 0.5
                    bw = float(x2 - x1)
                    bh = float(y2 - y1)
                    conf = float(confs[idx]) if confs is not None else 1.0
                    cls_id = int(cls_ids[idx]) if cls_ids is not None else -1
                    class_name = self.class_names.get(cls_id, str(cls_id))

                    det = Detection2D()
                    det.header.stamp = header.stamp
                    det.header.frame_id = self.detection_frame_id

                    det.bbox = BoundingBox2D()
                    # vision_msgs/Pose2D has center.position.{x,y} + theta.
                    det.bbox.center.position.x = float(cx)
                    det.bbox.center.position.y = float(cy)
                    det.bbox.center.theta = 0.0
                    det.bbox.size_x = bw
                    det.bbox.size_y = bh

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = class_name
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)

                    arr_msg.detections.append(det)

        self.detections_pub.publish(arr_msg)

        if arr_msg.detections:
            summary = {}
            for d in arr_msg.detections:
                cls = d.results[0].hypothesis.class_id
                summary[cls] = summary.get(cls, 0) + 1
            self.get_logger().info(
                'landmarks: %s' % summary, throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                'no landmarks detected', throttle_duration_sec=3.0)

    def _publish_annotated(self, results, cv_img, header):
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            annotated = results[0].plot() if results else cv_img
        except Exception:  # noqa: BLE001
            annotated = cv_img
        self._publish_raw_image(annotated, header)

    def _publish_raw_image(self, cv_img, header):
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            out.header.stamp = header.stamp
            out.header.frame_id = self.detection_frame_id
            self.image_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('failed to publish image_out: %s' % exc,
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloLandmarkDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
