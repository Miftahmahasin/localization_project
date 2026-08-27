#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 1 — GT-replay detector: publish the sidecar as if it were the YOLO node.

Reads the dataset ``gt_metadata.jsonl`` and republishes each frame's landmarks as
``soccer_msgs/BoundingBoxes`` on the SAME topic/format the real YOLO detector
emits, so the projection + geometric backend cannot tell the difference and can
be validated / tuned BEFORE the model is trusted. It also publishes the matching
head ``JointState`` (same stamp, for the projector's head-angle sync) and the GT
robot pose (for scoring). Degradation knobs (recall, false positives, L/T/X class
confusion, pixel noise) let you sweep the backend's tolerance live — the offline
counterpart is scripts/sweep_sensitivity.py.
"""
import json
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, CameraInfo
from nav_msgs.msg import Odometry
from soccer_msgs.msg import BoundingBox, BoundingBoxes

_NAME2ID = {'L': 0, 'T': 1, 'X': 2, 'goalpost': 3, 'center_circle': 4}
_LTX = (0, 1, 2)


class FakeDetector(Node):
    def __init__(self):
        super().__init__('fake_detector')
        p = self.declare_parameter
        self.dir = str(p('sidecar_dir',
                         '/media/miftah/backup/landmark_dataset/val').value)
        self.rate = float(p('rate_hz', 5.0).value)
        self.loop = bool(p('loop', True).value)
        self.recall = float(p('recall', 1.0).value)
        self.fp = int(p('fp_per_frame', 0).value)
        self.confuse = float(p('confuse', 0.0).value)
        self.pix_noise = float(p('pixel_noise_px', 0.0).value)
        self.bbox_topic = str(p('bbox_topic',
                                '/robot1/object_bounding_boxes').value)
        self.joints_topic = str(p('joint_states_topic',
                                  '/robotis_op3/joint_states').value)
        self.gt_topic = str(p('gt_pose_topic', '/ground_truth/odom').value)
        self.caminfo_topic = str(p('camera_info_topic',
                                   '/robotis_op3/camera/camera_info').value)
        self.rng = np.random.default_rng(int(p('seed', 0).value))

        path = os.path.join(self.dir, 'gt_metadata.jsonl')
        self.rows = []
        with open(path) as f:
            for line in f:
                self.rows.append(json.loads(line))
        self.i = 0
        self.W = self.rows[0]['camera']['image_wh'][0]
        self.H = self.rows[0]['camera']['image_wh'][1]

        # CameraInfo built from the sidecar K so the projector reproduces the
        # dataset geometry exactly (offline path is self-contained, no Webots).
        k = self.rows[0]['camera']['K_fx_fy_cx_cy']
        self._caminfo = CameraInfo()
        self._caminfo.width = int(self.W)
        self._caminfo.height = int(self.H)
        self._caminfo.k = [k[0], 0.0, k[2], 0.0, k[1], k[3], 0.0, 0.0, 1.0]
        self._caminfo.p = [k[0], 0.0, k[2], 0.0,
                           0.0, k[1], k[3], 0.0, 0.0, 0.0, 1.0, 0.0]
        self._caminfo.distortion_model = 'plumb_bob'

        self.pub_bb = self.create_publisher(BoundingBoxes, self.bbox_topic, 10)
        self.pub_js = self.create_publisher(JointState, self.joints_topic, 10)
        self.pub_gt = self.create_publisher(Odometry, self.gt_topic, 10)
        self.pub_ci = self.create_publisher(CameraInfo, self.caminfo_topic, 10)
        self.create_timer(1.0 / max(self.rate, 0.1), self._tick)
        self.get_logger().info(
            'fake_detector: %d frames from %s @ %.1fHz -> %s '
            '(recall=%.2f fp=%d confuse=%.2f noise=%.1fpx)'
            % (len(self.rows), self.dir, self.rate, self.bbox_topic,
               self.recall, self.fp, self.confuse, self.pix_noise))

    def _tick(self):
        if self.i >= len(self.rows):
            if not self.loop:
                return
            self.i = 0
        r = self.rows[self.i]
        self.i += 1
        now = self.get_clock().now().to_msg()

        # CameraInfo (same stamp as the frame) so landmark_projector has K
        self._caminfo.header.stamp = now
        self._caminfo.header.frame_id = 'cam_link'
        self.pub_ci.publish(self._caminfo)

        # head joint_states (same stamp as the image -> zero sync lag by design)
        js = JointState()
        js.header.stamp = now
        js.name = ['head_pan', 'head_tilt']
        js.position = [math.radians(r['camera']['head_pan_deg']),
                       math.radians(r['camera']['head_tilt_deg'])]
        self.pub_js.publish(js)

        # GT pose for scoring
        gp = r['gt_robot_pose']
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'map'
        od.pose.pose.position.x = float(gp['x'])
        od.pose.pose.position.y = float(gp['y'])
        yaw = math.radians(gp['yaw_deg'])
        od.pose.pose.orientation.z = math.sin(yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_gt.publish(od)

        # landmarks -> BoundingBoxes (with degradation)
        bbs = BoundingBoxes()
        bbs.header.stamp = now
        bbs.header.frame_id = 'cam_link'
        idx = 0
        for lm in r['landmarks']:
            if self.rng.random() > self.recall:
                continue
            cid = _NAME2ID[lm['class']]
            if cid in _LTX and self.rng.random() < self.confuse:
                cid = int(self.rng.choice([x for x in _LTX if x != cid]))
            cx, cy, w, h = lm['bbox_norm']
            cxp, cyp = cx * self.W, cy * self.H
            if self.pix_noise > 0:
                cxp += self.rng.normal(0, self.pix_noise)
                cyp += self.rng.normal(0, self.pix_noise)
            wp, hp = w * self.W, h * self.H
            bbs.bounding_boxes.append(self._box(
                cxp, cyp, wp, hp, cid, 0.95, idx))
            idx += 1
        for _ in range(self.fp):
            cxp = self.rng.uniform(0, self.W)
            cyp = self.rng.uniform(self.H * 0.3, self.H)
            bbs.bounding_boxes.append(self._box(
                cxp, cyp, 40, 40, int(self.rng.integers(0, 5)), 0.5, idx))
            idx += 1
        self.pub_bb.publish(bbs)

    def _box(self, cx, cy, w, h, cid, prob, idx):
        bb = BoundingBox()
        bb.xmin = int(round(cx - w / 2.0))
        bb.ymin = int(round(cy - h / 2.0))
        bb.xmax = int(round(cx + w / 2.0))
        bb.ymax = int(round(cy + h / 2.0))
        bb.probability = float(prob)
        bb.id = int(idx)
        bb.data = str(int(cid))
        return bb


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetector()
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
