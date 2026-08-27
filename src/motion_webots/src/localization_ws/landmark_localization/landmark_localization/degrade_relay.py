#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP H (live) — detector-degradation relay for the sim-solvable sweep harness.

Sits between the projector's ``/landmark_array`` and ``geometric_pose_node`` and
applies CONTROLLED degradation to the projected landmarks, so the FULL live pipeline
(real YOLO detector + projector + EKF + line_heading) can be measured under the
sim-to-real failure modes without touching the model. The landmark level is the
right stage: ``p_base`` is already metric, so range-based junction cutoff is exact,
and ``line_heading_node`` (which reads the IMAGE, not landmarks) is untouched — that
is precisely the point of the line-heading insurance experiment (S1).

Four independent axes (all off by default -> pass-through):
  * filter_classes      — drop these class ids entirely (total class filter).
                          [0,1,2] = remove all L/T/X junctions (the imgsz-320 mode).
  * cutoff_range_m + cutoff_classes — drop landmarks of cutoff_classes whose ground
                          range |p_base| exceeds R (junction distance cutoff, sweep R).
  * recall + recall_dist_slope — keep each landmark with prob
                          max(0, recall - slope*range); slope models detections
                          getting rarer with distance (sim-to-real turf).
  * fp_per_frame + fp_classes — inject N false-positive landmarks per frame at a
                          random plausible ground position (covariance from the same
                          error model), to stress the association / precision path.

Class ids: 0=L 1=T 2=X 3=goalpost 4=center_circle. Junction = {0,1,2}.
The published header (capture stamp + frame_id) is copied through UNCHANGED so the
active-vision head-pose lookup downstream stays correct.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point
from soccer_msgs.msg import Landmark, LandmarkArray

try:
    from landmark_geometry import error_model
    _HAS_EM = True
except Exception:                                  # pragma: no cover
    _HAS_EM = False

_JUNCTION = (0, 1, 2)


class DegradeRelay(Node):
    def __init__(self):
        super().__init__('degrade_relay')
        p = self.declare_parameter
        self.in_topic = str(p('in_topic', '/landmark_array').value)
        self.out_topic = str(p('out_topic', '/landmark_array_deg').value)
        # NB: default MUST be a non-empty int list (e.g. [-1], a class that never
        # matches) — an empty [] makes rclpy infer BYTE_ARRAY, and a later
        # `ros2 param set filter_classes [0,1,2]` (INTEGER_ARRAY) is then rejected.
        self.filter_classes = set(int(c) for c in p('filter_classes', [-1]).value)
        self.cutoff_classes = set(int(c) for c in
                                  p('cutoff_classes', list(_JUNCTION)).value)
        self.cutoff_range = float(p('cutoff_range_m', 0.0).value)   # 0 = off
        self.recall = float(p('recall', 1.0).value)
        self.recall_dist_slope = float(p('recall_dist_slope', 0.0).value)
        self.fp_per_frame = int(p('fp_per_frame', 0).value)
        self.fp_classes = [int(c) for c in p('fp_classes', list(_JUNCTION)).value]
        self.fp_range_m = float(p('fp_range_m', 3.0).value)
        self.rng = np.random.default_rng(int(p('seed', 0).value))

        # Live-tunable so a sweep can step a knob with `ros2 param set /degrade_relay
        # <name> <value>` between eval runs WITHOUT relaunching T1.
        self.add_on_set_parameters_callback(self._on_params)

        self._n_in = 0
        self._n_out = 0
        self._n_fp = 0
        self.sub = self.create_subscription(
            LandmarkArray, self.in_topic, self._cb, 10)
        self.pub = self.create_publisher(LandmarkArray, self.out_topic, 10)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'degrade_relay: %s -> %s | filter=%s cutoff=%s@%.1fm recall=%.2f'
            ' slope=%.3f fp=%d' % (
                self.in_topic, self.out_topic, sorted(self.filter_classes),
                sorted(self.cutoff_classes), self.cutoff_range, self.recall,
                self.recall_dist_slope, self.fp_per_frame))

    def _on_params(self, params):
        for pr in params:
            n, v = pr.name, pr.value
            if n == 'filter_classes':
                self.filter_classes = set(int(c) for c in v)
            elif n == 'cutoff_classes':
                self.cutoff_classes = set(int(c) for c in v)
            elif n == 'cutoff_range_m':
                self.cutoff_range = float(v)
            elif n == 'recall':
                self.recall = float(v)
            elif n == 'recall_dist_slope':
                self.recall_dist_slope = float(v)
            elif n == 'fp_per_frame':
                self.fp_per_frame = int(v)
            elif n == 'fp_classes':
                self.fp_classes = [int(c) for c in v]
            elif n == 'fp_range_m':
                self.fp_range_m = float(v)
        self._n_in = self._n_out = self._n_fp = 0     # reset counters per setting
        return SetParametersResult(successful=True)

    def _keep(self, lm) -> bool:
        cid = int(lm.class_id)
        if cid in self.filter_classes:
            return False
        rng = math.hypot(lm.p_base.x, lm.p_base.y)
        if self.cutoff_range > 0.0 and cid in self.cutoff_classes \
                and rng > self.cutoff_range:
            return False
        keep_p = self.recall - self.recall_dist_slope * rng
        if keep_p < 1.0 and self.rng.random() > max(0.0, keep_p):
            return False
        return True

    def _fp_landmark(self) -> Landmark:
        """A random-class landmark at a random plausible ground position."""
        cid = int(self.rng.choice(self.fp_classes))
        r = float(self.rng.uniform(0.6, self.fp_range_m))
        th = float(self.rng.uniform(-math.pi / 2, math.pi / 2))   # roughly in-view
        x, y = r * math.cos(th), r * math.sin(th)
        lm = Landmark()
        lm.class_id = cid
        lm.confidence = 0.5
        lm.p_base = Point(x=x, y=y, z=0.0)
        lm.p_cam = Point(x=x, y=y, z=0.0)
        if _HAS_EM:
            lm.covariance_2x2 = [float(v) for v in error_model.cov_2x2(x, y)]
        else:                                          # pragma: no cover
            s = 0.02 + 0.02 * r
            lm.covariance_2x2 = [s, 0.0, 0.0, s]
        lm.valid_range = True
        return lm

    def _cb(self, msg: LandmarkArray):
        out = LandmarkArray()
        out.header = msg.header                        # copy capture stamp + frame
        self._n_in += len(msg.landmarks)
        for lm in msg.landmarks:
            if self._keep(lm):
                out.landmarks.append(lm)
        for _ in range(self.fp_per_frame):
            out.landmarks.append(self._fp_landmark())
            self._n_fp += 1
        self._n_out += len(out.landmarks)
        self.pub.publish(out)

    def _report(self):
        self.get_logger().info(
            'degrade_relay: in=%d out=%d fp_injected=%d'
            % (self._n_in, self._n_out, self._n_fp))


def main(args=None):
    rclpy.init(args=args)
    node = DegradeRelay()
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
