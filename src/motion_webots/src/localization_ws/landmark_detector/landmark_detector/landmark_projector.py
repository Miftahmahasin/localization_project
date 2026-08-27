#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 0.3 — project YOLO landmark boxes to the ground plane (base_link).

Consumes ``soccer_msgs/BoundingBoxes`` (from the YOLO detector or the TAHAP 1
GT ``fake_detector``) and emits ``soccer_msgs/LandmarkArray`` where every
detection carries its position on the ground in ``base_link``. The camera model
is the SINGLE shared one (``landmark_geometry.projection.Projector``) — no second
projection implementation exists (prompt rule #2).

Head-angle synchronization (prompt 0.3, the most expensive active-vision bug):
the camera pose used to project a box is the pose AT THE IMAGE CAPTURE STAMP, not
at publish time. ``BoundingBoxes.header.stamp`` carries the image stamp (the
detector copies it from the Image), and head_pan/head_tilt are looked up from a
timestamped ``/joint_states`` buffer at that same stamp. In this stack the TF
``base_link->cam_link`` is a STATIC transform (head joints are not in TF), so the
joint-state chain — not TF — is what actually reflects neck motion. The
image<->joint stamp lag is measured and reported (median/p95).

Base pose is set to the origin with a nominal base height, so the shared
projector returns the ground point directly in base_link. Per-frame camera-height
uncertainty and a measured distance-covariance model land in TAHAP 2; here the
covariance is a documented placeholder.
"""
import bisect
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, JointState
from soccer_msgs.msg import BoundingBoxes, Landmark, LandmarkArray

from landmark_geometry.projection import Projector
from landmark_geometry import error_model

# dataset class ids (landmark_geometry / YOLO ordering)
_CLS_L, _CLS_T, _CLS_X, _CLS_GOALPOST, _CLS_CIRCLE = 0, 1, 2, 3, 4


class LandmarkProjector(Node):
    def __init__(self):
        super().__init__('landmark_projector')
        p = self.declare_parameter

        self.bbox_topic = str(p('bbox_topic',
                                '/robot1/object_bounding_boxes').value)
        self.caminfo_topic = str(p('camera_info_topic',
                                   '/robotis_op3/camera/camera_info').value)
        self.joints_topic = str(p('joint_states_topic',
                                  '/robotis_op3/joint_states').value)
        self.out_topic = str(p('output_topic', '/landmark_array').value)
        self.base_frame = str(p('base_frame', 'base_link').value)
        # nominal base_link height above the ground plane [m] (TAHAP 2 refines).
        self.base_height = float(p('base_height_m', 0.30).value)
        self.max_range = float(p('max_range_m', 9.0).value)
        self.ground_max_range = float(p('ground_max_range_m', 7.0).value)
        self.joint_buf_sec = float(p('joint_buffer_sec', 1.0).value)
        self.report_period = float(p('report_period_sec', 5.0).value)
        # FROZEN camera-extrinsic calibration — MUST match the dataset label
        # generator (landmark_dataset_sampler.py: base_z_offset=0.0,
        # pitch_bias_deg=-5.0, pan_bias_deg=0.0). These absorb the systematic
        # URDF-head-chain vs real-Webots-camera pitch residual; the same shared
        # Projector runs here, so omitting them makes every ground ray ~5 deg too
        # shallow -> distances blow up 3-40x near the horizon -> valid_range=False
        # for every landmark. Same single source of truth as the labels.
        self.pitch_bias_deg = float(p('pitch_bias_deg', -5.0).value)
        self.base_z_offset = float(p('base_z_offset_m', 0.0).value)
        self.pan_bias_deg = float(p('pan_bias_deg', 0.0).value)

        self.K = None                 # set from first CameraInfo
        self.W = self.H = None
        self.proj = None              # Projector, built once K is known
        self._pan_idx = self._tilt_idx = None
        # timestamped head-angle buffer: parallel arrays sorted by time
        self._jt = deque()            # (t_sec, pan, tilt)
        self._lags = []               # |img_stamp - joint_stamp| seconds

        self.sub_ci = self.create_subscription(
            CameraInfo, self.caminfo_topic, self._cb_caminfo,
            qos_profile_sensor_data)
        self.sub_js = self.create_subscription(
            JointState, self.joints_topic, self._cb_joints,
            qos_profile_sensor_data)
        self.sub_bb = self.create_subscription(
            BoundingBoxes, self.bbox_topic, self._cb_boxes, 10)
        self.pub = self.create_publisher(LandmarkArray, self.out_topic, 10)
        self.create_timer(self.report_period, self._report)

        self.get_logger().info(
            'landmark_projector: boxes=%s caminfo=%s joints=%s -> %s '
            '(base_h=%.3f max_range=%.1f ground=%.1f)'
            % (self.bbox_topic, self.caminfo_topic, self.joints_topic,
               self.out_topic, self.base_height, self.max_range,
               self.ground_max_range))

    # ── inputs ────────────────────────────────────────────────────────────────
    def _cb_caminfo(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        W, H = int(msg.width), int(msg.height)
        # Rebuild if resolution/intrinsics change (defensive re-latch): building
        # once and early-returning would silently keep a stale K if a different
        # CameraInfo (e.g. a resolution switch, or a wrong publisher winning the
        # race) arrives later — the exact non-determinism seen on the wire.
        if self.proj is not None:
            if (W == self.W and H == self.H
                    and np.allclose(K, self.K, atol=1e-6)):
                return
            self.get_logger().warn(
                'camera_info changed (%dx%d fx=%.1f -> %dx%d fx=%.1f); '
                'rebuilding projector' % (self.W, self.H, self.K[0, 0],
                                          W, H, K[0, 0]))
        self.K = K
        self.W, self.H = W, H
        self.proj = Projector(self.K, self.W, self.H,
                              max_range_m=self.max_range,
                              ground_max_range_m=self.ground_max_range,
                              base_z_offset=self.base_z_offset,
                              pitch_bias_deg=self.pitch_bias_deg,
                              pan_bias_deg=self.pan_bias_deg)
        self.get_logger().info(
            'camera model ready: fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d '
            '(calib pitch_bias=%.1f deg base_z_offset=%+.3f m pan_bias=%.1f deg)'
            % (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2],
               self.W, self.H, self.pitch_bias_deg, self.base_z_offset,
               self.pan_bias_deg))

    def _cb_joints(self, msg: JointState):
        if self._pan_idx is None:
            try:
                self._pan_idx = msg.name.index('head_pan')
                self._tilt_idx = msg.name.index('head_tilt')
            except ValueError:
                return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pan = float(msg.position[self._pan_idx])
        tilt = float(msg.position[self._tilt_idx])
        self._jt.append((t, pan, tilt))
        # evict old
        cutoff = t - self.joint_buf_sec
        while self._jt and self._jt[0][0] < cutoff:
            self._jt.popleft()

    def _lookup_head(self, t_img):
        """Nearest head (pan,tilt) to the image stamp; returns (pan,tilt,lag)."""
        if not self._jt:
            return None
        times = [e[0] for e in self._jt]
        i = bisect.bisect_left(times, t_img)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self._jt):
                lag = abs(self._jt[j][0] - t_img)
                if best is None or lag < best[2]:
                    best = (self._jt[j][1], self._jt[j][2], lag)
        return best

    # ── main ──────────────────────────────────────────────────────────────────
    def _cb_boxes(self, msg: BoundingBoxes):
        if self.proj is None:
            self.get_logger().warn('no CameraInfo yet — dropping boxes',
                                   throttle_duration_sec=2.0)
            return
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        head = self._lookup_head(t_img)
        if head is None:
            self.get_logger().warn('no joint_states buffered — dropping boxes',
                                   throttle_duration_sec=2.0)
            return
        pan, tilt, lag = head
        self._lags.append(lag)

        # camera pose at capture: base at origin -> projector returns base_link
        self.proj.set_pose(0.0, 0.0, self.base_height, 0.0, pan, tilt)

        out = LandmarkArray()
        out.header = msg.header            # stamp = image capture stamp
        out.header.frame_id = self.base_frame

        for bb in msg.bounding_boxes:
            try:
                cls = int(bb.data)
            except (ValueError, TypeError):
                continue
            u = 0.5 * (bb.xmin + bb.xmax)
            # goalpost reference point = base of the post (bottom edge), not center
            v = float(bb.ymax) if cls == _CLS_GOALPOST \
                else 0.5 * (bb.ymin + bb.ymax)
            w = self.proj.unproject_to_ground(float(u), float(v))
            if w is None:
                continue
            d = float(math.hypot(w[0], w[1]))
            cam = self.proj._to_cam(w[None, :])[0]

            lm = Landmark()
            lm.class_id = int(cls)
            lm.confidence = float(bb.probability)
            lm.u, lm.v = float(u), float(v)
            lm.p_cam.x, lm.p_cam.y, lm.p_cam.z = (float(cam[0]), float(cam[1]),
                                                  float(cam[2]))
            lm.p_base.x, lm.p_base.y, lm.p_base.z = (float(w[0]), float(w[1]),
                                                     0.0)
            # anisotropic distance covariance from the TAHAP 2 error model,
            # oriented along the camera->landmark bearing (range >> cross-range).
            lm.covariance_2x2 = error_model.cov_2x2(float(w[0]), float(w[1]))
            lm.valid_range = bool(d <= error_model.max_range(cls))
            out.landmarks.append(lm)

        self.pub.publish(out)

    def _report(self):
        if not self._lags:
            return
        a = np.asarray(self._lags)
        self.get_logger().info(
            'head-sync lag: n=%d  median=%.1f ms  p95=%.1f ms  max=%.1f ms'
            % (a.size, np.median(a) * 1e3, np.percentile(a, 95) * 1e3,
               a.max() * 1e3))


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkProjector()
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
