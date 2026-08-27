#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 6B — line-scan co-primary: field-line heading → EKF (yaw only).

The typed-landmark path blacks out when the head looks down to the ball
(gaze map 4.4-A: valid landmark fix ~0% below tilt −35°), yet near-field white
LINES are exactly what the camera sees looking down. This node turns those lines
into an absolute heading and feeds it straight to the EKF — raising line-scan
from the auxiliary AMCL channel (rule #5, kept) to a heading CO-PRIMARY, WITHOUT
touching the AMCL particle filter (rule #3).

Pipeline: rectified image → white-line pixels (grass-removed) → Hough segments →
each endpoint unprojected to the ground by the SINGLE shared
``landmark_geometry.Projector`` (rule #2; the camera pose comes from the head
pan/tilt in /joint_states at the image stamp, so it stays correct while the head
tilts down — the legacy TF-based fieldline detector cannot) → ground segments →
``line_heading.estimate_heading`` (dominant direction mod 90°, quadrant resolved
by the EKF prior, suppressed when ambiguous) → ``/line_heading``
PoseWithCovarianceStamped with only yaw informative (x/y variance huge). The EKF
takes it as a yaw-only ``pose3``.
"""
import bisect
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState

from landmark_geometry.projection import Projector

from .line_heading import estimate_heading


class LineHeadingNode(Node):
    def __init__(self):
        super().__init__('line_heading_node')
        p = self.declare_parameter

        self.image_topic = str(p('image_topic',
                                 '/robotis_op3/camera/image_rect').value)
        self.caminfo_topic = str(p('camera_info_topic',
                                   '/robotis_op3/camera/camera_info').value)
        self.joints_topic = str(p('joint_states_topic',
                                  '/robotis_op3/joint_states').value)
        self.ekf_topic = str(p('ekf_topic', '/odometry/filtered').value)
        self.out_topic = str(p('output_topic', '/line_heading').value)
        self.map_frame = str(p('map_frame', 'map').value)
        self.base_height = float(p('base_height_m', 0.30).value)
        # near-field only: lines are close, especially in down-gaze; a far cap
        # rejects horizon/grass noise that would corrupt the direction estimate.
        self.max_range = float(p('max_range_m', 9.0).value)
        self.ground_max_range = float(p('ground_max_range_m', 5.0).value)
        self.joint_buf_sec = float(p('joint_buffer_sec', 1.0).value)
        self.report_period = float(p('report_period_sec', 5.0).value)
        # FROZEN calibration — SAME single source of truth as landmark_projector
        # and the dataset sampler (pitch_bias −5°). Omitting it makes ground rays
        # ~5° too shallow near the horizon.
        self.pitch_bias_deg = float(p('pitch_bias_deg', -5.0).value)
        self.base_z_offset = float(p('base_z_offset_m', 0.0).value)
        self.pan_bias_deg = float(p('pan_bias_deg', 0.0).value)
        # white-line detection: min-RGB ACHROMATIC ridge (B4). A global gray>=200
        # threshold rendered 0 pixels on Webots grass (published=0/8500). The
        # proven extractor (landmark_dataset_gen overlay_lines.py::ridge_dist_map)
        # gates on color-NEUTRALITY (max-min small) at a much lower brightness
        # (min>=90), so faint/far lines on speckled grass survive.
        self.ridge_min = int(p('ridge_min', 90).value)   # min(R,G,B) >= this
        self.ridge_sat = int(p('ridge_sat', 80).value)   # max-min <= this (achrom)
        self.close_k = int(p('ridge_close_k', 5).value)  # morph-close kernel
        self.roi_top_frac = float(p('roi_top_frac', 0.30).value)  # cut sky/goal
        self.canny1 = int(p('canny1', 50).value)
        self.canny2 = int(p('canny2', 150).value)
        self.hough_thresh = int(p('hough_threshold', 40).value)
        self.hough_min_len = int(p('hough_min_len_px', 40).value)
        self.hough_max_gap = int(p('hough_max_gap_px', 10).value)
        # heading gating (precision>recall)
        self.min_seg_len_m = float(p('min_seg_len_m', 0.15).value)
        self.min_strength = float(p('min_strength', 0.6).value)
        self.min_segments = int(p('min_segments', 2).value)
        self.accept_margin = math.radians(
            float(p('accept_margin_deg', 30.0).value))
        self.prior_max_age = float(p('prior_max_age_s', 1.0).value)
        self.yaw_stddev = math.radians(float(p('yaw_stddev_deg', 5.0).value))

        self.K = None
        self.W = self.H = None
        self.proj = None
        self._pan_idx = self._tilt_idx = None
        self._jt = deque()                 # (t, pan, tilt)
        self._prior_yaw = None
        self._prior_t = None
        self.bridge = CvBridge()

        self._n_pub = 0
        self._n_suppress = 0
        self._strengths = []

        self.create_subscription(CameraInfo, self.caminfo_topic,
                                 self._cb_caminfo, qos_profile_sensor_data)
        self.create_subscription(JointState, self.joints_topic,
                                 self._cb_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.ekf_topic, self._cb_ekf, 20)
        self.create_subscription(Image, self.image_topic, self._cb_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.out_topic, 10)
        self.create_timer(self.report_period, self._report)

        self.get_logger().info(
            'line_heading_node: image=%s -> %s (yaw only, EKF pose3; NOT AMCL) '
            'ground<=%.1fm strength>=%.2f margin<=%.0fdeg'
            % (self.image_topic, self.out_topic, self.ground_max_range,
               self.min_strength, math.degrees(self.accept_margin)))

    # ── inputs ──────────────────────────────────────────────────────────────
    def _cb_caminfo(self, msg: CameraInfo):
        if self.proj is not None:
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.W, self.H = int(msg.width), int(msg.height)
        self.proj = Projector(self.K, self.W, self.H,
                              max_range_m=self.max_range,
                              ground_max_range_m=self.ground_max_range,
                              base_z_offset=self.base_z_offset,
                              pitch_bias_deg=self.pitch_bias_deg,
                              pan_bias_deg=self.pan_bias_deg)
        self.get_logger().info(
            'camera model ready: %dx%d (calib pitch_bias=%.1f deg)'
            % (self.W, self.H, self.pitch_bias_deg))

    def _cb_joints(self, msg: JointState):
        if self._pan_idx is None:
            try:
                self._pan_idx = msg.name.index('head_pan')
                self._tilt_idx = msg.name.index('head_tilt')
            except ValueError:
                return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._jt.append((t, float(msg.position[self._pan_idx]),
                         float(msg.position[self._tilt_idx])))
        cutoff = t - self.joint_buf_sec
        while self._jt and self._jt[0][0] < cutoff:
            self._jt.popleft()

    def _cb_ekf(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._prior_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._prior_t = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup_head(self, t_img):
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

    # ── line pixels → ground segments ────────────────────────────────────────
    def _white_mask(self, bgr):
        # min-RGB achromatic ridge: white lines are bright AND color-neutral
        # (max-min small); grass is green (large max-min) even where bright.
        b, g, r = bgr[:, :, 0], bgr[:, :, 1], bgr[:, :, 2]
        mn = np.minimum(np.minimum(b, g), r).astype(np.int16)
        mx = np.maximum(np.maximum(b, g), r).astype(np.int16)
        mask = ((mn >= self.ridge_min)
                & ((mx - mn) <= self.ridge_sat)).astype(np.uint8) * 255
        # bridge grass-texture gaps within a line band
        if self.close_k > 1:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                          (self.close_k, self.close_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        # blank the top ROI (sky, goal frame, crowd) to cut Hough noise
        top = int(self.roi_top_frac * self.H)
        mask[:top, :] = 0
        return mask

    def _ground_segments(self, bgr):
        mask = self._white_mask(bgr)
        edges = cv2.Canny(mask, self.canny1, self.canny2)
        lines = cv2.HoughLinesP(edges, 1, math.pi / 180.0, self.hough_thresh,
                                minLineLength=self.hough_min_len,
                                maxLineGap=self.hough_max_gap)
        segs = []
        if lines is None:
            return segs
        for x1, y1, x2, y2 in lines[:, 0, :]:
            w0 = self.proj.unproject_to_ground(float(x1), float(y1))
            w1 = self.proj.unproject_to_ground(float(x2), float(y2))
            if w0 is None or w1 is None:
                continue
            if (math.hypot(w0[0], w0[1]) > self.ground_max_range or
                    math.hypot(w1[0], w1[1]) > self.ground_max_range):
                continue
            segs.append(((float(w0[0]), float(w0[1])),
                         (float(w1[0]), float(w1[1]))))
        return segs

    # ── main ──────────────────────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        if self.proj is None:
            return
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        head = self._lookup_head(t_img)
        if head is None:
            return
        pan, tilt, _ = head
        # base at origin → projector returns ground points in base_link
        self.proj.set_pose(0.0, 0.0, self.base_height, 0.0, pan, tilt)

        # only usable while the EKF prior is fresh enough to break the 90° grid
        prior = None
        if (self._prior_yaw is not None and self._prior_t is not None and
                (self._now() - self._prior_t) <= self.prior_max_age):
            prior = self._prior_yaw

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn('cv_bridge: %s' % e,
                                   throttle_duration_sec=5.0)
            return

        segs = self._ground_segments(bgr)
        res = estimate_heading(segs, prior,
                               min_len_m=self.min_seg_len_m,
                               min_strength=self.min_strength,
                               min_segments=self.min_segments,
                               accept_margin_rad=self.accept_margin)
        if res is None:
            self._n_suppress += 1
            return
        yaw, strength, _ = res
        self._n_pub += 1
        self._strengths.append(strength)
        self._publish(msg.header.stamp, yaw, strength)

    def _publish(self, stamp, yaw, strength):
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()   # wall-time, like pose2
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x = 0.0
        out.pose.pose.position.y = 0.0
        out.pose.pose.orientation.z = math.sin(yaw * 0.5)
        out.pose.pose.orientation.w = math.cos(yaw * 0.5)
        # yaw-only: position variance huge so the EKF ignores x/y; yaw variance
        # tightens with the agreement strength.
        yaw_var = (self.yaw_stddev ** 2) / max(strength, 0.5)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = 1e6      # x, y, z
        cov[21] = cov[28] = 1e6              # roll, pitch
        cov[35] = float(yaw_var)             # yaw
        out.pose.covariance = cov
        self.pub.publish(out)

    def _report(self):
        n = self._n_pub + self._n_suppress
        if n == 0:
            return
        s = np.asarray(self._strengths) if self._strengths else np.array([0.0])
        self.get_logger().info(
            'line-heading: published=%d suppressed=%d (%.0f%% publish) '
            'strength med=%.2f'
            % (self._n_pub, self._n_suppress, 100.0 * self._n_pub / n,
               float(np.median(s))))


def main(args=None):
    rclpy.init(args=args)
    node = LineHeadingNode()
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
