#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
"""Automated stratified sampler for the auto-labeled landmark dataset.

Drives the Webots OP3 through many independent poses via the supervisor
teleport topic (``/robotis_op3/set_pose``, added to op3_extern_controller),
varies the head pan/tilt (active vision), then projects the known field
landmarks into each rendered frame and saves image + YOLO-label + debug
triplets — the SAME projection/labeling path as the interactive capture tool.

Sampling is stratified in three layers (see STRATEGI_PENGUMPULAN_DATA.md):
  A  coverage : robot spread over the whole field, all headings, varied head
  B  landmark-centric : poses deliberately facing each landmark class (corners,
     goals, center, penalty marks, sideline/goal-line T's) at varied distance
     — this is what guarantees the RARE, most valuable classes (X, corners)
  C  distance/edge : near/mid/far strata + some poses with the target pushed
     to the frame edge (partial landmarks)

Head range is taken from the REAL ball-search behavior
(op3_head_control_module: head_pan ∈ [-85,85]°, head_tilt ∈ [-75,30]°; scan
commands ~±51° pan, -60..-19° tilt), extended UP toward +30° for gaze at goals.

Confirm-first: run a SMALL batch (``num_samples:=200``) first; the node writes
``report.txt`` / ``report.json`` / ``report_montage.png`` with the class
distribution + distance histogram + sample overlays. Verify balance BEFORE a
full run.

Prereq: Webots + op3_extern_controller (rebuilt with teleport) running, robot
held in a standing pose (op3_manager). This node commands the head directly, so
disable any head-scan behavior while sampling.
"""

import os
import json
import glob
import math
import time
import random
import threading
from collections import Counter

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String, Float64MultiArray
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

from .field_landmarks import (build_line_intersections, build_goalposts,
                              build_center_circle, CLASS_NAMES,
                              FIELD_HALF_LEN, FIELD_HALF_WID, GOAL_HEIGHT,
                              PENALTY_MARK_DIST)
from .projection import Projector

# Camera height above the field (approx, from URDF chain) — used only to aim
# the head tilt so the target lands near frame center; labels use exact geometry.
_CAM_HEIGHT = 0.46

# Head mechanical limits (deg) — op3_head_control_module.cpp:49-52.
_PAN_MIN, _PAN_MAX = -85.0, 85.0
_TILT_MIN, _TILT_MAX = -75.0, 30.0


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def _ang_diff(a, b):
    d = a - b
    return abs((d + math.pi) % (2.0 * math.pi) - math.pi)


def _yaw_from_quat(z, w):
    return float(np.arctan2(2.0 * w * z, 1.0 - 2.0 * z * z))


class LandmarkDatasetSampler(Node):
    def __init__(self):
        super().__init__('landmark_dataset_sampler')
        p = self.declare_parameter

        # topics
        self.image_topic = str(p('image_topic',
                                 '/robotis_op3/camera/image_raw').value)
        self.info_topic = str(p('camera_info_topic',
                                '/robotis_op3/camera/camera_info').value)
        self.odom_topic = str(p('odom_topic', '/ground_truth/odom').value)
        self.joints_topic = str(p('joint_states_topic',
                                  '/robotis_op3/joint_states').value)
        self.set_pose_topic = str(p('set_pose_topic',
                                    '/robotis_op3/set_pose').value)
        self.head_pan_joint = str(p('head_pan_joint', 'head_pan').value)
        self.head_tilt_joint = str(p('head_tilt_joint', 'head_tilt').value)
        # 'manager' : command the head through op3_manager's head_control_module
        #   (/robotis/head_control/set_joint_states). REQUIRED when op3_manager is
        #   running, because robotis_controller republishes the low-level
        #   *_position/command topics every tick and would override direct writes.
        # 'direct'  : write Float64 to /robotis_op3/head_*_position/command
        #   (only for a bare op3_extern_controller with no manager).
        self.head_cmd_mode = str(p('head_cmd_mode', 'manager').value).lower()
        # head_mode 'aim' (default): tilt/pan aimed per-frame at the target.
        # 'fixed': head held at a CONSTANT tilt/pan for every frame (no per-frame
        # articulation) — body yaw still faces the target so landmarks stay in
        # view horizontally. Removes head motion/settling as a variable so the
        # projection geometry is identical every frame (see the pitch-residual
        # investigation). Vertical framing then depends only on distance.
        self.head_mode = str(p('head_mode', 'aim').value).lower()
        self.fixed_head_tilt_deg = float(p('fixed_head_tilt_deg', -15.0).value)
        self.fixed_head_pan_deg = float(p('fixed_head_pan_deg', 0.0).value)
        self.head_control_topic = str(p(
            'head_control_topic', '/robotis/head_control/set_joint_states').value)
        self.enable_module_topic = str(p(
            'enable_module_topic', '/robotis/enable_ctrl_module').value)
        self.enable_head_module = bool(p('enable_head_module', True).value)

        # output / run
        self.output_dir = os.path.expanduser(
            str(p('output_dir', '~/landmark_dataset').value))
        self.num_samples = int(p('num_samples', 200).value)
        self.seed = int(p('seed', 0).value)
        # resume: continue an interrupted collect. build_samples(num_samples) is
        # deterministic for a given seed, so the poses already saved are exactly
        # samples[0:next_index]. When True we SKIP those and render only the
        # remainder, appending at the next contiguous index (no re-render, no
        # duplicates). Requires the existing files to be the contiguous prefix
        # 0..next_index-1 of the SAME seed/num_samples (trim any orphan tail
        # labels first). Default False so fresh/val/smoke runs are unaffected.
        self.resume = bool(p('resume', False).value)

        # projection / calibration (match the verified capture calibration)
        self.max_range_m = float(p('max_range_m', 9.0).value)
        # Range gate for ground landmarks (junctions/marks/circle); goalposts keep
        # max_range. TAHAP 6: set from numbers, not intuition. The old 5.0 cap was
        # meant to hide a far-horizon "droop" that GATE 1 proved DOESN'T EXIST
        # (perpendicular residual ~2 px, signed 0, out to 9 m). 5.0 starved the
        # backend: only 79.5% of frames had >=2 ground junctions. Coverage plateaus
        # at 7.0 m (95% >=2, 90% >=3 junctions) while boxes stay >=~50 px (still
        # detectable after the detector's downscale). 9.0 adds only tinier far
        # boxes, no extra frame coverage.
        self.ground_max_range_m = float(p('ground_max_range_m', 7.0).value)
        self.min_box_px = float(p('min_box_px', 6.0).value)
        # min_emit_px: floor size (px) that far/tiny boxes are padded to. Scaled
        # to 18 for 1920x1080 capture (was 12 at 1280x720; same relative floor,
        # 1.5x). 8 made small X-marks / far circles too tight to capture; also the
        # min extent below which center_circle is dropped.
        self.min_emit_px = float(p('min_emit_px', 18.0).value)
        # FROZEN calibration for fixed-head 1920x1080 (GATE 1). -5.0 deg is a REAL
        # physical offset (URDF head mount vs Webots render): with it, the
        # perpendicular line-model residual is centered on the painted lines
        # (signed median 0.00 px at every distance 0-9 m, |perp| ~2 px noise
        # floor). Do NOT tune it to chase pixels — the earlier "droop" was box
        # SHAPE (fixed in TAHAP 4), not this. base_z_offset stays 0.0.
        self.pitch_bias_deg = float(p('pitch_bias_deg', -5.0).value)
        self.pan_bias_deg = float(p('pan_bias_deg', 0.0).value)
        self.base_z_offset = float(p('base_z_offset', 0.0).value)

        # layer mix (fractions; normalized)
        # A/coverage now = position-first EVEN coverage (jittered grid over the
        # whole inside-field area) with the head AIMED at a real landmark, so the
        # diverse standing positions also yield landmark-bearing frames. This is
        # the dominant layer (richest position<->frame pairing for localization).
        self.frac_a = float(p('frac_coverage', 0.55).value)
        self.frac_b = float(p('frac_landmark', 0.30).value)
        self.frac_c = float(p('frac_edge', 0.15).value)
        # small fraction of coverage poses that "free-look" (random yaw + head)
        # to keep some hard / near-empty frames for negatives.
        self.frac_free = float(p('frac_free', 0.08).value)
        # debug-overlay thinning: write an overlay only for the first
        # `debug_first` images plus every `debug_every`-th image (labels +
        # metadata are still written for EVERY frame). Halves disk footprint.
        self.debug_first = int(p('debug_first', 300).value)
        self.debug_every = max(1, int(p('debug_every', 25).value))

        # domain randomization (photometric): lighting, grass tint, camera noise.
        # Applied to the SAVED image; labels are geometric so unaffected. Seeded
        # by `seed`, so train/val runs get different distributions.
        self.domain_randomize = bool(p('domain_randomize', False).value)
        self.dr_brightness = float(p('dr_brightness', 0.12).value)  # ± frac of 255
        self.dr_contrast = float(p('dr_contrast', 0.20).value)      # ± alpha
        self.dr_gamma = float(p('dr_gamma', 0.25).value)            # ± gamma
        self.dr_hue = float(p('dr_hue', 6.0).value)                 # ± OpenCV hue
        self.dr_sat = float(p('dr_sat', 0.25).value)                # ± frac
        self.dr_val = float(p('dr_val', 0.15).value)                # ± frac
        # noise/blur kept LOW: field lines are thin, so heavy sensor noise or any
        # blur smears them out of the image. Mild noise only; blur OFF by default.
        self.dr_noise = float(p('dr_noise', 2.0).value)             # gauss sigma
        self.dr_blur_prob = float(p('dr_blur_prob', 0.0).value)

        # sim-to-real RENDER domain randomization: drives real Webots lighting
        # (directional light w/ shadows, ambient, background) via the supervisor
        # topic /robotis_op3/set_lighting. This produces variation photometric
        # post-processing cannot (real shadows, exposure, cast direction).
        # Ranges CLAMPED so the scene is never dark/washed enough to hide lines:
        # ambient floor raised (no near-black), intensity floor > 0 (light never
        # fully off) and ceiling lowered (no blow-out that erases white lines).
        self.sim_dr = bool(p('sim_dr', False).value)
        # _hi caps lowered (amb 1.3→1.1, int 1.8→1.4): the brightest renders
        # washed the scene until grass and robot merged in colour. Floors kept.
        self.dr_sim_amb = [float(p('dr_sim_amb_lo', 0.8).value),
                           float(p('dr_sim_amb_hi', 1.1).value)]
        self.dr_sim_int = [float(p('dr_sim_int_lo', 0.4).value),
                           float(p('dr_sim_int_hi', 1.4).value)]
        self.dr_sim_sky_prob = float(p('dr_sim_sky_prob', 0.5).value)

        # settle handshake
        self.settle_timeout_s = float(p('settle_timeout_s', 4.0).value)
        self.settle_extra_s = float(p('settle_extra_s', 3.0).value)
        # quiet gap AFTER capture, BEFORE the next teleport. Kept small (total
        # cadence settle+gap = 3.25 s); the next frame re-settles anyway.
        self.post_capture_s = float(p('post_capture_s', 0.25).value)
        self.pos_tol_m = float(p('pos_tol_m', 0.04).value)
        self.yaw_tol_deg = float(p('yaw_tol_deg', 2.5).value)
        self.head_tol_deg = float(p('head_tol_deg', 2.5).value)
        # keep robot inside these bounds (still INSIDE the painted lines at
        # ±4.5 / ±3.0; slightly wider than before to enrich edge/corner stances)
        self.place_half_len = float(p('place_half_len', 4.3).value)
        self.place_half_wid = float(p('place_half_wid', 2.8).value)

        self.rng = random.Random(self.seed)

        # landmark model (shared with capture tool)
        self.junctions = build_line_intersections()
        self.goalposts = build_goalposts()
        self.circles = build_center_circle()

        # label -> world position (for the distance histogram)
        self.label2pos = {}
        for j in self.junctions:
            self.label2pos[j.label] = (j.x, j.y, 0.0)
        for gp in self.goalposts:
            self.label2pos[gp.label] = (gp.x, gp.y, gp.top_z * 0.5)
        for c in self.circles:
            self.label2pos[c.label] = (c.cx, c.cy, 0.0)
        # label -> GROUND point (x,y,0) used for GT metadata: world_xy, pixel_uv
        # cross-check and distance are all taken at this single, unambiguous
        # point (post base for goalposts) so re-projection verification is exact.
        self.label2ground = {}
        for j in self.junctions:
            self.label2ground[j.label] = (j.x, j.y, 0.0)
        for gp in self.goalposts:
            self.label2ground[gp.label] = (gp.x, gp.y, 0.0)
        for c in self.circles:
            self.label2ground[c.label] = (c.cx, c.cy, 0.0)

        # Anchors for layer B (landmark-centric): (x, y, kind, weight).
        # L/T appear in nearly every field-facing frame, so they need no help;
        # X (marks), goalposts and the center circle are rare and are weighted
        # up so the batch stays class-balanced (STRATEGI §4).
        fhl, fhw = FIELD_HALF_LEN, FIELD_HALF_WID
        pmx = fhl - PENALTY_MARK_DIST      # penalty mark x (±3.0)
        self.anchors = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                self.anchors.append((sx * fhl, sy * fhw, 'corner', 0.6))   # L
                self.anchors.append((sx * fhl, sy * 1.3, 'post', 2.5))     # post
                self.anchors.append((sx * fhl, sy * 1.5, 'goalline', 0.5))  # T
                self.anchors.append((sx * fhl, sy * 2.5, 'goalline', 0.5))  # T
            self.anchors.append((sx * fhl, 0.0, 'post', 2.0))          # goal mouth
            self.anchors.append((sx * pmx, 0.0, 'mark', 4.0))          # X pen mark
            self.anchors.append((0.0, sx * fhw, 'sideline', 1.2))      # center T
        self.anchors.append((0.0, 0.0, 'mark', 4.0))                  # X center
        self.anchors.append((0.0, 0.0, 'circle', 18.0))              # circle frame
        self._anchor_cw = np.cumsum([a[3] for a in self.anchors])

        # Unified AIM-TARGET list for the position-first coverage layer: from any
        # standing position we pick a real, aim-able landmark to face so the frame
        # is non-empty. Rarer classes are up-weighted so even coverage keeps the
        # batch class-balanced. Each entry: (tx, ty, z_aim, yolo_class, kind).
        AIM_W = {0: 1.0, 1: 1.0, 2: 3.0, 3: 2.5, 4: 4.0}
        self.aim_targets = []
        self.aim_weights = []
        for j in self.junctions:
            self.aim_targets.append((j.x, j.y, 0.0, j.yolo_class, 'ground'))
            self.aim_weights.append(AIM_W.get(j.yolo_class, 1.0))
        for gp in self.goalposts:
            self.aim_targets.append((gp.x, gp.y, gp.top_z * 0.5,
                                     gp.yolo_class, 'post'))
            self.aim_weights.append(AIM_W.get(gp.yolo_class, 1.0))
        for c in self.circles:
            self.aim_targets.append((c.cx, c.cy, 0.0, c.yolo_class, 'circle'))
            self.aim_weights.append(AIM_W.get(c.yolo_class, 1.0))
        self.aim_weights = np.asarray(self.aim_weights, dtype=np.float64)

        # even-coverage position pool (built lazily for the requested count)
        self._pos_pool = []
        self._pos_i = 0

        # state
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._img = None
        self._K = None
        self._wh = None
        self._pose = None
        self._head = [0.0, 0.0]
        self._projector = None
        self._last_lighting = None   # last published render-DR vector (or None)
        self.meta_path = os.path.join(self.output_dir, 'gt_metadata.jsonl')

        # I/O
        self.dir_images = os.path.join(self.output_dir, 'images')
        self.dir_labels = os.path.join(self.output_dir, 'labels')
        self.dir_debug = os.path.join(self.output_dir, 'debug')
        for d in (self.dir_images, self.dir_labels, self.dir_debug):
            os.makedirs(d, exist_ok=True)
        self._write_class_files()
        poses_csv = os.path.join(self.output_dir, 'poses.csv')
        if not os.path.exists(poses_csv):
            with open(poses_csv, 'w') as f:
                f.write('stem,x,y,yaw,head_pan,head_tilt\n')
        self._save_index = self._next_index()

        # stats
        self.stat_class = Counter()
        self.stat_dist = []           # per-detection landmark distance (m)
        self.stat_per_class_dist = {i: [] for i in CLASS_NAMES}
        self.stat_labels_per_img = []
        self.stat_poses_xy = []       # (x,y) of every saved pose (coverage map)
        self.n_saved = 0
        self.n_empty = 0

        # publishers / subscribers
        self.pub_pose = self.create_publisher(Pose2D, self.set_pose_topic, 1)
        self.pub_light = self.create_publisher(
            Float64MultiArray, '/robotis_op3/set_lighting', 1)
        if self.head_cmd_mode == 'manager':
            self.pub_head = self.create_publisher(
                JointState, self.head_control_topic, 1)
            self.pub_enable = self.create_publisher(
                String, self.enable_module_topic, 1)
        else:
            self.pub_pan = self.create_publisher(
                Float64, '/robotis_op3/head_pan_position/command', 1)
            self.pub_tilt = self.create_publisher(
                Float64, '/robotis_op3/head_tilt_position/command', 1)
        self.create_subscription(Image, self.image_topic,
                                 self._cb_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic,
                                 self._cb_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic,
                                 self._cb_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, self.joints_topic,
                                 self._cb_joints, qos_profile_sensor_data)

        self.get_logger().info(
            'Sampler ready: %d junctions + %d goalposts + %d circle; '
            'output=%s (next idx %06d); target=%d samples, seed=%d'
            % (len(self.junctions), len(self.goalposts), len(self.circles),
               self.output_dir, self._save_index, self.num_samples, self.seed))

    # ── callbacks ──────────────────────────────────────────────────────────
    def _cb_image(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc,
                                   throttle_duration_sec=2.0)
            return
        with self._lock:
            self._img = cv_img
            self._wh = (cv_img.shape[1], cv_img.shape[0])

    def _cb_info(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        with self._lock:
            self._K = K

    def _cb_odom(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        with self._lock:
            self._pose = (pos.x, pos.y, pos.z, _yaw_from_quat(ori.z, ori.w))

    def _cb_joints(self, msg):
        pan = tilt = None
        for name, position in zip(msg.name, msg.position):
            if name == self.head_pan_joint:
                pan = position
            elif name == self.head_tilt_joint:
                tilt = position
        with self._lock:
            if pan is not None:
                self._head[0] = pan
            if tilt is not None:
                self._head[1] = tilt

    # ── sampling ───────────────────────────────────────────────────────────
    def _inside(self, x, y):
        return (abs(x) <= self.place_half_len and abs(y) <= self.place_half_wid)

    def _aim_head(self, kind, dist):
        """Return (pan_deg, tilt_deg) aiming the target near frame center."""
        if kind == 'post':
            # look toward the mid-height of the post
            tilt = math.degrees(math.atan2(GOAL_HEIGHT * 0.5 - _CAM_HEIGHT,
                                           max(dist, 0.3)))
        else:
            # ground target: depress the gaze toward it
            tilt = -math.degrees(math.atan2(_CAM_HEIGHT, max(dist, 0.3)))
        tilt += self.rng.uniform(-8.0, 8.0)
        pan = 0.0
        return pan, tilt

    def _weighted_anchor(self):
        r = self.rng.uniform(0.0, float(self._anchor_cw[-1]))
        idx = int(np.searchsorted(self._anchor_cw, r, side='right'))
        idx = min(idx, len(self.anchors) - 1)
        return self.anchors[idx]

    def _sample_layerB(self, edge=False):
        """A pose deliberately facing a random anchor at a stratified distance."""
        ax, ay, kind, _w = self._weighted_anchor()
        if kind == 'circle':
            # the whole circle only frames as a compact ellipse from a narrow
            # distance band; keep it there so the pose actually yields a label
            d = self.rng.uniform(2.3, 4.5)
        else:
            stratum = self.rng.choice(['near', 'mid', 'far'])
            d = {'near': self.rng.uniform(1.0, 2.0),
                 'mid': self.rng.uniform(2.0, 4.0),
                 'far': self.rng.uniform(4.0, 7.0)}[stratum]
        rx = ry = None
        for _ in range(40):
            bearing = self.rng.uniform(-math.pi, math.pi)
            cx = ax - d * math.cos(bearing)
            cy = ay - d * math.sin(bearing)
            if self._inside(cx, cy):
                rx, ry, yaw = cx, cy, bearing
                break
        if rx is None:
            return self._sample_layerA()
        pan_deg, tilt_deg = self._aim_head(kind, d)
        # push target toward the frame edge for layer C by panning the head away
        # — but keep the circle centered (a partial circle won't frame at all)
        if kind == 'circle':
            pan_deg += self.rng.uniform(-6.0, 6.0)
        else:
            pan_deg += self.rng.uniform(-35.0, 35.0) if edge \
                else self.rng.uniform(-10.0, 10.0)
        pan_deg = _clip(pan_deg, _PAN_MIN, _PAN_MAX)
        tilt_deg = _clip(tilt_deg, _TILT_MIN, _TILT_MAX)
        return (rx, ry, yaw, math.radians(pan_deg), math.radians(tilt_deg))

    def _sample_layerA(self):
        """Broad coverage: random position/heading/head over the field."""
        rx = self.rng.uniform(-self.place_half_len, self.place_half_len)
        ry = self.rng.uniform(-self.place_half_wid, self.place_half_wid)
        yaw = self.rng.uniform(-math.pi, math.pi)
        pan_deg = _clip(self.rng.uniform(-55.0, 55.0), _PAN_MIN, _PAN_MAX)
        # bias tilt downward (most landmarks are on the ground) but allow up
        tilt_deg = _clip(self.rng.uniform(-50.0, 15.0), _TILT_MIN, _TILT_MAX)
        return (rx, ry, yaw, math.radians(pan_deg), math.radians(tilt_deg))

    def _build_position_pool(self, n):
        """Even (low-discrepancy) standing positions over the inside-field area.

        A jittered grid: pick nx,ny so nx*ny >= n with the cell aspect matching
        the field, then jitter each cell centre inside its own cell. This spreads
        positions MUCH more uniformly than plain uniform-random sampling (no
        clumping), which is what makes the teleport spots "berkali-kali lipat
        beragam". Shuffled; recycled with fresh jitter if n exceeds the cells.
        """
        w = 2.0 * self.place_half_len
        h = 2.0 * self.place_half_wid
        nx = max(1, int(round(math.sqrt(n * w / h))))
        ny = max(1, int(math.ceil(n / nx)))
        cw, ch = w / nx, h / ny
        pool = []
        for iy in range(ny):
            for ix in range(nx):
                cx = -self.place_half_len + (ix + 0.5) * cw
                cy = -self.place_half_wid + (iy + 0.5) * ch
                jx = cx + self.rng.uniform(-0.5, 0.5) * cw
                jy = cy + self.rng.uniform(-0.5, 0.5) * ch
                pool.append((_clip(jx, -self.place_half_len, self.place_half_len),
                             _clip(jy, -self.place_half_wid, self.place_half_wid)))
        self.rng.shuffle(pool)
        return pool

    def _next_position(self):
        if self._pos_i >= len(self._pos_pool):
            # exhausted: re-jitter the grid for another even pass
            self._pos_pool = self._build_position_pool(len(self._pos_pool) or 1)
            self._pos_i = 0
        pos = self._pos_pool[self._pos_i]
        self._pos_i += 1
        return pos

    def _aimable_targets(self, rx, ry):
        """Indices of aim-able landmarks from (rx,ry) + their per-target distance."""
        idxs, dists = [], []
        for k, (tx, ty, _tz, _cls, kind) in enumerate(self.aim_targets):
            d = math.hypot(tx - rx, ty - ry)
            if d < 0.8:
                continue
            if kind == 'post':
                if d > self.max_range_m:
                    continue
            elif kind == 'circle':
                if not (2.3 <= d <= 4.5):
                    continue
            else:  # ground junction/mark
                if d > self.ground_max_range_m:
                    continue
            idxs.append(k)
            dists.append(d)
        return idxs, dists

    def _sample_coverage(self):
        """Position-first: an evenly-spread standing pose that FACES a real,
        rarity-weighted landmark so the frame is non-empty and the true pose is
        richly paired with its view (localization data enrichment)."""
        rx, ry = self._next_position()
        # a small fraction free-looks (random yaw/head) → hard/near-empty frames
        if self.rng.random() < self.frac_free:
            yaw = self.rng.uniform(-math.pi, math.pi)
            pan_deg = _clip(self.rng.uniform(-55.0, 55.0), _PAN_MIN, _PAN_MAX)
            tilt_deg = _clip(self.rng.uniform(-50.0, 15.0), _TILT_MIN, _TILT_MAX)
            return (rx, ry, yaw, math.radians(pan_deg), math.radians(tilt_deg))
        idxs, dists = self._aimable_targets(rx, ry)
        if not idxs:
            return self._sample_layerA()
        w = self.aim_weights[idxs]
        pick = int(self.rng.choices(range(len(idxs)), weights=w, k=1)[0])
        k = idxs[pick]
        d = dists[pick]
        tx, ty, _tz, _cls, kind = self.aim_targets[k]
        yaw = math.atan2(ty - ry, tx - rx) + math.radians(self.rng.uniform(-8.0, 8.0))
        _pan0, tilt_deg = self._aim_head(kind, d)
        pan_deg = _clip(self.rng.uniform(-12.0, 12.0), _PAN_MIN, _PAN_MAX)
        tilt_deg = _clip(tilt_deg, _TILT_MIN, _TILT_MAX)
        return (rx, ry, yaw, math.radians(pan_deg), math.radians(tilt_deg))

    def build_samples(self, n):
        tot = self.frac_a + self.frac_b + self.frac_c
        na = int(round(n * self.frac_a / tot))
        nc = int(round(n * self.frac_c / tot))
        nb = n - na - nc
        # even-coverage position pool sized to the coverage-layer count
        self._pos_pool = self._build_position_pool(max(na, 1))
        self._pos_i = 0
        samples = ([self._sample_coverage() for _ in range(na)] +
                   [self._sample_layerB(edge=False) for _ in range(nb)] +
                   [self._sample_layerB(edge=True) for _ in range(nc)])
        self.rng.shuffle(samples)
        if self.head_mode == 'fixed':
            # pin the head for every frame; keep each pose's body yaw (still
            # faces its target) so landmarks remain in view horizontally.
            ft = math.radians(_clip(self.fixed_head_tilt_deg, _TILT_MIN, _TILT_MAX))
            fp = math.radians(_clip(self.fixed_head_pan_deg, _PAN_MIN, _PAN_MAX))
            samples = [(x, y, yaw, fp, ft) for (x, y, yaw, _p, _t) in samples]
            self.get_logger().info(
                'head_mode=fixed: tilt=%.1f pan=%.1f deg for all frames'
                % (self.fixed_head_tilt_deg, self.fixed_head_pan_deg))
        return samples

    # ── teleport + capture ───────────────────────────────────────────────────
    def _ensure_projector(self, K, w, h):
        if (self._projector is None or
                self._projector.W != w or self._projector.H != h):
            self._projector = Projector(
                K, w, h, max_range_m=self.max_range_m,
                ground_max_range_m=self.ground_max_range_m,
                min_box_px=self.min_box_px, min_emit_px=self.min_emit_px,
                base_z_offset=self.base_z_offset,
                pitch_bias_deg=self.pitch_bias_deg,
                pan_bias_deg=self.pan_bias_deg)
        else:
            self._projector.fx = float(K[0, 0])
            self._projector.fy = float(K[1, 1])
            self._projector.cx = float(K[0, 2])
            self._projector.cy = float(K[1, 2])
        return self._projector

    def _command_head(self, pan, tilt):
        if self.head_cmd_mode == 'manager':
            js = JointState()
            js.name = [self.head_pan_joint, self.head_tilt_joint]
            js.position = [float(pan), float(tilt)]
            self.pub_head.publish(js)
        else:
            self.pub_pan.publish(Float64(data=float(pan)))
            self.pub_tilt.publish(Float64(data=float(tilt)))

    def _sample_lighting(self):
        """Random render lighting → [amb,int,cr,cg,cb,dx,dy,dz,sr,sg,sb]."""
        r = self.rng
        amb = r.uniform(self.dr_sim_amb[0], self.dr_sim_amb[1])
        inten = r.uniform(self.dr_sim_int[0], self.dr_sim_int[1])
        # color temperature: t=0 warm (less blue), t=1 cool (less red)
        t = r.uniform(0.0, 1.0)
        cr = 1.0 - 0.15 * t
        cb = 1.0 - 0.15 * (1.0 - t)
        cg = r.uniform(0.92, 1.0)
        # sun direction: random azimuth, elevation 25-70° above, pointing down
        az = r.uniform(-math.pi, math.pi)
        el = r.uniform(math.radians(25.0), math.radians(70.0))
        dx = math.cos(el) * math.cos(az)
        dy = math.cos(el) * math.sin(az)
        dz = -math.sin(el)
        # background: usually black, sometimes a lifted grey/blue ambient sky
        if r.random() < self.dr_sim_sky_prob:
            base = r.uniform(0.02, 0.25)
            sr, sg, sb = (base * r.uniform(0.7, 1.0),
                          base * r.uniform(0.8, 1.0),
                          base * r.uniform(0.9, 1.2))
        else:
            sr = sg = sb = 0.0
        return [amb, inten, cr, cg, cb, dx, dy, dz, sr, sg, sb]

    def _publish_lighting(self, vals):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in vals]
        self.pub_light.publish(msg)

    def teleport_and_settle(self, sample):
        x, y, yaw, pan, tilt = sample
        self.pub_pose.publish(Pose2D(x=float(x), y=float(y), theta=float(yaw)))
        self._command_head(pan, tilt)
        if self.sim_dr:
            # randomize the render before the settle wait so it re-renders
            self._last_lighting = self._sample_lighting()
            self._publish_lighting(self._last_lighting)

        t0 = time.time()
        settled = False
        while time.time() - t0 < self.settle_timeout_s:
            time.sleep(0.05)
            with self._lock:
                pose = self._pose
                head = list(self._head)
            if pose is None:
                continue
            if (abs(pose[0] - x) < self.pos_tol_m and
                    abs(pose[1] - y) < self.pos_tol_m and
                    _ang_diff(pose[3], yaw) < math.radians(self.yaw_tol_deg) and
                    abs(head[0] - pan) < math.radians(self.head_tol_deg) and
                    abs(head[1] - tilt) < math.radians(self.head_tol_deg)):
                settled = True
                break
        # let the render refresh & any residual bob damp out
        time.sleep(self.settle_extra_s)
        return settled

    def compute(self):
        with self._lock:
            img = None if self._img is None else self._img.copy()
            K = None if self._K is None else self._K.copy()
            wh = self._wh
            pose = self._pose
            head = list(self._head)
        if img is None or K is None or wh is None or pose is None:
            return None, None, None, None, None
        proj = self._ensure_projector(K, wh[0], wh[1])
        proj.set_pose(pose[0], pose[1], pose[2], pose[3], head[0], head[1])
        dets = proj.project_all(self.junctions, self.goalposts, self.circles)
        campos = proj._cam_pos.copy()
        return img, dets, pose, head, campos

    def _augment(self, img):
        """Photometric domain randomization (lighting / grass tint / noise)."""
        if not self.domain_randomize:
            return img
        r = self.rng
        out = img.astype(np.float32)
        # lighting: contrast (alpha) + brightness (beta)
        alpha = r.uniform(1.0 - self.dr_contrast, 1.0 + self.dr_contrast)
        beta = r.uniform(-self.dr_brightness, self.dr_brightness) * 255.0
        out = np.clip(out * alpha + beta, 0, 255)
        # gamma
        if self.dr_gamma > 0:
            g = r.uniform(1.0 - self.dr_gamma, 1.0 + self.dr_gamma)
            lut = np.clip(((np.arange(256) / 255.0) ** (1.0 / g)) * 255.0,
                          0, 255).astype(np.uint8)
            out = lut[out.astype(np.uint8)]
        else:
            out = out.astype(np.uint8)
        # grass tint / white balance: HSV jitter
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[..., 0] = (hsv[..., 0] + r.uniform(-self.dr_hue, self.dr_hue)) % 180
        hsv[..., 1] = np.clip(hsv[..., 1] *
                              r.uniform(1.0 - self.dr_sat, 1.0 + self.dr_sat),
                              0, 255)
        hsv[..., 2] = np.clip(hsv[..., 2] *
                              r.uniform(1.0 - self.dr_val, 1.0 + self.dr_val),
                              0, 255)
        out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        # camera noise
        if self.dr_noise > 0:
            noise = np.random.normal(0.0, self.dr_noise, out.shape)
            out = np.clip(out.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        # occasional slight blur
        if r.random() < self.dr_blur_prob:
            out = cv2.GaussianBlur(out, (3, 3), 0)
        return out

    @staticmethod
    def _mat2quat(R):
        """3x3 rotation matrix -> quaternion (w, x, y, z)."""
        t = R[0, 0] + R[1, 1] + R[2, 2]
        if t > 0.0:
            s = math.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [float(w), float(x), float(y), float(z)]

    def _write_metadata(self, stem, dets, pose, head):
        """Append one GT-metadata line (JSONL). SEPARATE from YOLO labels."""
        proj = self._projector
        w, h = self._wh
        Tmc = proj._T_map_cam
        cam_pos = [float(v) for v in Tmc[:3, 3]]
        cam_quat = self._mat2quat(Tmc[:3, :3])
        lm = []
        for d in dets:
            g = self.label2ground.get(d.label)
            if g is None:
                continue
            pt = np.asarray(g, dtype=np.float64)
            uv, _ = proj._project(pt[None, :], range_limit=self.max_range_m)
            dist = float(np.linalg.norm(pt - proj._cam_pos))
            bx = ((d.x1 + d.x2) * 0.5) / w
            by = ((d.y1 + d.y2) * 0.5) / h
            bw = (d.x2 - d.x1) / w
            bh = (d.y2 - d.y1) / h
            lm.append({
                'class': d.class_name,
                'label': d.label,
                'world_xy': [float(g[0]), float(g[1])],
                'pixel_uv': [float(uv[0, 0]), float(uv[0, 1])],
                'bbox_norm': [round(bx, 6), round(by, 6),
                              round(bw, 6), round(bh, 6)],
                'distance_m': round(dist, 4),
            })
        rec = {
            'image': stem + '.png',
            'gt_robot_pose': {'x': round(float(pose[0]), 4),
                              'y': round(float(pose[1]), 4),
                              'yaw_deg': round(math.degrees(pose[3]), 3)},
            'camera': {
                'head_pan_deg': round(math.degrees(head[0]), 3),
                'head_tilt_deg': round(math.degrees(head[1]), 3),
                'cam_world_pos': [round(v, 5) for v in cam_pos],
                'cam_world_quat': [round(v, 6) for v in cam_quat],
                # intrinsics + image size: makes the sidecar fully self-describing
                # so relabel.py can regenerate YOLO labels WITHOUT Webots/render.
                'K_fx_fy_cx_cy': [round(proj.fx, 4), round(proj.fy, 4),
                                  round(proj.cx, 4), round(proj.cy, 4)],
                'image_wh': [int(w), int(h)],
            },
            # projector params used at generation (provenance; relabel may override)
            'projector': {
                'min_emit_px': round(proj.min_emit_px, 3),
                'ground_max_range_m': round(proj.ground_max_range, 3),
                'max_range_m': round(proj.max_range, 3),
                'pitch_bias_deg': round(self.pitch_bias_deg, 4),
                'base_z_offset': round(self.base_z_offset, 4),
            },
            'landmarks': lm,
            'domain_rand': {
                'seed': self.seed,
                'sim_dr': self.sim_dr,
                'photometric': self.domain_randomize,
                'lighting': ([round(v, 4) for v in self._last_lighting]
                             if (self.sim_dr and self._last_lighting) else None),
            },
        }
        with open(self.meta_path, 'a') as f:
            f.write(json.dumps(rec) + '\n')

    def save(self, img, dets, pose, head, campos):
        idx = self._save_index
        w, h = img.shape[1], img.shape[0]
        stem = '%06d' % idx
        lines = []
        # Write GT metadata BEFORE augmenting the image (projection uses the
        # current projector pose; augmentation only touches pixels, not geometry).
        self._write_metadata(stem, dets, pose, head)
        img = self._augment(img)   # training image gets domain randomization
        for d in dets:
            xc = ((d.x1 + d.x2) * 0.5) / w
            yc = ((d.y1 + d.y2) * 0.5) / h
            bw = (d.x2 - d.x1) / w
            bh = (d.y2 - d.y1) / h
            lines.append('%d %.6f %.6f %.6f %.6f' %
                         (d.class_id, xc, yc, bw, bh))
            self.stat_class[d.class_id] += 1
            pos = self.label2pos.get(d.label)
            if pos is not None:
                dist = float(np.linalg.norm(np.asarray(pos) - campos))
                self.stat_dist.append(dist)
                self.stat_per_class_dist[d.class_id].append(dist)

        cv2.imwrite(os.path.join(self.dir_images, stem + '.png'), img)
        label_path = os.path.join(self.dir_labels, stem + '.txt')
        with open(label_path, 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        # thin the debug overlays (labels + metadata still written for every
        # frame); keeps disk footprint down on large full runs.
        if idx < self.debug_first or (idx % self.debug_every) == 0:
            self._write_debug_overlay(label_path, img, stem, w, h)
        # per-pose metadata (enables re-projection / re-calibration later)
        with open(os.path.join(self.output_dir, 'poses.csv'), 'a') as f:
            f.write('%s,%.4f,%.4f,%.5f,%.5f,%.5f\n' %
                    (stem, pose[0], pose[1], pose[3], head[0], head[1]))
        self.stat_labels_per_img.append(len(lines))
        self.stat_poses_xy.append((float(pose[0]), float(pose[1])))
        self._save_index += 1
        if lines:
            self.n_saved += 1
        else:
            self.n_empty += 1

    _DEBUG_COLORS = {0: (0, 220, 0), 1: (0, 160, 255), 2: (255, 0, 200),
                     3: (255, 220, 0), 4: (0, 255, 255)}

    def _write_debug_overlay(self, label_path, img, stem, w, h):
        """Draw the debug overlay by RE-READING the written YOLO .txt and
        denormalising — so the overlay provably reflects the label FILE, not an
        in-memory box (TAHAP 0 honesty requirement)."""
        overlay = img.copy()
        with open(label_path) as f:
            for line in f:
                p = line.split()
                if len(p) != 5:
                    continue
                cid = int(float(p[0]))
                xc, yc, bw, bh = (float(p[1]) * w, float(p[2]) * h,
                                  float(p[3]) * w, float(p[4]) * h)
                x1, y1 = int(round(xc - bw * 0.5)), int(round(yc - bh * 0.5))
                x2, y2 = int(round(xc + bw * 0.5)), int(round(yc + bh * 0.5))
                color = self._DEBUG_COLORS.get(cid, (255, 255, 255))
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                cv2.putText(overlay, CLASS_NAMES.get(cid, str(cid)),
                            (x1, max(0, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.dir_debug, stem + '.png'), overlay)

    # ── run ───────────────────────────────────────────────────────────────────
    def run(self):
        # wait for the first camera + pose so we don't teleport blind
        self.get_logger().info('Waiting for camera/pose/joint topics...')
        t0 = time.time()
        while time.time() - t0 < 30.0:
            with self._lock:
                ready = (self._img is not None and self._K is not None and
                         self._pose is not None)
            if ready:
                break
            time.sleep(0.1)
        else:
            self.get_logger().error('Timed out waiting for topics; is Webots + '
                                    'op3_extern_controller running?')
            return

        # Hand the head joints to op3_manager's head_control_module so our
        # /robotis/head_control/set_joint_states commands take effect.
        if self.head_cmd_mode == 'manager' and self.enable_head_module:
            for _ in range(3):
                self.pub_enable.publish(String(data='head_control_module'))
                time.sleep(0.3)
            self.get_logger().info(
                'enabled head_control_module for head commands')

        # Fixed-head mode: drive the head to its constant angle ONCE and let it
        # come fully to rest before any capture, so it never moves during the run.
        if self.head_mode == 'fixed':
            ft = math.radians(_clip(self.fixed_head_tilt_deg, _TILT_MIN, _TILT_MAX))
            fp = math.radians(_clip(self.fixed_head_pan_deg, _PAN_MIN, _PAN_MAX))
            for _ in range(4):
                self._command_head(fp, ft)
                time.sleep(0.4)
            self.get_logger().info('fixed head parked at tilt=%.1f pan=%.1f deg'
                                   % (self.fixed_head_tilt_deg,
                                      self.fixed_head_pan_deg))

        # Restore baseline lighting when sim-DR is OFF (a previous sim-DR run may
        # have left the render lights changed until the next Webots restart).
        if not self.sim_dr:
            self._publish_lighting([1.0, 0.0, 1.0, 1.0, 1.0,
                                    0.3, -0.4, -1.0, 0.0, 0.0, 0.0])
            time.sleep(0.2)
        else:
            self.get_logger().info('sim-to-real render DR enabled')

        samples = self.build_samples(self.num_samples)
        # Resume: the first `resume_skip` deterministic poses are already on disk
        # (contiguous prefix 0..next_index-1). Skip them (no teleport/render) and
        # append the remainder at the next index. resume_skip is 0 unless resume
        # is requested, so fresh runs are unaffected.
        resume_skip = self._save_index if self.resume else 0
        if resume_skip:
            self.get_logger().info(
                'RESUME: %d poses already saved; skipping to sample %d/%d'
                % (resume_skip, resume_skip, len(samples)))
        self.get_logger().info('Generated %d stratified poses; sampling...'
                               % len(samples))
        for i, s in enumerate(samples):
            if not rclpy.ok():
                break
            if i < resume_skip:
                continue
            settled = self.teleport_and_settle(s)
            img, dets, pose, head, campos = self.compute()
            if img is None:
                self.get_logger().warn('sample %d: no frame, skipping' % i)
                continue
            self.save(img, dets, pose, head, campos)
            if self.post_capture_s > 0.0 and i < len(samples) - 1:
                time.sleep(self.post_capture_s)
            if (i + 1) % 20 == 0 or i == len(samples) - 1:
                self.get_logger().info(
                    '[%d/%d] saved (settled=%s) dets=%d  class=%s'
                    % (i + 1, len(samples), settled, len(dets),
                       dict(self.stat_class)))
        self.write_report()

    # ── report ────────────────────────────────────────────────────────────────
    def _dist_hist(self, dists):
        edges = [0, 1, 2, 3, 4, 5, 6, 7, 99]
        h = [0] * (len(edges) - 1)
        for d in dists:
            for k in range(len(edges) - 1):
                if edges[k] <= d < edges[k + 1]:
                    h[k] += 1
                    break
        labels = ['%d-%dm' % (edges[k], edges[k + 1])
                  for k in range(len(edges) - 2)] + ['7m+']
        return list(zip(labels, h))

    def write_report(self):
        total_labels = sum(self.stat_class.values())
        by_name = {CLASS_NAMES[i]: self.stat_class.get(i, 0)
                   for i in sorted(CLASS_NAMES)}
        report = {
            'output_dir': self.output_dir,
            'images_with_labels': self.n_saved,
            'images_empty': self.n_empty,
            'total_labels': total_labels,
            'class_counts': by_name,
            'labels_per_image_mean': (float(np.mean(self.stat_labels_per_img))
                                      if self.stat_labels_per_img else 0.0),
            'distance_hist_all': self._dist_hist(self.stat_dist),
            'distance_hist_by_class': {
                CLASS_NAMES[i]: self._dist_hist(self.stat_per_class_dist[i])
                for i in sorted(CLASS_NAMES)},
        }
        with open(os.path.join(self.output_dir, 'report.json'), 'w') as f:
            json.dump(report, f, indent=2)

        maxc = max(by_name.values()) if by_name else 0
        weak = [n for n, c in by_name.items() if maxc and c < 0.5 * maxc]
        lines = []
        lines.append('=== landmark dataset sampling report ===')
        lines.append('output_dir        : %s' % self.output_dir)
        lines.append('images w/ labels  : %d' % self.n_saved)
        lines.append('images empty      : %d' % self.n_empty)
        lines.append('total labels      : %d' % total_labels)
        lines.append('labels / image    : %.2f'
                     % report['labels_per_image_mean'])
        lines.append('')
        lines.append('class distribution:')
        for n, c in by_name.items():
            bar = '#' * int(40 * c / maxc) if maxc else ''
            lines.append('  %-14s %6d  %s' % (n, c, bar))
        if weak:
            lines.append('')
            lines.append('WEAK classes (< 50%% of max): %s' % ', '.join(weak))
            lines.append('  -> add layer-B batches for these before full run.')
        lines.append('')
        lines.append('distance histogram (all detections):')
        for lbl, cnt in report['distance_hist_all']:
            lines.append('  %-6s %5d' % (lbl, cnt))
        # position spread (localization data-richness): nearest-neighbour spacing
        nn_min, nn_med = self._nn_spacing()
        if nn_min is not None:
            lines.append('')
            lines.append('position spread (%d poses over ±%.1f×±%.1f m):'
                         % (len(self.stat_poses_xy), self.place_half_len,
                            self.place_half_wid))
            lines.append('  nearest-neighbour spacing  min=%.3fm  median=%.3fm'
                         % (nn_min, nn_med))
            report['pose_nn_min_m'] = round(nn_min, 4)
            report['pose_nn_median_m'] = round(nn_med, 4)
            with open(os.path.join(self.output_dir, 'report.json'), 'w') as f:
                json.dump(report, f, indent=2)
        text = '\n'.join(lines) + '\n'
        with open(os.path.join(self.output_dir, 'report.txt'), 'w') as f:
            f.write(text)
        self.get_logger().info('\n' + text)
        self._write_montage()
        self._write_coverage_map()

    def _nn_spacing(self):
        """Min / median nearest-neighbour distance among saved (x,y) poses."""
        pts = np.asarray(self.stat_poses_xy, dtype=np.float64)
        if len(pts) < 2:
            return None, None
        # subsample for O(n^2) safety on big runs
        if len(pts) > 2000:
            sel = np.random.RandomState(0).choice(len(pts), 2000, replace=False)
            pts = pts[sel]
        d2 = ((pts[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
        np.fill_diagonal(d2, np.inf)
        nn = np.sqrt(d2.min(axis=1))
        return float(nn.min()), float(np.median(nn))

    def _write_coverage_map(self, scale=140):
        """Scatter of every standing (x,y) pose over the field rectangle — direct
        visual proof that teleport positions are spread, not clumped."""
        if not self.stat_poses_xy:
            return
        fhl, fhw = FIELD_HALF_LEN, FIELD_HALF_WID
        pad = 0.4
        W = int((2 * (fhl + pad)) * scale)
        H = int((2 * (fhw + pad)) * scale)
        img = np.full((H, W, 3), 30, dtype=np.uint8)

        def to_px(x, y):
            u = int((x + fhl + pad) * scale)
            v = int((fhw + pad - y) * scale)   # +y up
            return u, v

        # field outline + centre line/circle
        cv2.rectangle(img, to_px(-fhl, fhw), to_px(fhl, -fhw), (80, 160, 80), 2)
        cv2.line(img, to_px(0, fhw), to_px(0, -fhw), (80, 160, 80), 1)
        cv2.circle(img, to_px(0.0, 0.0), int(0.75 * scale), (80, 160, 80), 1)
        for pt in self.stat_poses_xy:
            u, v = to_px(pt[0], pt[1])
            cv2.circle(img, (u, v), 2, (0, 200, 255), -1)
        out = os.path.join(self.output_dir, 'coverage_map.png')
        cv2.imwrite(out, img)
        self.get_logger().info('coverage map: %s' % out)

    def _write_montage(self, cols=4, rows=4, cell=320):
        debug_files = sorted(glob.glob(os.path.join(self.dir_debug, '*.png')))
        if not debug_files:
            return
        pick = debug_files[-min(len(debug_files), cols * rows):]
        canvas = np.zeros((rows * cell, cols * cell, 3), dtype=np.uint8)
        for k, fp in enumerate(pick):
            im = cv2.imread(fp)
            if im is None:
                continue
            im = cv2.resize(im, (cell, cell))
            r, c = divmod(k, cols)
            canvas[r * cell:(r + 1) * cell, c * cell:(c + 1) * cell] = im
        out = os.path.join(self.output_dir, 'report_montage.png')
        cv2.imwrite(out, canvas)
        self.get_logger().info('montage: %s' % out)

    # ── helpers ─────────────────────────────────────────────────────────────
    def _next_index(self):
        existing = glob.glob(os.path.join(self.dir_labels, '*.txt'))
        existing += glob.glob(os.path.join(self.dir_images, '*.png'))
        mx = -1
        for pth in existing:
            stem = os.path.splitext(os.path.basename(pth))[0]
            if stem.isdigit():
                mx = max(mx, int(stem))
        return mx + 1

    def _write_class_files(self):
        names = [CLASS_NAMES[i] for i in sorted(CLASS_NAMES)]
        with open(os.path.join(self.output_dir, 'classes.txt'), 'w') as f:
            f.write('\n'.join(names) + '\n')
        with open(os.path.join(self.output_dir, 'data.yaml'), 'w') as f:
            f.write('# Auto-labeled OP3 Webots landmark dataset\n')
            f.write('path: %s\n' % self.output_dir)
            f.write('train: images\n')
            f.write('val: images\n')
            f.write('nc: %d\n' % len(names))
            f.write('names: [%s]\n' % ', '.join("'%s'" % n for n in names))


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDatasetSampler()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
