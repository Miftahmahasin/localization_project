#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
"""Interactive auto-labeled landmark dataset capture (Webots, manual posing).

You position the OP3 robot MANUALLY in the Webots editor. This tool reads the
live ground-truth pose + head angles, projects the known field landmarks
(L/T/X junctions and goalposts) into the camera image, and shows a LIVE overlay
so you can verify the labels before saving. Press ``s`` to save an
image + YOLO-label + debug-overlay triplet.

Topics (defaults, Webots):
  sub  /robotis_op3/camera/image_raw     sensor_msgs/Image (BGRA8)
  sub  /robotis_op3/camera/camera_info   sensor_msgs/CameraInfo  (K)
  sub  /ground_truth/odom                nav_msgs/Odometry  (map→base_link)
  sub  /robotis_op3/joint_states         sensor_msgs/JointState (head_pan/tilt)
  pub  /robotis_op3/head_{pan,tilt}_position/command  (optional head control)

Keys (in the OpenCV window):
  s        save current frame (image + label + debug overlay)
  q / ESC  quit
  a / d    head pan  + / -   (if head control enabled)
  w / x    head tilt + / -   (if head control enabled)
  [ / ]    pitch_bias  - / +  (deg)  — live calibration
  ; / '    base_z_offset - / + (m)   — live calibration
  r        reset calibration offsets to 0

Output layout (under ~output_dir):
  images/NNNNNN.png   labels/NNNNNN.txt   debug/NNNNNN.png
  classes.txt   data.yaml
"""

import os
import glob
import threading

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image, CameraInfo, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from cv_bridge import CvBridge

from .field_landmarks import (build_line_intersections, build_goalposts,
                              build_center_circle, CLASS_NAMES)
from .projection import Projector

# BGR colors per class id.
_CLASS_COLOR = {
    0: (0, 220, 0),      # L  — green
    1: (0, 160, 255),    # T  — orange
    2: (255, 0, 200),    # X  — magenta
    3: (255, 220, 0),    # goalpost — cyan
    4: (0, 255, 255),    # center_circle — yellow
}


def _yaw_from_quat(z: float, w: float) -> float:
    """Planar yaw from a (qz, qw) quaternion (qx=qy=0 for ground truth)."""
    return float(np.arctan2(2.0 * w * z, 1.0 - 2.0 * z * z))


class LandmarkDatasetCapture(Node):
    def __init__(self):
        super().__init__('landmark_dataset_capture')

        # ── parameters ────────────────────────────────────────────────────────
        p = self.declare_parameter
        self.image_topic = str(p('image_topic',
                                 '/robotis_op3/camera/image_raw').value)
        self.info_topic = str(p('camera_info_topic',
                                '/robotis_op3/camera/camera_info').value)
        self.odom_topic = str(p('odom_topic', '/ground_truth/odom').value)
        self.joints_topic = str(p('joint_states_topic',
                                  '/robotis_op3/joint_states').value)
        self.output_dir = os.path.expanduser(
            str(p('output_dir', '~/landmark_dataset').value))
        self.head_pan_joint = str(p('head_pan_joint', 'head_pan').value)
        self.head_tilt_joint = str(p('head_tilt_joint', 'head_tilt').value)
        self.max_range_m = float(p('max_range_m', 9.0).value)
        self.min_box_px = float(p('min_box_px', 6.0).value)
        self.base_z_offset = float(p('base_z_offset', 0.0).value)
        self.pitch_bias_deg = float(p('pitch_bias_deg', 0.0).value)
        self.pan_bias_deg = float(p('pan_bias_deg', 0.0).value)
        self.enable_head_control = bool(p('enable_head_control', True).value)
        self.head_step_deg = float(p('head_step_deg', 3.0).value)

        # ── landmark model ────────────────────────────────────────────────────
        self.junctions = build_line_intersections()
        self.goalposts = build_goalposts()
        self.circles = build_center_circle()
        self.get_logger().info(
            'Loaded %d junctions + %d goalposts + %d center-circle'
            % (len(self.junctions), len(self.goalposts), len(self.circles)))

        # ── state (latest messages) ───────────────────────────────────────────
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._img = None            # latest cv bgr image
        self._K = None              # 3x3 intrinsics
        self._img_wh = None         # (w, h)
        self._pose = None           # (x, y, z, yaw)
        self._head = [0.0, 0.0]     # [pan, tilt]
        self._projector = None

        # commanded head targets (for optional head control)
        self._pan_cmd = 0.0
        self._tilt_cmd = 0.0

        # ── I/O dirs & index ──────────────────────────────────────────────────
        self.dir_images = os.path.join(self.output_dir, 'images')
        self.dir_labels = os.path.join(self.output_dir, 'labels')
        self.dir_debug = os.path.join(self.output_dir, 'debug')
        for d in (self.dir_images, self.dir_labels, self.dir_debug):
            os.makedirs(d, exist_ok=True)
        self._write_class_files()
        self._save_index = self._next_index()
        self.get_logger().info('Output dir: %s (next index %06d)'
                               % (self.output_dir, self._save_index))

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Image, self.image_topic,
                                 self._cb_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic,
                                 self._cb_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic,
                                 self._cb_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, self.joints_topic,
                                 self._cb_joints, qos_profile_sensor_data)

        # ── head-control publishers (optional) ────────────────────────────────
        if self.enable_head_control:
            self.pub_pan = self.create_publisher(
                Float64, '/robotis_op3/head_pan_position/command', 1)
            self.pub_tilt = self.create_publisher(
                Float64, '/robotis_op3/head_tilt_position/command', 1)

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc,
                                   throttle_duration_sec=2.0)
            return
        with self._lock:
            self._img = cv_img
            self._img_wh = (cv_img.shape[1], cv_img.shape[0])

    def _cb_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        with self._lock:
            self._K = K

    def _cb_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = _yaw_from_quat(ori.z, ori.w)
        with self._lock:
            self._pose = (pos.x, pos.y, pos.z, yaw)

    def _cb_joints(self, msg: JointState):
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

    # ── projection ────────────────────────────────────────────────────────────
    def _ensure_projector(self, K, w, h):
        if (self._projector is None or
                self._projector.W != w or self._projector.H != h):
            self._projector = Projector(
                K, w, h, max_range_m=self.max_range_m,
                min_box_px=self.min_box_px, base_z_offset=self.base_z_offset,
                pitch_bias_deg=self.pitch_bias_deg,
                pan_bias_deg=self.pan_bias_deg)
        else:
            # keep intrinsics fresh + live-tunable calibration
            self._projector.fx = float(K[0, 0])
            self._projector.fy = float(K[1, 1])
            self._projector.cx = float(K[0, 2])
            self._projector.cy = float(K[1, 2])
            self._projector.base_z_offset = self.base_z_offset
            self._projector.pitch_bias = np.radians(self.pitch_bias_deg)
            self._projector.pan_bias = np.radians(self.pan_bias_deg)
        return self._projector

    def compute_detections(self):
        """Snapshot state, project, return (image, detections, pose, head)."""
        with self._lock:
            img = None if self._img is None else self._img.copy()
            K = None if self._K is None else self._K.copy()
            wh = self._img_wh
            pose = self._pose
            head = list(self._head)
        if img is None or K is None or wh is None or pose is None:
            return img, None, pose, head
        proj = self._ensure_projector(K, wh[0], wh[1])
        proj.set_pose(pose[0], pose[1], pose[2], pose[3], head[0], head[1])
        dets = proj.project_all(self.junctions, self.goalposts, self.circles)
        return img, dets, pose, head

    # ── drawing / saving ──────────────────────────────────────────────────────
    def draw_overlay(self, img, dets, pose, head):
        vis = img.copy()
        if dets:
            for d in dets:
                color = _CLASS_COLOR.get(d.class_id, (255, 255, 255))
                x1, y1, x2, y2 = (int(d.x1), int(d.y1), int(d.x2), int(d.y2))
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                tag = d.class_name
                if d.class_id == 3 and d.base_clipped:
                    tag += '!'   # base out of frame
                cv2.putText(vis, tag, (x1, max(0, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                            cv2.LINE_AA)
        self._draw_hud(vis, dets, pose, head)
        return vis

    def _draw_hud(self, vis, dets, pose, head):
        h = vis.shape[0]
        lines = []
        if pose is not None:
            lines.append('pose x=%.2f y=%.2f yaw=%.1fdeg' %
                         (pose[0], pose[1], np.degrees(pose[3])))
        lines.append('head pan=%.1f tilt=%.1f deg' %
                     (np.degrees(head[0]), np.degrees(head[1])))
        lines.append('calib pitch_bias=%.1f base_z=%+.3f' %
                     (self.pitch_bias_deg, self.base_z_offset))
        n = 0 if dets is None else len(dets)
        lines.append('dets=%d  saved=%d' % (n, self._save_index))
        lines.append("[s]ave  [q]uit  head:a/d w/x  calib:[ ] ; '")
        y = 20
        for ln in lines:
            cv2.putText(vis, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += 20
        _ = h

    def save(self, img, dets, overlay):
        if img is None or dets is None:
            self.get_logger().warn('nothing to save (no image/pose/detections)')
            return
        idx = self._save_index
        w = img.shape[1]
        h = img.shape[0]
        stem = '%06d' % idx
        img_path = os.path.join(self.dir_images, stem + '.png')
        lbl_path = os.path.join(self.dir_labels, stem + '.txt')
        dbg_path = os.path.join(self.dir_debug, stem + '.png')

        lines = []
        for d in dets:
            xc = ((d.x1 + d.x2) * 0.5) / w
            yc = ((d.y1 + d.y2) * 0.5) / h
            bw = (d.x2 - d.x1) / w
            bh = (d.y2 - d.y1) / h
            lines.append('%d %.6f %.6f %.6f %.6f' %
                         (d.class_id, xc, yc, bw, bh))

        cv2.imwrite(img_path, img)
        cv2.imwrite(dbg_path, overlay)
        with open(lbl_path, 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))

        self._save_index += 1
        self.get_logger().info('saved %s  (%d labels)' % (stem, len(lines)))

    # ── head control ──────────────────────────────────────────────────────────
    def nudge_head(self, d_pan_deg=0.0, d_tilt_deg=0.0):
        if not self.enable_head_control:
            return
        with self._lock:
            self._pan_cmd = self._head[0] + np.radians(d_pan_deg)
            self._tilt_cmd = self._head[1] + np.radians(d_tilt_deg)
        self.pub_pan.publish(Float64(data=float(self._pan_cmd)))
        self.pub_tilt.publish(Float64(data=float(self._tilt_cmd)))

    # ── helpers ───────────────────────────────────────────────────────────────
    def _next_index(self):
        existing = glob.glob(os.path.join(self.dir_labels, '*.txt'))
        existing += glob.glob(os.path.join(self.dir_images, '*.png'))
        mx = -1
        for p in existing:
            stem = os.path.splitext(os.path.basename(p))[0]
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
    node = LandmarkDatasetCapture()

    # Spin ROS in a background thread; run the OpenCV GUI loop in the main thread.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    win = 'landmark_dataset_capture'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    node.get_logger().info('GUI ready. Position the robot in Webots; press '
                           's to save, q to quit.')
    try:
        while rclpy.ok():
            img, dets, pose, head = node.compute_detections()
            if img is None:
                # nothing yet — show a placeholder so the window is responsive
                placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder,
                            'waiting for camera/pose...', (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                cv2.imshow(win, placeholder)
            else:
                overlay = node.draw_overlay(img, dets, pose, head)
                cv2.imshow(win, overlay)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                img2, dets2, pose2, head2 = node.compute_detections()
                overlay2 = (node.draw_overlay(img2, dets2, pose2, head2)
                            if img2 is not None else None)
                node.save(img2, dets2, overlay2)
            elif key == ord('a'):
                node.nudge_head(d_pan_deg=+node.head_step_deg)
            elif key == ord('d'):
                node.nudge_head(d_pan_deg=-node.head_step_deg)
            elif key == ord('w'):
                node.nudge_head(d_tilt_deg=+node.head_step_deg)
            elif key == ord('x'):
                node.nudge_head(d_tilt_deg=-node.head_step_deg)
            elif key == ord('['):
                node.pitch_bias_deg -= 0.5
            elif key == ord(']'):
                node.pitch_bias_deg += 0.5
            elif key == ord(';'):
                node.base_z_offset -= 0.01
            elif key == ord("'"):
                node.base_z_offset += 0.01
            elif key == ord('r'):
                node.pitch_bias_deg = 0.0
                node.base_z_offset = 0.0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
