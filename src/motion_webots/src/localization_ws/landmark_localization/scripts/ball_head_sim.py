#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""S6 (opsi 1) — simulate a BALL-TRACKING head so localization can be measured under the
most realistic untested gameplay condition: in a match the head chases the ball
(ball_tracker), tilting deep-down (valid-fix ~0% below -35 deg, GATE 4.4-A) and panning to
follow it, so landmarks are seen only intermittently. Publishes a ball-following pattern to
the SAME ``/robotis/head_control/set_joint_states`` (JointState [head_pan, head_tilt]) the
gaze policy and ball_tracker use, so the gaze node's take-over-the-head contention is
exercised for real.

  head_tilt = tilt_deg +/- tilt_amp_deg * sin(2pi t / tilt_period)   (ball near <-> far)
  head_pan  =           pan_rad     * sin(2pi t / pan_period)        (ball left <-> right)

Usage:
  ball_head_sim.py --duration 60 [--enable-module] [--tilt-deg -45] [--tilt-amp-deg 12]
                   [--pan-rad 0.6] [--tilt-period 5] [--pan-period 3] [--rate 10]
--enable-module publishes /robotis/enable_ctrl_module 'head_control_module' first (needed for
the gaze-OFF condition; with gaze ON the gaze node already enables it)."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--tilt-deg', type=float, default=-45.0)      # nominal down-gaze
    ap.add_argument('--tilt-amp-deg', type=float, default=12.0)   # -33..-57 deg
    ap.add_argument('--pan-rad', type=float, default=0.6)
    ap.add_argument('--tilt-period', type=float, default=5.0)
    ap.add_argument('--pan-period', type=float, default=3.0)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--enable-module', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('ball_head_sim')
    pub = n.create_publisher(JointState, '/robotis/head_control/set_joint_states', 10)
    if a.enable_module:
        emp = n.create_publisher(String, '/robotis/enable_ctrl_module', 5)
        for _ in range(3):
            emp.publish(String(data='head_control_module'))
            time.sleep(0.1)
        n.get_logger().info('enabled head_control_module')

    tilt0 = math.radians(a.tilt_deg)
    tamp = math.radians(a.tilt_amp_deg)
    n.get_logger().info(
        'ball_head_sim: tilt=%.0f+/-%.0f deg (T=%.1fs), pan=%.2f rad (T=%.1fs), %.1f Hz, %.0fs'
        % (a.tilt_deg, a.tilt_amp_deg, a.tilt_period, a.pan_rad, a.pan_period,
           a.rate, a.duration))
    t0 = time.time()
    dt = 1.0 / a.rate
    while rclpy.ok() and time.time() - t0 < a.duration:
        t = time.time() - t0
        pan = a.pan_rad * math.sin(2 * math.pi * t / max(0.1, a.pan_period))
        tilt = tilt0 + tamp * math.sin(2 * math.pi * t / max(0.1, a.tilt_period))
        js = JointState()
        js.header.stamp = n.get_clock().now().to_msg()
        js.name = ['head_pan', 'head_tilt']
        js.position = [float(pan), float(tilt)]
        pub.publish(js)
        time.sleep(dt)
    n.get_logger().info('ball_head_sim done')
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
