#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP B2.1 — deliver an AUTHORITATIVE side seed reliably.

`/initialpose` is the one message that commits the field side: it reaches BOTH the
EKF (its `set_pose` is remapped to `/initialpose`, resetting the filter state) and
geometric_pose_node (`_cb_initialpose` -> mirror.set_ref, committing the anti-mirror
side). The 8c failure `gON_run2` was a genuine mirror lock (robot at GT x=-2.48, but
localization stuck at +1.54) that a single `ros2 topic pub --once /initialpose`
FAILED to break — one volatile message, published against a long-running node, is
easily missed, so the stale cross-run `ref` was never overridden.

This helper makes the seed bulletproof: it publishes the SAME `/initialpose` several
times over a short window (default 6x @ 0.25 s) so both subscribers latch it, waits
for at least one subscriber to be connected before the first send, and reports. Use
it INSTEAD of a bare `pub --once` in every seeded run.

USAGE (own half, facing +x, at x=-2.5):
  python3 seed_side.py --x -2.5 --y 0 --yaw 0
  # after teleporting the robot to the same pose; then start walk_op3.py.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def _cov(pos_var: float, yaw_var: float):
    c = [0.0] * 36
    c[0] = pos_var          # x
    c[7] = pos_var          # y
    c[35] = yaw_var         # yaw
    c[14] = c[21] = c[28] = 1.0e6   # z / roll / pitch unused
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True, help='seed x [m]')
    ap.add_argument('--y', type=float, default=0.0, help='seed y [m]')
    ap.add_argument('--yaw', type=float, default=0.0, help='seed yaw [deg]')
    ap.add_argument('--topic', default='/initialpose')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--n', type=int, default=10, help='repeats (default 10)')
    ap.add_argument('--period', type=float, default=0.3, help='s between repeats')
    ap.add_argument('--pos-var', type=float, default=0.04, help='(0.2 m)^2 seed pos var')
    ap.add_argument('--yaw-var', type=float, default=0.0076, help='(5 deg)^2 seed yaw var')
    ap.add_argument('--wait', type=float, default=3.0,
                    help='max s to wait for a subscriber before first send')
    args = ap.parse_args()

    rclpy.init()
    node = Node('seed_side')
    pub = node.create_publisher(PoseWithCovarianceStamped, args.topic, 10)

    # wait for at least one subscriber (the EKF + geometric_pose_node) to connect,
    # so the first send is not dropped into the void.
    t0 = time.time()
    while pub.get_subscription_count() < 1 and time.time() - t0 < args.wait \
            and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    nsub = pub.get_subscription_count()

    yaw = math.radians(args.yaw)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = args.frame
    msg.pose.pose.position.x = float(args.x)
    msg.pose.pose.position.y = float(args.y)
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
    msg.pose.covariance = _cov(args.pos_var, args.yaw_var)

    for i in range(args.n):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.01)
        node.get_logger().info(
            'seed %d/%d -> %s (%.2f, %.2f, %.1f deg)  subs=%d'
            % (i + 1, args.n, args.topic, args.x, args.y, args.yaw,
               pub.get_subscription_count()))
        if i < args.n - 1:
            time.sleep(args.period)

    if nsub < 1:
        node.get_logger().warn(
            'no subscriber was connected when seeding began — is the stack up? '
            'the repeats may still have landed, but VERIFY the T2 report shows '
            '"mirror ref set from /initialpose seed".')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
