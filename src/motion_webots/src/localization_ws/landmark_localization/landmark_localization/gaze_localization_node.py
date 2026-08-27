#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""TAHAP 6A — active-vision gaze policy for localization recovery.

The no-odom stack tracks well WHILE landmarks are in view, but blacks out when
the camera stops seeing them — the body faces the goal net (measured 16 s gap),
or the head tilts down to track the ball (gaze map 4.4-A: valid-fix ~0% below
tilt -35 deg). During a blackout the EKF only coasts, so localization decays.

This node closes the loop the cheap way: it watches localization health and,
when it degrades, TEMPORARILY takes the head to a localization gaze (tilt at the
horizon, where the 4.4 map shows the fix-rate plateau ~79%, while slowly sweeping
pan to sweep landmarks into view), then RELEASES the head the moment a confident
fix returns so ball-tracking / gameplay resumes. It is NOT a new head controller:
it publishes to the SAME ``/robotis/head_control/set_joint_states`` the soccer
ball-tracker uses, and only while in the LOCALIZING state — so there is no
publisher fight when localization is healthy.

Health = EKF position covariance (``/odometry/filtered``) below a threshold AND a
recent landmark fix (``/landmark_pose``). Trigger = covariance too high OR no fix
for too long. It reports trigger frequency, the recovery-time distribution, and
the fraction of time spent localizing (the cost to gameplay — head off the ball).
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class GazeLocalizationNode(Node):
    def __init__(self):
        super().__init__('gaze_localization_node')
        p = self.declare_parameter
        self.ekf_topic = str(p('ekf_topic', '/odometry/filtered').value)
        self.fix_topic = str(p('fix_topic', '/landmark_pose').value)
        self.head_topic = str(
            p('head_topic', '/robotis/head_control/set_joint_states').value)
        # trigger localization gaze when uncertain...
        self.trigger_pos_var = float(p('trigger_pos_var', 0.5).value)   # m^2
        self.fix_gap_trigger = float(p('fix_gap_trigger_s', 1.5).value)
        # ...and release it back to gameplay when confidently recovered
        self.release_pos_var = float(p('release_pos_var', 0.2).value)
        self.release_fixes = int(p('release_fixes', 5).value)
        self.release_window = float(p('release_window_s', 2.0).value)
        # localization gaze: horizon tilt (4.4 map plateau) + slow pan sweep
        self.gaze_tilt = math.radians(float(p('gaze_tilt_deg', -15.0).value))
        self.pan_min = float(p('pan_min_rad', -1.0).value)
        self.pan_max = float(p('pan_max_rad', 1.0).value)
        self.pan_period = float(p('pan_period_s', 5.0).value)
        self.rate = float(p('control_rate_hz', 5.0).value)
        self.enable_head_module = bool(p('enable_head_module', True).value)

        self.cov_x = None
        self.last_fix_t = None
        self.fix_times = deque()
        self.state = 'NORMAL'
        self.t_localize_start = None
        self._episodes = []           # recovery times [s]
        self._n_trigger = 0
        self._localizing_time = 0.0
        self._t0 = self._now()

        self.create_subscription(Odometry, self.ekf_topic, self._cb_ekf, 20)
        self.create_subscription(PoseWithCovarianceStamped, self.fix_topic,
                                 self._cb_fix, 20)
        self.pub_head = self.create_publisher(JointState, self.head_topic, 10)
        self.pub_mod = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 5)

        self._enable_count = 0
        if self.enable_head_module:
            # re-publish a few times (startup subscriber may miss a single msg)
            self._enable_timer = self.create_timer(1.0, self._enable_head)
        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'gaze policy: trigger cov>%.2f m^2 or gap>%.1fs; gaze tilt=%.0f deg, '
            'pan sweep [%.2f,%.2f] rad; release cov<%.2f & %d fixes'
            % (self.trigger_pos_var, self.fix_gap_trigger,
               math.degrees(self.gaze_tilt), self.pan_min, self.pan_max,
               self.release_pos_var, self.release_fixes))

    # ── helpers ────────────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _enable_head(self):
        m = String()
        m.data = 'head_control_module'
        self.pub_mod.publish(m)
        self._enable_count += 1
        if self._enable_count == 1:
            self.get_logger().info('enabling head_control_module')
        if self._enable_count >= 3:
            self._enable_timer.cancel()

    # ── inputs ─────────────────────────────────────────────────────────────────
    def _cb_ekf(self, msg: Odometry):
        self.cov_x = float(msg.pose.covariance[0])       # x-x variance

    def _cb_fix(self, msg: PoseWithCovarianceStamped):
        t = self._now()
        self.last_fix_t = t
        self.fix_times.append(t)
        cutoff = t - self.release_window
        while self.fix_times and self.fix_times[0] < cutoff:
            self.fix_times.popleft()

    def _recent_fixes(self):
        t = self._now()
        cutoff = t - self.release_window
        while self.fix_times and self.fix_times[0] < cutoff:
            self.fix_times.popleft()
        return len(self.fix_times)

    def _sweep_pan(self, t):
        """Triangle sweep across [pan_min, pan_max]."""
        if self.pan_period <= 0:
            return 0.0
        ph = (t % self.pan_period) / self.pan_period      # 0..1
        tri = 2.0 * ph if ph < 0.5 else 2.0 * (1.0 - ph)  # 0..1..0
        return self.pan_min + (self.pan_max - self.pan_min) * tri

    def _command_gaze(self, t):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['head_pan', 'head_tilt']
        js.position = [self._sweep_pan(t), self.gaze_tilt]
        self.pub_head.publish(js)

    def _center_head(self):
        """On release, return the head to a neutral forward gaze (pan=0, horizon)
        once, then stop publishing — otherwise it freezes wherever the pan sweep
        happened to be at recovery (askew). In gameplay the ball-tracker overrides
        this immediately; standalone it leaves a deterministic forward pose."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['head_pan', 'head_tilt']
        js.position = [0.0, self.gaze_tilt]
        self.pub_head.publish(js)

    # ── main loop ──────────────────────────────────────────────────────────────
    def _tick(self):
        t = self._now()
        gap = (t - self.last_fix_t) if self.last_fix_t is not None else 1e9
        uncertain = ((self.cov_x is not None and self.cov_x > self.trigger_pos_var)
                     or gap > self.fix_gap_trigger)
        recovered = (self.cov_x is not None
                     and self.cov_x < self.release_pos_var
                     and self._recent_fixes() >= self.release_fixes)

        if self.state == 'NORMAL':
            if uncertain:
                self.state = 'LOCALIZING'
                self.t_localize_start = t
                self._n_trigger += 1
                self.get_logger().warn(
                    'localization degraded (cov=%s gap=%.1fs) -> GAZE takeover'
                    % ('%.2f' % self.cov_x if self.cov_x is not None else '?',
                       gap))
        else:  # LOCALIZING
            self._command_gaze(t)
            if recovered:
                rec = t - self.t_localize_start
                self._episodes.append(rec)
                self._localizing_time += rec
                self.state = 'NORMAL'
                self._center_head()          # return head to forward, not askew
                self.get_logger().info(
                    'localization recovered in %.1fs -> release head (centered)'
                    % rec)

    def _report(self):
        t = self._now()
        live = (t - self.t_localize_start) if self.state == 'LOCALIZING' else 0.0
        total = self._localizing_time + live
        span = max(t - self._t0, 1e-9)
        eps = self._episodes
        rec = ('mean=%.1fs max=%.1fs n=%d' %
               (sum(eps) / len(eps), max(eps), len(eps))) if eps else 'n=0'
        self.get_logger().info(
            'gaze: state=%s triggers=%d recovery(%s) localizing=%.0f%% of run'
            % (self.state, self._n_trigger, rec, 100.0 * total / span))


def main(args=None):
    rclpy.init(args=args)
    node = GazeLocalizationNode()
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
