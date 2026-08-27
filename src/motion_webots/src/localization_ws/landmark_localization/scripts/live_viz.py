#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Live localization visualizer — draws the RoboCup KidSize field to the SAME scale as
the Webots world (dimensions from landmark_geometry.field_landmarks) and overlays, in
real time:
  * EKF pose      (/odometry/filtered)      — solid arrow + trail = what localization believes
  * ground truth  (/ground_truth/odom)      — hollow arrow            = the real pose (sim only)
  * raw geom fix  (/landmark_pose)           — small dot               = the instantaneous fix

The pos/yaw error (EKF vs GT) is printed live. Matplotlib so it runs anywhere with a
display; no Gazebo/RViz needed.

Run (with the localization stack up):
  python3 live_viz.py [--no-gt] [--no-fix] [--trail 200]
"""
import argparse
import math
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# --- field dimensions (m), mirror of landmark_geometry.field_landmarks -------------
HALF_LEN, HALF_WID = 4.5, 3.0
GA_DEPTH, GA_HW = 1.0, 1.5          # goal area   -> front x=±3.5, y=±1.5
PA_DEPTH, PA_HW = 2.0, 2.5          # penalty area-> front x=±2.5, y=±2.5
PEN_MARK = 3.0                      # penalty mark x=±3.0
CIRCLE_R = 0.75
GOAL_HW, GOAL_DEPTH = 1.3, 0.6      # posts y=±1.3, goal box behind line


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def draw_field(ax):
    ax.set_facecolor('#2e7d32')                          # grass green
    W = dict(color='white', lw=2, zorder=2)
    # outer boundary + halfway line
    ax.add_patch(mpatches.Rectangle((-HALF_LEN, -HALF_WID), 2 * HALF_LEN, 2 * HALF_WID,
                                    fill=False, **W))
    ax.plot([0, 0], [-HALF_WID, HALF_WID], **W)
    ax.add_patch(mpatches.Circle((0, 0), CIRCLE_R, fill=False, **W))
    ax.plot(0, 0, '+', color='white', ms=10, mew=2, zorder=2)
    for s in (-1, 1):                                    # both ends
        gx = s * HALF_LEN
        # goal area, penalty area (rectangles opening toward the goal line)
        ax.add_patch(mpatches.Rectangle((gx - s * GA_DEPTH, -GA_HW), s * GA_DEPTH,
                                        2 * GA_HW, fill=False, **W))
        ax.add_patch(mpatches.Rectangle((gx - s * PA_DEPTH, -PA_HW), s * PA_DEPTH,
                                        2 * PA_HW, fill=False, **W))
        ax.plot(s * PEN_MARK, 0, '+', color='white', ms=8, mew=2, zorder=2)
        # goal (behind the goal line) + posts
        ax.add_patch(mpatches.Rectangle((gx, -GOAL_HW), s * GOAL_DEPTH, 2 * GOAL_HW,
                                        fill=False, color='#ffd54f', lw=2, zorder=2))
        for py in (-GOAL_HW, GOAL_HW):
            ax.plot(gx, py, 'o', color='#ffd54f', ms=6, zorder=3)
    ax.set_xlim(-HALF_LEN - 0.8, HALF_LEN + 0.8)
    ax.set_ylim(-HALF_WID - 0.6, HALF_WID + 0.6)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]  (own goal x<0  →  opponent x>0)')
    ax.set_ylabel('y [m]')


class Viz:
    def __init__(self, args):
        self.args = args
        rclpy.init()
        self.n = rclpy.create_node('live_viz')
        self.ekf = None
        self.gt = None
        self.fix = None
        self.n.create_subscription(Odometry, '/odometry/filtered', self._ekf, 10)
        if not args.no_gt:
            self.n.create_subscription(Odometry, '/ground_truth/odom', self._gt, 10)
        if not args.no_fix:
            self.n.create_subscription(PoseWithCovarianceStamped, '/landmark_pose',
                                       self._fix, 10)
        self.trail = deque(maxlen=args.trail)

    def _ekf(self, m):
        p = m.pose.pose
        self.ekf = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _fix(self, m):
        p = m.pose.pose
        self.fix = (p.position.x, p.position.y)

    def _arrow(self, ax, pose, color, fill, label):
        if pose is None:
            return []
        x, y, th = pose
        dx, dy = 0.55 * math.cos(th), 0.55 * math.sin(th)
        arts = [ax.arrow(x, y, dx, dy, head_width=0.22, head_length=0.18,
                         fc=(color if fill else 'none'), ec=color, lw=2.5,
                         length_includes_head=True, zorder=5, label=label)]
        arts.append(ax.add_patch(mpatches.Circle((x, y), 0.10, fill=fill, color=color,
                                                 zorder=5)))
        return arts

    def run(self):
        fig, ax = plt.subplots(figsize=(10, 7))
        draw_field(ax)
        fig.canvas.manager.set_window_title('Localization live — EKF vs GT')
        dyn = []
        txt = ax.text(0.01, 0.99, '', transform=ax.transAxes, va='top', ha='left',
                      fontsize=10, family='monospace',
                      bbox=dict(boxstyle='round', fc='white', alpha=0.8), zorder=10)

        def update(_):
            for _ in range(6):
                rclpy.spin_once(self.n, timeout_sec=0.0)
            for a in dyn:
                try:
                    a.remove()
                except Exception:
                    pass
            dyn.clear()
            if self.ekf is not None:
                self.trail.append((self.ekf[0], self.ekf[1]))
            if len(self.trail) > 1:
                xs, ys = zip(*self.trail)
                dyn.extend(ax.plot(xs, ys, '-', color='#00e5ff', lw=1.2, alpha=0.7,
                                   zorder=4))
            if self.fix is not None:
                dyn.extend(ax.plot(self.fix[0], self.fix[1], '.', color='#ff8f00',
                                   ms=9, zorder=4))
            dyn.extend(self._arrow(ax, self.gt, '#ffffff', False, 'GT'))
            dyn.extend(self._arrow(ax, self.ekf, '#00e5ff', True, 'EKF'))
            # error readout
            lines = []
            if self.ekf:
                lines.append('EKF  x=%+.2f y=%+.2f yaw=%+.0f°'
                             % (self.ekf[0], self.ekf[1], math.degrees(self.ekf[2])))
            if self.gt and self.ekf:
                pe = math.hypot(self.ekf[0] - self.gt[0], self.ekf[1] - self.gt[1])
                ye = abs(math.degrees(math.atan2(
                    math.sin(self.ekf[2] - self.gt[2]),
                    math.cos(self.ekf[2] - self.gt[2]))))
                lines.append('GT   x=%+.2f y=%+.2f yaw=%+.0f°'
                             % (self.gt[0], self.gt[1], math.degrees(self.gt[2])))
                lines.append('err  pos=%.3f m  yaw=%.1f°' % (pe, ye))
            elif self.ekf is None:
                lines = ['(waiting for /odometry/filtered …)']
            txt.set_text('\n'.join(lines))
            return dyn + [txt]

        self.ani = FuncAnimation(fig, update, interval=100, blit=False,
                                 cache_frame_data=False)
        ax.legend(loc='lower right', fontsize=9, framealpha=0.8)
        try:
            plt.show()
        finally:
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-gt', action='store_true', help='hide ground-truth arrow')
    ap.add_argument('--no-fix', action='store_true', help='hide raw /landmark_pose dot')
    ap.add_argument('--trail', type=int, default=200, help='EKF trail length (frames)')
    Viz(ap.parse_args()).run()


if __name__ == '__main__':
    main()
