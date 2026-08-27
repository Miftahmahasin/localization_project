#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Match-play demo v2 — LONG continuous walking, minimal teleport. Seeds ONCE at
kickoff, then the robot walks a continuous tour of the field by steering toward random
waypoints (turn + forward), updating the gait params LIVE (no stop/start per segment,
so the walk is smooth). Watch localization track the whole way in live_viz.py.

Unlike match_demo.py (teleport-per-scene), v2 teleports only once; everything after is
real walking, so it exercises SUSTAINED tracking (like 8c) across the field.

Run (localization stack up; watch live_viz.py):
  python3 match_demo2.py [--duration 240] [--no-seed] [--ball-head]
"""
import argparse
import math
import random
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from op3_walking_module_msgs.msg import WalkingParam

SCR = "/home/miftah/basbot/src/motion_webots/src/localization_ws/landmark_localization/scripts"
# stay inside this box so long walks don't leave the field / enter the goals
BX, BY = 3.3, 1.8
X_FWD = 0.014          # forward step when roughly on-heading
X_TURN = 0.004         # tiny forward while pivoting to a far bearing
MAX_TURN_DEG = 11.0    # per-step turn amplitude cap


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def base_param():
    p = WalkingParam()
    p.init_x_offset = -0.02; p.init_y_offset = 0.015; p.init_z_offset = 0.035
    p.period_time = 0.65; p.dsp_ratio = 0.2; p.step_fb_ratio = 0.28
    p.z_move_amplitude = 0.06; p.y_swap_amplitude = 0.028; p.z_swap_amplitude = 0.006
    p.arm_swing_gain = 0.2
    p.balance_hip_roll_gain = 0.35; p.balance_knee_gain = 0.3
    p.balance_ankle_roll_gain = 0.7; p.balance_ankle_pitch_gain = 0.9
    p.hip_pitch_offset = math.radians(5.0)
    p.balance_enable = True
    return p


def region(x, y):
    half = "sendiri" if x < -0.5 else ("lawan" if x > 0.5 else "TENGAH")
    wing = "" if abs(y) < 1.0 else (" sayap+" if y > 0 else " sayap-")
    return "paruh %s%s" % (half, wing)


class Demo2(Node):
    def __init__(self, args):
        super().__init__('match_demo2')
        self.args = args
        self.gt = None
        self.create_subscription(Odometry, '/ground_truth/odom', self._gt, 10)
        self.pub_pose = self.create_publisher(Pose2D, '/robotis_op3/set_pose', 5)
        self.pub_mod = self.create_publisher(String, '/robotis/enable_ctrl_module', 5)
        self.pub_par = self.create_publisher(WalkingParam, '/robotis/walking/set_params', 5)
        self.pub_cmd = self.create_publisher(String, '/robotis/walking/command', 5)
        self.turn_sign = 1.0

    def _gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_gait(self, x, angle_deg):
        p = base_param()
        p.x_move_amplitude = float(x)
        p.y_move_amplitude = 0.0
        p.angle_move_amplitude = math.radians(float(angle_deg))
        self.pub_par.publish(p)

    def start_walk(self):
        m = String(); m.data = 'walking_module'; self.pub_mod.publish(m)
        self.spin(2.0)
        self.set_gait(0.0, 0.0)
        self.spin(0.5)
        m = String(); m.data = 'start'; self.pub_cmd.publish(m)
        self.spin(0.5)

    def stop_walk(self):
        self.set_gait(0.0, 0.0)
        m = String(); m.data = 'stop'; self.pub_cmd.publish(m)
        self.spin(0.5)

    def calibrate_turn(self):
        """Command a +turn briefly and see which way yaw actually moves."""
        y0 = self.gt[2] if self.gt else 0.0
        self.set_gait(0.0, 8.0)
        self.spin(3.0)
        y1 = self.gt[2] if self.gt else 0.0
        self.turn_sign = 1.0 if wrap(y1 - y0) >= 0 else -1.0
        print("   turn-sign calibrated: +angle -> %s"
              % ("CCW (+yaw)" if self.turn_sign > 0 else "CW (-yaw)"))

    def new_target(self, cur):
        for _ in range(20):
            tx = random.uniform(-BX, BX)
            ty = random.uniform(-BY, BY)
            if cur is None or math.hypot(tx - cur[0], ty - cur[1]) > 2.5:
                return (tx, ty)
        return (tx, ty)

    def run(self):
        self.spin(1.0)
        if not self.args.no_seed and self.gt is not None:
            # single kickoff placement + re-seed at actual pose
            for _ in range(10):
                self.pub_pose.publish(Pose2D(x=-2.5, y=0.0, theta=0.0)); self.spin(1.0)
                if math.hypot(self.gt[0] + 2.5, self.gt[1]) < 0.3:
                    break
            self.spin(1.0)
            g = self.gt
            subprocess.run([sys.executable, f"{SCR}/seed_side.py", "--x", "%.3f" % g[0],
                            "--y", "%.3f" % g[1], "--yaw", "%.1f" % math.degrees(g[2])],
                           capture_output=True)
            self.spin(2.0)
            print("[KICKOFF] seeded at (%.2f, %.2f, %.0f deg); now CONTINUOUS walking"
                  % (g[0], g[1], math.degrees(g[2])))

        head = None
        if self.args.ball_head:
            head = subprocess.Popen(
                [sys.executable, f"{SCR}/ball_head_sim.py", "--duration",
                 str(int(self.args.duration) + 5), "--enable-module"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.start_walk()
        self.calibrate_turn()
        target = self.new_target(self.gt[:2] if self.gt else None)
        print("   -> menuju (%.1f, %.1f)" % target)
        t0 = time.time(); last_report = 0.0; t_target = time.time()
        try:
            while time.time() - t0 < self.args.duration:
                self.spin(0.6)
                if self.gt is None:
                    continue
                x, y, yaw = self.gt
                # boundary guard: if near edge, aim back to center
                if abs(x) > BX + 0.4 or abs(y) > BY + 0.4:
                    target = (0.0, 0.0)
                d = math.hypot(target[0] - x, target[1] - y)
                if d < 0.8 or time.time() - t_target > 30:
                    target = self.new_target((x, y))
                    t_target = time.time()
                    print("   -> menuju (%.1f, %.1f)  [%s]"
                          % (target[0], target[1], region(*target)))
                bearing = math.atan2(target[1] - y, target[0] - x)
                e = wrap(bearing - yaw)
                turn = self.turn_sign * max(-MAX_TURN_DEG,
                                            min(MAX_TURN_DEG, math.degrees(e) * 0.5))
                xstep = X_TURN if abs(e) > math.radians(70) else X_FWD
                self.set_gait(xstep, turn)
                if time.time() - last_report > 6:
                    last_report = time.time()
                    print("   di (%.2f, %.2f, %.0f°) %s | target(%.1f,%.1f) d=%.1fm"
                          % (x, y, math.degrees(yaw), region(x, y),
                             target[0], target[1], d))
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_walk()
            if head:
                try:
                    head.wait(timeout=3)
                except Exception:
                    head.terminate()
            print("\nmatch_demo2 done (walked %.0f s)." % (time.time() - t0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=240.0,
                    help='total walking time (s)')
    ap.add_argument('--no-seed', action='store_true',
                    help='skip the kickoff teleport+seed (assume already localized)')
    ap.add_argument('--ball-head', action='store_true',
                    help='also run ball_head_sim (deep down-gaze) throughout')
    rclpy.init()
    d = Demo2(ap.parse_args())
    try:
        d.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
