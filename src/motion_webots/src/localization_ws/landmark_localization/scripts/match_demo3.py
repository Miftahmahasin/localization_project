#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Match-play demo v3 — AGGRESSIVE route + FASTER walk. Like v2 (one kickoff seed, then
continuous steered walking, live gait updates), but: (1) faster forward speed + shorter
gait period, (2) sharper turns, (3) waypoints target the field EXTREMES (corners / far
ends) so the robot sprints long diagonals corner-to-corner instead of gentle wandering.
Stresses localization tracking under quick, dynamic motion.

Run (localization stack up; watch live_viz.py):
  python3 match_demo3.py [--duration 240] [--speed 0.018] [--turn 18] [--period 0.60]
                         [--no-seed] [--ball-head]
Tip: if the robot wobbles/falls, lower --speed (e.g. 0.015) or raise --period (0.63).
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
BX, BY = 3.3, 1.8
# aggressive target set: corners, far ends, far wings — forces long diagonal sprints
EXTREMES = [(BX, BY), (BX, -BY), (-BX, BY), (-BX, -BY),
            (BX, 0.0), (-BX, 0.0), (0.0, BY), (0.0, -BY),
            (2.4, 1.5), (-2.4, -1.5), (2.4, -1.5), (-2.4, 1.5)]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def region(x, y):
    half = "sendiri" if x < -0.5 else ("lawan" if x > 0.5 else "TENGAH")
    wing = "" if abs(y) < 1.0 else (" sayap+" if y > 0 else " sayap-")
    return "paruh %s%s" % (half, wing)


class Demo3(Node):
    def __init__(self, args):
        super().__init__('match_demo3')
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

    def base_param(self):
        p = WalkingParam()
        p.init_x_offset = -0.02; p.init_y_offset = 0.015; p.init_z_offset = 0.035
        p.period_time = float(self.args.period)
        p.dsp_ratio = 0.2; p.step_fb_ratio = 0.28
        p.z_move_amplitude = 0.06; p.y_swap_amplitude = 0.028; p.z_swap_amplitude = 0.006
        p.arm_swing_gain = 0.2
        p.balance_hip_roll_gain = 0.35; p.balance_knee_gain = 0.3
        p.balance_ankle_roll_gain = 0.7; p.balance_ankle_pitch_gain = 0.9
        p.hip_pitch_offset = math.radians(5.0)
        p.balance_enable = True
        return p

    def set_gait(self, x, angle_deg):
        p = self.base_param()
        p.x_move_amplitude = float(x)
        p.y_move_amplitude = 0.0
        p.angle_move_amplitude = math.radians(float(angle_deg))
        self.pub_par.publish(p)

    def start_walk(self):
        self.pub_mod.publish(String(data='walking_module')); self.spin(2.0)
        self.set_gait(0.0, 0.0); self.spin(0.5)
        self.pub_cmd.publish(String(data='start')); self.spin(0.5)

    def stop_walk(self):
        self.set_gait(0.0, 0.0)
        self.pub_cmd.publish(String(data='stop')); self.spin(0.5)

    def calibrate_turn(self):
        y0 = self.gt[2] if self.gt else 0.0
        self.set_gait(0.0, 8.0); self.spin(3.0)
        y1 = self.gt[2] if self.gt else 0.0
        self.turn_sign = 1.0 if wrap(y1 - y0) >= 0 else -1.0
        print("   turn-sign: +angle -> %s"
              % ("CCW" if self.turn_sign > 0 else "CW"))

    def new_target(self, cur):
        far = [t for t in EXTREMES if cur is None or math.hypot(t[0]-cur[0], t[1]-cur[1]) > 3.5]
        return random.choice(far if far else EXTREMES)

    def run(self):
        self.spin(1.0)
        if not self.args.no_seed and self.gt is not None:
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
            print("[KICKOFF] seeded (%.2f,%.2f,%.0f°); AGGRESSIVE sprint x=%.3f turn=%.0f period=%.2f"
                  % (g[0], g[1], math.degrees(g[2]), self.args.speed, self.args.turn,
                     self.args.period))

        head = None
        if self.args.ball_head:
            head = subprocess.Popen(
                [sys.executable, f"{SCR}/ball_head_sim.py", "--duration",
                 str(int(self.args.duration) + 5), "--enable-module"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.start_walk()
        self.calibrate_turn()
        target = self.new_target(self.gt[:2] if self.gt else None)
        print("   -> SPRINT ke (%.1f, %.1f) [%s]" % (target[0], target[1], region(*target)))
        t0 = time.time(); last = 0.0; t_tgt = time.time()
        try:
            while time.time() - t0 < self.args.duration:
                self.spin(0.5)
                if self.gt is None:
                    continue
                x, y, yaw = self.gt
                if abs(x) > BX + 0.5 or abs(y) > BY + 0.5:
                    target = (0.0, 0.0)
                d = math.hypot(target[0] - x, target[1] - y)
                if d < 0.9 or time.time() - t_tgt > 22:
                    target = self.new_target((x, y)); t_tgt = time.time()
                    print("   -> SPRINT ke (%.1f, %.1f) [%s]"
                          % (target[0], target[1], region(*target)))
                e = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
                turn = self.turn_sign * max(-self.args.turn,
                                            min(self.args.turn, math.degrees(e) * 0.7))
                # aggressive: keep speed up through turns; only ease off for near-reverse
                xstep = (0.006 if abs(e) > math.radians(100)
                         else self.args.speed * (0.6 if abs(e) > math.radians(55) else 1.0))
                self.set_gait(xstep, turn)
                if time.time() - last > 5:
                    last = time.time()
                    print("   di (%.2f,%.2f,%.0f°) %s | tgt(%.1f,%.1f) d=%.1fm"
                          % (x, y, math.degrees(yaw), region(x, y), target[0], target[1], d))
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_walk()
            if head:
                try:
                    head.wait(timeout=3)
                except Exception:
                    head.terminate()
            print("\nmatch_demo3 done (%.0f s)." % (time.time() - t0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=240.0)
    # NOTE: op3 walking in this Webots is gait-limited to ~0.02 m/s; forward speed
    # barely scales past x~0.02 (tested x=0.024/period=0.56 -> still ~0.02 m/s, stable
    # but no faster). These defaults are the fastest STABLE setting; higher --speed
    # only adds fall risk. v3's "aggressive" comes from the route (corner sprints +
    # sharp turns), not raw locomotion speed.
    ap.add_argument('--speed', type=float, default=0.020, help='forward step amplitude')
    ap.add_argument('--turn', type=float, default=18.0, help='max turn deg/step')
    ap.add_argument('--period', type=float, default=0.58, help='gait period s (lower=faster)')
    ap.add_argument('--no-seed', action='store_true')
    ap.add_argument('--ball-head', action='store_true')
    rclpy.init()
    d = Demo3(ap.parse_args())
    try:
        d.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
