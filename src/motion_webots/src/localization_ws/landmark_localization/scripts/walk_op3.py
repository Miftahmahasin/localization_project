#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Bascorro — Apache-2.0
"""Drive the OP3 gait for the TAHAP 4 walking convergence test.

Runs EXACTLY the GUI demo's walk sequence (NO ini_pose — setting the walking_module
mode is itself what stands the robot into the walking-ready stance; the base_module
ini_pose is an unwanted crouch that, across teleport + multi-run, accumulates and
topples the robot):

  1) /robotis/enable_ctrl_module       "walking_module"  -> set mode = STAND + gait
  2) /robotis/walking/set_params       WalkingParam      -> the frozen op3
       defaults (op3_walking_module/config/param.yaml) + the requested
       x/y/angle move amplitudes
  3) /robotis/walking/command          "start"
  ... walk for --duration s (or until Ctrl-C) ...
  4) /robotis/walking/command          "stop"

All balance / offset / timing fields are the ROBOT'S OWN DEFAULTS — only the
move amplitudes are yours to set. Defaults give a slow, stable straight walk
(2 cm/step). This is a convenience wrapper around the same topics the GUI demo
uses; if you already have your usual gait launch, use that instead.

Example:
  # slow straight walk forward for 180 s:
  python3 walk_op3.py --x 0.02 --duration 180
  # gentle forward + slight left turn (keeps landmarks sweeping into view):
  python3 walk_op3.py --x 0.015 --angle 5 --duration 180
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from op3_walking_module_msgs.msg import WalkingParam
except ImportError as e:                       # pragma: no cover
    raise SystemExit('op3_walking_module_msgs not found — source the OP3 '
                     'workspace (op3_manager) first: %r' % e)


# ── frozen op3 defaults, from op3_walking_module/config/param.yaml ────────────
def default_param() -> WalkingParam:
    p = WalkingParam()
    p.init_x_offset = -0.02
    p.init_y_offset = 0.015
    p.init_z_offset = 0.035
    p.init_roll_offset = 0.0
    p.init_pitch_offset = 0.0
    p.init_yaw_offset = 0.0
    # UNIT TRAP: op3_walking_module reads WalkingParam.period_time in SECONDS
    # (its internal default is 600*0.001=0.6; it logs period_time*1000 as ms).
    # param.yaml's `period_time: 650` is milliseconds -> send 0.65 s here.
    # Sending 650.0 makes the gait clock nonsensical.
    p.period_time = 0.65             # SECONDS (param.yaml 650 ms)
    p.dsp_ratio = 0.2
    p.step_fb_ratio = 0.28
    p.z_move_amplitude = 0.06        # foot lift (param.yaml: foot_height)
    p.y_swap_amplitude = 0.028       # param.yaml: swing_right_left
    p.z_swap_amplitude = 0.006       # param.yaml: swing_top_down
    p.pelvis_offset = 0.0
    p.arm_swing_gain = 0.2
    p.balance_hip_roll_gain = 0.35
    p.balance_knee_gain = 0.3
    p.balance_ankle_roll_gain = 0.7
    p.balance_ankle_pitch_gain = 0.9
    # UNIT TRAP: hip_pitch_offset is RADIANS in WalkingParam (module default is
    # 13*DEGREE2RADIAN; it logs hip_pitch_offset*RADIAN2DEGREE). param.yaml's
    # `hip_pitch_offset: 5.0` is DEGREES -> convert. Sending 5.0 as radians
    # (=286 deg) drove LegUpper to +-5.42 rad and folded the robot.
    p.hip_pitch_offset = math.radians(5.0)      # 5 deg -> 0.0873 rad
    p.p_gain = 0
    p.i_gain = 0
    p.d_gain = 0
    p.move_aim_on = False
    p.balance_enable = True
    return p


class Walker(Node):
    def __init__(self, args):
        super().__init__('walk_op3')
        self.args = args
        self.pub_mod = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10)
        self.pub_par = self.create_publisher(
            WalkingParam, '/robotis/walking/set_params', 10)
        self.pub_cmd = self.create_publisher(
            String, '/robotis/walking/command', 10)

    def _say(self, pub, data, wait):
        m = String(); m.data = data
        pub.publish(m)
        self.get_logger().info('-> %s' % data)
        t0 = time.time()
        while time.time() - t0 < wait and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        a = self.args
        # NO ini_pose. Setting the walking_module mode is what stands the robot into
        # the walking-ready stance (this IS the GUI "Set Mode: walking_module" step).
        self._say(self.pub_mod, 'walking_module', 2.0)
        par = default_param()
        par.x_move_amplitude = float(a.x)
        par.y_move_amplitude = float(a.y)
        par.angle_move_amplitude = float(a.angle) * 3.14159265 / 180.0  # deg->rad
        self.pub_par.publish(par)
        self.get_logger().info(
            '-> set_params x=%.3f y=%.3f angle=%.1fdeg (period=%.0fms, '
            'hip_pitch=%.1fdeg)'
            % (a.x, a.y, a.angle, par.period_time * 1000.0,
               math.degrees(par.hip_pitch_offset)))
        time.sleep(0.5)
        self._say(self.pub_cmd, 'start', 0.2)
        self.get_logger().info('WALKING for %.0f s (Ctrl-C to stop early)…'
                               % a.duration)
        t0 = time.time()
        try:
            while time.time() - t0 < a.duration and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        self._say(self.pub_cmd, 'stop', 0.5)
        self.get_logger().info('STOP sent.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=0.02,
                    help='forward move amplitude per step [m] (default 0.02)')
    ap.add_argument('--y', type=float, default=0.0,
                    help='lateral move amplitude [m] (default 0.0)')
    ap.add_argument('--angle', type=float, default=0.0,
                    help='turn amplitude [deg/step] (default 0.0)')
    ap.add_argument('--duration', type=float, default=180.0,
                    help='seconds to walk before auto-stop (default 180)')
    args = ap.parse_args()
    rclpy.init()
    node = Walker(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
