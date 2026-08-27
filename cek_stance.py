#!/usr/bin/env python3
"""
cek_stance.py — Diagnosa nilai knee dan stance detection saat berjalan

Jalankan saat robot BERJALAN dan amati:
  1. Nilai knee kiri/kanan (radian)
  2. Apakah stance berubah-ubah (R, L, RL, ..)

Usage: python3 cek_stance.py
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState


class CekStance(Node):
    def __init__(self):
        super().__init__('cek_stance')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(JointState, '/robotis_op3/joint_states', self.cb, qos)
        self.count = 0
        self.get_logger().info("Jalankan robot — amati nilai knee dan stance")
        self.get_logger().info(f"{'step':>5} | {'L_knee':>8} | {'R_knee':>8} | {'L_hip':>8} | {'R_hip':>8} | stance@0.3 | stance@0.5 | stance@0.8")
        self.get_logger().info("-" * 85)

    def cb(self, msg):
        self.count += 1
        if self.count % 5 != 1:  # print ~20Hz
            return

        j = dict(zip(msg.name, msg.position))
        lk = j.get('l_knee', 0.0)
        rk = j.get('r_knee', 0.0)
        lh = j.get('l_hip_pitch', 0.0)
        rh = j.get('r_hip_pitch', 0.0)

        def stance_str(thresh):
            l = 'L' if abs(lk) > thresh else '.'
            r = 'R' if abs(rk) > thresh else '.'
            return r + l

        print(
            f"{self.count:5d} | "
            f"{lk:+8.3f} | {rk:+8.3f} | "
            f"{lh:+8.3f} | {rh:+8.3f} | "
            f"  {stance_str(0.3):^9} | "
            f"  {stance_str(0.5):^9} | "
            f"  {stance_str(0.8):^9}"
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CekStance())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()