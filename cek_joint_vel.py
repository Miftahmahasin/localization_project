#!/usr/bin/env python3
"""
cek_joint_vel.py — Cek kecepatan joint dari Webots saat robot DIAM

Usage: python3 cek_joint_vel.py
Biarkan robot DIAM, amati nilai velocity.
Kalau velocity >> 0 saat diam → noise besar → ini penyebab divergensi
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState

JOINTS_KAKI = [
    'l_hip_pitch', 'l_knee', 'l_ank_pitch',
    'r_hip_pitch', 'r_knee', 'r_ank_pitch',
]

class CekJointVel(Node):
    def __init__(self):
        super().__init__('cek_joint_vel')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(JointState, '/robotis_op3/joint_states', self.cb, qos)
        self.count = 0
        self.get_logger().info("Biarkan robot DIAM — amati velocity joint kaki")
        self.get_logger().info("-" * 60)

    def cb(self, msg):
        self.count += 1
        if self.count % 20 != 1:
            return
        
        joint_map = dict(zip(msg.name, zip(msg.position, msg.velocity)))
        
        parts = []
        max_vel = 0.0
        for j in JOINTS_KAKI:
            if j in joint_map:
                pos, vel = joint_map[j]
                parts.append(f"{j.replace('_pitch','_p').replace('l_','L').replace('r_','R')}:"
                             f"v={vel:+.3f}")
                max_vel = max(max_vel, abs(vel))
        
        flag = "✅ OK" if max_vel < 0.01 else ("⚠️  NOISY" if max_vel < 0.1 else "❌ SANGAT NOISY")
        print(f"[{self.count:4d}] max_vel={max_vel:.4f} {flag}")
        if parts:
            print(f"       {' | '.join(parts[:3])}")
            print(f"       {' | '.join(parts[3:])}")

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CekJointVel())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()