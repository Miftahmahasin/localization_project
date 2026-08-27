#!/usr/bin/env python3
"""
diagnosa_odom.py — Script sederhana untuk cek arah odom
Jalankan: python3 diagnosa_odom.py
Lalu perhatikan output saat robot jalan maju/mundur/kanan/kiri
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomDiag(Node):
    def __init__(self):
        super().__init__('odom_diag')
        self.prev = None
        self.count = 0
        self.create_subscription(Odometry, '/odom', self.cb,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.get_logger().info("Monitoring /odom... suruh robot jalan MAJU")

    def cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.degrees(2.0 * math.atan2(q.z, q.w))

        if self.prev:
            dx = x - self.prev[0]
            dy = y - self.prev[1]
            dist = math.sqrt(dx**2 + dy**2)
            if dist > 0.005:  # hanya log kalau bergerak > 5mm
                self.count += 1
                # Arah gerak dalam derajat
                dir_deg = math.degrees(math.atan2(dy, dx))
                self.get_logger().info(
                    f"[{self.count:3d}] pos=({x:.3f},{y:.3f}) "
                    f"yaw={yaw:.1f}° | "
                    f"delta=({dx:+.3f},{dy:+.3f}) "
                    f"arah_gerak={dir_deg:.1f}°")

        self.prev = (x, y, yaw)

rclpy.init()
rclpy.spin(OdomDiag())