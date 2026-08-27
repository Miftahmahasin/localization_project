#!/usr/bin/env python3
"""
footstep_odom_node.py — Odometry dari FootStepCommand (Fase 1B)

Sumber: /robotis/online_walking/foot_step_command
  - step_length (m) dan step_time (s) per step → kecepatan yang di-command
  - command: "forward" / "backward" / "stop" / "left" / "right" / dll.
Yaw: dari /robotis_op3/imu (integrasi angular_velocity.z)

Ini BUKAN ground-truth. Ada systematic error dari:
  - Slip (robot tidak selalu capai commanded speed)
  - Balance controller yang memodifikasi gait
  - IMU yaw drift ringan
→ Cocok sebagai sumber odom realistis untuk Fase 1B EKF parameter tuning.

Parameter:
  slip_ratio   (float, default 1.0): koreksi slip (odom = commanded * slip_ratio).
               Kalibrasi dari GT data. Mulai 1.0, ukur k dari run pertama.
  zero_tf_yaw  (bool,  default True): publikasikan yaw=0 ke TF (sama dgn legged_kf).
  imu_topic    (str, default /robotis_op3/imu)
"""

import math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

try:
    from op3_online_walking_module_msgs.msg import FootStepCommand
    _HAS_MSG = True
except ImportError:
    _HAS_MSG = False


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class FootstepOdomNode(Node):
    def __init__(self):
        super().__init__('footstep_odom_node')

        self.declare_parameter('slip_ratio',  1.0)
        self.declare_parameter('zero_tf_yaw', True)
        self.declare_parameter('imu_topic',   '/robotis_op3/imu')

        self.slip       = self.get_parameter('slip_ratio').value
        self.zero_yaw   = self.get_parameter('zero_tf_yaw').value
        imu_topic       = self.get_parameter('imu_topic').value

        self.get_logger().info(
            f"[FootstepOdom] slip_ratio={self.slip:.3f}  zero_tf_yaw={self.zero_yaw}")

        # Odometry state
        self.x    = 0.0
        self.y    = 0.0
        self.yaw  = 0.0  # integrated from IMU

        # Walking state
        self.vx_body  = 0.0  # current commanded body velocity (m/s)
        self.vy_body  = 0.0
        self.wz_body  = 0.0

        # IMU
        self.imu_yaw0  = None  # first IMU yaw (to zero out absolute heading)
        self.last_imu_time = None

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # IMU for yaw integration
        self.sub_imu = self.create_subscription(Imu, imu_topic, self._imu_cb, qos_be)

        # FootStepCommand for velocity
        if _HAS_MSG:
            self.sub_fs = self.create_subscription(
                FootStepCommand,
                '/robotis/online_walking/foot_step_command',
                self._footstep_cb, qos_rel)
        else:
            self.get_logger().warn(
                "[FootstepOdom] op3_online_walking_module_msgs not found! "
                "Velocity will be 0 — check package installation.")

        self.pub_odom = self.create_publisher(Odometry, '/odom', qos_rel)
        self.tf_br    = TransformBroadcaster(self)

        # Odometry integration at 50Hz
        self.timer = self.create_timer(0.02, self._timer_cb)
        self._last_timer = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Extract absolute yaw from quaternion for initialization only
        q = msg.orientation
        yaw_abs = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw_abs
            self.last_imu_time = now
            return

        # Integrate angular velocity (more accurate than quaternion derivative)
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0 < dt < 0.5:
                self.yaw += msg.angular_velocity.z * dt
        self.last_imu_time = now

    # ─────────────────────────────────────────────────────────────
    def _footstep_cb(self, msg):
        """Compute body velocity from step command."""
        cmd = msg.command.lower() if msg.command else ""
        step_len  = msg.step_length   # m per step
        side_len  = msg.side_length   # m per step (lateral)
        step_time = msg.step_time     # s per step

        if step_time <= 0.001:
            self.vx_body = 0.0
            self.vy_body = 0.0
            return

        vx = step_len / step_time * self.slip
        vy = side_len / step_time * self.slip

        if cmd == "forward":
            self.vx_body =  vx
            self.vy_body =  0.0
        elif cmd == "backward":
            self.vx_body = -vx
            self.vy_body =  0.0
        elif cmd in ("left",):
            self.vx_body = 0.0
            self.vy_body =  vy
        elif cmd in ("right",):
            self.vx_body = 0.0
            self.vy_body = -vy
        elif cmd in ("stop", ""):
            self.vx_body = 0.0
            self.vy_body = 0.0
        else:
            # turn_left / turn_right: kecepatan translasi kecil
            self.vx_body = 0.0
            self.vy_body = 0.0

    # ─────────────────────────────────────────────────────────────
    def _timer_cb(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self._last_timer.nanoseconds) * 1e-9
        self._last_timer = now

        if dt <= 0 or dt > 0.5:
            return

        # Rotate body velocity to world frame
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_world = cy * self.vx_body - sy * self.vy_body
        vy_world = sy * self.vx_body + cy * self.vy_body

        # Integrate position
        self.x += vx_world * dt
        self.y += vy_world * dt

        self._publish(now, vx_world, vy_world)

    # ─────────────────────────────────────────────────────────────
    def _publish(self, stamp, vx_world, vy_world):
        yaw_out = 0.0 if self.zero_yaw else self.yaw
        qx, qy, qz, qw = _quat_from_yaw(yaw_out)

        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        cov_p = [0.0] * 36
        cov_p[0]  = 0.05   # x — maior karena slip uncertainty
        cov_p[7]  = 0.05   # y
        cov_p[35] = 0.1    # yaw
        odom.pose.covariance = cov_p

        odom.twist.twist.linear.x  = vx_world
        odom.twist.twist.linear.y  = vy_world
        odom.twist.twist.angular.z = self.wz_body

        cov_t = [0.0] * 36
        cov_t[0]  = 0.02
        cov_t[7]  = 0.02
        cov_t[35] = 0.1
        odom.twist.covariance = cov_t

        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp    = stamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FootstepOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
