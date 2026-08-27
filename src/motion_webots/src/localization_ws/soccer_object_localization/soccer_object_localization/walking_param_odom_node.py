#!/usr/bin/env python3
"""
walking_param_odom_node.py — Odometry dari op3_walking_module (Fase 1B)

Source:
  /robotis/walking/set_params  (op3_walking_module_msgs/WalkingParam)
    → x_move_amplitude (m/step-amplitude), period_time (s/cycle)
  /robotis/walking/command     (std_msgs/String)
    → "start" / "stop"
  /robotis_op3/imu             (sensor_msgs/Imu)
    → angular_velocity.z untuk integrasi yaw

Velocity formula:
  vx = x_move_amplitude * 2.0 / period_time * slip_ratio
  (x_move_amplitude = amplitude sinusoidal per kaki;
   satu periode penuh = 2 * amplitude net body displacement)

Ini TIDAK circular (bukan GT). Ada systematic error dari:
  - Slip (robot tidak selalu capai commanded speed)
  - Balance controller
  - IMU yaw drift ringan
→ Cocok untuk Fase 1B Gate G1: ukur apakah EKF x-capture > 70%.

Parameter:
  slip_ratio   (float, default 1.0): koreksi slip. Kalibrasi dari GT setelah run pertama.
               slip_ratio = GT_disp / odom_disp (jika odom over-report) atau sebaliknya.
  zero_tf_yaw  (bool,  default True): publikasikan yaw=0 ke TF.
  imu_topic    (str, default /robotis_op3/imu)
  param_timeout (float, default 0.5): s tanpa set_params → anggap stop.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

try:
    from op3_walking_module_msgs.msg import WalkingParam
    _HAS_WALKING_MSG = True
except ImportError:
    _HAS_WALKING_MSG = False


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class WalkingParamOdomNode(Node):
    def __init__(self):
        super().__init__('walking_param_odom_node')

        self.declare_parameter('slip_ratio',    1.0)
        self.declare_parameter('zero_tf_yaw',   True)
        self.declare_parameter('imu_topic',     '/robotis_op3/imu')
        self.declare_parameter('param_timeout', 0.5)

        self.slip          = self.get_parameter('slip_ratio').value
        self.zero_yaw      = self.get_parameter('zero_tf_yaw').value
        imu_topic          = self.get_parameter('imu_topic').value
        self.param_timeout = self.get_parameter('param_timeout').value

        self.get_logger().info(
            f"[WalkingParamOdom] slip_ratio={self.slip:.3f}  "
            f"zero_tf_yaw={self.zero_yaw}  imu={imu_topic}")
        self.get_logger().info(
            "[WalkingParamOdom] Menunggu /robotis/walking/command 'start' "
            "dan /robotis/walking/set_params dari GUI...")

        # Odometry state
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # Walking state from set_params
        self.x_amplitude  = 0.0   # m (step sinusoidal amplitude)
        self.y_amplitude  = 0.0
        self.period_time  = 0.6   # s (default from ball_follower)
        self.is_walking   = False
        self.last_param_t = None  # wall time of last set_params msg

        # IMU
        self.last_imu_t = None

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self._imu_cb, qos_be)

        self.sub_cmd = self.create_subscription(
            String, '/robotis/walking/command', self._cmd_cb, qos_rel)

        if _HAS_WALKING_MSG:
            self.sub_param = self.create_subscription(
                WalkingParam, '/robotis/walking/set_params', self._param_cb, qos_rel)
        else:
            self.get_logger().warn(
                "[WalkingParamOdom] op3_walking_module_msgs not found! "
                "Velocity will be 0 — source install/setup.bash and rebuild.")

        self.pub_odom = self.create_publisher(Odometry, '/odom', qos_rel)
        self.tf_br    = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self._timer_cb)  # 50 Hz
        self._last_timer = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == "start":
            self.is_walking = True
            self.get_logger().info("[WalkingParamOdom] walking START")
        elif cmd == "stop":
            self.is_walking = False
            self.get_logger().info("[WalkingParamOdom] walking STOP")

    # ─────────────────────────────────────────────────────────────
    def _param_cb(self, msg):
        """Update velocity from walking params.
        GUI publishes this ONCE per "Apply" click — not continuously.
        We keep the last known values until a new message or 'stop' command.
        """
        self.x_amplitude = msg.x_move_amplitude
        self.y_amplitude = msg.y_move_amplitude
        if msg.period_time > 0.05:
            self.period_time = msg.period_time
        self.last_param_t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"[WalkingParamOdom] params: x_amp={self.x_amplitude:.4f}m  "
            f"period={self.period_time:.2f}s  "
            f"→ vx_cmd={self.x_amplitude*2/self.period_time:.4f} m/s")

    # ─────────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_t is not None:
            dt = now - self.last_imu_t
            if 0 < dt < 0.5:
                self.yaw += msg.angular_velocity.z * dt
        self.last_imu_t = now

    # ─────────────────────────────────────────────────────────────
    def _timer_cb(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self._last_timer.nanoseconds) * 1e-9
        self._last_timer = now

        if dt <= 0 or dt > 0.5:
            return

        # Gate: integrate only when walking AND params received at least once.
        # GUI publishes set_params ONCE per "Apply" — no continuous stream.
        # param_timeout is only a safety fallback for very stale params (default 300s).
        now_s = now.nanoseconds * 1e-9
        param_received = self.last_param_t is not None
        param_not_stale = (
            param_received
            and (now_s - self.last_param_t) < self.param_timeout
        )

        vx_world = 0.0
        vy_world = 0.0

        if self.is_walking and param_received and param_not_stale and self.period_time > 0.05:
            # velocity = amplitude * 2 / period_time  (one full period = 2 half-steps)
            vx_body = self.x_amplitude * 2.0 / self.period_time * self.slip
            vy_body = self.y_amplitude * 2.0 / self.period_time * self.slip

            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            vx_world = cy * vx_body - sy * vy_body
            vy_world = sy * vx_body + cy * vy_body

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
        cov_p[0]  = 0.05   # x — slip uncertainty
        cov_p[7]  = 0.05   # y
        cov_p[35] = 0.1    # yaw
        odom.pose.covariance = cov_p

        odom.twist.twist.linear.x  = vx_world
        odom.twist.twist.linear.y  = vy_world

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
    node = WalkingParamOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
