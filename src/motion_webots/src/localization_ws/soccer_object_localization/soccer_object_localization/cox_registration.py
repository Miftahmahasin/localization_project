#!/usr/bin/env python3
"""
cox_registration.py — Cox Line Point Registration Node
=======================================================
Referensi: Paper 3 (Whelan dkk.) — "Line Point Registration"

CHANGELOG:
  [FIX 1–4] Parameter image/camera, LUT path, log informatif (dari versi sebelumnya)

  [FIX 5] pixel_to_world — konvensi koordinat diperbaiki total
          Bug lama: semua titik ter-project ke ~2–3mm dari robot
          → Cox menghitung dθ=-17.6° yang konsisten (systematic error)

          Root cause:
            ray_z = -(px_v - cy) dikombinasikan dengan cos(-pitch)/sin(-pitch)
            → world_ray_z ≈ 250-300 untuk semua pixel
            → t = 0.475/250 = 0.0019 → cam_x ≈ 0.002m (2mm!) untuk semua titik
            → Semua line points jatuh di satu titik LUT → Cox menghasilkan
               rotasi besar yang tidak berarti

          Fix: konvensi kamera yang konsisten
            Camera optical frame: z=maju, x=kanan, y=bawah(image)
            Robot body frame:     x=depan, y=kiri, z=atas
            Transform: xb=zc, yb=-xc, zb=-yc, lalu rotasi pitch di bidang xz
            Hasil: pixel bawah frame → jarak dekat (0.5-1m),
                   pixel atas frame  → jarak jauh (2-5m) ✓
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import os


class CoxRegistration(Node):
    def __init__(self):
        super().__init__('cox_registration')

        # ── Parameters ───────────────────────────────────────────────
        default_lut = os.path.expanduser(
            '~/ros2_ws/src/soccer_object_localization/config/voronoi_lut.npz')

        self.declare_parameter('voronoi_lut_path',  default_lut)
        self.declare_parameter('rate_hz',           10.0)
        self.declare_parameter('max_delta_x',       0.30)
        self.declare_parameter('max_delta_y',       0.30)
        self.declare_parameter('max_delta_theta',   15.0)   # deg
        self.declare_parameter('min_points',        10)
        self.declare_parameter('outlier_dist',      0.5)    # m
        self.declare_parameter('eta',               0.01)
        self.declare_parameter('zeta',              0.001)
        self.declare_parameter('cov_x',             0.04)
        self.declare_parameter('cov_y',             0.04)
        self.declare_parameter('cov_yaw',           0.02)
        self.declare_parameter('min_confidence',    0.3)
        self.declare_parameter('image_width',       1280)
        self.declare_parameter('image_height',      720)
        self.declare_parameter('focal_length',      793.3)
        self.declare_parameter('cam_pitch_deg',    -20.0)
        self.declare_parameter('camera_height_m',   0.475)
        self.declare_parameter('field_half_len',    4.5)
        self.declare_parameter('field_half_wid',    3.0)
        self.declare_parameter('correct_yaw',       True)
        # inactive_center_radius_m: jika > 0, Cox tidak aktif saat robot berada
        # dalam radius ini dari titik tengah lapangan (0,0). Lingkaran tengah
        # (radius 1.5m) menyebabkan chord approximation oleh HoughLinesP yang
        # jatuh DI DALAM lingkaran → Voronoi normal menunjuk keluar → Cox memberi
        # dx positif palsu (+0.1-0.27m/step) dan dy negatif → AMCL kolaps.
        # Set 1.6m agar Cox tidak aktif selama robot masih di dalam zona lingkaran.
        self.declare_parameter('inactive_center_radius_m', 0.0)
        # prior_pose_topic: pose prior yang dipakai Cox sebagai titik awal.
        # Default: /amcl_pose — lebih baik dari /odom karena AMCL sudah
        # mengintegrasikan scan matching. /odom (legged KF) selalu 0,0 di
        # Webots sehingga Cox selalu anchor ke center → AMCL stuck center.
        self.declare_parameter('prior_pose_topic', '/amcl_pose')

        lut_path      = self.get_parameter('voronoi_lut_path').value
        self.rate_hz  = self.get_parameter('rate_hz').value
        self.max_dx   = self.get_parameter('max_delta_x').value
        self.max_dy   = self.get_parameter('max_delta_y').value
        self.max_dth  = math.radians(self.get_parameter('max_delta_theta').value)
        self.min_pts  = self.get_parameter('min_points').value
        self.out_dist = self.get_parameter('outlier_dist').value
        self.eta      = self.get_parameter('eta').value
        self.zeta     = self.get_parameter('zeta').value
        self.cov_x    = self.get_parameter('cov_x').value
        self.cov_y    = self.get_parameter('cov_y').value
        self.cov_yaw  = self.get_parameter('cov_yaw').value
        self.correct_yaw = self.get_parameter('correct_yaw').value
        self.inactive_center_radius = self.get_parameter('inactive_center_radius_m').value
        self.declare_parameter('use_odom_for_deadzone', False)
        self.use_odom_for_deadzone = self.get_parameter('use_odom_for_deadzone').value
        self.legged_x = 0.0
        self.legged_y = 0.0
        # [FIX J] Geometric world-space center circle filter
        self.declare_parameter('center_circle_filter_radius_m', 0.0)
        self.center_circle_filter_radius = self.get_parameter('center_circle_filter_radius_m').value
        # [FIX K] Startup delay — Cox silent for N seconds after node start
        self.declare_parameter('startup_delay_s', 0.0)
        self.startup_delay = self.get_parameter('startup_delay_s').value
        self._node_start_time: float | None = None
        # [FIX M] Rate-limit /initialpose to prevent AMCL oscillation.
        # /cox_pose published every tick; only /initialpose gated.
        self.declare_parameter('initialpose_cooldown_s', 0.0)
        self.initialpose_cooldown = self.get_parameter('initialpose_cooldown_s').value
        self._last_initialpose_sec: float = 0.0
        # [FIX O] Suppress δy when y is geometrically unconstrained (no ny≠0 lines visible)
        self.declare_parameter('min_y_constraint', 0.0)
        self.min_y_constraint = self.get_parameter('min_y_constraint').value
        self.min_conf = self.get_parameter('min_confidence').value
        self.img_w    = float(self.get_parameter('image_width').value)
        self.img_h    = float(self.get_parameter('image_height').value)
        self.focal    = float(self.get_parameter('focal_length').value)
        self.pitch    = math.radians(self.get_parameter('cam_pitch_deg').value)
        self.cam_h    = self.get_parameter('camera_height_m').value
        self.fhl      = self.get_parameter('field_half_len').value
        self.fhw      = self.get_parameter('field_half_wid').value

        # Precompute pitch rotation constants
        self._cp = math.cos(self.pitch)   # cos(-20°) =  0.940
        self._sp = math.sin(self.pitch)   # sin(-20°) = -0.342

        # ── Load LUT ─────────────────────────────────────────────────
        self.lut = None
        self._load_lut(lut_path)

        # ── State ────────────────────────────────────────────────────
        self.odom_x          = 0.0
        self.odom_y          = 0.0
        self.odom_yaw        = 0.0
        self.latest_segments = None

        prior_topic = self.get_parameter('prior_pose_topic').value

        # ── ROS I/O ──────────────────────────────────────────────────
        # Pilih QoS sesuai publisher:
        #   /amcl_pose   → RELIABLE/TRANSIENT_LOCAL (nav2 default)
        #   /odom        → BEST_EFFORT (sensor_data)
        if prior_topic == '/amcl_pose':
            prior_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1)
            self.sub_prior = self.create_subscription(
                PoseWithCovarianceStamped, prior_topic,
                self._cb_prior_pwcs, prior_qos)
        elif prior_topic in ('/odom', '/odom_throttled'):
            self.sub_prior = self.create_subscription(
                Odometry, prior_topic,
                self._cb_prior_odom, qos_profile_sensor_data)
        else:
            # /odometry/filtered dan sejenisnya — diterbitkan dengan RELIABLE QoS
            reliable_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10)
            self.sub_prior = self.create_subscription(
                Odometry, prior_topic,
                self._cb_prior_odom, reliable_qos)

        self.get_logger().info(f'  prior_pose_topic: {prior_topic}')

        self.sub_seg      = self.create_subscription(
            Float32MultiArray, '/field_line_segments', self._cb_segments, 10)
        self.pub_pose     = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_cox_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/cox_pose', 10)
        self.timer        = self.create_timer(1.0 / self.rate_hz, self._cb_timer)

        if self.center_circle_filter_radius > 0.0:
            self.get_logger().info(
                f'  [FIX J] center_circle_filter: r={self.center_circle_filter_radius:.2f}m '
                f'(world-space, no position estimate needed)')
        if self.startup_delay > 0.0:
            self.get_logger().info(
                f'  [FIX K] startup_delay: {self.startup_delay:.0f}s (Cox inactive at first)')
        if self.initialpose_cooldown > 0.0:
            self.get_logger().info(
                f'  [FIX M] initialpose_cooldown: {self.initialpose_cooldown:.0f}s '
                f'(/cox_pose still at {self.rate_hz:.0f}Hz)')
        if self.min_y_constraint > 0.0:
            self.get_logger().info(
                f'  [FIX O] min_y_constraint: {self.min_y_constraint:.1f} '
                f'(δy zeroed when no ny≠0 lines visible)')

        self.get_logger().info('CoxRegistration v2 started (Paper 3)')
        self.get_logger().info(
            f'  cam: {int(self.img_w)}×{int(self.img_h)}px  '
            f'f={self.focal:.0f}px  pitch={self.get_parameter("cam_pitch_deg").value}°  '
            f'h={self.cam_h}m')
        if self.lut is not None:
            self.get_logger().info(
                f'  LUT: {self.lut["distance"].shape}  '
                f'res={self.lut["resolution"]:.4f} m/px  '
                f'origin=({self.lut["origin_x"]:.2f},{self.lut["origin_y"]:.2f})')
        else:
            self.get_logger().warn('  LUT not loaded')

    # ════════════════════════════════════════════════════════════════
    # LUT
    # ════════════════════════════════════════════════════════════════

    def _load_lut(self, path: str):
        if not path.endswith('.npz'):
            path += '.npz'
        if not os.path.exists(path):
            self.get_logger().warn(
                f'Voronoi LUT tidak ditemukan: {path}\n'
                f'  → Jalankan voronoi_precompute.py terlebih dahulu')
            return
        try:
            data = np.load(path)
            self.lut = {
                'distance':   data['distance'],
                'normal_x':   data['normal_x'],
                'normal_y':   data['normal_y'],
                'r_offset':   data['r_offset'],
                'origin_x':   float(data['origin_x']),
                'origin_y':   float(data['origin_y']),
                'resolution': float(data['resolution']),
                'map_w':      int(data['map_width']),
                'map_h':      int(data['map_height']),
            }
            self.get_logger().info(f'LUT loaded: {path}')
        except Exception as e:
            self.get_logger().error(f'Gagal load LUT: {e}')

    # ════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════

    def _cb_prior_pwcs(self, msg: PoseWithCovarianceStamped):
        """Callback untuk /amcl_pose (PoseWithCovarianceStamped)."""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _cb_prior_odom(self, msg: Odometry):
        """Callback untuk /odom atau /odometry/filtered (Odometry)."""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _cb_segments(self, msg):
        self.latest_segments = msg.data

    def _cb_timer(self):
        if self.lut is None or self.latest_segments is None:
            return
        if len(self.latest_segments) < 8:
            return

        # [FIX K] Startup delay: hold fire until AMCL has time to converge
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._node_start_time is None:
            self._node_start_time = now_sec
        if self.startup_delay > 0.0 and (now_sec - self._node_start_time) < self.startup_delay:
            self.get_logger().info(
                f'[COX] startup delay: {now_sec - self._node_start_time:.1f}s / {self.startup_delay:.0f}s',
                throttle_duration_sec=5.0)
            return

        # ── Sample line points ───────────────────────────────────────
        SEG_FIELDS  = 8
        data        = self.latest_segments
        line_points = []

        for i in range(0, len(data) - SEG_FIELDS + 1, SEG_FIELDS):
            x1, y1, x2, y2 = data[i], data[i+1], data[i+2], data[i+3]
            seg_len    = data[i+4]
            confidence = data[i+5]
            if confidence < self.min_conf:
                continue
            n_sample = max(3, int(seg_len / 30))
            for k in range(n_sample):
                t  = k / max(n_sample - 1, 1)
                px = x1 + t * (x2 - x1)
                py = y1 + t * (y2 - y1)
                wp = self._pixel_to_world(px, py)
                if wp is not None:
                    line_points.append(wp)

        if len(line_points) < self.min_pts:
            self.get_logger().debug(
                f'Cox: only {len(line_points)} pts < min={self.min_pts}, skip',
                throttle_duration_sec=2.0)
            return

        # ── Cox registration ─────────────────────────────────────────
        pts   = np.array(line_points, dtype=np.float64)
        delta = self._cox_registration(pts)
        if delta is None:
            return

        dx, dy, dtheta = delta

        if not self.correct_yaw:
            dtheta = 0.0

        # ── Safety cap: clip moderate deltas, skip wildly large ones ──
        # Clip (bukan skip) agar koreksi besar bisa diaplikasikan inkremental
        # (Cox 10Hz: tiap step max 0.30m; 2-3 step cukup untuk koreksi 0.5m).
        # Skip hanya jika >3× max = scan/kondisi sangat buruk.
        if abs(dx) > 3*self.max_dx or abs(dy) > 3*self.max_dy:
            self.get_logger().warn(
                f'Cox delta terlalu besar (>3×max): '
                f'dx={dx:.3f}m dy={dy:.3f}m → skip',
                throttle_duration_sec=2.0)
            return
        if abs(dx) > self.max_dx or abs(dy) > self.max_dy or abs(dtheta) > self.max_dth:
            self.get_logger().info(
                f'Cox delta di-clip: '
                f'dx={dx:.3f}→{float(np.clip(dx,-self.max_dx,self.max_dx)):.3f}m '
                f'dy={dy:.3f}→{float(np.clip(dy,-self.max_dy,self.max_dy)):.3f}m',
                throttle_duration_sec=1.0)
        dx     = float(np.clip(dx,     -self.max_dx,  self.max_dx))
        dy     = float(np.clip(dy,     -self.max_dy,  self.max_dy))
        dtheta = float(np.clip(dtheta, -self.max_dth, self.max_dth))

        # ── Corrected pose ───────────────────────────────────────────
        corrected_x   = float(np.clip(self.odom_x + dx,  -self.fhl, self.fhl))
        corrected_y   = float(np.clip(self.odom_y + dy,  -self.fhw, self.fhw))
        corrected_yaw = math.atan2(
            math.sin(self.odom_yaw + dtheta),
            math.cos(self.odom_yaw + dtheta))

        # ── Publish /initialpose ─────────────────────────────────────
        msg_out                              = PoseWithCovarianceStamped()
        msg_out.header.stamp                 = self.get_clock().now().to_msg()
        msg_out.header.frame_id              = 'map'
        msg_out.pose.pose.position.x         = corrected_x
        msg_out.pose.pose.position.y         = corrected_y
        msg_out.pose.pose.position.z         = 0.0
        msg_out.pose.pose.orientation.z      = math.sin(corrected_yaw / 2.0)
        msg_out.pose.pose.orientation.w      = math.cos(corrected_yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = self.cov_x; cov[7] = self.cov_y
        cov[14] = 0.01; cov[21] = 0.01; cov[28] = 0.01
        cov[35] = self.cov_yaw
        msg_out.pose.covariance = cov
        # [FIX M] Gate /initialpose by cooldown; /cox_pose always at full rate
        now_pub = self.get_clock().now().nanoseconds * 1e-9
        if (self.initialpose_cooldown <= 0.0 or
                (now_pub - self._last_initialpose_sec) >= self.initialpose_cooldown):
            self.pub_pose.publish(msg_out)
            self._last_initialpose_sec = now_pub
        self.pub_cox_pose.publish(msg_out)

        self.get_logger().info(
            f'[COX] pts={len(line_points)}  '
            f'dx={dx:.3f} dy={dy:.3f} dθ={math.degrees(dtheta):.1f}°  '
            f'→ pose=({corrected_x:.2f},{corrected_y:.2f},'
            f'{math.degrees(corrected_yaw):.1f}°)',
            throttle_duration_sec=1.0)

    # ════════════════════════════════════════════════════════════════
    # [FIX 5] PIXEL → WORLD — konvensi diperbaiki
    # ════════════════════════════════════════════════════════════════

    def _pixel_to_world(self, px_u: float, px_v: float):
        """
        Back-project pixel (u,v) → koordinat world (x,y) di lantai.

        [FIX 5] Konvensi yang benar:
          Camera optical frame: z=maju, x=kanan, y=BAWAH (image convention)
          Robot body frame:     x=depan, y=kiri,  z=ATAS

          Step 1 — Normalized ray di camera frame:
            xc = (u - cx) / f    (kanan dari pusat)
            yc = (v - cy) / f    (bawah dari pusat, image convention)
            zc = 1.0             (maju, optical axis)

          Step 2 — Transform ke robot body frame (pre-pitch):
            xb = zc   (maju  = optical axis)
            yb = -xc  (kiri  = -kanan)
            zb = -yc  (atas  = -bawah_image)

          Step 3 — Rotasi pitch (kamera tilt bawah = pitch negatif)
            Rotasi Ry(pitch) di bidang xz (sagittal plane):
            xb' = xb * cos(pitch) - zb * sin(pitch)
            zb' = xb * sin(pitch) + zb * cos(pitch)

          Step 4 — Interseksi dengan lantai:
            Kamera di z = cam_h, lantai di z = 0
            t = -cam_h / zb'   (t > 0 jika zb' < 0 = ray mengarah ke bawah)

          Step 5 — Koordinat di lantai:
            gnd_front = t * xb'  (depan)
            gnd_left  = t * yb   (kiri, tidak berubah saat pitch)

          Step 6 — Transform ke world frame via odom yaw

        Bug lama: ray_z = -(px_v-cy) dan cos(-pitch)/sin(-pitch)
          → world_ray_z ≈ 270 untuk semua pixel
          → t ≈ 0.002m → semua titik di 2mm dari robot
          → Cox menghasilkan dθ=-17.6° yang konsisten
        """
        cx = self.img_w / 2.0
        cy = self.img_h / 2.0

        # Step 1: normalized ray di camera frame
        xc = (px_u - cx) / self.focal
        yc = (px_v - cy) / self.focal
        zc = 1.0

        # Step 2: camera → body frame (pre-pitch)
        xb0 =  zc    # depan
        yb  = -xc    # kiri (tidak berubah saat pitch)
        zb0 = -yc    # atas

        # Step 3: pitch rotation (Ry di bidang xz)
        xb = xb0 * self._cp - zb0 * self._sp
        zb = xb0 * self._sp + zb0 * self._cp

        # Step 4: interseksi dengan lantai
        if abs(zb) < 1e-9:
            return None
        t = -self.cam_h / zb
        if t < 0.05 or t > 15.0:   # min 5cm, max 15m
            return None

        # Step 5: koordinat di lantai (robot frame)
        gnd_front = t * xb
        gnd_left  = t * yb

        # Step 6: transform ke world frame
        cos_yaw = math.cos(self.odom_yaw)
        sin_yaw = math.sin(self.odom_yaw)
        world_x = self.odom_x + gnd_front * cos_yaw - gnd_left * sin_yaw
        world_y = self.odom_y + gnd_front * sin_yaw + gnd_left * cos_yaw

        # Filter: harus di dalam lapangan (+20% margin)
        if abs(world_x) > self.fhl * 1.2 or abs(world_y) > self.fhw * 1.2:
            return None

        # [FIX J] Buang titik yang ter-project di dalam area lingkaran tengah.
        # Chord HoughLinesP dari arc lingkaran jatuh di DALAM lingkaran (r<1.5m).
        # Normal Voronoi di titik dalam mengarah keluar → WLS beri false +dx.
        # Filter ini bekerja tanpa perlu estimasi posisi robot.
        if self.center_circle_filter_radius > 0.0:
            if world_x * world_x + world_y * world_y < self.center_circle_filter_radius ** 2:
                return None

        return (world_x, world_y)

    # ════════════════════════════════════════════════════════════════
    # COX WEIGHTED LEAST SQUARES
    # ════════════════════════════════════════════════════════════════

    def _cox_registration(self, points: np.ndarray):
        """
        Cox's weighted least squares registration.
        Paper 3 Eq. 5: b̂ = (XᵀWX + ζI)⁻¹ XᵀWY
        """
        lut      = self.lut
        origin_x = lut['origin_x']
        origin_y = lut['origin_y']
        res      = lut['resolution']
        map_h    = lut['map_h']
        map_w    = lut['map_w']

        X_list, Y_list, W_list = [], [], []
        n_outlier = 0

        for (wx, wy) in points:
            col = int((wx - origin_x) / res)
            row = int(map_h - 1 - (wy - origin_y) / res)

            if not (0 <= col < map_w and 0 <= row < map_h):
                n_outlier += 1
                continue

            d  = float(lut['distance'][row, col])
            ux = float(lut['normal_x'][row, col])
            uy = float(lut['normal_y'][row, col])
            r  = float(lut['r_offset'][row, col])

            if d > self.out_dist:
                n_outlier += 1
                continue

            w_i = 1.0 / (d * d + self.eta)
            X_list.append([ux, uy, -ux * wy + uy * wx])
            Y_list.append(r - (ux * wx + uy * wy))
            W_list.append(w_i)

        n_inlier = len(X_list)
        if n_inlier < self.min_pts:
            self.get_logger().debug(
                f'Cox: {n_inlier} inliers ({n_outlier} outliers) < min={self.min_pts}, skip',
                throttle_duration_sec=2.0)
            return None

        X = np.array(X_list, dtype=np.float64)
        Y = np.array(Y_list, dtype=np.float64)
        W = np.diag(W_list)

        XtW = X.T @ W
        A   = XtW @ X + self.zeta * np.eye(3)
        b   = XtW @ Y

        cond = float(np.linalg.cond(A))
        if cond > 1e8:
            self.get_logger().warn(
                f'Cox matrix ill-conditioned (cond={cond:.1e}), skip',
                throttle_duration_sec=5.0)
            return None

        try:
            delta = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Cox LS singular', throttle_duration_sec=5.0)
            return None

        # [FIX O] Zero δy when A[1,1] ≈ ζ (no y-constraining lines, amplifies noise 1000×)
        y_info = float(A[1, 1]) - self.zeta
        if self.min_y_constraint > 0.0 and y_info < self.min_y_constraint:
            delta[1] = 0.0

        self.get_logger().debug(
            f'Cox LS OK: {n_inlier} inliers {n_outlier} outliers cond={cond:.1e}',
            throttle_duration_sec=2.0)

        return float(delta[0]), float(delta[1]), float(delta[2])


def main(args=None):
    rclpy.init(args=args)
    node = CoxRegistration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()