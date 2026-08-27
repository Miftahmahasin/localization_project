#!/usr/bin/env python3
"""
localization_v14_surrogate.launch.py  [Fase 1B Diagnostik]
============================================================
IDENTIK dengan localization_v14.launch.py KECUALI:
  - kf_odom_node (legged_odometry_kf) diganti surrogate_odom_node
  - surrogate_odom_node menggunakan /ground_truth/odom dengan k=1.042
    scale error + Gaussian noise untuk mensimulasikan odom yang bekerja

TUJUAN: Gate G1 — "Jika odom reliable, apakah EKF bisa tracking (x-capture>70%)?"
JANGAN dipakai sebagai konfigurasi produksi. Kembali ke v14 setelah G1 dikonfirmasi.

Basis: localization_v14.launch.py  [v10.20]
=====================================
Basis: v13 (v10.7b — AMCL rollback + scan_gate + goal_localizer)

PERUBAHAN v10.9 — lanjutan v10.8 (fix bug odometry + voronoi + initial pose):

  FIX A — legged_odometry_kf_node v3.2: Absolute-Height Stance Detection (Webots)
    Bug v3.1: Di Webots, fz_L ≈ fz_R sepanjang waktu (gait simetris di simulasi)
    → double-support terdeteksi terus → foot anchor tidak pernah diperbarui
    → KF stuck di (0,0) selamanya.
    Fix v3.2: mode simulasi menggunakan perbandingan tiap kaki vs minimumnya sendiri
    (leaky running minimum). Kaki stance = dalam 1.5cm dari minimum historis.

  FIX B — cox_registration v2: pixel_to_world konvensi diperbaiki (FIX 5)
    Bug lama: konvensi ray_z salah → semua titik ter-project ke ~2mm dari robot
    → Cox menghitung dθ=-17.6° yang konsisten (systematic rotation error)
    Fix: konvensi kamera yang benar:
      Camera optical: z=maju, x=kanan, y=bawah(image)
      Body: x=depan, y=kiri, z=atas
      Pitch rotation benar di bidang sagittal
    Hasil: pixel bawah frame → 0.5-1m, pixel atas frame → 2-5m ✓

  FIX C — goal_yaw_corrector v2.2: prior_pose_topic = /amcl_pose (default)
    Sebelumnya: anchor yaw dari /odom (raw legged KF output)
    Sekarang: anchor yaw dari /amcl_pose (sudah include scan matching global)
    Alasan: AMCL pose jauh lebih akurat untuk anchor karena sudah terintegrasi
    dengan scan matching global. /odom di Webots selalu 0,0 saat baru start.
    QoS disesuaikan: RELIABLE/TRANSIENT_LOCAL untuk /amcl_pose.

  FIX D — voronoi_precompute.py: normal_y dikonversi dari image space → world space
    Bug: normal_y = -grad_y/mag (image space, y-down positif) disimpan ke LUT.
    cox_registration.py membaca LUT dan menggunakan normal_y sebagai world space
    (y-up positif). Sign flip ini menyebabkan uy terbalik → Cox mendorong AMCL
    ke +y secara sistematis → AMCL drift +0.67m sementara GT y≈0.
    Fix: normal_y = +grad_y/mag (world space). LUT di-regenerate.

  FIX E — AMCL initial pose: diubah dari (0,0,0) ke (-0.36, 0, 0)

  FIX F — IMU yaw drift fix (v10.10):
    Bug: legged KF (v3.2) publish TF odom→base_link dengan IMU yaw yang drift
    (~16°/1.6s di startup Webots) → AMCL motion model memakai yaw salah
    → AMCL yaw konvergen ke 90° false minimum.
    Fix 1: legged_odometry_kf_node: tambah zero_tf_yaw parameter (default False),
           set True di launch → TF dan /odom pakai yaw=0, bukan IMU yaw.
           AMCL motion model bersih; EKF yaw hanya dari AMCL (pose0).
    Fix 2: ekf_soccer.yaml: odom0_config[5] (yaw) = false → EKF tidak pakai
           yaw dari /odom. EKF yaw hanya dari AMCL scan matching (pose0).
    Bug: robot spawn Webots di translation (-0.36, 0, 0.3) → ROS x=-0.36.
    AMCL mulai di (0,0) → error awal 0.36m → Cox harus koreksi dari posisi salah.
    Fix: initial_pose.x = -0.36, initial_pose.y = 0.0 (sesuai Webots spawn).

  FIX G — v10.11: revert initial_pose.x dan disable Cox yaw:
    Bug: initial_pose.x=-0.36 menyebabkan inkonsistensi TF/AMCL di startup.
    Legged KF mulai di (0,0) → map→odom = identity → TF says base_link at (0,0).
    Detector memproyeksikan scan dengan referensi TF (0,0), bukan (-0.36,0).
    Sehingga scan points offset +0.36m dalam x → AMCL score false minimum di -37°.
    Fix 1: revert initial_pose.x ke 0.0 (baseline). TF dan AMCL konsisten di (0,0).
           Cox akan koreksi posisi x,y secara bertahap → acceptable (baseline RMSE=0.51m).
    Fix 2: cox correct_yaw=False → Cox hanya kirim dx,dy ke AMCL, tidak dtheta.
           Cox WLS yaw tidak reliable saat AMCL sudah di posisi salah.
           AMCL menangani yaw sendiri via scan matching.

  FIX H — v10.12: cox inactive_center_radius_m=1.6 (zona lingkaran tengah):
    Bug: arc lingkaran tengah (r=1.5m) ter-deteksi sebagai chord lurus oleh
    HoughLinesP. Chord jatuh DI DALAM lingkaran → Voronoi LUT cell memiliki
    normal mengarah keluar (outward) → Cox memberi dx positif besar palsu
    (+0.12-0.27m/step) dan dy negatif → AMCL kolaps dari posisi benar (0.008m)
    ke posisi salah (0.58m) dalam 10 detik (t=40→50s).
    Setelah AMCL salah, semua koreksi Cox berikutnya juga salah (anchored ke
    posisi AMCL yang salah) → cascade failure.
    Fix: nonaktifkan Cox saat ||robot_pos|| < 1.6m (dalam zona lingkaran tengah).
    AMCL menangani sendiri (terbukti: RMSE=0.008m di t=40s tanpa Cox).
    Setelah robot keluar lingkaran (x>1.5m), Cox aktif kembali dengan AMCL benar.

  FIX I — v10.13: gunakan legged odom (bukan AMCL) untuk deadzone check + clip:
    Bug v10.12: deadzone check memakai self.odom_x = AMCL pose. AMCL dimulai di
    (0,0) dan tidak pernah dikoreksi Cox (Cox tidak aktif). AMCL selalu melaporkan
    dist < 1.6m → Cox tidak pernah aktif sepanjang test → deadlock.
    Tambahan: safety cap Cox (skip jika |delta| > max_delta) menyebabkan Cox tidak
    pernah bisa koreksi error besar (~0.37m, 0.50m) saat pertama aktif.
    Fix 1: use_odom_for_deadzone=True → Cox baca /odom (legged KF pure, tidak fused
           AMCL) untuk deadzone. Legged KF v3.2 tracking displacement aktual.
           Cox aktif saat odom_dist > 2.0m = GT_x ≈ 1.64m (lewat lingkaran tengah).
    Fix 2: safety cap di-clip (bukan skip): koreksi besar di-clip ke max_delta dan
           tetap diaplikasikan. Dalam 2-3 firing (0.2-0.3s), AMCL konvergen ke posisi
           benar. Skip hanya jika |delta| > 3×max_delta (scan sangat buruk).

  FIX J — v10.14: geometric world-space center circle filter + Cox→EKF direct:
    Bug v10.13: threshold 2.0m (legged odom) tidak pernah tercapai — odom hanya
    ~1.59m di akhir test (~62% efisiensi). Cox ZERO aktivasi dalam 169s.
    RMSE=2.12m (terburuk sepanjang sejarah). AMCL self-jump 4.3m di t=97s.
    Root flaw: semua deadzone berbasis posisi adalah bootstrapping-dependent —
    estimasi posisi menjadi tidak reliable persis saat dibutuhkan.

    Fix 1 [center_circle_filter_radius_m=1.6]: Di _pixel_to_world, buang titik
           yang ter-project ke dalam r=1.6m dari pusat lapangan (0,0). Chord
           HoughLinesP dari arc lingkaran jatuh di r<1.5m → dibuang otomatis.
           TIDAK perlu estimasi posisi robot — filter murni geometris.
           Garis penalty area (x=2.55m, r>2.55m) dan garis tepi (y=3m, r≥3m)
           TIDAK difilter → Cox tetap aktif di penalty area.

    Fix 2 [prior_pose_topic=/odometry/filtered]: Cox memakai EKF output sebagai
           prior WLS. EKF stabil (tidak loncat 4m seperti AMCL). Saat AMCL
           salah, EKF ≈ legged odom saja → prior masih wajar.

    Fix 3 [/cox_pose → EKF pose1]: Cox mempublish langsung ke EKF via /cox_pose,
           tidak lagi mengandalkan AMCL sebagai perantara. AMCL tetap menerima
           /initialpose (koreksi sekunder) tapi EKF tidak tunggu AMCL.

    Fix 4 [rate_hz=5.0]: 1Hz terlalu lambat. Di 5Hz, koreksi penalty area tiba
           setiap 0.2s → konvergensi ~1s setelah Cox aktif.

  FIX K — v10.15: AMCL wide initial spread + break Cox↔EKF circular dependency:
    Bug v10.14: AMCL converged to false minimum (0.255, -0.294) at t=3s.
    prior=/odometry/filtered created Cox→EKF→/odometry/filtered→Cox feedback loop.
    Both EKF and AMCL locked at wrong position (~0.33, -0.30) from t=6s. RMSE=1.05m.

    Fix 1 [AMCL covariance x,y 0.05→0.5]: wider initial particle spread (σ=0.71m,
           3σ=2.1m) covers true spawn (-0.363, 0) and resists early false minimum.

    Fix 2 [prior_pose_topic=/amcl_pose]: Cox reads AMCL, not EKF-fused output.
           /amcl_pose is NOT contaminated by /cox_pose feedback → loop broken.

    Fix 3 [pose1=/cox_pose removed from ekf_soccer.yaml]: EKF = odom + AMCL only.
           EKF no longer an amplification stage in the Cox feedback loop.

    Fix 4 [startup_delay_s=20.0]: Cox silent for first 20s after node start (t=26s
           from launch). Allows AMCL 23s to find correct position before any reinit.

  FIX L — v10.16: Asymmetric covariance + Cox /initialpose cooldown + reinstate EKF Cox:
    Bug v10.15: y covariance 0.5 (σ=0.71m) → AMCL drifted to y=0.86m false minimum.
    Bug v10.15: /initialpose at 5Hz → AMCL oscillated +0.86→-0.83 in 4s. RMSE=1.444m.
    Bug v10.15: pose1=/cox_pose removed from EKF → EKF follows AMCL errors.

    Fix L [y covariance 0.5→0.05]: y σ=0.22m → y=0.86m mirror is 3.9σ (≈0 particles).
      Asymmetric: x=0.5 (wide, covers spawn at -0.363m), y=0.05 (tight, prevents mirror).

    Fix M [initialpose_cooldown_s=5.0]: /cox_pose still 5Hz to EKF. /initialpose gated:
      max 1 AMCL reinit per 5s → AMCL converges fully between corrections. No oscillation.

    Fix N [pose1=/cox_pose reinstated in ekf_soccer.yaml]: EKF = odom + AMCL + Cox.
      Loop safe: Cox prior=/amcl_pose, NOT /odometry/filtered (no Cox↔EKF feedback).
      v10.12 best EKF (0.654m) was achieved with this architecture.

  FIX O — v10.17: Suppress Cox δy when y is geometrically unconstrained:
    Bug v10.16: WLS A[1,1]=ζ=0.001 when no ny≠0 lines → δy=b[1]/0.001 (noise×1000).
    Robot at center field sees only x-lines (ny=0). δy clipped at ±0.30m → AMCL pushed
    +0.30m wrong every 5s → AMCL_y drifted to 1.499m RMSE by t=120s.
    Fix: after WLS solve, if (A[1,1]-ζ) < min_y_constraint=5.0: zero δy.
    Only trust δy when ≥1 clear y-line observation (w_i≈2500 for d=0.02m >> 5.0).
    δx still computed normally — x-lines (ny=0) constrain x well.

  FIX P — v10.18: FAILED — Disabled /initialpose entirely.
    v10.18 disaster: AMCL converged to false x≈-0.04 (true x=-0.363) at startup.
    Without Cox /initialpose x-correction, AMCL x diverged as robot moved forward.
    AMCL_x went -0.04 → -0.43 → ±1.6 → ±5.4m. RMSE AMCL=3.83m. Do not repeat.
    LESSON: Cox /initialpose is ESSENTIAL for AMCL x-correction. Disabling it is wrong.

  FIX Q — v10.19: Restore /initialpose; fix cov_y 0.04→0.001 to prevent y reconvergence:
    Bug v10.17: /initialpose with cov_y=0.04 (σ=0.2m) spread particles ±0.6m in y.
    False y minimum at |Δy|=0.211m = 4.7σ → particles there → AMCL reconverged there.
    Fix: cov_y=0.001 (σ=0.032m, 3σ=0.095m). False minimum at |Δy|=0.211m = 6.6σ → ~0 particles.
    AMCL cannot reconverge to wrong y. cov_x=0.04 unchanged → x-correction intact.
    Restore initialpose_cooldown_s=5.0 → Cox still corrects AMCL x every 5s.

  FIX R — v10.20: Reduce center_circle_filter_radius_m 1.6→0.9 to restore center-line visibility:
    Bug v10.19: filter_radius=1.6m discards ALL center-line observations.
    Center line runs at world x=0, y∈[-3,3]. Projected point distance from origin = |y|.
    With filter=1.6m: only |y|>1.6m passes — outside typical camera FOV → Cox sees 0 points.
    Cox has NO data for entire trajectory x=-0.363→+2.28m → no /initialpose corrections.
    AMCL only moves via odom dead-reckoning (humanoid odom ~21% accuracy) → pose frozen.
    v10.19 result: AMCL_x peaked at 0.378m, GT_x reached 1.599m. RMSE=0.581m.
    Fix: filter_radius=0.9m. Center circle arc (r=0.75m) still filtered (0.75<0.9). ✓
    Center line at |y|>0.9m now passes → 10-20 points visible → Cox computes δx. ✓

Timing (tidak berubah dari v13):
  t=0  : static_tf, kf_odom, odom_throttle, map_server, ekf, camera, rectify
  t=1  : lifecycle_manager_map
  t=2  : detector, pc2scan, scan_stabilizer, field_boundary,
          segment_classifier, scan_gate
  t=3  : amcl
  t=4  : lifecycle_manager_amcl
  t=5  : particle_converter
  t=6  : goal_yaw_corrector, cox_registration
  t=7  : crossing_detector, crossing_amcl_constraint, goal_localizer
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_loc     = get_package_share_directory("soccer_object_localization")
    config_file = os.path.join(pkg_loc, "config", "op3_sim.yaml")
    map_file    = os.path.join(pkg_loc, "maps", "soccer_field.yaml")
    ekf_config  = os.path.join(pkg_loc, "config", "ekf_soccer.yaml")
    voronoi_lut = os.path.join(pkg_loc, "config", "voronoi_lut.npz")
    camera_yaml = os.path.join(pkg_loc, "config", "camera.yaml")

    white_threshold_arg = DeclareLaunchArgument(
        "white_threshold", default_value="165",
        description="Threshold putih untuk deteksi garis lapangan"
    )

    # ── t=0: Core nodes ──────────────────────────────────────────────────────
    static_tf_publisher = Node(
        package="op3_utra_bridge", executable="op3_static_transforms.py",
        name="static_tf_publisher", output="screen"
    )

    # [SURROGATE] Ganti legged_odometry_kf dengan surrogate dari GT (Fase 1B diagnostik)
    # k=1.042: odom under-report 4% (sesuai hasil Fase 0)
    # noise_pos=3mm, noise_yaw=0.005rad: simulasi noise encoder/IMU ringan
    kf_odom_node = Node(
        package="soccer_object_localization", executable="surrogate_odom_node",
        name="surrogate_odom", output="screen",
        parameters=[{
            "k_scale":    1.042,
            "noise_pos":  0.003,
            "noise_yaw":  0.005,
            "noise_vel":  0.002,
            "zero_tf_yaw": True,
        }]
    )

    odom_throttle_node = Node(
        package="topic_tools", executable="throttle", name="odom_throttle",
        output="screen",
        arguments=["messages", "/odom", "20.0", "/odom_throttled"],
    )

    camera_info_publisher_node = Node(
        package="soccer_object_localization", executable="camera_info_publisher",
        name="camera_info_publisher", output="screen",
        parameters=[{"camera_yaml_path": camera_yaml}]
    )

    rectify_node = Node(
        package="image_proc", executable="rectify_node", name="rectify_node",
        output="screen",
        remappings=[
            ("image",       "/robotis_op3/camera/image_raw"),
            ("camera_info", "/robotis_op3/camera/camera_info"),
            ("image_rect",  "/robotis_op3/camera/image_rect"),
        ]
    )

    map_server_node = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename":map_file,"topic_name":"map","frame_id":"map"}]
    )

    ekf_node = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[ekf_config],
        remappings=[("odometry/filtered","/odometry/filtered"),("/set_pose","/initialpose")]
    )

    # ── t=1: Map lifecycle ───────────────────────────────────────────────────
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"autostart":True,"node_names":["map_server"]}]
    )

    # ── t=2: Perception pipeline ─────────────────────────────────────────────
    detector_node = Node(
        package="soccer_object_localization", executable="detector_fieldline_enhanced2",
        name="detector_fieldline", output="screen",
        parameters=[config_file, {
            "use_dynamic_tf": False,
            "camera.height": 0.475, "camera.tilt": -0.349,
            "camera.offset_x": 0.08, "camera.offset_y": 0.0,
            "camera.focal_length": 793.3,
            "camera.image_width": 1280, "camera.image_height": 720,
            "detection.white_threshold": LaunchConfiguration("white_threshold"),
            "detection.use_enhanced": True,
            "detection.roi_top_cut": 0.15, "detection.roi_bottom_cut": 0.08,
            "detection.min_line_length": 15, "detection.max_line_gap": 25,
            "detection.canny_low": 60, "detection.canny_high": 180,
            "detection.hough_threshold": 60, "detection.remove_grass": True,
            "detection.grass_h_low": 35, "detection.grass_h_high": 85,
            "detection.grass_s_low": 40,
            "point_cloud.spacing": 12, "point_cloud.max_distance": 4.5,
            "point_cloud.min_points": 5,
            "publish.debug_image": True, "publish.point_cloud": True,
        }],
        remappings=[
            ("/camera/image_raw",   "/robotis_op3/camera/image_rect"),
            ("/camera/camera_info", "/robotis_op3/camera/camera_info"),
        ]
    )

    simple_pc2scan_node = Node(
        package="soccer_object_localization", executable="simple_pc2scan",
        name="simple_pc2scan", output="screen",
        parameters=[{"angle_min":-3.14159,"angle_max":3.14159,"angle_increment":0.0174533,
                     "range_min":0.3,"range_max":4.5,"scan_height":0.0}]
    )

    scan_stabilizer_node = Node(
        package="soccer_object_localization", executable="scan_stabilizer",
        name="scan_stabilizer", output="screen",
        parameters=[{"imu_topic":"/robotis_op3/imu","roll_threshold_deg":5.0,
                     "pitch_threshold_deg":6.0,"max_hold_sec":0.8,"min_stable_count":3}]
    )

    field_boundary_node = Node(
        package="soccer_object_localization", executable="field_boundary_detector",
        name="field_boundary_detector", output="screen",
        parameters=[{
            "sky_v_thresh":     55,
            "t_sky":            0.35,
            "green_h_low":      35,
            "green_h_high":     85,
            "green_s_low":      50,
            "green_v_low":      50,
            "t_green_verify":   0.30,
            "goal_bright_v":    190,
            "goal_bright_frac": 0.10,
            "goal_top_ratio":   0.50,
            "subsample_x":      8,
            "subsample_y":      8,
            "boundary_margin_px": 8,
            "min_boundary_row": 0.04,
            "max_boundary_row": 0.85,
            "smooth_kernel":    9,
            "convex_iters":     8,
            "publish_debug":    True,
        }],
        remappings=[("/robotis_op3/camera/image_raw","/robotis_op3/camera/image_rect")]
    )

    segment_classifier_node = Node(
        package="soccer_object_localization", executable="segment_classifier_gw",
        name="segment_classifier_gw", output="screen",
        parameters=[{
            "max_angle_deg":40.0,"min_projection":0.15,"vote_threshold":0.45,"scan_half":4,
            "max_angle_deg_far":45.0,"min_projection_far":0.10,"vote_threshold_far":0.35,"scan_half_far":6,
            "hough_threshold":60,"hough_min_line":60,"hough_max_gap":15,
            "min_segment_len":30,"max_segment_len":600,
            "hough_threshold_far":20,"hough_min_line_far":15,"hough_max_gap_far":25,"min_segment_len_far":12,
            "far_zone_split":0.60,"nms_dist":15,"auto_calibrate_gw":True,
            "white_threshold":200,"use_boundary_roi":True,"publish_debug":True,
            "use_line_image":True,"roi_top_fallback":0.35,
        }]
    )

    # ── t=3-5: AMCL — parameter sama dengan v10.7b (proven stable) ──────────
    amcl_node = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[{
            "odom_frame_id":"odom","base_frame_id":"cam_link","global_frame_id":"map",
            "scan_topic":"field_scan_gated",
            "min_particles":1000,"max_particles":8000,
            "recovery_alpha_slow":0.001,"recovery_alpha_fast":0.2,
            "robot_model_type":"nav2_amcl::DifferentialMotionModel",
            "alpha1":0.000001,"alpha2":0.000001,"alpha3":0.000001,
            "alpha4":0.000001,"alpha5":0.000001,
            "update_min_d":0.01,"update_min_a":0.01,"resample_interval":1,
            "laser_model_type":"likelihood_field",
            "laser_likelihood_max_dist":1.0,"laser_max_range":5.0,"laser_min_range":0.3,
            "laser_max_beams":180,"laser_z_hit":0.7,"laser_z_rand":0.3,"laser_sigma_hit":0.25,
            # [FIX G] initial_pose.x reverted to 0.0 (baseline) — TF/AMCL consistency
            "set_initial_pose":True,"initial_pose.x":0.0,"initial_pose.y":0.0,
            "initial_pose.z":0.0,"initial_pose.yaw":0.0,
            "initial_pose_covariance":[
                0.5,0.0,0.0,0.0,0.0,0.0, 0.0,0.05,0.0,0.0,0.0,0.0,  # [FIX L] x=0.5(keep), y 0.5→0.05 (σ=0.22m)
                0.0,0.0,0.01,0.0,0.0,0.0, 0.0,0.0,0.0,0.01,0.0,0.0,
                0.0,0.0,0.0,0.0,0.01,0.0, 0.0,0.0,0.0,0.0,0.0,0.02,
            ],
            "transform_tolerance":0.3,"tf_broadcast":False,
            "always_reset_initial_pose":False,"first_map_only":False,"save_pose_rate":2.0,
        }]
    )

    lifecycle_manager_amcl = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_amcl", output="screen",
        parameters=[{"autostart":True,"node_names":["amcl"]}]
    )

    particle_converter_node = Node(
        package="soccer_object_localization", executable="particle_converter",
        name="particle_converter", output="screen"
    )

    # ── t=6: Correctors ──────────────────────────────────────────────────────
    # [FIX C] v2.2: prior_pose_topic = /amcl_pose (default)
    # goal_yaw_corrector menggunakan AMCL pose sebagai anchor yaw, bukan raw odom
    goal_yaw_corrector_node = Node(
        package="soccer_object_localization", executable="goal_yaw_corrector",
        name="goal_yaw_corrector", output="screen",
        parameters=[{
            "focal_length":793.3,"image_width":1280,"image_height":720,
            "roi_top":0.02,"roi_bottom":0.42,"white_threshold":200,
            "min_line_length_px":200,"max_line_angle_deg":25.0,
            "max_yaw_delta_deg":3.0,"min_confidence":0.7,"cooldown_sec":10.0,
            "cov_xy":9.0,"cov_yaw":0.015,
            "prior_pose_topic": "/amcl_pose",  # [FIX C] v2.2: explicit AMCL prior
        }],
        remappings=[("/robotis_op3/camera/image_raw","/robotis_op3/camera/image_rect")]
    )

    # [FIX B] v2 FIX5: pixel_to_world konvensi diperbaiki
    # prior_pose_topic = /amcl_pose (sama dengan goal_yaw_corrector)
    cox_registration_node = Node(
        package="soccer_object_localization", executable="cox_registration",
        name="cox_registration", output="screen",
        parameters=[{
            "voronoi_lut_path":voronoi_lut,
            "image_width":1280,"image_height":720,"focal_length":793.3,
            "cam_pitch_deg":-20.0,"camera_height_m":0.475,
            "rate_hz":5.0,"max_delta_x":0.30,"max_delta_y":0.30,"max_delta_theta":20.0,  # [FIX J] rate 1→5Hz
            "min_points":10,"min_confidence":0.30,"outlier_dist":0.5,
            "eta":0.01,"zeta":0.001,"cov_x":0.04,"cov_y":0.001,"cov_yaw":0.02,  # [FIX Q] cov_y 0.04→0.001
            "field_half_len":4.5,"field_half_wid":3.0,
            "prior_pose_topic": "/amcl_pose",            # [FIX K] revert ke AMCL (bukan EKF/odometry/filtered)
            "startup_delay_s":  20.0,                  # [FIX K] tunggu AMCL konvergen dulu
            "initialpose_cooldown_s": 5.0,             # [FIX M] /initialpose max 1/5s (restored v10.19)
            "min_y_constraint":       5.0,             # [FIX O] zero δy when y unconstrained
            "correct_yaw": False,                      # [FIX G] Cox hanya koreksi x,y
            "inactive_center_radius_m": 0.0,           # [FIX J] geometric filter mengganti deadzone
            "use_odom_for_deadzone": False,             # [FIX J] tidak diperlukan lagi
            "center_circle_filter_radius_m": 0.9,      # [FIX R] 1.6→0.9: center line |y|>0.9m now visible
        }]
    )

    scan_gate_node = Node(
        package="soccer_object_localization", executable="scan_gate",
        name="scan_gate", output="screen",
        parameters=[{
            "near_goal_x_m":   2.5,
            "full_filter_x_m": 3.5,
            "close_range_m":   0.8,   # [v1.1] 1.5→0.8m — jaring ~0.5-0.8m
            "full_close_range": 1.0,  # [v1.1] 2.0→1.0m
            "field_half_len":   4.5,
            "input_topic":  "/field_scan_stable",
            "output_topic": "/field_scan_gated",
        }]
    )

    crossing_detector_node = Node(
        package="soccer_object_localization", executable="crossing_detector",
        name="crossing_detector", output="screen",
        parameters=[{
            "image_width":          1280,
            "image_height":          720,
            "focal_length":          793.3,
            "cam_pitch_deg":         -20.0,
            "camera_height_m":        0.475,
            "field_half_len":          4.5,
            "field_half_wid":          3.0,
            "min_seg_confidence":     0.20,
            "min_seg_length":         25.0,
            "cluster_dist_px":         60,
            "min_angle_diff_deg":      25.0,
            "near_endpoint_ratio":      0.30,
            "goal_filter_angle_deg":   70.0,
            "goal_filter_top_ratio":    0.30,
            "world_merge_dist_m":       0.25,
            "roi_top_ratio":            0.25,
            "roi_bottom_ratio":         0.97,
            "min_confidence":           0.40,
            "max_crossings_per_frame":  6,
            "publish_debug":            True,
        }]
    )

    crossing_constraint_node = Node(
        package="soccer_object_localization", executable="crossing_amcl_constraint",
        name="crossing_amcl_constraint", output="screen",
        parameters=[{
            "field_half_len":  4.5,
            "field_half_wid":  3.0,
            "penalty_x":       3.1,
            "penalty_y":       1.3,
            "sigma_d":         0.20,
            "lambda_d":        0.05,
            "sigma_beta":      0.15,
            "min_likelihood":  0.25,
            "min_confidence":  0.50,
            "max_match_dist_m": 1.50,
            "cooldown_sec":    2.0,
            "cov_x":           0.10,
            "cov_y":           0.10,
            "cov_yaw":         0.05,
            "max_correction_m": 0.50,
        }]
    )

    goal_localizer_node = Node(
        package="soccer_object_localization", executable="goal_localizer",
        name="goal_localizer", output="screen",
        parameters=[{
            "image_width":          1280,
            "image_height":          720,
            "focal_length":          793.3,
            "field_half_len":          4.5,
            "goal_width_m":            2.6,
            "goal_height_m":           1.2,
            "activation_x_m":          2.0,
            "max_yaw_deg":            60.0,
            "min_goal_width_px":        80,
            "max_goal_dist_m":           3.0,
            "white_threshold":          200,
            "post_min_height_ratio":    0.08,
            "post_max_width_px":         40,
            "post_roi_top":             0.02,
            "post_roi_bottom":          0.45,
            "cov_x":    0.10,
            "cov_y":    0.20,
            "cov_yaw":  0.10,
            "cooldown_sec": 1.5,
            "publish_debug": True,
        }],
        remappings=[("/robotis_op3/camera/image_rect","/robotis_op3/camera/image_rect")]
    )

    goal_detector_node = Node(
        package="soccer_object_localization", executable="goal_detector",
        name="goal_detector", output="screen",
        parameters=[{
            "goal_width_m":2.6,"goal_height_m":1.2,"focal_length":793.3,
            "image_width":1280,"image_height":720,
            "field_half_length":4.5,"field_half_width":3.0,
            "min_goal_width_px":80,"min_confidence":0.6,"correction_interval":0.5,
            "yaw_only_threshold":0.75,"max_yaw_correction_deg":30.0,"white_threshold":200,
        }],
        remappings=[("/robotis_op3/camera/image_raw","/robotis_op3/camera/image_rect")]
    )

    # ── Launch Description ────────────────────────────────────────────────────
    return LaunchDescription([
        white_threshold_arg,

        # t=0
        static_tf_publisher,
        kf_odom_node,
        odom_throttle_node,
        map_server_node,
        ekf_node,
        camera_info_publisher_node,
        rectify_node,

        # t=1
        TimerAction(period=1.0, actions=[lifecycle_manager_map]),

        # t=2: perception + scan_gate
        TimerAction(period=2.0, actions=[
            detector_node,
            simple_pc2scan_node,
            scan_stabilizer_node,
            field_boundary_node,
            segment_classifier_node,
            scan_gate_node,
        ]),

        # t=3: AMCL
        TimerAction(period=3.0, actions=[amcl_node]),

        # t=4: AMCL lifecycle
        TimerAction(period=4.0, actions=[lifecycle_manager_amcl]),

        # t=5: particle converter
        TimerAction(period=5.0, actions=[particle_converter_node]),

        # t=6: correctors
        TimerAction(period=6.0, actions=[
            goal_yaw_corrector_node,
            cox_registration_node,
        ]),

        # t=7: crossing detection + goal localizer
        TimerAction(period=7.0, actions=[
            crossing_detector_node,
            crossing_constraint_node,
            goal_localizer_node,
        ]),
    ])
