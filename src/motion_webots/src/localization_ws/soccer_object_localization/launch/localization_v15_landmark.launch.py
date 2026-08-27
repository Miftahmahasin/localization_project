#!/usr/bin/env python3
"""
localization_v15_landmark.launch.py  [TAHAP 4 — geometric landmark, NO body odom]
=================================================================================
The geometric-landmark localization stack. Replaces the white-line-AMCL-as-primary
architecture (v10-v14) with typed-corner GEOMETRIC pose fixes fused by the EKF, and
removes body odometry entirely (plan 4.0: walking_param_odom / legged-KF are gait
command-echoes, blind to slip — dropped). White-line AMCL is kept only as an
OPTIONAL auxiliary channel (rule #5), off by default.

TF ownership (no odom):
  * EKF (ekf_soccer_landmark.yaml, world_frame=map, publish_tf=true) owns map->odom.
  * With no wheel/leg odometry there is no dynamic odom->base_link source, so a
    STATIC IDENTITY odom->base_link is published; map->odom then carries the full
    pose estimate (standard "map-only" wiring). AMCL runs tf_broadcast:=False.
  * Verify in live sim that no other node also publishes odom->base_link.

Detector source (arg `detector`):
  * fake (default) — landmark_localization/fake_detector replays the GT sidecar
    (boxes + head joints + CameraInfo + GT pose) with NO Webots. This is a PLUMBING
    smoke test: sidecar poses are teleports, so the no-teleport gate rejects most
    fixes and the EKF will NOT track temporally — it validates that topics connect
    and /landmark_pose is produced, not convergence. Temporal validation needs
    Webots (a walked trajectory).
  * yolo — the real detector (needs Webots + trained model): camera_info + rectify
    + landmark_detector_node -> /robot1/object_bounding_boxes.

Args:
  detector=fake|yolo   sidecar_dir=<path>   recall=<0..1>
  single_corner_mode=partial|coast          use_line_scan=false|true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_loc = get_package_share_directory("soccer_object_localization")
    map_file = os.path.join(pkg_loc, "maps", "soccer_field.yaml")
    ekf_config = os.path.join(pkg_loc, "config", "ekf_soccer_landmark.yaml")
    config_file = os.path.join(pkg_loc, "config", "op3_sim.yaml")
    voronoi_lut = os.path.join(pkg_loc, "config", "voronoi_lut.npz")
    camera_yaml = os.path.join(pkg_loc, "config", "camera.yaml")

    # ── args ─────────────────────────────────────────────────────────────────
    detector_arg = DeclareLaunchArgument("detector", default_value="fake")
    sidecar_arg = DeclareLaunchArgument(
        "sidecar_dir", default_value="/media/miftah/backup/landmark_dataset/val")
    recall_arg = DeclareLaunchArgument("recall", default_value="1.0")
    single_arg = DeclareLaunchArgument(
        "single_corner_mode", default_value="partial")
    line_arg = DeclareLaunchArgument("use_line_scan", default_value="false")
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/media/miftah/Project/landmark_deploy/"
                      "best_landmark_v8n_int8_openvino_model")
    imgsz_arg = DeclareLaunchArgument("imgsz", default_value="640")
    conf_arg = DeclareLaunchArgument("conf", default_value="0.25")
    # TAHAP 5 — committed field-side (own half). -1 = x<0, +1 = x>0, 0 = require
    # an /initialpose seed. Auto-commits the mirror mode so no manual seed is
    # needed; an /initialpose still re-commits it.
    startside_arg = DeclareLaunchArgument("start_x_sign", default_value="-1.0")
    # TAHAP 6A — active-vision gaze policy (off by default). When on, takes the
    # head to a horizon localization gaze on a blackout and releases on recovery.
    gaze_arg = DeclareLaunchArgument("use_gaze", default_value="false")
    # S1 DECISION (Aug 24): line-heading KEPT as essential heading insurance and
    # defaulted ON for the match profile. Cutoff-sweep harness proved it prevents a
    # ~108deg-median heading collapse when the robot rotates in total landmark
    # blackout (yaw p95 176->11deg, criterion-Z keep branch); null when landmarks are
    # present (C6-live). See S1_PRAREGISTRASI_LINE_HEADING.md.
    lineheading_arg = DeclareLaunchArgument(
        "use_line_heading", default_value="true")
    # EKF-trap watchdog OFF by default: TAHAP 8 diagnosis proved the failure is a
    # WRONG raw geometric fix (backend association false-minimum from a uniform
    # prior), not an EKF trap — resetting the EKF to that wrong fix only harms.
    trapwd_arg = DeclareLaunchArgument(
        "use_trap_watchdog", default_value="false")
    diagassoc_arg = DeclareLaunchArgument("diag_assoc", default_value="false")
    # B3.1 chi-square (Mahalanobis) fix-vs-prior gate threshold, 3 DOF. DEFAULT
    # 0 = DISABLED: live A/B (seeded, 5x each) showed the gate ON made tracking
    # WORSE (mirror 16.8% / flips 5.8 / converge 12 s) vs OFF (5/5 TRUE / 0% /
    # 3 s) once the reliable seed (B2.1) is in place. Opt in (>0) only with a
    # calibrated threshold.
    chi2gate_arg = DeclareLaunchArgument("chi2_gate", default_value="0.0")
    # Mirror-tracker persistent per-frame diagnostic CSV (ref/kind/contra/raw/
    # chosen/reloc/prior). Empty = off. Set a path to record the exact mirror-flip.
    mirrordiag_arg = DeclareLaunchArgument("mirror_diag_csv", default_value="")
    # C4 — cond-number covariance inflation, exposed as an arg ONLY to run the S4.2
    # decision test (ill-conditioned 2-landmark view): 0.0 = disabled (match default).
    condinflate_arg = DeclareLaunchArgument("cond_inflate_at", default_value="0.0")
    # This Webots setup does NOT publish /clock (the topic exists but is idle), so
    # the whole stack runs on WALL time (default false). Only set :=true if you
    # have verified `ros2 topic hz /clock` actually ticks. Note geometric_pose_node
    # stamps /landmark_pose with the node clock (output_stamp=now) so the EKF
    # accepts it on wall time regardless.
    sim_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    st = {"use_sim_time": ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)}

    # TAHAP H — detector-degradation relay (sweep harness). use_degrade:=true inserts
    # degrade_relay between the projector's /landmark_array and geometric_pose_node
    # (which then reads /landmark_array_deg). Knobs are live-tunable via
    # `ros2 param set /degrade_relay ...` so a sweep needs no relaunch.
    degrade_arg = DeclareLaunchArgument("use_degrade", default_value="false")
    use_degrade = IfCondition(LaunchConfiguration("use_degrade"))
    # geometric_pose_node reads the degraded topic iff use_degrade, else the raw one.
    lm_topic = PythonExpression(
        ["'/landmark_array_deg' if '", LaunchConfiguration("use_degrade"),
         "' == 'true' else '/landmark_array'"])

    detector = LaunchConfiguration("detector")
    is_fake = IfCondition(PythonExpression(["'", detector, "' == 'fake'"]))
    is_yolo = IfCondition(PythonExpression(["'", detector, "' == 'yolo'"]))
    use_line = IfCondition(LaunchConfiguration("use_line_scan"))
    use_gaze = IfCondition(LaunchConfiguration("use_gaze"))
    use_line_heading = IfCondition(LaunchConfiguration("use_line_heading"))

    bbox_topic = "/robot1/object_bounding_boxes"
    caminfo_topic = "/robotis_op3/camera/camera_info"
    joints_topic = "/robotis_op3/joint_states"

    # ── core: TF, map, EKF (no odom) ─────────────────────────────────────────
    static_tf_publisher = Node(
        package="op3_utra_bridge", executable="op3_static_transforms.py",
        name="static_tf_publisher", output="screen")

    # STATIC IDENTITY odom->base_link (replaces the dropped legged-KF). map->odom
    # (from the EKF) then carries the full pose. See header note.
    odom_base_static = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="odom_base_identity", output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"])

    map_server_node = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file, "topic_name": "map",
                     "frame_id": "map"}, st])

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}, st])

    ekf_node = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[ekf_config, st],
        remappings=[("odometry/filtered", "/odometry/filtered"),
                    ("/set_pose", "/initialpose")])

    # ── detector source ──────────────────────────────────────────────────────
    fake_detector_node = Node(
        package="landmark_localization", executable="fake_detector",
        name="fake_detector", output="screen", condition=is_fake,
        parameters=[{
            "sidecar_dir": LaunchConfiguration("sidecar_dir"),
            "rate_hz": 5.0, "loop": True,
            "recall": LaunchConfiguration("recall"),
            "bbox_topic": bbox_topic,
            "joint_states_topic": joints_topic,
            "camera_info_topic": caminfo_topic,
            "gt_pose_topic": "/ground_truth/odom",
        }, st])

    camera_info_publisher_node = Node(
        package="soccer_object_localization", executable="camera_info_publisher",
        name="camera_info_publisher", output="screen", condition=is_yolo,
        parameters=[{"camera_yaml_path": camera_yaml}, st])

    rectify_node = Node(
        package="image_proc", executable="rectify_node", name="rectify_node",
        output="screen", condition=is_yolo, parameters=[st],
        remappings=[("image", "/robotis_op3/camera/image_raw"),
                    ("camera_info", caminfo_topic),
                    ("image_rect", "/robotis_op3/camera/image_rect")])

    yolo_node = Node(
        package="landmark_detector", executable="landmark_detector_node",
        name="landmark_detector", output="screen", condition=is_yolo,
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
            "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
            "image_topic": "/robotis_op3/camera/image_rect"}, st])

    # ── geometric landmark path (always) ─────────────────────────────────────
    landmark_projector_node = Node(
        package="landmark_detector", executable="landmark_projector",
        name="landmark_projector", output="screen",
        parameters=[{
            "bbox_topic": bbox_topic,
            "camera_info_topic": caminfo_topic,
            "joint_states_topic": joints_topic,
            "output_topic": "/landmark_array",
            "base_frame": "base_link",
            # FROZEN camera-extrinsic calibration — MUST match the dataset label
            # generator (landmark_dataset_sampler.py). Without it every ground
            # ray is ~5 deg too shallow -> p_base blows up -> valid_range=False.
            "pitch_bias_deg": -5.0,
            "base_z_offset_m": 0.0,
            "pan_bias_deg": 0.0,
        }, st])

    geometric_pose_node = Node(
        package="landmark_localization", executable="geometric_pose_node",
        name="geometric_pose_node", output="screen",
        parameters=[{
            "landmark_topic": lm_topic,
            "prior_topic": "/odometry/filtered",
            "output_topic": "/landmark_pose",
            "map_frame": "map",
            "single_corner_mode": LaunchConfiguration("single_corner_mode"),
            "no_teleport_m": 1.0,
            "chi2_gate": ParameterValue(
                LaunchConfiguration("chi2_gate"), value_type=float),
            "output_stamp": "now",
            # TAHAP 5 mirror-mode hold
            "start_x_sign": ParameterValue(
                LaunchConfiguration("start_x_sign"), value_type=float),
            "commit_from_initialpose": True,
            "ekf_trap_watchdog": ParameterValue(
                LaunchConfiguration("use_trap_watchdog"), value_type=bool),
            "diag_assoc": ParameterValue(
                LaunchConfiguration("diag_assoc"), value_type=bool),
            "mirror_diag_csv": LaunchConfiguration("mirror_diag_csv"),
            # C5 — per-profile association: SIM = {L,T}=[0,1] (X distinctive -> by-type
            # only, kills the sparse-view X<->T false match). HARDWARE: set [0, 1, 2] to
            # absorb a confused X. Risk is asymmetric (small gain, no recall loss on sim).
            "assoc_agnostic_group": [0, 1],
            # C4 — cond-number cov inflation; 0.0 = DISABLED (default, no regression).
            "cond_inflate_at": ParameterValue(
                LaunchConfiguration("cond_inflate_at"), value_type=float),
            "cond_inflate_cap": 10.0,
            # C2 — only a fix with WLS residual <= this updates the side-belief ref
            # (a crouch/fall fix is still published but can't poison ref). Vision-only.
            "blend_resid_max": 0.30,
            # S2 — behavior-layer /fall event contract (IMU stays OUT of localization).
            # ON by default; inert until a separate fall-detector publishes /fall.
            # /fall=true FREEZES fix-ingest (no garbage while down); recovery from a
            # heading-changing fall is a RE-SEED (/initialpose, OPSI 1, instant). An
            # autonomous heading reloc is NOT attempted (= global relocalization, 8a).
            "fall_enable": True,
            "fall_topic": "/fall",
        }, st])

    # TAHAP H — detector-degradation relay (only when use_degrade:=true). All knobs
    # default to pass-through; step them live with `ros2 param set /degrade_relay ...`.
    degrade_relay_node = Node(
        package="landmark_localization", executable="degrade_relay",
        name="degrade_relay", output="screen", condition=use_degrade,
        parameters=[{
            "in_topic": "/landmark_array",
            "out_topic": "/landmark_array_deg",
        }, st])

    # TAHAP 6A — gaze-localization policy (reuses the soccer head-control topic)
    gaze_node = Node(
        package="landmark_localization", executable="gaze_localization_node",
        name="gaze_localization_node", output="screen", condition=use_gaze,
        parameters=[{
            "ekf_topic": "/odometry/filtered",
            "fix_topic": "/landmark_pose",
            "head_topic": "/robotis/head_control/set_joint_states",
            "enable_head_module": True,
        }, st])

    # TAHAP 6B — line-scan CO-PRIMARY: field-line heading -> EKF pose3 (yaw only).
    # Reuses the SHARED Projector (correct under down-gaze, unlike the TF-based
    # legacy fieldline detector); NEVER touches AMCL (rule #3).
    line_heading_node = Node(
        package="landmark_detector", executable="line_heading_node",
        name="line_heading_node", output="screen", condition=use_line_heading,
        parameters=[{
            "image_topic": "/robotis_op3/camera/image_rect",
            "camera_info_topic": caminfo_topic,
            "joint_states_topic": joints_topic,
            "ekf_topic": "/odometry/filtered",
            "output_topic": "/line_heading",
            "map_frame": "map",
            "pitch_bias_deg": -5.0,
            "base_z_offset_m": 0.0,
            "pan_bias_deg": 0.0,
            "ground_max_range_m": 5.0,
        }, st])

    # ── OPTIONAL white-line AMCL auxiliary channel (rule #5) ─────────────────
    # off by default; needs Webots camera. tf_broadcast:=False (EKF owns TF).
    detector_fieldline = Node(
        package="soccer_object_localization",
        executable="detector_fieldline_enhanced2", name="detector_fieldline",
        output="screen", condition=use_line,
        parameters=[config_file, {
            "use_dynamic_tf": False, "camera.height": 0.475,
            "camera.tilt": -0.349, "camera.focal_length": 793.3,
            "camera.image_width": 1280, "camera.image_height": 720,
            "detection.white_threshold": 165, "detection.use_enhanced": True,
            "point_cloud.max_distance": 4.5, "publish.point_cloud": True}, st],
        remappings=[("/camera/image_raw", "/robotis_op3/camera/image_rect"),
                    ("/camera/camera_info", caminfo_topic)])

    simple_pc2scan = Node(
        package="soccer_object_localization", executable="simple_pc2scan",
        name="simple_pc2scan", output="screen", condition=use_line,
        parameters=[{"angle_min": -3.14159, "angle_max": 3.14159,
                     "angle_increment": 0.0174533, "range_min": 0.3,
                     "range_max": 4.5, "scan_height": 0.0}, st])

    scan_stabilizer = Node(
        package="soccer_object_localization", executable="scan_stabilizer",
        name="scan_stabilizer", output="screen", condition=use_line,
        parameters=[{"imu_topic": "/robotis_op3/imu", "roll_threshold_deg": 5.0,
                     "pitch_threshold_deg": 6.0, "max_hold_sec": 0.8,
                     "min_stable_count": 3}, st])

    scan_gate = Node(
        package="soccer_object_localization", executable="scan_gate",
        name="scan_gate", output="screen", condition=use_line,
        parameters=[{"near_goal_x_m": 2.5, "full_filter_x_m": 3.5,
                     "close_range_m": 0.8, "full_close_range": 1.0,
                     "field_half_len": 4.5, "input_topic": "/field_scan_stable",
                     "output_topic": "/field_scan_gated"}, st])

    amcl_node = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        condition=use_line,
        parameters=[{
            "odom_frame_id": "odom", "base_frame_id": "cam_link",
            "global_frame_id": "map", "scan_topic": "field_scan_gated",
            "min_particles": 1000, "max_particles": 8000,
            "robot_model_type": "nav2_amcl::DifferentialMotionModel",
            "update_min_d": 0.01, "update_min_a": 0.01, "resample_interval": 1,
            "laser_model_type": "likelihood_field",
            "laser_likelihood_max_dist": 1.0, "laser_max_range": 5.0,
            "laser_min_range": 0.3, "laser_max_beams": 180,
            "set_initial_pose": False, "transform_tolerance": 0.3,
            "tf_broadcast": False, "first_map_only": False}, st])

    lifecycle_manager_amcl = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_amcl", output="screen", condition=use_line,
        parameters=[{"autostart": True, "node_names": ["amcl"]}, st])

    return LaunchDescription([
        detector_arg, sidecar_arg, recall_arg, single_arg, line_arg,
        model_arg, imgsz_arg, conf_arg, sim_arg, startside_arg, gaze_arg,
        lineheading_arg, trapwd_arg, diagassoc_arg, chi2gate_arg, mirrordiag_arg,
        condinflate_arg, degrade_arg,

        # t=0 core
        static_tf_publisher, odom_base_static, map_server_node, ekf_node,

        # t=1 map lifecycle
        TimerAction(period=1.0, actions=[lifecycle_manager_map]),

        # t=2 detector + geometric path
        # NOTE: camera_info_publisher_node is intentionally NOT launched — Webots
        # (op3_webots_extern_controller) already publishes the correct 1920x1080
        # camera_info (fx=1185.6). Running our own too gives TWO publishers on
        # /robotis_op3/camera/camera_info; the projector latches whichever arrives
        # first and never updates, so a wrong (1280x720/fx=900 default) latch would
        # break projection non-deterministically. Keep a single source of truth.
        TimerAction(period=2.0, actions=[
            fake_detector_node, rectify_node,
            yolo_node, landmark_projector_node, degrade_relay_node,
            geometric_pose_node, gaze_node,
            line_heading_node]),

        # t=3 optional line-scan auxiliary
        TimerAction(period=3.0, actions=[
            detector_fieldline, simple_pc2scan, scan_stabilizer, scan_gate]),
        TimerAction(period=4.0, actions=[amcl_node]),
        TimerAction(period=5.0, actions=[lifecycle_manager_amcl]),
    ])
