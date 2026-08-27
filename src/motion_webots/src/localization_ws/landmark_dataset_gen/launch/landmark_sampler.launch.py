# Automated stratified landmark-dataset sampler.
#
# Teleports the OP3 through many poses via the supervisor teleport topic
# (/robotis_op3/set_pose, added to op3_extern_controller), varies the head,
# projects the known landmarks, and saves image+label+debug triplets plus a
# report (report.txt / report.json / report_montage.png).
#
# Prereq: Webots + op3_extern_controller (rebuilt with teleport) running,
# robot held standing (op3_manager) with head-scan behavior DISABLED.
#
# Confirm-first — run a SMALL batch and inspect the report before a full run:
#   ros2 launch landmark_dataset_gen landmark_sampler.launch.py num_samples:=200
#
# Full run (after verifying balance), plus a SEPARATE val run:
#   ros2 launch landmark_dataset_gen landmark_sampler.launch.py \
#        num_samples:=4000 output_dir:=~/landmark_dataset/train seed:=1
#   ros2 launch landmark_dataset_gen landmark_sampler.launch.py \
#        num_samples:=600  output_dir:=~/landmark_dataset/val   seed:=777

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    def val(name):
        return LaunchConfiguration(name).perform(context)

    params = {
        'image_topic': val('image_topic'),
        'camera_info_topic': val('camera_info_topic'),
        'odom_topic': val('odom_topic'),
        'joint_states_topic': val('joint_states_topic'),
        'set_pose_topic': val('set_pose_topic'),
        'output_dir': val('output_dir'),
        'num_samples': int(val('num_samples')),
        'seed': int(val('seed')),
        'resume': val('resume') in ('true', 'True', '1'),
        'max_range_m': float(val('max_range_m')),
        'ground_max_range_m': float(val('ground_max_range_m')),
        'min_emit_px': float(val('min_emit_px')),
        'pitch_bias_deg': float(val('pitch_bias_deg')),
        'pan_bias_deg': float(val('pan_bias_deg')),
        'base_z_offset': float(val('base_z_offset')),
        'frac_coverage': float(val('frac_coverage')),
        'frac_landmark': float(val('frac_landmark')),
        'frac_edge': float(val('frac_edge')),
        'frac_free': float(val('frac_free')),
        'place_half_len': float(val('place_half_len')),
        'place_half_wid': float(val('place_half_wid')),
        'debug_first': int(val('debug_first')),
        'debug_every': int(val('debug_every')),
        'settle_extra_s': float(val('settle_extra_s')),
        'post_capture_s': float(val('post_capture_s')),
        'head_cmd_mode': val('head_cmd_mode'),
        'head_mode': val('head_mode'),
        'fixed_head_tilt_deg': float(val('fixed_head_tilt_deg')),
        'fixed_head_pan_deg': float(val('fixed_head_pan_deg')),
        'enable_head_module': val('enable_head_module').lower() in
        ('1', 'true', 'yes'),
        'domain_randomize': val('domain_randomize').lower() in
        ('1', 'true', 'yes'),
        'dr_brightness': float(val('dr_brightness')),
        'dr_contrast': float(val('dr_contrast')),
        'dr_gamma': float(val('dr_gamma')),
        'dr_hue': float(val('dr_hue')),
        'dr_sat': float(val('dr_sat')),
        'dr_val': float(val('dr_val')),
        'dr_noise': float(val('dr_noise')),
        'dr_blur_prob': float(val('dr_blur_prob')),
        'sim_dr': val('sim_dr').lower() in ('1', 'true', 'yes'),
        'dr_sim_amb_lo': float(val('dr_sim_amb_lo')),
        'dr_sim_amb_hi': float(val('dr_sim_amb_hi')),
        'dr_sim_int_lo': float(val('dr_sim_int_lo')),
        'dr_sim_int_hi': float(val('dr_sim_int_hi')),
        'dr_sim_sky_prob': float(val('dr_sim_sky_prob')),
    }
    return [Node(package='landmark_dataset_gen',
                 executable='landmark_dataset_sampler',
                 name='landmark_dataset_sampler',
                 output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic',
                              default_value='/robotis_op3/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/robotis_op3/camera/camera_info'),
        DeclareLaunchArgument('odom_topic',
                              default_value='/ground_truth/odom'),
        DeclareLaunchArgument('joint_states_topic',
                              default_value='/robotis_op3/joint_states'),
        DeclareLaunchArgument('set_pose_topic',
                              default_value='/robotis_op3/set_pose'),
        DeclareLaunchArgument('output_dir', default_value='~/landmark_dataset'),
        DeclareLaunchArgument('num_samples', default_value='200'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('resume', default_value='false'),
        DeclareLaunchArgument('max_range_m', default_value='9.0'),
        # TAHAP 6: coverage plateaus at 7.0 m (95% frames >=2 ground junctions,
        # vs 79.5% at 5.0). Residual ~2 px to 9 m (GATE 1), so range is set by
        # coverage + box detectability, not by a phantom far-horizon droop.
        DeclareLaunchArgument('ground_max_range_m', default_value='7.0'),
        DeclareLaunchArgument('min_emit_px', default_value='18.0'),
        # FROZEN (GATE 1): -5.0 deg is a real URDF-mount-vs-render offset; with it
        # the line-model residual is centered (signed 0.00 px, 0-9 m). Do NOT tune
        # it — the old "droop" was box SHAPE (TAHAP 4). base_z_offset stays 0.0.
        DeclareLaunchArgument('pitch_bias_deg', default_value='-5.0'),
        DeclareLaunchArgument('pan_bias_deg', default_value='0.0'),
        DeclareLaunchArgument('base_z_offset', default_value='0.0'),
        DeclareLaunchArgument('frac_coverage', default_value='0.55'),
        DeclareLaunchArgument('frac_landmark', default_value='0.30'),
        DeclareLaunchArgument('frac_edge', default_value='0.15'),
        DeclareLaunchArgument('frac_free', default_value='0.08'),
        # standing bounds — INSIDE the painted lines (±4.5 / ±3.0)
        DeclareLaunchArgument('place_half_len', default_value='4.3'),
        DeclareLaunchArgument('place_half_wid', default_value='2.8'),
        # debug-overlay thinning (labels + metadata still written every frame)
        DeclareLaunchArgument('debug_first', default_value='300'),
        DeclareLaunchArgument('debug_every', default_value='25'),
        DeclareLaunchArgument('settle_extra_s', default_value='3.0'),
        # quiet gap after capture, before next teleport (total cadence 3.25 s)
        DeclareLaunchArgument('post_capture_s', default_value='0.25'),
        # 'manager' (op3_manager running, default) or 'direct' (bare controller)
        DeclareLaunchArgument('head_cmd_mode', default_value='manager'),
        DeclareLaunchArgument('head_mode', default_value='aim'),
        DeclareLaunchArgument('fixed_head_tilt_deg', default_value='-15.0'),
        DeclareLaunchArgument('fixed_head_pan_deg', default_value='0.0'),
        DeclareLaunchArgument('enable_head_module', default_value='true'),
        # domain randomization (photometric) — enable for full train/val runs
        DeclareLaunchArgument('domain_randomize', default_value='false'),
        DeclareLaunchArgument('dr_brightness', default_value='0.12'),
        DeclareLaunchArgument('dr_contrast', default_value='0.20'),
        DeclareLaunchArgument('dr_gamma', default_value='0.25'),
        DeclareLaunchArgument('dr_hue', default_value='6.0'),
        DeclareLaunchArgument('dr_sat', default_value='0.25'),
        DeclareLaunchArgument('dr_val', default_value='0.15'),
        DeclareLaunchArgument('dr_noise', default_value='2.0'),
        DeclareLaunchArgument('dr_blur_prob', default_value='0.0'),
        # sim-to-real RENDER DR (real Webots lighting/shadows); needs the
        # rebuilt controller + updated world (Webots restart required).
        DeclareLaunchArgument('sim_dr', default_value='false'),
        DeclareLaunchArgument('dr_sim_amb_lo', default_value='0.8'),
        DeclareLaunchArgument('dr_sim_amb_hi', default_value='1.1'),
        DeclareLaunchArgument('dr_sim_int_lo', default_value='0.4'),
        DeclareLaunchArgument('dr_sim_int_hi', default_value='1.4'),
        DeclareLaunchArgument('dr_sim_sky_prob', default_value='0.5'),
        OpaqueFunction(function=_setup),
    ])
