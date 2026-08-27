# Launch the interactive landmark dataset capture tool.
#
# Assumes Webots + op3_extern_controller are ALREADY running (so the camera,
# camera_info and /ground_truth/odom topics are published). Position the robot
# MANUALLY in the Webots editor; this tool overlays the projected landmark
# labels live and saves image+label pairs on the 's' key.
#
# Examples:
#   ros2 launch landmark_dataset_gen landmark_capture.launch.py
#   ros2 launch landmark_dataset_gen landmark_capture.launch.py \
#        output_dir:=/data/op3_landmarks pitch_bias_deg:=-2.0
#   ros2 launch landmark_dataset_gen landmark_capture.launch.py \
#        max_range_m:=5.0 min_box_px:=8

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
        'output_dir': val('output_dir'),
        'max_range_m': float(val('max_range_m')),
        'min_box_px': float(val('min_box_px')),
        'base_z_offset': float(val('base_z_offset')),
        'pitch_bias_deg': float(val('pitch_bias_deg')),
        'pan_bias_deg': float(val('pan_bias_deg')),
        'enable_head_control': val('enable_head_control').lower() in
        ('1', 'true', 'yes'),
    }

    node = Node(
        package='landmark_dataset_gen',
        executable='landmark_dataset_capture',
        name='landmark_dataset_capture',
        output='screen',
        parameters=[params],
    )
    return [node]


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
        DeclareLaunchArgument('output_dir', default_value='~/landmark_dataset'),
        DeclareLaunchArgument('max_range_m', default_value='9.0'),
        DeclareLaunchArgument('min_box_px', default_value='6.0'),
        DeclareLaunchArgument('base_z_offset', default_value='0.0'),
        DeclareLaunchArgument('pitch_bias_deg', default_value='-5.0'),
        DeclareLaunchArgument('pan_bias_deg', default_value='0.0'),
        DeclareLaunchArgument('enable_head_control', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
