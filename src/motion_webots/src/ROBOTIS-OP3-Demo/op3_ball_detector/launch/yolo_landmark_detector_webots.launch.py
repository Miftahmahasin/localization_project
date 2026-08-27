# Launch the YOLO landmark detector for Webots simulation (ONNX backend).
#
# Subscribes to the raw Image published by Webots:
#   /robotis_op3/camera/image_raw  (sensor_msgs/Image, BGRA8, 1280×720)
#
# Publishes:
#   /landmark_detector_node/detections   (vision_msgs/Detection2DArray)
#   /landmark_detector_node/image_out    (sensor_msgs/Image, annotated debug)
#
# Default model: <share>/model/field8n.onnx (8 soccer-field landmark classes).
#
# Examples:
#   ros2 launch op3_ball_detector yolo_landmark_detector_webots.launch.py
#   # Override confidence threshold:
#   ros2 launch op3_ball_detector yolo_landmark_detector_webots.launch.py \
#        confidence_threshold:=0.5
#   # Only detect goal posts and corners:
#   ros2 launch op3_ball_detector yolo_landmark_detector_webots.launch.py \
#        target_classes:="['Gawang','Corner','left corner','right corner']"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')

    param_path = PathJoinSubstitution(
        [pkg_share, 'config', 'yolo_landmark_detector_webots_params.yaml'])

    overrides = {}

    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path:
        overrides['model_path'] = model_path

    conf = LaunchConfiguration('confidence_threshold').perform(context)
    if conf:
        overrides['confidence_threshold'] = float(conf)

    rate = LaunchConfiguration('detection_rate').perform(context)
    if rate:
        overrides['detection_rate'] = float(rate)

    image_topic = LaunchConfiguration('image_topic').perform(context)

    node_params = [param_path]
    if overrides:
        node_params.append(overrides)

    landmark_node = Node(
        package='op3_ball_detector',
        namespace='landmark_detector_node',
        executable='yolo_landmark_detector.py',
        name='yolo_landmark_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', image_topic)],
    )

    return [landmark_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/robotis_op3/camera/image_raw',
            description='Raw Image topic from Webots camera.'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override model path. Empty = use YAML default '
                        '(model/field8n.onnx relative to package share).'),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='',
            description='Override detection confidence threshold. '
                        'Empty = use YAML default (0.40).'),
        DeclareLaunchArgument(
            'detection_rate', default_value='',
            description='Override inference rate (Hz). Empty = use YAML default.'),
        OpaqueFunction(function=_launch_setup),
    ])
