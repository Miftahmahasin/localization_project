# Launch the YOLO landmark detector for Webots simulation (OpenVINO backend).
#
# Identical to yolo_landmark_detector_webots.launch.py but uses the OpenVINO
# IR model (field8n_openvino_model/) for faster CPU inference.
#
# Subscribes to:
#   /robotis_op3/camera/image_raw  (sensor_msgs/Image, BGRA8, 1280×720)
# Publishes:
#   /landmark_detector_node/detections   (vision_msgs/Detection2DArray)
#   /landmark_detector_node/image_out    (sensor_msgs/Image, annotated debug)
#
# Export the model once if not present in <share>/model/:
#   yolo export model=field8n.pt format=openvino imgsz=512
#
# Examples:
#   ros2 launch op3_ball_detector yolo_landmark_detector_openvino_webots.launch.py
#   # Limit CPU cores to leave room for the walking controller:
#   ros2 launch op3_ball_detector yolo_landmark_detector_openvino_webots.launch.py \
#        cpu_affinity:=0-3 omp_threads:=2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('op3_ball_detector')

    param_path = PathJoinSubstitution(
        [pkg_share, 'config',
         'yolo_landmark_detector_openvino_webots_params.yaml'])

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

    # Optionally cap OpenMP/BLAS threads and pin to specific cores so the
    # detector does not starve the walking controller.
    cpu_affinity = LaunchConfiguration('cpu_affinity').perform(context)
    omp_threads = LaunchConfiguration('omp_threads').perform(context)
    node_extra = {}
    if omp_threads:
        node_extra['additional_env'] = {
            'OMP_NUM_THREADS': omp_threads,
            'OPENBLAS_NUM_THREADS': omp_threads,
        }
    if cpu_affinity:
        node_extra['prefix'] = 'taskset -c %s' % cpu_affinity

    landmark_node = Node(
        package='op3_ball_detector',
        namespace='landmark_detector_node',
        executable='yolo_landmark_detector.py',
        name='yolo_landmark_detector',
        output='screen',
        parameters=node_params,
        remappings=[('image_in', image_topic)],
        **node_extra,
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
            description='Override model path (abs path or relative to package '
                        'share). Empty = use YAML default '
                        '(model/field8n_openvino_model).'),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='',
            description='Override confidence threshold. Empty = use YAML (0.40).'),
        DeclareLaunchArgument(
            'detection_rate', default_value='',
            description='Override inference rate (Hz). Empty = use YAML (20.0).'),
        DeclareLaunchArgument(
            'omp_threads', default_value='2',
            description='Cap OpenMP/BLAS threads for pre/post-processing. '
                        'Empty = no cap.'),
        DeclareLaunchArgument(
            'cpu_affinity', default_value='',
            description='taskset core list (e.g. "0-3") to pin detector threads. '
                        'Empty = no pinning.'),
        OpaqueFunction(function=_launch_setup),
    ])
