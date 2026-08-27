# Landmark detector — TensorRT INT8 backend (Jetson Orin Nano).
#
#   ros2 launch landmark_detector detect_tensorrt.launch.py \
#        model_path:=/path/to/best_int8.engine \
#        imgsz:=<chosen by benchmark> image_topic:=/camera/image_raw
#
# The .engine is built ON the Orin by export_tensorrt_orin.sh (not portable).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('iou', default_value='0.5'),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('class_map', default_value='identity'),
        DeclareLaunchArgument('publish_debug', default_value='true'),
    ]
    node = Node(
        package='landmark_detector', executable='landmark_detector_node',
        name='landmark_detector', output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'backend': 'tensorrt',
            'image_topic': LaunchConfiguration('image_topic'),
            'imgsz': LaunchConfiguration('imgsz'),
            'conf': LaunchConfiguration('conf'),
            'iou': LaunchConfiguration('iou'),
            # TensorRT precision is baked into the .engine at build time; keep
            # half=false here so predict() doesn't try to re-cast inputs.
            'half': False,
            'device': LaunchConfiguration('device'),
            'class_map': LaunchConfiguration('class_map'),
            'publish_debug': LaunchConfiguration('publish_debug'),
        }])
    return LaunchDescription(args + [node])
