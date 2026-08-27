# Landmark detector — OpenVINO INT8 backend (Intel NUC).
#
#   ros2 launch landmark_detector detect_openvino.launch.py \
#        model_path:=/path/to/best_int8_openvino_model \
#        imgsz:=<chosen by benchmark> image_topic:=/camera/image_raw
#
# model_path is the OpenVINO IR *directory* produced by export_openvino.py.
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
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('class_map', default_value='identity'),
        DeclareLaunchArgument('publish_debug', default_value='true'),
    ]
    node = Node(
        package='landmark_detector', executable='landmark_detector_node',
        name='landmark_detector', output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'backend': 'openvino',
            'image_topic': LaunchConfiguration('image_topic'),
            'imgsz': LaunchConfiguration('imgsz'),
            'conf': LaunchConfiguration('conf'),
            'iou': LaunchConfiguration('iou'),
            'half': False,
            'device': LaunchConfiguration('device'),
            'class_map': LaunchConfiguration('class_map'),
            'publish_debug': LaunchConfiguration('publish_debug'),
        }])
    return LaunchDescription(args + [node])
