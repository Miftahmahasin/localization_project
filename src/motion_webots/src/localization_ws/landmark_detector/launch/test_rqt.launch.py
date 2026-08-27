# Landmark detector — one-command test on the live Webots camera, viewed in rqt.
#
# Prereq (separate terminal): ros2 launch op3_webots_ros2 robot_launch.py
#
#   ros2 launch landmark_detector test_rqt.launch.py            # .pt (default)
#
#   ros2 launch landmark_detector test_rqt.launch.py \
#        backend:=openvino \
#        model_path:=/media/miftah/Project/landmark_deploy/best_landmark_v8n_int8_openvino_model
#
# Publishes /robot1/detection_image (annotated) + /robot1/object_bounding_boxes,
# and auto-opens rqt_image_view on the detection image (open_rqt:=false to skip).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEPLOY = '/media/miftah/Project/landmark_deploy'
_DETECTION_TOPIC = '/robot1/detection_image'


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'model_path',
            default_value='%s/best_landmark_v8n.pt' % _DEPLOY),
        DeclareLaunchArgument('backend', default_value='pt'),
        DeclareLaunchArgument('image_topic',
                              default_value='/robotis_op3/camera/image_raw'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('iou', default_value='0.5'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('class_map', default_value='identity'),
        DeclareLaunchArgument('open_rqt', default_value='true'),
    ]
    detector = Node(
        package='landmark_detector', executable='landmark_detector_node',
        name='landmark_detector', output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'backend': LaunchConfiguration('backend'),
            'image_topic': LaunchConfiguration('image_topic'),
            'imgsz': LaunchConfiguration('imgsz'),
            'conf': LaunchConfiguration('conf'),
            'iou': LaunchConfiguration('iou'),
            'half': False,
            'device': LaunchConfiguration('device'),
            'class_map': LaunchConfiguration('class_map'),
            'publish_debug': True,
        }])
    rqt = Node(
        package='rqt_image_view', executable='rqt_image_view',
        name='rqt_image_view', output='screen',
        arguments=[_DETECTION_TOPIC],
        condition=IfCondition(LaunchConfiguration('open_rqt')))
    return LaunchDescription(args + [detector, rqt])
