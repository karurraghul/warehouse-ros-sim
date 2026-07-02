from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='camera_topic',
            default_value='/delivery_camera/image_raw',
            description=(
                'Camera image topic to subscribe to. Run `ros2 topic list` '
                'after starting the sim and override this if it differs.'),
        ),
        Node(
            package='robo_ai_vision',
            executable='marker_detector_node',
            name='marker_detector_node',
            output='screen',
            parameters=[{'camera_topic': camera_topic}],
        ),
    ])
