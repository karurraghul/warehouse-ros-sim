from ament_index_python import get_package_share_directory
import launch
import os
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    delivery_robot_xacro = os.path.join(
        get_package_share_directory('robo_ai'),
        'models', 'urdf', 'delivery_robot_description.xacro'
    )

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro', ' ', delivery_robot_xacro]), value_type=str)
        }],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Flag to enable use_sim_time'
        ),
        robot_state_publisher,
    ])
