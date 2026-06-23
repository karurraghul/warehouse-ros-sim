from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    robo_ai_pkg = get_package_share_directory('robo_ai')
    rviz_launch_file = os.path.join(robo_ai_pkg, 'launch', 'rviz.launch.py')
    state_publisher_launch_file = os.path.join(robo_ai_pkg, 'launch', 'state_publisher.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(state_publisher_launch_file)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch_file)),
        

    ])