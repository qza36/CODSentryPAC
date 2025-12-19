from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('trajectory_generation'),
        'config',
        'planner_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='trajectory_generation',
            executable='trajectory_generation',
            name='trajectory_generator',
            parameters=[config],
            output='screen'
        )
    ])