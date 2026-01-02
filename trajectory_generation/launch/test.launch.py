from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('trajectory_generation')
    config_file = os.path.join(pkg_dir, 'config', 'planner_config.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'test.rviz')

    return LaunchDescription([
        # 假里程计节点
        Node(
            package='trajectory_generation',
            executable='fake_odom_node',
            name='fake_odom',
            parameters=[{'x': 5.0, 'y': 8.0}],
            output='screen'
        ),
        # 轨迹生成节点
        Node(
            package='trajectory_generation',
            executable='trajectory_generation_node',
            name='trajectory_generator',
            parameters=[config_file],
            output='screen'
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
