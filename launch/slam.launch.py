import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rb300_nav')
    config_file = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        slam_toolbox_node,
    ])
