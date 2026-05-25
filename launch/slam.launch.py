import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rb300_nav')
    config_file = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    slam_controller_node = Node(
        package='rb300_nav',
        executable='slam_controller.py',
        name='slam_controller',
        output='screen',
        parameters=[{
            'slam_params_file': config_file,
            'command_topic': '/rb300_webui/slam_command',
        }],
    )

    return LaunchDescription([
        slam_controller_node,
    ])
