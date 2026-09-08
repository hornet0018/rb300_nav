import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rb300_nav')
    config_file = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    slam_mode = LaunchConfiguration('slam_mode')
    map_dir = LaunchConfiguration('map_dir')

    slam_controller_node = Node(
        package='rb300_nav',
        executable='slam_controller.py',
        name='slam_controller',
        output='screen',
        parameters=[{
            'slam_params_file': config_file,
            'command_topic': '/rb300_webui/slam_command',
            'slam_mode': slam_mode,
            'map_dir': map_dir,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_mode',
            default_value='slam_toolbox',
            description='Scan matcher: slam_toolbox or cartographer'
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value='~/.rb300/maps',
            description='Directory where saved maps (pbstream / pgm) are stored'
        ),
        slam_controller_node,
    ])
