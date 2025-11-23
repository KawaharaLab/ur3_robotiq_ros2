import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ros2_gelsight_package')
    default_config = os.path.join(package_share, 'config', 'default_config.json')

    return LaunchDescription([
        Node(
            package='ros2_gelsight_package',
            executable='ros2_gelsight_publisher',
            name='gelsight_mini_publisher',
            output='screen',
            parameters=[{'gs_config': default_config}],
        ),
    ])