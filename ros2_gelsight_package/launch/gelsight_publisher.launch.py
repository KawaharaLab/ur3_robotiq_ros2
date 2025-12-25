import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('ros2_gelsight_package')
    default_config = os.path.join(package_share, 'config', 'default_config.json')

    args = [
        DeclareLaunchArgument('gs_config', default_value=default_config, description='Path to GelSight config JSON'),
        DeclareLaunchArgument('device_path_1', default_value='/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._GelSight_Mini_R0B_2BJB-TRZT_2BJBTRZT-video-index0', description='Device path for camera 1 (/dev/v4l/by-id/...)'),
        DeclareLaunchArgument('device_index_1', default_value='0', description='Device index for camera 1 (fallback if path empty)'),
        DeclareLaunchArgument('topic_1', default_value='gelsight/left/image_raw', description='Topic for camera 1 images'),
        DeclareLaunchArgument('frame_1', default_value='gelsight1', description='Frame id for camera 1'),

        DeclareLaunchArgument('device_path_2', default_value='/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._GelSight_Mini_R0B_2BML-JPDL_2BMLJPDL-video-index0', description='Device path for camera 2 (/dev/v4l/by-id/...)'),
        DeclareLaunchArgument('device_index_2', default_value='1', description='Device index for camera 2 (fallback if path empty)'),
        DeclareLaunchArgument('topic_2', default_value='gelsight/right/image_raw', description='Topic for camera 2 images'),
        DeclareLaunchArgument('frame_2', default_value='gelsight2', description='Frame id for camera 2'),

        DeclareLaunchArgument('publish_rate', default_value='15.0', description='Publish rate (Hz) for both cameras'),
        DeclareLaunchArgument('enable_compressed', default_value='true', description='If true, also publish compressed topics directly from the GelSight node'),
        DeclareLaunchArgument('compressed_quality', default_value='90', description='JPEG quality (1-100) for compressed output'),
    ]

    node1_args = [
        '--gs-config', LaunchConfiguration('gs_config'),
        '--device-path', LaunchConfiguration('device_path_1'),
        '--device-index', LaunchConfiguration('device_index_1'),
        '--topic-name', LaunchConfiguration('topic_1'),
        '--frame-id', LaunchConfiguration('frame_1'),
        '--publish-rate', LaunchConfiguration('publish_rate'),
        '--publish-compressed', LaunchConfiguration('enable_compressed'),
        '--compressed-quality', LaunchConfiguration('compressed_quality'),
    ]

    node2_args = [
        '--gs-config', LaunchConfiguration('gs_config'),
        '--device-path', LaunchConfiguration('device_path_2'),
        '--device-index', LaunchConfiguration('device_index_2'),
        '--topic-name', LaunchConfiguration('topic_2'),
        '--frame-id', LaunchConfiguration('frame_2'),
        '--publish-rate', LaunchConfiguration('publish_rate'),
        '--publish-compressed', LaunchConfiguration('enable_compressed'),
        '--compressed-quality', LaunchConfiguration('compressed_quality'),
    ]

    nodes = [
        Node(
            package='ros2_gelsight_package',
            executable='ros2_gelsight_publisher',
            name='gelsight_mini_publisher_1',
            output='screen',
            arguments=node1_args,
        ),
        Node(
            package='ros2_gelsight_package',
            executable='ros2_gelsight_publisher',
            name='gelsight_mini_publisher_2',
            output='screen',
            arguments=node2_args,
        ),
    ]

    return LaunchDescription(args + nodes)