from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('ns1', default_value='camera_wrist', description='Namespace for camera 1'),
        DeclareLaunchArgument('ns2', default_value='camera_fixed', description='Namespace for camera 2'),
        DeclareLaunchArgument('serial1', default_value='143122066280', description='Serial number for camera 1 (empty = first found)'),
        DeclareLaunchArgument('serial2', default_value='141722062040', description='Serial number for camera 2 (empty = next found)'),
        DeclareLaunchArgument('color_profile1', default_value='640x360x15', description='RGB color profile for camera 1'),
        DeclareLaunchArgument('color_profile2', default_value='640x360x15', description='RGB color profile for camera 2'),
        DeclareLaunchArgument('enable_depth1', default_value='false', description='Enable depth for camera 1'),
        DeclareLaunchArgument('enable_depth2', default_value='false', description='Enable depth for camera 2'),
    ]

    node1 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=LaunchConfiguration('ns1'),
        name='realsense2_camera',
        output='screen',
        parameters=[
            {'serial_no': ParameterValue(LaunchConfiguration('serial1'), value_type=str)},
            {'rgb_camera.color_profile': LaunchConfiguration('color_profile1')},
            {'enable_depth': LaunchConfiguration('enable_depth1')},
        ],
    )

    node2 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=LaunchConfiguration('ns2'),
        name='realsense2_camera',
        output='screen',
        parameters=[
            {'serial_no': ParameterValue(LaunchConfiguration('serial2'), value_type=str)},
            {'rgb_camera.color_profile': LaunchConfiguration('color_profile2')},
            {'enable_depth': LaunchConfiguration('enable_depth2')},
        ],
    )

    return LaunchDescription(args + [node1, node2])
