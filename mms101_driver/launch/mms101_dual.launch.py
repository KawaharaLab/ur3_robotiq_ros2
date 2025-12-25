import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port1_arg = DeclareLaunchArgument(
        'port1', default_value='/dev/ttyUSB1', description='Serial port for MMS101 #1'
    )
    port2_arg = DeclareLaunchArgument(
        'port2', default_value='/dev/ttyUSB2', description='Serial port for MMS101 #2'
    )
    frame1_arg = DeclareLaunchArgument(
        'frame1', default_value='ft_sensor_left', description='frame_id for MMS101 #1'
    )
    frame2_arg = DeclareLaunchArgument(
        'frame2', default_value='ft_sensor_right', description='frame_id for MMS101 #2'
    )
    force_scale_arg = DeclareLaunchArgument(
        'force_scale', default_value='0.001', description='Force scale (shared)'
    )
    torque_scale_arg = DeclareLaunchArgument(
        'torque_scale', default_value='0.00001', description='Torque scale (shared)'
    )

    node1 = Node(
        package='mms101_driver',
        executable='mms101_node',
        name='mms101_driver_1',
        output='screen',
        remappings=[('force_torque', LaunchConfiguration('topic1'))],
        parameters=[
            {
                'port': LaunchConfiguration('port1'),
                'frame_id': LaunchConfiguration('frame1'),
                'force_scale': LaunchConfiguration('force_scale'),
                'torque_scale': LaunchConfiguration('torque_scale'),
            }
        ],
    )

    node2 = Node(
        package='mms101_driver',
        executable='mms101_node',
        name='mms101_driver_2',
        output='screen',
        remappings=[('force_torque', LaunchConfiguration('topic2'))],
        parameters=[
            {
                'port': LaunchConfiguration('port2'),
                'frame_id': LaunchConfiguration('frame2'),
                'force_scale': LaunchConfiguration('force_scale'),
                'torque_scale': LaunchConfiguration('torque_scale'),
            }
        ],
    )

    return LaunchDescription([
        port1_arg,
        port2_arg,
        frame1_arg,
        frame2_arg,
        force_scale_arg,
        torque_scale_arg,
        DeclareLaunchArgument('topic1', default_value='/force_torque/left', description='Output topic for sensor 1'),
        DeclareLaunchArgument('topic2', default_value='/force_torque/right', description='Output topic for sensor 2'),
        node1,
        node2,
    ])
