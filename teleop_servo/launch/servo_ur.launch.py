import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_param_builder import ParameterBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    ur_type = "ur3"

    home_max_joint_speed = LaunchConfiguration("home_max_joint_speed", default="8.0")
    home_deadband = LaunchConfiguration("home_deadband", default="0.01")
    home_min_joint_speed = LaunchConfiguration("home_min_joint_speed", default="0.5")
    home_velocity_gain = LaunchConfiguration("home_velocity_gain", default="2.0")

    robot_description = (
        # ParameterBuilder("ur_description")
        ParameterBuilder("teleop_servo")
        .xacro_parameter(
            "robot_description",
            # "urdf/ur.urdf.xacro",
            "config/ur3_with_robotiq_2f_140.urdf.xacro",
            mappings={
                "name": "ur",
                "ur_type": ur_type,
                "prefix": "",
                "tf_prefix": "",
                "use_fake_hardware": "false",
                "headless_mode": "false",
                "gripper_prefix": "robotiq_",
                "gripper_static_fingers": "true",
                "flange_to_gripper_xyz": "0 0 0.136",
                "flange_to_gripper_rpy": "0 0 0",
            },
        )
        .to_dict()
    )

    robot_description_semantic = (
        ParameterBuilder("ur_moveit_config")
        .xacro_parameter(
            "robot_description_semantic",
            "srdf/ur.srdf.xacro",
            mappings={"name": "ur", "prefix": ""},
        )
        .to_dict()
    )

    kinematics_data = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    if kinematics_data is None:
        raise RuntimeError("Failed to load UR kinematics configuration")

    # The upstream YAML stores the actual kinematics entries beneath /**/ros__parameters,
    # so flatten it to the format expected by the Servo node parameters list.
    robot_description_kinematics_params = (
        kinematics_data.get("/**", {})
        .get("ros__parameters", {})
        .get("robot_description_kinematics")
    )
    if robot_description_kinematics_params is None:
        raise RuntimeError("UR kinematics YAML is missing robot_description_kinematics data")

    robot_description_kinematics = {
        "robot_description_kinematics": robot_description_kinematics_params
    }

    # Get parameters for the Servo node
    servo_yaml = load_yaml("teleop_servo", "config/ur_real_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="teleop_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="teleop_servo",
                plugin="teleop_servo::JoyToServoPubUr",
                name="controller_to_servo_node",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {
                        "home_max_joint_speed": home_max_joint_speed,
                        "home_deadband": home_deadband,
                        "home_min_joint_speed": home_min_joint_speed,
                        "home_velocity_gain": home_velocity_gain,
                    },
                ],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("home_max_joint_speed", default_value="8.0"),
            DeclareLaunchArgument("home_deadband", default_value="0.01"),
            DeclareLaunchArgument("home_min_joint_speed", default_value="0.5"),
            DeclareLaunchArgument("home_velocity_gain", default_value="2.0"),
            servo_node,
            container,
        ]
    )
