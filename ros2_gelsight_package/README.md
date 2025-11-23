# README for ros2_gelsight_package

# ros2_gelsight_package

This package provides a ROS 2 node that publishes images from a GelSight Mini tactile camera as `sensor_msgs/Image` messages. It is designed to facilitate the integration of GelSight Mini into robotic systems, enabling the use of tactile sensing data in various applications.

## Package Structure

The package contains the following files and directories:

- **CMakeLists.txt**: Configures the build process of the ROS 2 package, specifying the package name, version, dependencies, and build instructions.
- **package.xml**: Contains metadata about the package, including its name, version, description, maintainers, and dependencies required for the package to function.
- **README.md**: This documentation file, providing instructions on how to build, run, and use the GelSight Mini publisher node.
- **launch/**: Contains launch scripts for starting the GelSight Mini publisher node.
  - **gelsight_publisher.launch.py**: Defines how to launch the node and any parameters or remappings needed.
- **resource/**: Typically contains additional resources for the package, such as configuration files or other assets.
  - **ros2_gelsight_package**: Placeholder for additional resources.
- **src/**: Contains the implementation of the GelSight Mini publisher node.
  - **ros2_gelsight_publisher.py**: Implements the `GelSightMiniPublisher` class, which handles the camera stream and publishes images to a specified ROS 2 topic.
  - **__init__.py**: Marks the `src` directory as a Python package.
- **config/**: Contains configuration settings for the GelSight Mini camera.
  - **default_config.json**: Default configuration settings, such as camera width, height, and border fraction.

## Installation

1. **Clone the repository**:
   ```
   git clone <repository_url>
   cd ros2_gelsight_package
   ```

2. **Build the package**:
   Make sure you have a ROS 2 workspace set up. Place the `ros2_gelsight_package` directory inside the `src` folder of your workspace, then build the workspace:
   ```
   cd ~/ros2_workspace
   colcon build --packages-select ros2_gelsight_package
   ```

3. **Source the workspace**:
   After building, source the workspace to make the package available:
   ```
   source install/setup.bash
   ```

## Usage

To run the GelSight Mini publisher node, use the following command:
```
ros2 launch ros2_gelsight_package gelsight_publisher.launch.py
```

You can customize the launch parameters by modifying the launch file or passing arguments directly in the command line.

## Configuration

The default configuration for the GelSight Mini camera can be found in `config/default_config.json`. You can modify this file to adjust camera settings such as width, height, and border fraction.

## License

This package is licensed under the MIT License. See the LICENSE file for more details.

## Acknowledgments

Special thanks to the developers and contributors of the GelSight Mini and ROS 2 community for their support and resources.