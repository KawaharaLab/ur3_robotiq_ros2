# robotiq_2f_gripper_ros2

A ROS2 package for use with the Robotiq 2-Finger 140mm gripper.

## Overview

The repo contains four ROS2 packages (detailed description below):

| Package Name | Description |
| --- | --- |
| robotiq_2f_gripper_description | A description package containing the URDF and meshes of the gripper to be viewed in RViz  |
| robotiq_2f_gripper_hardware | A hardware package creating a ROS2 action server to control the gripper |
| robotiq_2f_gripper_interfaces | A package containing the driver of the gripper |
| robotiq_2f_gripper_msgs | A packages defining a message template for gripper communication  |

## Getting Started

These instrcutions assume you have ROS2 (tested with ROS2 Humble) installed. The repo depends on the [serial-ros2](https://github.com/RoverRobotics-forks/serial-ros2) package for communication with the gripper. An updated fork of the serial-ros2 repo is included as a submodule to this repo.

Clone this repo to your ROS2 workspace and build the ROS2 packages:

```bash
colcon build
```

Don't forget to source your workspace after the built:

```bash
source install/setup.bash
```

## Detailed Package Descriptions

### robotiq_2f_gripper_description

Adopted from [ros-industrial](https://github.com/ros-industrial/robotiq) for ROS2. The original license remains active.

To launch RViz2 and to inspect the gripper run:

```bash
ros2 launch robotiq_2f_gripper_description visualize.launch
```

### robotiq_2f_gripper_hardware

#### Action Server and Sending Move Requests
 
To launch the ROS2 action server (this will initialize/move the gripper) run:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py
```

To send a command to the gripper run (remeber to source any new terminal you open with `source install/setup.bash`):

```bash
ros2 action send_goal /robotiq_2f_gripper_action robotiq_2f_gripper_msgs/action/MoveTwoFingerGripper "{target_position: 0.05, target_speed: 0.1, target_force: 0.1}"
```

You can specify a target postion in meters, i.e. distance between the gripper fingers, and a target speed and force in percent (between 0 to 1).

#### Joint State and Gripper State Publishers

Listen to the current joint states:

```bash
ros2 topic echo /robotiq_2f_gripper/joint_states
```

Notice that the published joint states contain one joint (called ```finger_joint```) in the range of 0 to 0.7 radians. At 0.7 rad the gripper is fully closed.

Listen to the current gripper state. The gripper state here is defined as a boolean that is ```true``` when the gripper is holding an object and ```false``` when it is not holding an object.

```bash
ros2 topic echo /robotiq_2f_gripper/object_grasped
```

#### Controlling the Gripper via Topic Subscribers

In addition to the action server, the gripper can also be controlled by publishing to the following topics:

##### Confidence-based Control

You can control the gripper by publishing confidence values between -1.0 and 1.0 to the confidence command topic:

```bash
ros2 topic pub /robotiq_2f_gripper/confidence_command std_msgs/msg/Float32MultiArray "data: [0.8]"  # Open with high confidence
ros2 topic pub /robotiq_2f_gripper/confidence_command std_msgs/msg/Float32MultiArray "data: [-0.8]"  # Close with high confidence
ros2 topic pub /robotiq_2f_gripper/confidence_command std_msgs/msg/Float32MultiArray "data: [0.0]"  # Neutral confidence
```

This command mode uses hysteresis to avoid oscillation:

- Values > 0.2: Opens the gripper fully
- Values < -0.2: Closes the gripper fully
- Values between -0.2 and 0.2: Maintains current state (hysteresis band)

##### Binary Control

For direct binary control without hysteresis, publish exactly 1.0 (open) or -1.0 (close) to the binary command topic:

```bash
ros2 topic pub /robotiq_2f_gripper/binary_command std_msgs/msg/Float32MultiArray "data: [1.0]"  # Open
ros2 topic pub /robotiq_2f_gripper/binary_command std_msgs/msg/Float32MultiArray "data: [-1.0]"  # Close
```

#### Launch Arguments when Starting the Hardware

*fake_hardware* (default: false)

If you don't want to move the hardware, or don't have the hardware available, but still want to test the ROS2 action server you can launch the node with `fake_hardware:=true`:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py fake_hardware:=true
```

Alongside the action server this also publishes mocked joint states and gripper states.

*serial_port* (default: /dev/ttyUSB0)

It's assumed that you work on a Linux machine. If you have multiple USB devices connected to your computer find out to which serial port your Robotiq gripper is connected and set the ```serial_port:=<your_port>``` launch argument:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py serial_port:=/dev/ttyUSB1
```

*rviz2* (default: false)

If you would like to look at a visualization of the Robotiq gripper (for example in combination with fake_hardware mode) that als listens to the joint states publised under the joint_states ROS2 topic, set ```rviz2:=true```:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py rviz2:=true
```

### robotiq_2f_gripper_interfaces

Adopted from [PickNikRobotics](https://github.com/PickNikRobotics/ros2_robotiq_gripper). The original license remains active.

This package should not be run or launched. Instead it provides the driver code for the gripper.

### robotiq_2f_gripper_msgs

This package should not be run or launched. Instead it provides a message template for the communication with the gripper.

## Troubleshooting

1. When launching the gripper:

```bash
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py serial_port:=/dev/ttyUSB0

```

and you receive:

```bash
[robotiq_2f_gripper_node-1] terminate called after throwing an instance of 'serial::IOException'
[robotiq_2f_gripper_node-1]   what():  IO Exception (2): No such file or directory, file /home/jannis.haberhausen/GitHub/robotiq_2f_gripper_ros2/src/serial-ros2/src/impl/unix.cc, line 152.
```

Fix: connect the USB of the robotiq gripper.

2. When sending an action to the gripper action server:

```bash
ros2 action send_goal /robotiq_2f_gripper_action robotiq_2f_gripper_msgs/action/MoveTwoFingerGripper "{target_position: 0.05, target_speed: 0.1, target_force: 0.1}"
```

and you receive:

```bash
The passed action type is invalid
```

Fix: source your workspace in the new terminal

```bash
source install/setup.bash
```
