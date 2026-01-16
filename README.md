# ur3_robotiq_ros2
## 実行コマンド
### URのドライバノード
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.1.102 launch_rviz:=false
```

### グリッパのドライバノード
```
ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_launch.py
```
### カメラ
```
source /opt/ros/humble/setup.bash && source /home/user/ur3_robotiq_ros2/install/setup.bash && ros2 launch realsense_dual_launch realsense_dual.launch.py
```
一つずつやるときはこんな感じ
```
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=camera_wrist -p rgb_camera.color_profile:=640x360x15 -p enable_depth:=false
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=camera_fixed -p rgb_camera.color_profile:=640x360x15 -p enable_depth:=false
```
ストリーミング
```
ros2 run image_tools showimage --ros-args -r image:=/camera_fixed/realsense2_camera/color/image_raw
```
### GelSight起動
```
ros2 launch ros2_gelsight_package gelsight_publisher.launch.py
```
### 力覚センサ
```
ros2 launch mms101_driver mms101_dual.launch.py
```
単体でやるなら、
```
ros2 run mms101_driver mms101_node –ros-args --remap port:=/dev/ttyUSB1 # left
ros2 run mms101_driver mms101_node –ros-args --remap port:=/dev/ttyUSB2 # right
```
### テレオペのコマンドを出すノード
```
ros2 launch teleop_servo servo_ur.launch.py
```
### テレオペのデータ記録
```
ros2 run data_capture_tools data_capture_manager --ros-args -p config:=/home/user/ur3_robotiq_ros2/data_capture_tools/config/data_capture.yaml
```
### VLAノード
```
ros2 run vla_adapter pi0_inference_node
```
–ros-args -p debug:=trueでdata/examplesの入力をもとに一回推論してpublishする
### foxglove
```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
### controller変更
```
ros2 control switch_controllers --activate forward_position_controller --deactivate scaled_joint_trajectory_controller
```