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

## Optional local acquisition-record copy

With `acquisition_records:=true`, local metadata now defaults to **bag-linked**
mode (`acquisition_record_mode:=bag`). Camera publication and the existing ROS
acquisition_timing messages continue while idle; local files are created only
when a compatible collector requests recording. No JPEG pixels are duplicated.
Image headers, ROS timing payloads, camera settings and motion are unchanged.

### Normal collection

Build both packages, source the workspace, and restart the GelSight publishers
and data_capture_manager_10. Other drivers do not need restarting for this change.

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_gelsight_package data_capture_tools
source install/setup.bash
ros2 launch ros2_gelsight_package gelsight_publisher.launch.py acquisition_records:=true
```

Both cameras must log `Local acquisition copy: bag-linked standby; no files until
a collector starts recording`. Never run duplicate publishers on the devices.
The manager defaults to `gelsight_local_records: true`, including custom YAML
files where that key is absent. Set it to false to disable control requests.
Normal ac/ar/ap/af recording through manager_10 needs no extra save commands.

For each recording session, the manager creates:

- `gelsight_acquisition_records/<node>_<stream_id>.jsonl` and `.status.json`;
- `gelsight_local_recording.json`, recording the requests and node acknowledgments.

These paths are inside the existing session directory alongside bag/logs, not
inside every trial export. A new bag session gets a new directory. The writer
starts before rosbag launches and stops after the recorder exits, so boundary
records outside the bag interval are expected. Recorder startup/shutdown can
wait up to 2.5 seconds each for acknowledgments. Missing local acknowledgments
are warnings; they do not stop or retry the primary collection or motion.

The collector renews a 10-second lease every second while its recorder process
is alive. A normal stop closes the local writers. If the collector crashes or
communication ceases, capture stops adding local records at lease expiry even
if the ROS control timer is delayed. The camera keeps publishing. Both collector
and GelSight nodes must run on the same workstation. No files are auto-deleted;
previously captured files remain, but idle nodes do not keep appending in bag mode.

A bare `ros2 bag record` does **not** send these control requests. Use the passive
helper below for manual checks. The old `acquisition_record_dir` option alone no
longer enables continuous saving: that requires explicit
`acquisition_record_mode:=continuous` plus an absolute directory and
`acquisition_records:=true`. Continuous mode writes throughout node lifetime;
`acquisition_record_mode:=off` disables only the local copy.

### Files, overhead and timing meanings

Each JSONL starts with startup metadata. `kind:frame` holds the exact JSON sent
on acquisition_timing plus clocks immediately before local queue submission.
`kind:publish_result` records host clocks around ROS message construction/publish
and success/failure. These are not exposure times or DDS delivery acknowledgments.
`writer_monotonic_ns` is the worker's formatting time. Clock tuples use
`[monotonic_before_ns, ros_ns, monotonic_after_ns]`. Match records by stream_id,
header_ns and frame_seq; local data does not repair or replace bag messages.

Capture only submits immutable text with put_nowait to a 256-envelope queue.
File opening/draining occurs on the control callback, and a separate writer
batches up to 32 envelopes, flushes/status-updates about once per second, and
fsyncs on close. Queue overflow drops local records with counters; disk failure
disables local acceptance. Neither intentionally stops image acquisition, though
CPU/GIL/disk use still adds overhead and must be checked on the workstation.
Control close waits at most two seconds; a stuck writer blocks new local writers
until drained. A forced kill/power loss can lose buffered data.

After normal bag stop, check error:null, closed:true, clean_close:true,
close_timed_out:false, pending_records:0, dropped_records:0, rejected_records:0,
and matching accepted/written/flushed counts. For bag-linked stop,
capture_source_detached:true is expected; capture_thread_stopped:false is normal
because the camera remains active. Clean close alone does not prove no drops.

### Passive validation without robot motion

Keep the GelSight and MMS101 publishers running with acquisition_records:=true.
UR/gripper/action nodes and init/ac are not required. In a separate sourced
workstation terminal, run:

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/record_gelsight_local_check.py \
  --seconds 40 --out ./gs_local_link_check_20261105_run1
```

The helper waits for both local writers, records the eight GelSight/MMS101 image,
force and timing topics, stops both recording paths, and compares them. It sends
no robot commands. If startup acknowledgments are missing it fails before bag
recording and reports why. All files are under the specified output directory;
share its `comparison.json`. Do not manually mkdir a missing local directory,
and do not stop GelSight to finalize this test. Choose a new output name for each
run. Keep JSONL/status files if more detailed investigation is needed.
