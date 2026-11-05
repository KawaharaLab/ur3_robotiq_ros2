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

`acquisition_record_dir` is empty by default. Set both options below to enable a
local copy on the workstation. This does not change camera properties, image
headers, image topics, ROS timing payloads, or robot motion. The output contains
metadata only, not a second copy of JPEG pixels.

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_gelsight_package
source install/setup.bash
ros2 launch ros2_gelsight_package gelsight_publisher.launch.py \
  acquisition_records:=true \
  acquisition_record_dir:=/home/tsumura/my_robotiq_ws/gelsight_acquisition_records
```

Restart only the GelSight launch process to load the new code. Never run two
camera publishers against the same devices. The log must print `Local acquisition
copy: ...` for each camera. A directory without `acquisition_records:=true` is a
configuration error. An unwritable path fails at startup instead of pretending
that local recording is available. Existing setup without the directory option
has no writer thread or files.

### Files and timing meanings

Each node/startup stream creates `<node>_<stream_id>.jsonl` plus
`<node>_<stream_id>.status.json` in that directory, without overwriting an old log.
The JSONL begins with `kind:start` metadata. Each frame then has:

- `kind:frame`: the exact JSON object also sent on acquisition_timing, plus an
  `enqueue_clock` sampled immediately before its local queue submission, before
  the ROS publish call. The serialized ROS payload itself is reused unchanged.
- `kind:publish_result`: that frame's header/sequence and host clocks bracketing
  construction/publish of the ROS String message, with success/failure. These
  clocks are not physical exposure timestamps or DDS delivery acknowledgements.
- `writer_monotonic_ns`: when the worker formats the local line, not a sensor
  timestamp. Clock samples use `[monotonic_before_ns, ros_ns, monotonic_after_ns]`.

The capture thread submits immutable text to a bounded queue of 256 envelopes
with put_nowait. It never waits for queue space or performs disk writes. The
worker writes batches of up to 32 envelopes, buffers file output, flushes about
once per second, and updates the small status JSON atomically. It fsyncs the data
on normal shutdown, not for every frame. A queue overflow drops the local record
and increments counters; a disk failure disables local acceptance and is logged.
Neither condition intentionally stops image acquisition. Python scheduling/GIL,
shared CPU/disk use, and clock calls still add some overhead; zero effect is not
guaranteed. A forced kill/power loss can lose buffered data and leave stale status.

Check `error:null`, `dropped_records:0`, `rejected_records:0`, and after normal
shutdown `closed:true`, `clean_close:true`, `capture_thread_stopped:true`,
`close_timed_out:false`, `pending_records:0`, with accepted/written/flushed counts
matching. Counts include both frame and publish_result envelopes. During recording,
flushed counts may lag by up to one interval; startup metadata is not counted.
Closing the writer waits at most five seconds; a blocked disk can still prevent
all final records from being saved and is reported as an incomplete close.

Local logging covers a whole node lifetime, not one bag. Match via stream_id,
header_ns and frame_seq; preserve the matching JSONL/status files together with
research bags. Nothing automatically copies these files into per-trial folders
or repairs/replaces bag messages. No files are silently cleaned up.

### Passive validation without robot motion

With the GelSight nodes and the normal MMS101 publishers running, record a static
40-second bag from another sourced workstation terminal. UR/gripper/action nodes
and init/ac are not required for this test.

```bash
cd /home/tsumura/my_robotiq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout --signal=INT 40s ros2 bag record \
  -s sqlite3 --compression-mode file --compression-format zstd \
  -o ./gs_local_check_20261105_run1 \
  /gelsight/left/image_raw/compressed /gelsight/right/image_raw/compressed \
  /gelsight_mini_publisher_1/acquisition_timing \
  /gelsight_mini_publisher_2/acquisition_timing \
  /force_torque/left /force_torque/right \
  /mms101_driver_1/acquisition_timing /mms101_driver_2/acquisition_timing \
  </dev/null > ./gs_local_check_20261105_run1.log 2>&1
```

Exit code 124 from timeout is expected. Wait for compression/normal exit. For this
validation, stop the GelSight launch with Ctrl+C afterwards to flush and finalize
its local status. Do not rebuild or change motion conditions for this test.
Then compare existing records without ROS subscriptions or robot commands:

```bash
/usr/bin/python3 src/ur3_robotiq_ros2/utils/diagnostics/compare_gelsight_local_bag.py \
  --bag ./gs_local_check_20261105_run1 \
  --local-dir ./gelsight_acquisition_records \
  --out ./gs_local_check_20261105_comparison.json
```

The script supports SQLite db3 and file-compressed db3.zstd via the existing
read-only QA reader. It uses Python YAML and zstd installed in the normal
workstation environment; no ROS Python imports. It refuses to overwrite output.
Share the comparison JSON first. It includes the matching filenames and status,
image FPS, local queue-before-publish/header timing, bag delivery timing, and ROS
publish-call duration. Missing bag metadata with local metadata present supports
a post-generation delivery/recording problem; it does not identify which DDS or
rosbag component failed. If both are missing, inspect local drop/error counters
before blaming generation. No physical synchronization guarantee is implied.
