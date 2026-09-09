# ROS 2 time synchronization audit

Audit date: 2026-09-08 (Asia/Tokyo)

Workspace: `/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2`

Representative existing bag:
`/home/tsumura/data/10times_test/20260505_123511/bag/session_20260505_123511`
(62.610 s, 33,677 messages, sqlite3 with file-level zstd compression).

No existing bag or message timestamp was changed. The QA reader decompresses a
file-compressed bag into a temporary directory and opens sqlite in read-only mode.

## 1. Topic dependency map

The rates below are measured from the representative bag unless marked "new".
For image entries, the capture manager rewrites configured raw image names to
their `/compressed` transport before starting `ros2 bag record`.

| Recorded or source topic | Type | Publisher / package | Source | Header | Timestamp generation | Rate |
|---|---|---|---|:---:|---|---:|
| `/force_torque/left` | `geometry_msgs/msg/WrenchStamped` | `/mms101_driver_1`, `mms101_driver` | `mms101_driver/src/mms101_node.cpp`, `process_packet()` | yes | `Node::now()` after a complete 25-byte serial packet is framed, immediately before parsing/publish; not a device timestamp | 100.006 Hz |
| `/force_torque/right` | `geometry_msgs/msg/WrenchStamped` | `/mms101_driver_2`, `mms101_driver` | same source, second launch instance | yes | same as left | 100.006 Hz |
| `/gelsight/left/image_raw` | `sensor_msgs/msg/Image` | `/gelsight_mini_publisher_1`, `ros2_gelsight_package` | `ros2_gelsight_package/ros2_gelsight_publisher.py`, `_capture_loop()` | yes | `self.get_clock().now()` after OpenCV `read()` returns and after `cv2_to_imgmsg`; this is host-side post-read time, not exposure time | source, nominal 15 Hz |
| `/gelsight/right/image_raw` | `sensor_msgs/msg/Image` | `/gelsight_mini_publisher_2`, `ros2_gelsight_package` | same source, second launch instance | yes | same as left | source, nominal 15 Hz |
| `/gelsight/left/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | same GelSight node | same source | yes | `cmsg.header = msg.header`; raw header is preserved across JPEG encoding | 14.993 Hz median while present |
| `/gelsight/right/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | same GelSight node | same source | yes | raw header preserved | 15.004 Hz |
| `/camera_wrist/realsense2_camera/color/image_raw` | `sensor_msgs/msg/Image` | `/camera_wrist/realsense2_camera`, `realsense2_camera` 4.57.7 | upstream `base_realsense_node.cpp` | yes | RealSense frame timestamp converted by `frameSystemTimeSec()`; live metadata says `Global Time`, and `rgb_camera.global_time_enabled=true` | source, configured 15 Hz |
| `/camera_fixed/realsense2_camera/color/image_raw` | `sensor_msgs/msg/Image` | `/camera_fixed/realsense2_camera`, `realsense2_camera` 4.57.7 | same upstream source, second device | yes | same mapping, independently running camera | source, configured 15 Hz |
| both RealSense `/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | image transport plugin in each RealSense node | `compressed_image_transport` 2.5.5, `compressed_publisher.cpp` | yes | `compressed.header = message.header`; compression does not restamp | 14.882 Hz |
| both RealSense `/color/metadata` | `realsense2_camera_msgs/msg/Metadata` | each RealSense node | upstream wrapper | yes | same frame time; newly added to recording so timestamp-domain evidence is retained | new, frame rate |
| `/joint_states` | `sensor_msgs/msg/JointState` | `/joint_state_broadcaster`, `joint_state_broadcaster` 2.53.1 | upstream `joint_state_broadcaster.cpp` | yes | controller update `time` argument; controller manager 2.54.0 invokes `update(cm->now(), measured_period)` | 125.012 Hz |
| `/robotiq_2f_gripper/finger_distance_mm` | `std_msgs/msg/Float32` | `/robotiq_2f_gripper_node`, `robotiq_2f_gripper_hardware` | `robotiq_2f_gripper_node.cpp` | no | published after the status polling conversion; only rosbag receive time is available | 20.806 Hz |
| `/robotiq_2f_gripper/status` | `robotiq_2f_gripper_msgs/msg/GripperStatus` | same Robotiq node | same source plus `GripperStatus.msg` | yes | `Node::now()` immediately after synchronous Modbus `read_status()` returns; raw `gOBJ/gFLT/gPR/gPO/gCU` from that packet | new, configured 20 Hz |
| `/robotiq_2f_gripper/joint_states` | `sensor_msgs/msg/JointState` | same Robotiq node | same source | yes | now uses the exact same acquisition stamp as `/status` | new, configured 20 Hz |
| `/robot_action_event` | `robotiq_2f_gripper_msgs/msg/RobotActionEvent` | `/pick_place_client`, `gripper_client` | `gripper_client/src/gripper_client_node.cpp` | yes | this node's ROS clock at each command/result/hold boundary | new, event driven |
| `/trial_event` | `robotiq_2f_gripper_msgs/msg/TrialEvent` | `/data_capture_manager`, `data_capture_tools` | `data_capture_node_10times.py` | yes | this node's ROS clock immediately before the corresponding legacy marker/command | new, two per trial |
| `/current_phase` | `std_msgs/msg/Int32` | `/pick_place_client` | `gripper_client_node.cpp` | no | legacy phase publication; bag receive time only | new recording, event driven |
| `/trial_marker` | `std_msgs/msg/String` | `/data_capture_manager` | `data_capture_node_10times.py` | no | legacy compatibility topic; bag receive time only | 20 messages / 10 trials |

The live ROS graph verified the two FT publisher names, the two RealSense node
names, `/joint_state_broadcaster` as the sole `/joint_states` publisher, and
`/pick_place_client` as the `/current_phase` publisher. The Robotiq node was not
running during the graph query, so its dependency was verified from its launch,
header, implementation, and package dependencies rather than inferred from a
live endpoint.

## 2. Clock domains

| Producer | Header/event clock | Other clocks in the implementation | Assessment |
|---|---|---|---|
| MMS101 FT | node ROS clock; live `use_sim_time=false`, therefore host system/Unix epoch | `steady_clock` only for initialization deadlines/diagnostics | common ROS epoch; stamp is packet-processing time |
| GelSight | node ROS clock; live `use_sim_time=false` | `time.time()` only gates publication/health; it is not written to messages | common ROS epoch; stamp is post-camera-read time |
| RealSense color | librealsense `Global Time` in the current live configuration | device hardware clock is converted by librealsense/global-time; wrapper has a ROS-base mapping fallback for `HARDWARE_CLOCK` domain | Unix-like ROS epoch, but two devices are not hardware-trigger synchronized |
| UR `/joint_states` | controller-manager ROS clock passed into broadcaster update | controller-manager uses steady time for measured loop period, not the message epoch | common ROS epoch; not a UR controller hardware timestamp |
| Robotiq status/state | node ROS clock after Modbus response | `steady_clock` measures read duration and action timeout | common ROS epoch; 20 Hz packet completion granularity |
| Action/trial events | publishing node ROS clock | wall/steady timers implement the 1/4/1 s waits | common ROS epoch |
| rosbag2 0.15.16 | recorder node ROS clock in generic subscription callback | storage/cache/compression time is not used as the database timestamp | receive/take time, separate from message header |

`data_capture_node_10times.py` does not pass `--use-sim-time` to rosbag. Live
checks found `use_sim_time=false` on FT, GelSight, RealSense, and
`pick_place_client`; the bag stamps are Unix-epoch values. Thus there is no
steady-time or raw-device-uptime value mixed directly into the audited header
columns.

The RealSense wrapper has two paths. With `HARDWARE_CLOCK`, its first frame
anchors camera milliseconds to `_node.now()` and subsequent stamps add camera
elapsed time. With the observed `Global Time` domain, the SDK-provided timestamp
is converted directly to nanoseconds. Recording `/color/metadata` in future bags
is important because it records `clock_domain` rather than requiring a live
configuration assumption.

## 3. Can headers be compared directly?

For FT, GelSight, UR state, Robotiq stamped status, action events, and trial
events on this host, yes at the ROS/system-clock epoch level. This does **not**
mean simultaneous physical acquisition:

- FT is stamped after serial packet reception/framing.
- GelSight is stamped after a blocking OpenCV camera read and conversion, not at exposure.
- UR state is stamped at the ros2_control update.
- Robotiq is stamped after a synchronous Modbus response.
- RealSense is capture/global-time based and then passes through USB and JPEG compression.

Consequently, header comparison is suitable for a common analysis axis, while
the measured pipeline offsets and sensor sampling periods are the accuracy
bounds. The two RealSense devices and the two GelSight cameras are free-running;
nearest-frame matching must not be described as hardware synchronization.

Headerless `/finger_distance_mm`, `/trial_marker`, and `/current_phase` can only
use bag receive time. New analyses should prefer `/status`, `/trial_event`, and
`/robot_action_event` respectively.

## 4. Existing bag: bag timestamp minus header

| Topic | Messages | Median Hz | Median offset | IQR | p95 absolute | Max absolute | Robust drift |
|---|---:|---:|---:|---:|---:|---:|---:|
| RealSense fixed compressed | 929 | 14.882 | 69.984 ms | 3.529 ms | 70.529 ms | 1419.286 ms | -3.790 ms/min |
| RealSense wrist compressed | 926 | 14.882 | 69.670 ms | 3.377 ms | 70.285 ms | 1395.354 ms | +1.311 ms/min |
| FT left | 6263 | 100.006 | 0.075 ms | 0.049 ms | 0.322 ms | 15.431 ms | -0.005 ms/min |
| FT right | 6175 | 100.006 | 0.075 ms | 0.058 ms | 0.322 ms | 4.095 ms | -0.003 ms/min |
| GelSight left compressed | 425 | 14.993 | 5.003 ms | 0.122 ms | 5.163 ms | 14.120 ms | +0.092 ms/min |
| GelSight right compressed | 940 | 15.004 | 5.301 ms | 0.147 ms | 5.875 ms | 6.750 ms | +0.028 ms/min |
| UR `/joint_states` | 7828 | 125.012 | 0.186 ms | 0.107 ms | 0.543 ms | 14.225 ms | -0.010 ms/min |
| Robotiq float width | 1223 | 20.806 | unavailable | unavailable | unavailable | unavailable | unavailable |

The RealSense median includes capture-to-recorder transport and JPEG work
because compression preserves the earlier capture header. The approximately
1.4 s maximum is a startup outlier; it is not representative of the p95.

No backwards or duplicate timestamps occurred in the sensor topics above.
`/tf_static` contains old/static transform stamps and `/scaled_joint_trajectory_controller/controller_state`
contains one stale transient-local sample (80.9 s old); these are lifecycle/
latched semantics, not evidence of an 80 s sensor clock error.

Coverage defects in this old bag are material:

- FT right begins 0.869 s after bag start.
- GelSight left ends 34.330 s before bag end (425 frames versus 940 right).
- This historical left-camera dropout belongs to the pre-fix bag; no FPS change
  was made in this audit.

## 5. Same-sensor and cross-sensor effective timing

Nearest-neighbor header offsets over overlapping coverage:

| Pair (A minus nearest B) | Median | IQR | p95 absolute | Max absolute |
|---|---:|---:|---:|---:|
| FT left - right | -1.815 ms | 0.023 ms | 1.865 ms | 4.032 ms |
| GelSight left - right | +13.272 ms | 8.968 ms | 22.823 ms | 31.684 ms |
| RealSense wrist - fixed | +23.867 ms | 0.076 ms | 24.051 ms | 43.396 ms |

These are free-running sample-phase differences, not calibrated clock offsets.
The very stable FT value shows consistent serial/thread scheduling, but it does
not prove simultaneous ADC sampling. GelSight and RealSense differences are
below one frame but are not hardware synchronization.

The QA image-activity/force cross-correlations were weak:

- FT left to GelSight left: correlation 0.193; lag not identifiable.
- FT right to GelSight right: correlation 0.103; lag not identifiable.
- FT left to FT right force-shape correlation: 0.113; lag not identifiable.

The reported numerical argmax lags are deliberately marked unreliable and are
not applied. FT force response and GelSight deformation contain real mechanical
lag, different spatial contact, and different signal definitions. The old bag
also lacks an action-defined loading window and loses the left stream halfway,
so no defensible FT-GelSight timestamp offset can be isolated from it.

## 6. Action and phase semantics after the fix

Normal `run` behavior remains:

1. phase 1 open action, then 1 s delay;
2. phase 2 close action, then 4 s hold;
3. phase 3 open action, then 1 s delay.

`/robot_action_event` now records:

1. `trial_run_received`
2. `phase1_goal_sent`
3. `phase1_action_succeeded`
4. `phase2_close_goal_sent`
5. `phase2_close_action_succeeded`
6. `hold_start`
7. `hold_end`
8. `phase3_open_goal_sent`
9. `phase3_open_action_succeeded`
10. `sequence_finished`

The pre-audit Robotiq action server returned success immediately after writing
the command registers. That made the previous "action result" a command-write
acknowledgement, not physical completion. The server now polls status at the
configured 20 Hz and succeeds only when `gPR` matches this goal and either
`gOBJ` reports an object-detected stop, or `gOBJ` reports requested-position
stop while `gPO` is within one raw count of the target. It publishes each polled
raw status with the packet-reception stamp. Therefore
`phase2_close_action_succeeded` is now an action-defined physical motor-stop
boundary and `hold_start` follows it immediately. It is not proof of calibrated
object indentation.

`hold_end` is emitted by the existing 4 s timer immediately before the phase 3
open goal; `phase3_open_goal_sent` is the release command start. Existing
`/current_phase` remains for compatibility and is now also recorded.

## 7. Gripper and trial observability

The old float width topic remains unchanged. Its value is a polynomial conversion
of raw `gPO`; it is an estimated finger aperture and must not be treated as true
sample indentation.

The already-defined stamped `/robotiq_2f_gripper/status` is now included in
capture and continues during action execution. It contains the raw position,
requested position, object detection, fault, current, and Modbus read duration.
The gripper-specific `/joint_states` uses the same acquisition stamp. No second
competing custom width message was added because this raw stamped status is the
more faithful source.

`/trial_event` preserves START/END, trial index, mode, target position, and exact
target speed. `/trial_marker` remains unchanged. Normal commands now include the
trial index as an optional third field; old publishers that send only position
and speed remain compatible.

Random loading speed remains the default. Configuration now supports:

```yaml
loading_speed_mode: random  # or fixed
fixed_loading_speed: 0.100
random_loading_speed_min: 0.050
random_loading_speed_max: 0.200
```

The selected value is used identically in the stamped trial event, action event,
legacy marker, and action goal.

## 8. Synchronization QA

Run after sourcing the workspace:

```bash
ros2 run data_capture_tools synchronization_qa BAG_PATH \
  --decode-images \
  --json synchronization_qa.json \
  --markdown synchronization_qa.md \
  --timeline synchronization_timeline.png
```

It reports message count, median rate, header availability, median/IQR/p95/max
bag-header offset, robust endpoint drift, backwards/duplicate stamps, large gaps,
topic start/end coverage, required action events, action-defined hold duration,
loading-window sample coverage, same-type nearest-stamp behavior, and explicitly
uncorrected observed correlations. The timeline overlays gripper aperture,
events/current phase, FT, GelSight frame times, and UR joint state.

Representative outputs are:

- `docs/synchronization_qa_20260505.md`
- `docs/synchronization_qa_20260505.json`
- `docs/synchronization_timeline_20260505.png`

## 9. Impact on old and new data

Old bags remain usable on original message headers for FT, GelSight, RealSense,
and UR state. `/finger_distance_mm` and `/trial_marker` are restricted to bag
receive time. The old phase-2 close/hold boundary cannot be reconstructed exactly:
cross-correlation or threshold estimates may be reported as estimates, but must
not be silently substituted as action timestamps.

New bags contain exact software/action boundaries and packet-stamped gripper
status. Expected nominal resolution is approximately 10 ms for FT, 8 ms for UR
joint state, 50 ms plus serial-read/result-delivery latency for Robotiq physical
completion, and 66.7 ms per 15 Hz GelSight/RealSense frame. Event timestamps have
nanosecond representation, but their physical accuracy is limited by those
sampling and pipeline latencies. For tighter-than-frame tactile alignment,
camera hardware capture timestamps or an external shared trigger would still be
required.
