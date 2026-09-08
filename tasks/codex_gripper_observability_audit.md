# Codex task: Robotiq gripper-width data provenance and observability audit

## Objective

Do **not** modify or implement the staged-probing code yet.

The immediate goal is to establish, from source code and read-only runtime inspection, exactly what gripper-related data can currently be observed and recorded.

Questions to answer:

1. Where does `robotiq_2f_gripper_finger_distance_mm.csv` come from?
2. Which ROS 2 publisher produces `/robotiq_2f_gripper/finger_distance_mm`?
3. What physical/controller quantity does its `Float32.data` represent?
4. How is that quantity calculated from the Robotiq driver / registers / joint state?
5. Is it commanded position, controller-estimated position, measured actuator position, estimated fingertip distance, or something else?
6. What is its unit, range, resolution, update rate, latency, and timestamp semantics?
7. Is it suitable as a time-varying jaw-width measurement for the tactile experiments?
8. What other gripper state/feedback signals are available and potentially preferable?
9. What must be calibrated experimentally before this signal can be called physical jaw opening or used as a displacement proxy?

Do not call any signal “object indentation” unless an independent calibration supports that interpretation.

---

## Canonical experiment files

Current collection entry point:

`/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/data_capture_tools/data_capture_tools/data_capture_node_10times.py`

Current gripper command client:

`/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/gripper_client/src/gripper_client_node.cpp`

Potentially relevant post-processing file mentioned by the experimenter:

`/home/tsumura/my_robotiq_ws/build/data_capture_tools/build/lib/data_capture_tools/data_processer.py`

Sensor packages are not the main target of this task:

- `/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/mms101_driver`
- `/home/tsumura/my_robotiq_ws/src/ur3_robotiq_ros2/ros2_gelsight_package`

The current operational workflow is:

`gripper_client_node -> data_capture_node_10times.py -> i -> ap -> repetition count`

The unused single-trial `s` path and `as` path are not relevant unless needed to understand shared infrastructure.

The Python collection file contains commented-out historical code. Treat only the currently executable `i -> ap` path as canonical.

---

## Known facts from the current code

Verify these rather than assuming more than they establish.

### Python collection node

`data_capture_node_10times.py` already subscribes to:

`/robotiq_2f_gripper/finger_distance_mm`

as `std_msgs/msg/Float32`.

Its callback converts `msg.data / 1000.0` and stores the latest value in `_current_gripper_width_m`.

This proves that the current code *assumes* the topic is in mm. It does **not** prove where that value originates or whether it is a calibrated physical jaw gap.

### Gripper command client

`gripper_client_node.cpp` sends `robotiq_2f_gripper_msgs/action/MoveTwoFingerGripper` goals containing at least:

- `target_position`
- `target_speed`
- `target_force`

This is command-side information. Do not assume the command target equals measured jaw width.

### `data_processer.py`

The current processor reads:

`csv/robotiq_2f_gripper_finger_distance_mm.csv`

and expects the data column:

`robotiq_2f_gripper_finger_distance_mm.data`

It uses changes in that CSV to estimate a stable time.

This processor appears to **consume** the CSV, not generate the underlying gripper measurement. Trace the CSV generation separately.

---

# Investigation tasks

## Task A — Trace CSV provenance end-to-end

Trace:

`Robotiq hardware/driver`
→ `ROS 2 publisher`
→ `/robotiq_2f_gripper/finger_distance_mm`
→ `rosbag`
→ bag converter
→ `robotiq_2f_gripper_finger_distance_mm.csv`
→ `data_processer.py`

Identify the exact file/function at each step.

Inspect at minimum:

- `data_capture_tools/config.py`
- the actual `data_capture.yaml` loaded by the collection node
- `data_capture_tools/bag_converter.py`
- any source file that publishes `finger_distance_mm`
- package launch files that start that publisher
- relevant Robotiq driver/message packages

Use `rg`/`grep` to search the repository for:

```text
finger_distance_mm
robotiq_2f_gripper
MoveTwoFingerGripper
target_position
current_position
actual_position
gPO
gPR
position
joint_state
```

Also determine whether the file under:

`build/data_capture_tools/build/lib/data_capture_tools/data_processer.py`

is a generated build copy, symlink, or the canonical source being executed. Find its source counterpart and compare them. Do not edit build artifacts.

---

## Task B — Identify the publisher and its computation

Find the exact node and source code that publishes:

`/robotiq_2f_gripper/finger_distance_mm`

Answer with code references:

1. Publisher node/package.
2. Input value used to construct `Float32.data`.
3. Formula or lookup used to convert the driver state to millimeters.
4. Whether that input ultimately comes from:
   - a hardware status register,
   - commanded setpoint,
   - action feedback,
   - joint state,
   - inferred encoder position,
   - or another source.
5. Whether the value is independently measured or model-derived.
6. Expected numerical range and resolution.
7. Whether finger geometry assumptions are embedded in the conversion.
8. Whether the value is total finger-to-finger distance, one-finger displacement, actuator position, or another convention.

Do not infer these from variable names alone. Trace the value to its source.

If the source eventually leaves this repository and enters an installed ROS package, inspect the installed package/source if locally available and report where the trace stops.

---

## Task C — Determine timestamp semantics

`std_msgs/msg/Float32` has no message header.

Determine:

1. What timestamp appears as `stamp_ns` in `robotiq_2f_gripper_finger_distance_mm.csv`.
2. Whether it is:
   - publisher acquisition time,
   - ROS publish time,
   - rosbag receive/write time,
   - converter timestamp,
   - or something else.
3. Whether this timestamp can be aligned meaningfully with:
   - MMS101 `WrenchStamped`,
   - GelSight image timestamps,
   - trial markers.
4. The expected timing uncertainty introduced by the publisher, executor, DDS, rosbag, throttling, or converter.

Explicitly distinguish message-internal timestamps from bag timestamps.

---

## Task D — Runtime observability audit

Only use read-only ROS commands. Do not command or move the robot/gripper.

With the gripper driver running, inspect:

```bash
ros2 topic list
ros2 topic list | grep -i robotiq
ros2 action list | grep -i robotiq
ros2 node list
```

For the width topic:

```bash
ros2 topic type /robotiq_2f_gripper/finger_distance_mm
ros2 topic info -v /robotiq_2f_gripper/finger_distance_mm
ros2 topic echo --once /robotiq_2f_gripper/finger_distance_mm
ros2 topic hz /robotiq_2f_gripper/finger_distance_mm
```

Run `topic hz` only long enough to estimate the rate.

After identifying the publisher node:

```bash
ros2 node info <publisher_node>
```

Inspect the gripper action and message definitions:

```bash
ros2 action info /robotiq_2f_gripper_action
ros2 interface show robotiq_2f_gripper_msgs/action/MoveTwoFingerGripper
```

Also inspect all available gripper-related topics/interfaces that may contain:

- actual/current position,
- requested position,
- object detection/status,
- motor current,
- force/current feedback,
- joint states,
- fault/status registers.

Do not assume all of these exist. Report only what is actually available.

---

## Task E — Inspect an existing recorded trial/bag

Use one existing dataset trial that contains:

`robotiq_2f_gripper_finger_distance_mm.csv`

and, if the corresponding bag still exists, inspect the bag with read-only tools.

Report:

- CSV columns;
- sample count;
- first/last timestamps;
- median/min/max sampling interval;
- effective sampling rate;
- min/max width;
- number of unique width values;
- smallest non-zero observed step;
- whether the trajectory is smooth, quantized, or piecewise constant;
- whether the reported width reaches the commanded target;
- delay between command/phase/marker events and observed width change, if reconstructable.

If multiple existing trials are readily available, compare a few without turning this into a large data analysis task.

This is a measurement-interface audit, not a machine-learning experiment.

---

## Task F — Inventory all alternative gripper feedback

Create a table of every relevant signal actually available in this system.

For each signal report:

| Signal | ROS interface | Source | Unit | Rate | Timestamp | Command or feedback? | Physical interpretation | Main limitations |
|---|---|---|---|---|---|---|---|---|

Candidates may include, only if actually present:

- `finger_distance_mm`
- action feedback/result
- `/joint_states` entries for the gripper
- Robotiq status registers
- requested/actual position
- motor current
- object-detection flags
- force/current estimate

For each candidate, evaluate whether it is useful for:

1. current jaw opening;
2. contact-relative jaw closure;
3. detecting contact;
4. force/displacement analysis;
5. estimating true object indentation.

Use one of:

- suitable;
- usable with calibration;
- weak proxy;
- unsuitable.

Give the reason.

---

## Task G — Calibration requirements

Based on the source trace, state what can and cannot be claimed from `finger_distance_mm`.

At minimum distinguish:

1. commanded target position;
2. controller-reported gripper state;
3. physical finger-to-finger gap;
4. contact-relative jaw closure;
5. true deformation/indentation of the object.

If `finger_distance_mm` is a driver-derived conversion rather than a directly calibrated physical measurement, propose a minimal offline calibration experiment, but do not implement it yet.

For example, determine whether comparison against a ruler/caliper/gauge at several commanded openings would establish:

- offset;
- scale;
- monotonicity;
- repeatability;
- hysteresis/backlash;
- effective resolution.

Do not propose a calibration method that cannot validate the quantity being claimed.

---

# Required final report

Do not change code.

Return a concise technical report with:

## 1. Provenance
Exact origin of `robotiq_2f_gripper_finger_distance_mm.csv`.

## 2. Meaning
Exactly what its numeric value represents and what it does not represent.

## 3. Timing
Source and quality of its timestamps and measured update rate.

## 4. Alternative data
Other gripper feedback available in this ROS setup.

## 5. Suitability
Whether the current width signal is suitable for:
- logging jaw motion;
- constructing contact-relative closure;
- using as a displacement-conditioned tactile feature.

## 6. Calibration
What calibration is necessary before interpreting it physically.

## 7. Recommendation
Choose one:
- use the current topic as-is;
- use it after calibration;
- replace/supplement it with another available signal.

Support the recommendation with the source-code/runtime evidence found above.

## 8. Evidence
List:
- exact files/functions inspected;
- exact ROS commands run;
- one representative existing CSV/bag inspected;
- unresolved questions.

Do not implement staged probing until this audit has been reviewed.
