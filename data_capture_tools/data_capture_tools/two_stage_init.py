"""Opt-in two-axis init. No ROS imports until runtime; geometry is unit-testable."""

import math
import select
import sys
import threading
import time

import numpy as np


def settings_from(raw):
    s = dict(raw.get("initialization") or {})
    mode = s.get("mode", "legacy")
    if mode not in ("legacy", "two_stage"):
        raise ValueError("initialization.mode must be legacy or two_stage")
    s["mode"] = mode
    for key, default in (("test_only", False), ("require_gelsight", True)):
        if type(s.get(key, default)) is not bool:
            raise ValueError(key + " must be a YAML boolean")
        s.setdefault(key, default)
    if not s["require_gelsight"] and not (mode == "two_stage" and s["test_only"]):
        raise ValueError("GelSight can be omitted only in two_stage test_only mode")
    if mode == "legacy":
        if s["test_only"]:
            raise ValueError("test_only requires two_stage")
        return s
    s.setdefault("rotation_deg", 90.0)
    if s["rotation_deg"] not in (-90, 90):
        raise ValueError("rotation_deg must be +90 or -90 (base Z axis)")
    for key in ("clamp_center_width_m", "stage1_probe_width_m", "stage2_probe_width_m"):
        if not isinstance(s.get(key), (float, int)) or not 0 < s[key] <= 0.135:
            raise ValueError(key + " must be explicitly set within (0, 0.135] m")
    if s["stage1_probe_width_m"] <= s["clamp_center_width_m"]:
        raise ValueError("stage1 probe must exceed clamp-axis sample width")
    for key in ("pivot_tcp_m", "retreat_base_m"):
        v = np.asarray(s.get(key), dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError(key + " must be an explicit finite 3-vector")
        s[key] = v.tolist()
    if np.linalg.norm(s["pivot_tcp_m"]) > 0.5:
        raise ValueError("pivot_tcp_m exceeds 0.5 m")
    if not 0.01 <= np.linalg.norm(s["retreat_base_m"]) <= 0.15:
        raise ValueError("retreat_base_m must have length 10 to 150 mm")
    s.setdefault("rotation_duration_s", 10.0)
    s.setdefault("max_init_force_n", 5.0)
    if not 5 <= s["rotation_duration_s"] <= 30:
        raise ValueError("rotation_duration_s must be 5 to 30 seconds")
    if not 0 < s["max_init_force_n"] <= 30:
        raise ValueError("max_init_force_n must be within (0, 30] N")
    s.setdefault("tcp_frame", "base")
    return s


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.01:
        raise ValueError("Invalid TCP quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_targets(position, rotation, pivot_tcp, retreat, degrees):
    """Return retreat/rotate/return endpoints preserving the configured physical pivot."""
    a = math.radians(degrees)
    yaw = np.array(
        [[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]]
    )
    p, r = np.asarray(position), np.asarray(rotation)
    pivot, away = np.asarray(pivot_tcp), np.asarray(retreat)
    new_r = yaw @ r
    new_p = p + r @ pivot - new_r @ pivot
    if np.linalg.norm(new_p - p) > 0.15:
        raise ValueError("Pivot compensation exceeds 150 mm; check TCP geometry")
    return [(p + away, r), (new_p + away, new_r), (new_p, new_r)]


def rotvec(r):
    # Quaternion extraction by largest diagonal handles rotations close to pi.
    k = (
        np.array(
            [
                [
                    r[0, 0] - r[1, 1] - r[2, 2],
                    r[1, 0] + r[0, 1],
                    r[2, 0] + r[0, 2],
                    r[2, 1] - r[1, 2],
                ],
                [
                    r[1, 0] + r[0, 1],
                    r[1, 1] - r[0, 0] - r[2, 2],
                    r[2, 1] + r[1, 2],
                    r[0, 2] - r[2, 0],
                ],
                [
                    r[2, 0] + r[0, 2],
                    r[2, 1] + r[1, 2],
                    r[2, 2] - r[0, 0] - r[1, 1],
                    r[1, 0] - r[0, 1],
                ],
                [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], np.trace(r)],
            ]
        )
        / 3
    )
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    if q[3] < 0:
        q = -q
    n = np.linalg.norm(q[:3])
    return np.zeros(3) if n < 1e-12 else q[:3] / n * (2 * math.atan2(n, q[3]))


class _Publisher:
    def __init__(self, owner, original):
        self.owner, self.original = owner, original

    def publish(self, msg):
        self.owner._two_stage_check()
        self.original.publish(msg)


class TwoStageInitMixin:
    def _setup_two_stage_init(self):
        self._two_stage_settings = settings_from(self.config.raw)
        if self._two_stage_settings["mode"] == "two_stage":
            if (
                not 0
                < self.config.target_diameter
                < self._two_stage_settings["stage2_probe_width_m"]
            ):
                raise ValueError(
                    "target_diameter must be positive and smaller than stage2_probe_width_m"
                )
        self._two_stage_running = False
        self._two_stage_abort = None
        self._two_stage_tcp = None
        if self._two_stage_settings["mode"] == "two_stage":
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import qos_profile_sensor_data

            self._two_stage_tcp_sub = self.create_subscription(
                PoseStamped,
                "/tcp_pose_broadcaster/pose",
                self._two_stage_tcp_cb,
                qos_profile_sensor_data,
            )
            self.get_logger().warning(
                "TWO-STAGE INIT: rotation=%s deg; test_only=%s; GelSight required=%s"
                % (
                    self._two_stage_settings["rotation_deg"],
                    self._two_stage_settings["test_only"],
                    self._two_stage_settings["require_gelsight"],
                )
            )

    def _two_stage_tcp_cb(self, msg):
        self._two_stage_tcp = (time.monotonic(), msg)

    def _two_stage_pose(self):
        row = self._two_stage_tcp
        if row is None or time.monotonic() - row[0] > 0.25:
            raise RuntimeError("Fresh /tcp_pose_broadcaster/pose required (250 ms)")
        msg = row[1]
        if msg.header.frame_id.lstrip("/") != self._two_stage_settings[
            "tcp_frame"
        ].lstrip("/"):
            raise RuntimeError("TCP pose frame mismatch: " + msg.header.frame_id)
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age_ms = (self.get_clock().now().nanoseconds - stamp) / 1e6
        if not -10 <= age_ms <= 250:
            raise RuntimeError("TCP source timestamp outside accepted range")
        p, q = msg.pose.position, msg.pose.orientation
        p = np.array([p.x, p.y, p.z])
        if not np.isfinite(p).all():
            raise RuntimeError("Non-finite TCP position")
        return p, quaternion_matrix([q.x, q.y, q.z, q.w])

    def _two_stage_check(self):
        if self._two_stage_abort:
            raise RuntimeError(self._two_stage_abort)
        if self._stop_event.is_set():
            raise RuntimeError("Node shutdown requested")
        if getattr(self, "_auto_in_init", False) and self._auto_abort_reason:
            raise RuntimeError(self._auto_abort_reason)
        if time.monotonic() - self._two_stage_begin > 350:
            raise RuntimeError("Two-stage init time limit (350 s)")
        self._two_stage_pose()
        if self._auto_robot_mode != 7 or self._auto_safety_mode not in (1, 2):
            raise RuntimeError("UR must be RUNNING in NORMAL/REDUCED safety mode")
        # Records are generated by independent callbacks; no new sensor topics required.
        with self._init_record_lock:
            rows = {
                k: self._init_latest.get(k)
                for k in ("force_left", "force_right", "gripper_status")
            }
        now = time.monotonic_ns()
        for k, row in rows.items():
            if row is None or (now - row["receive_monotonic_ns"]) / 1e9 > 0.25:
                raise RuntimeError("Two-stage init stale " + k)
            if k.startswith("force_") and not all(
                math.isfinite(x) for x in row["wrench"]
            ):
                raise RuntimeError("Invalid force sample")
        if rows["gripper_status"]["status"]["g_flt"] != 0:
            raise RuntimeError("Gripper fault during two-stage init")
        if self._two_stage_force_guard:
            f = max(
                self._force_monitor.get_calibrated_left(),
                self._force_monitor.get_calibrated_right(),
            )
            if not math.isfinite(f) or f > self._two_stage_settings["max_init_force_n"]:
                raise RuntimeError("Two-stage init force limit")
        if (
            self._two_stage_settings["require_gelsight"]
            and not self._check_sensors_health()
        ):
            raise RuntimeError("Sensor health failed during two-stage init")

    def _two_stage_wait_pose(self, target, timeout, position_tolerance=0.0003):
        p, r = target
        begin = time.monotonic()
        stable = None
        while time.monotonic() - begin < timeout:
            self._two_stage_check()
            actual_p, actual_r = self._two_stage_pose()
            angle = math.acos(float(np.clip((np.trace(r.T @ actual_r) - 1) / 2, -1, 1)))
            near = np.linalg.norm(
                p - actual_p
            ) < position_tolerance and angle < math.radians(0.5)
            stable = (stable or time.monotonic()) if near else None
            if stable and time.monotonic() - stable >= 0.2:
                return
            time.sleep(0.02)
        raise RuntimeError("Two-stage arm endpoint not reached; no further motion")

    def _two_stage_wait_translation(self, before, dx, dy, dz):
        self._two_stage_wait_pose(
            (before[0] + [dx, dy, dz], before[1]),
            8.0,
            position_tolerance=min(0.00005, np.linalg.norm([dx, dy, dz]) * 0.2),
        )

    def _two_stage_move(self, target, label, duration=None):
        from std_msgs.msg import String

        self._two_stage_check()
        p, r = target
        values = ",".join("%.10f" % x for x in np.r_[p, rotvec(r)])
        timed = ", t=%.3f" % duration if duration is not None else ""
        self._init_observe(
            "two_stage_arm_target",
            stage=label,
            position_m=p.tolist(),
            rotation_matrix=r.tolist(),
        )
        self._urscript_pub.publish(
            String(
                data="def two_stage_move():\n  movel(p[%s], a=0.05, v=0.01%s)\nend\n"
                % (values, timed)
            )
        )
        self._two_stage_wait_pose(target, (duration or 20.0) + 8.0)
        self._init_observe(
            "two_stage_arm_reached",
            stage=label,
            position_m=self._two_stage_pose()[0].tolist(),
        )

    def _two_stage_gripper(self, width):
        from std_msgs.msg import String

        self._sequence_finished_event.clear()
        self._sequence_failed_event.clear()
        self._cmd_pub.publish(String(data="gripper %.5f 0.100" % width))
        if not self._wait_for_sequence(15.0, "Two-stage gripper move"):
            raise RuntimeError("Two-stage gripper move failed")
        self._two_stage_check()
        if abs(self._current_gripper_width_m - width) > 0.002:
            raise RuntimeError("Gripper did not reach release/probe width")

    def _run_two_stage_init_motion_sequence(self):
        from std_msgs.msg import String

        s = self._two_stage_settings
        self._two_stage_begin = time.monotonic()
        self._two_stage_force_guard = False
        self._two_stage_abort = None
        original_cmd, original_script = self._cmd_pub, self._urscript_pub
        done = threading.Event()
        self._two_stage_running = True

        def stop(reason):
            self._two_stage_abort = str(reason)
            # These are unwrapped publishers, so a latched abort cannot suppress STOP.
            original_cmd.publish(String(data="stop"))
            original_script.publish(String(data="stopj(2.0)"))
            self._sequence_failed_event.set()
            self._sequence_finished_event.set()

        def watch():
            while not done.wait(0.05):
                try:
                    if (
                        not getattr(self, "_auto_in_init", False)
                        and select.select([sys.stdin], [], [], 0)[0]
                    ):
                        sys.stdin.readline()
                        raise RuntimeError("Manual Enter requested STOP")
                    self._two_stage_check()
                except Exception as exc:
                    stop(exc)
                    return

        watcher = None
        try:
            if not self._check_sensors_health():
                raise RuntimeError("Required sensors unavailable")
            self._two_stage_check()  # Validate all feedback before the first command.
            self._cmd_pub = _Publisher(self, original_cmd)
            self._urscript_pub = _Publisher(self, original_script)
            watcher = threading.Thread(target=watch, daemon=True)
            watcher.start()
            self._init_observe("two_stage_start", settings=s)
            self.get_logger().warning(
                "Two-stage init starts: open, first-axis center, retreat/rotate, second-axis center. Enter stops manual init."
            )
            self._sequence_finished_event.clear()
            self._sequence_failed_event.clear()
            joints = " ".join(
                str(math.radians(v)) for v in self.config.initial_arm_pose
            )
            self._cmd_pub.publish(String(data="init 0.135 " + joints))
            if not self._wait_for_sequence(20.0, "Two-stage initial pose"):
                raise RuntimeError("Initial arm/gripper move failed")
            # Coarse correction remains on the FIXED clamp axis, even after yaw changes.
            before = self._two_stage_pose()
            offset = getattr(self, "_auto_init_offset", None) or {}
            distance = s["clamp_center_width_m"] / 2 + self.config.arm_offset
            tangent = math.radians(45 + s["rotation_deg"])
            delta = np.array(
                [
                    distance * math.cos(math.radians(135)),
                    distance * math.sin(math.radians(135)),
                    0.0,
                ]
            )
            # Apply only height here: stage-1 centering would cancel a tangent shift
            # because the final tangent is parallel to the original clamp axis.
            delta += [0, 0, offset.get("vertical_m", 0)]
            self._two_stage_move((before[0] + delta, before[1]), "clamp_coarse_center")
            self._two_stage_gripper(s["stage1_probe_width_m"])
            time.sleep(0.5)
            self._force_monitor.tare()
            self._two_stage_force_guard = True
            width = self._execute_active_centering(
                axis_yaw_deg=135.0,
                probe_width_m=s["stage1_probe_width_m"],
                edge_only=True,
                strict=True,
            )
            if width is None:
                raise RuntimeError("Stage 1 centering failed")
            self._two_stage_gripper(0.135)
            targets = rotation_targets(
                *self._two_stage_pose(),
                s["pivot_tcp_m"],
                s["retreat_base_m"],
                s["rotation_deg"]
            )
            self._init_observe(
                "two_stage_rotation_plan",
                pivot_tcp_m=s["pivot_tcp_m"],
                retreat_base_m=s["retreat_base_m"],
                rotation_deg=s["rotation_deg"],
            )
            for target, label in zip(targets, ("retreat", "rotate", "return_center")):
                self._two_stage_move(
                    target,
                    label,
                    s["rotation_duration_s"] if label == "rotate" else None,
                )
            lateral = offset.get("lateral_m", 0)
            if lateral:
                before = self._two_stage_pose()
                self._two_stage_move(
                    (
                        before[0]
                        + [lateral * math.cos(tangent), lateral * math.sin(tangent), 0],
                        before[1],
                    ),
                    "planned_tangent_offset",
                )
            self._two_stage_gripper(s["stage2_probe_width_m"])
            time.sleep(0.5)
            self._force_monitor.tare()
            width = self._execute_active_centering(
                axis_yaw_deg=135 + s["rotation_deg"],
                probe_width_m=s["stage2_probe_width_m"],
                nominal_width_m=self.config.target_diameter,
                strict=True,
            )
            if width is None:
                raise RuntimeError("Stage 2 centering failed")
            self._mark_init_contact(width)
            self._two_stage_gripper(s["stage2_probe_width_m"])
            self._two_stage_check()
            self._effective_object_width_m = width
            self._calibrated_target_m = max(0.0, width - 2 * self.config.push_depth)
            self._saved_base_joints = list(self._current_joints)
            self._init_observe(
                "two_stage_complete", effective_width_m=width, test_only=s["test_only"]
            )
            self.get_logger().info(
                "Two-stage init complete"
                + (
                    " (TEST ONLY; not reusable for collection)"
                    if s["test_only"]
                    else ""
                )
            )
            return True
        except Exception as exc:
            stop(exc)
            self.get_logger().error(
                "Two-stage init stopped: "
                + str(exc)
                + "; no automatic release/rotation retry"
            )
            self._init_observe("two_stage_abort", reason=str(exc))
            return False
        finally:
            done.set()
            if watcher is not None:
                watcher.join(timeout=1.0)
            self._cmd_pub, self._urscript_pub = original_cmd, original_script
            self._two_stage_running = False
