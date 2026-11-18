"""Hardware-free regression checks for the opt-in two-stage init."""

import importlib.util
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch

import numpy as np

spec = importlib.util.spec_from_file_location(
    "two_stage_under_test",
    Path(__file__).parents[1] / "data_capture_tools" / "two_stage_init.py",
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def settings():
    return dict(
        mode="two_stage",
        test_only=True,
        require_gelsight=False,
        rotation_deg=90,
        clamp_center_width_m=0.04,
        stage1_probe_width_m=0.06,
        stage2_probe_width_m=0.06,
        pivot_tcp_m=[0.01, 0.02, 0.1],
        retreat_base_m=[0, 0, 0.05],
    )


class GeometryTests(unittest.TestCase):
    def test_default_preserves_legacy(self):
        self.assertEqual(
            m.settings_from({}),
            dict(mode="legacy", test_only=False, require_gelsight=True),
        )

    def test_no_camera_bypass_for_collection(self):
        for raw in ({"require_gelsight": False}, {**settings(), "test_only": False}):
            with self.assertRaises(ValueError):
                m.settings_from({"initialization": raw})

    def test_small_angles_only_for_test_mode(self):
        for angle in (-20, 20, -0.5, 0.5):
            self.assertEqual(
                m.settings_from(
                    {"initialization": {**settings(), "rotation_deg": angle}}
                )["rotation_deg"],
                angle,
            )
            with self.assertRaises(ValueError):
                m.settings_from(
                    {
                        "initialization": {
                            **settings(),
                            "rotation_deg": angle,
                            "test_only": False,
                            "require_gelsight": True,
                        }
                    }
                )
        for angle in (0, True, "-20", float("nan"), float("inf"), -91):
            with self.assertRaises(ValueError):
                m.settings_from(
                    {"initialization": {**settings(), "rotation_deg": angle}}
                )

    def test_height_offset_defaults_and_validation(self):
        self.assertEqual(
            m.settings_from({"initialization": settings()})["post_rotation_z_offset_m"],
            0,
        )
        for dz in (-0.02, 0.02):
            self.assertEqual(
                m.settings_from(
                    {"initialization": {**settings(), "post_rotation_z_offset_m": dz}}
                )["post_rotation_z_offset_m"],
                dz,
            )
        for dz in (True, "-0.02", float("nan"), float("inf"), -0.101, 0.101):
            with self.assertRaises(ValueError):
                m.settings_from(
                    {"initialization": {**settings(), "post_rotation_z_offset_m": dz}}
                )

    def test_missing_pivot_and_retreat_rejected(self):
        for key in (
            "pivot_tcp_m",
            "retreat_base_m",
            "stage1_probe_width_m",
            "clamp_center_width_m",
            "stage2_probe_width_m",
        ):
            s = settings()
            del s[key]
            with self.assertRaises(ValueError):
                m.settings_from({"initialization": s})

    def test_invalid_parameters(self):
        for key, value in [
            ("rotation_deg", 180),
            ("rotation_duration_s", 1),
            ("test_only", "false"),
            ("retreat_base_m", [0, 0, 0]),
            ("stage1_probe_width_m", 0.03),
            ("pivot_tcp_m", [0, float("nan"), 0]),
            ("max_init_force_n", 100),
        ]:
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    m.settings_from({"initialization": {**settings(), key: value}})

    def test_both_yaw_signs_preserve_physical_center_and_z(self):
        p = np.array([0.3, -0.2, 0.4])
        r = m.quaternion_matrix([0.5, 0.5, 0.5, 0.5])
        pivot = np.array([0.01, 0.02, 0.1])
        retreat = np.array([0, 0, 0.05])
        for deg in (-90, 90):
            targets = m.rotation_targets(p, r, pivot, retreat, deg)
            np.testing.assert_allclose(targets[0][0], p + retreat)
            np.testing.assert_allclose(
                targets[-1][0] + targets[-1][1] @ pivot, p + r @ pivot
            )
            np.testing.assert_allclose(targets[1][0], targets[2][0] + retreat)
            a = math.radians(deg)
            np.testing.assert_allclose(
                targets[2][1] @ r.T,
                [
                    [math.cos(a), -math.sin(a), 0],
                    [math.sin(a), math.cos(a), 0],
                    [0, 0, 1],
                ],
                atol=1e-12,
            )

    def test_rotvec_round_trip_including_pi(self):
        for q in (
            [0, 0, 0, 1],
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0.5, 0.5, 0.5, 0.5],
            [0.01, 0.3, 0.4, 0.8],
        ):
            q = np.asarray(q) / np.linalg.norm(q)
            r = m.quaternion_matrix(q)
            v = m.rotvec(r)
            a = np.linalg.norm(v)
            back = (
                [0, 0, 0, 1]
                if a < 1e-10
                else [*(v / a * math.sin(a / 2)), math.cos(a / 2)]
            )
            np.testing.assert_allclose(m.quaternion_matrix(back), r, atol=1e-12)

    def test_bad_quaternion(self):
        for q in ([0, 0, 0, 0], [0, 0, 0, 2], [float("nan"), 0, 0, 1]):
            with self.assertRaises(ValueError):
                m.quaternion_matrix(q)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class Fake(m.TwoStageInitMixin):
    def __init__(self):
        self._two_stage_settings = m.settings_from({"initialization": settings()})
        self.config = types.SimpleNamespace(
            initial_arm_pose=[0] * 6,
            arm_offset=0.001,
            target_diameter=0.04,
            push_depth=0.004,
        )
        self._cmd_pub, self._urscript_pub = Publisher(), Publisher()
        self._sequence_failed_event, self._sequence_finished_event = (
            threading.Event(),
            threading.Event(),
        )
        self._force_monitor = types.SimpleNamespace(tare=lambda: None)
        self._current_joints = [1] * 6
        self.moves, self.centers, self.records = [], [], []
        self.fail = None

    def _check_sensors_health(self):
        return True

    def _two_stage_check(self):
        if self._two_stage_abort:
            raise RuntimeError(self._two_stage_abort)

    def _two_stage_pose(self):
        return np.array([0.3, 0.2, 0.4]), np.eye(3)

    def _wait_for_sequence(self, *args):
        return True

    def _two_stage_gripper(self, width):
        self.moves.append(("gripper", width))

    def _two_stage_move(self, target, label, duration=None):
        self.moves.append((label, target))
        if label == self.fail:
            raise RuntimeError("injected arm failure")

    def _execute_active_centering(self, **kw):
        self.centers.append(kw)
        return 0.04

    def _mark_init_contact(self, width):
        self.records.append(("contact", width))

    def _init_observe(self, kind, **kwargs):
        self.records.append((kind, kwargs))

    def get_logger(self):
        return types.SimpleNamespace(
            warning=lambda *x: None, error=lambda *x: None, info=lambda *x: None
        )


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.modules = patch.dict(
            sys.modules,
            {
                "std_msgs": types.ModuleType("std_msgs"),
                "std_msgs.msg": types.SimpleNamespace(
                    String=lambda **kw: types.SimpleNamespace(**kw)
                ),
            },
        )
        self.modules.start()

    def tearDown(self):
        self.modules.stop()

    def run_fake(self, f):
        with patch.object(m.time, "sleep"), patch.object(
            m.select, "select", return_value=([], [], [])
        ):
            return f._run_two_stage_init_motion_sequence()

    def test_stage_order_and_axis_sign(self):
        for sign in (-90, 90):
            f = Fake()
            f._two_stage_settings["rotation_deg"] = sign
            self.assertTrue(self.run_fake(f))
            self.assertEqual([x["axis_yaw_deg"] for x in f.centers], [135, 135 + sign])
            self.assertTrue(f.centers[0]["edge_only"])
            self.assertNotIn("edge_only", f.centers[1])
            self.assertEqual(
                len([x for x in f._cmd_pub.messages if x.startswith("init ")]), 1
            )
            self.assertEqual(
                [x[0] for x in f.moves],
                [
                    "clamp_coarse_center",
                    "gripper",
                    "gripper",
                    "retreat",
                    "rotate",
                    "return_center",
                    "gripper",
                    "gripper",
                ],
            )
            coarse = f.moves[0][1][0] - f._two_stage_pose()[0]
            np.testing.assert_allclose(
                coarse,
                [
                    0.021 * math.cos(math.radians(135)),
                    0.021 * math.sin(math.radians(135)),
                    0,
                ],
            )
            self.assertEqual(len([x for x in f.records if x[0] == "contact"]), 1)

    def test_small_angle_stops_open_at_retreat_before_return_and_stage_two(self):
        for angle in (-20, 20):
            f = Fake()
            f._two_stage_settings["rotation_deg"] = angle
            self.assertTrue(self.run_fake(f))
            self.assertEqual(
                [x[0] for x in f.moves],
                ["clamp_coarse_center", "gripper", "gripper", "retreat", "rotate"],
            )
            self.assertEqual(len(f.centers), 1)
            self.assertEqual(f.moves[2], ("gripper", 0.135))
            kinds = [x[0] for x in f.records]
            self.assertIn("two_stage_rotation_test_complete", kinds)
            self.assertNotIn("contact", kinds)
            self.assertNotIn("two_stage_complete", kinds)
            self.assertFalse(f._two_stage_running)

    def test_height_change_after_rotation_before_return_for_both_signs(self):
        for angle in (-90, 90):
            for dz in (-0.02, 0.02):
                f = Fake()
                f._two_stage_settings.update(
                    rotation_deg=angle, post_rotation_z_offset_m=dz
                )
                self.assertTrue(self.run_fake(f))
                labels = [x[0] for x in f.moves]
                self.assertLess(
                    labels.index("rotate"), labels.index("post_rotation_height")
                )
                self.assertLess(
                    labels.index("post_rotation_height"), labels.index("return_center")
                )
                self.assertEqual(f.moves[2], ("gripper", 0.135))
                pose0 = f._two_stage_pose()
                pivot = np.array(f._two_stage_settings["pivot_tcp_m"])
                expected = m.rotation_targets(
                    *pose0, pivot, f._two_stage_settings["retreat_base_m"], angle
                )
                moves = {label: target for label, target in f.moves}
                np.testing.assert_allclose(
                    moves["post_rotation_height"][0], expected[1][0] + [0, 0, dz]
                )
                final_p, final_r = moves["return_center"]
                np.testing.assert_allclose(
                    final_p + final_r @ pivot, pose0[0] + pose0[1] @ pivot + [0, 0, dz]
                )
                np.testing.assert_allclose(
                    moves["clamp_coarse_center"][0][2], pose0[0][2]
                )
                self.assertEqual(len(f.centers), 2)

    def test_small_angle_ignores_height_change(self):
        f = Fake()
        f._two_stage_settings.update(rotation_deg=-20, post_rotation_z_offset_m=-0.02)
        self.assertTrue(self.run_fake(f))
        self.assertNotIn("post_rotation_height", [x[0] for x in f.moves])
        self.assertNotIn("return_center", [x[0] for x in f.moves])

    def test_height_failure_stops_before_return_and_second_centering(self):
        f = Fake()
        f._two_stage_settings["post_rotation_z_offset_m"] = -0.02
        f.fail = "post_rotation_height"
        self.assertFalse(self.run_fake(f))
        self.assertEqual(len(f.centers), 1)
        self.assertNotIn("return_center", [x[0] for x in f.moves])
        self.assertIn("stop", f._cmd_pub.messages)

    def test_failed_rotation_never_returns_or_runs_stage_two(self):
        f = Fake()
        f.fail = "rotate"
        self.assertFalse(self.run_fake(f))
        self.assertEqual(len(f.centers), 1)
        self.assertNotIn("return_center", [x[0] for x in f.moves])
        self.assertIn("stop", f._cmd_pub.messages)
        self.assertIn("stopj(2.0)", f._urscript_pub.messages)
        self.assertFalse(f._two_stage_running)

    def test_aborted_publisher_cannot_send_more_motion(self):
        f = Fake()
        f._two_stage_abort = "stopped"
        pub = Publisher()
        with self.assertRaises(RuntimeError):
            m._Publisher(f, pub).publish(types.SimpleNamespace(data="move"))
        self.assertEqual(pub.messages, [])

    def test_pose_frame_and_age_checked(self):
        f = Fake()
        f._two_stage_tcp = None
        pose_reader = m.TwoStageInitMixin._two_stage_pose
        with self.assertRaises(RuntimeError):
            pose_reader(f)
        msg = types.SimpleNamespace(header=types.SimpleNamespace(frame_id="world"))
        f._two_stage_tcp = (m.time.monotonic(), msg)
        with self.assertRaises(RuntimeError):
            pose_reader(f)
        f._two_stage_tcp = (m.time.monotonic() - 1, msg)
        with self.assertRaises(RuntimeError):
            pose_reader(f)


class HealthTests(unittest.TestCase):
    def node(self):
        f = Fake()
        f._two_stage_abort = None
        f._two_stage_begin = m.time.monotonic()
        f._two_stage_force_guard = True
        f._stop_event = threading.Event()
        f._auto_robot_mode = 7
        f._auto_safety_mode = 1
        f._init_record_lock = threading.RLock()
        now = m.time.monotonic_ns()
        f._init_latest = {
            k: dict(receive_monotonic_ns=now, wrench=[0] * 6)
            for k in ("force_left", "force_right")
        }
        f._init_latest["gripper_status"] = dict(
            receive_monotonic_ns=now, status={"g_flt": 0}
        )
        f._force_monitor = types.SimpleNamespace(
            get_calibrated_left=lambda: 0.1, get_calibrated_right=lambda: 0.1
        )
        return f

    def test_no_images_still_requires_force_and_gripper(self):
        f = self.node()
        f._check_sensors_health = lambda: self.fail(
            "Image check must not run in no-camera test"
        )
        m.TwoStageInitMixin._two_stage_check(f)
        f._init_latest["force_left"]["receive_monotonic_ns"] -= 300_000_000
        with self.assertRaisesRegex(RuntimeError, "stale force_left"):
            m.TwoStageInitMixin._two_stage_check(f)

    def test_gripper_fault_force_limit_and_robot_stop(self):
        f = self.node()
        f._init_latest["gripper_status"]["status"]["g_flt"] = 9
        with self.assertRaisesRegex(RuntimeError, "Gripper fault"):
            m.TwoStageInitMixin._two_stage_check(f)
        f = self.node()
        f._force_monitor.get_calibrated_left = lambda: 6
        with self.assertRaisesRegex(RuntimeError, "force limit"):
            m.TwoStageInitMixin._two_stage_check(f)
        f = self.node()
        f._auto_safety_mode = 3
        with self.assertRaisesRegex(RuntimeError, "UR must"):
            m.TwoStageInitMixin._two_stage_check(f)

    def test_camera_required_mode_calls_health_check(self):
        f = self.node()
        f._two_stage_settings["require_gelsight"] = True
        f._check_sensors_health = lambda: False
        with self.assertRaisesRegex(RuntimeError, "Sensor health"):
            m.TwoStageInitMixin._two_stage_check(f)


class CenteringIntegrationTests(unittest.TestCase):
    def test_real_edge_only_path_uses_requested_axis_and_never_final_grips(self):
        import ast
        import re
        from typing import Optional

        source = (
            Path(__file__).parents[1]
            / "data_capture_tools"
            / "data_capture_node_10times.py"
        )
        cls = next(
            x
            for x in ast.parse(source.read_text()).body
            if isinstance(x, ast.ClassDef) and x.name == "DataCaptureNode"
        )
        method = next(
            x
            for x in cls.body
            if isinstance(x, ast.FunctionDef) and x.name == "_execute_active_centering"
        )
        env = dict(
            Optional=Optional,
            math=math,
            time=m.time,
            rclpy=types.SimpleNamespace(ok=lambda: True),
            String=lambda **kw: types.SimpleNamespace(**kw),
        )
        exec(
            compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"),
            env,
        )
        for degrees in (45, 135, 225):
            n = Fake()
            axis = np.array(
                [math.cos(math.radians(degrees)), math.sin(math.radians(degrees)), 0]
            )
            location = np.zeros(3)
            n.config.contact_threshold = 0.03
            n.config.contact_margin = 0.2
            n.config.centering_step_arm = 0.001
            n.config.centering_min_step_arm = 0.0001
            n.config.safe_margin = 0.02
            n._two_stage_pose = lambda: (location.copy(), np.eye(3))
            n._two_stage_wait_translation = lambda *args: None
            n._force_monitor = types.SimpleNamespace(
                tare=lambda: None,
                get_calibrated_left=lambda: max(0, float(location @ axis) * 100),
                get_calibrated_right=lambda: max(0, -float(location @ axis) * 100),
            )

            def publish(msg):
                match = re.search(r"pose_add\(p_curr, p\[([^]]+)\]", msg.data)
                delta = [float(x) for x in match.group(1).split(",")][:3]
                location[:] += delta

            n._urscript_pub = types.SimpleNamespace(publish=publish)
            with patch.object(m.time, "sleep"):
                width = env["_execute_active_centering"](
                    n,
                    axis_yaw_deg=degrees,
                    probe_width_m=0.06,
                    edge_only=True,
                    strict=True,
                )
            self.assertAlmostEqual(width, 0.058, places=5)
            np.testing.assert_allclose(location, [0, 0, 0], atol=1e-9)
            self.assertEqual(len(n._cmd_pub.messages), 1)
            self.assertTrue(n._cmd_pub.messages[0].startswith("step_gripper 0.06000"))
            self.assertEqual(
                [kind for kind, _ in n.records], ["two_stage_clamp_center"]
            )


if __name__ == "__main__":
    unittest.main()
