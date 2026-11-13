"""Bounded retry after confirmed stop; completed trials and init are preserved."""

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from data_capture_tools import object_automation as module

spec = importlib.util.spec_from_file_location(
    "retry_fixture", ROOT / "test/test_object_automation.py"
)
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class RetryTests(unittest.TestCase):
    def runner(self, root):
        n = fixture.FakeRunner(root)
        n.config.raw["automatic_retry"] = dict(enabled=True)
        n.waits = []
        n._auto_force_fault = None
        n._auto_retry_wait = lambda policy, record: n.waits.append(record)
        return n

    def test_transient_retry_keeps_completed_trials_and_saved_init(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n.fail_once = True
            n._auto_run()
            self.assertEqual(n._auto_job["status"], "complete")
            first = n._auto_job["plan"]["positions"][0]["trials"]
            self.assertEqual(len(first[0]["attempts"]), 1)
            self.assertEqual(len(first[1]["attempts"]), 2)
            self.assertEqual(first[1]["attempts"][0]["status"], "aborted")
            self.assertEqual(n.init_calls, 4)  # Each planned position exactly once.
            self.assertEqual(len(n.waits), 1)
            self.assertEqual(n._auto_job["automatic_retries"][0]["status"], "recovered")

    def test_repeated_fault_stops_after_three_retries(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n._auto_execute_trial = lambda *args: (_ for _ in ()).throw(
                module.Paused("Invalid image header age left: test")
            )
            n._auto_run()
            self.assertEqual(len(n.waits), 3)
            self.assertEqual(
                len(n._auto_job["plan"]["positions"][0]["trials"][0]["attempts"]), 4
            )
            self.assertEqual(n._auto_job["status"], "paused")

    def test_total_retry_limit_across_different_trials(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n.config.raw["automatic_retry"]["max_per_run"] = 1
            original = n._auto_execute_trial
            failed = set()

            def trial(row, attempt):
                if row["id"] not in failed:
                    failed.add(row["id"])
                    raise module.Paused("Stale force left")
                return original(row, attempt)

            n._auto_execute_trial = trial
            n._auto_run()
            self.assertEqual(len(n.waits), 1)
            self.assertEqual(n._auto_job["status"], "paused")

    def test_manual_force_fault_and_shutdown_are_not_retryable(self):
        for reason in (
            "Manual pause requested",
            "Node shutdown requested",
            "Force hard limit left",
            "Gripper fault",
            "Sensor stream restarted gelsight_left",
            "Bag recorder exited",
            "Nonfinite force left",
            "Arm position changed; reference cannot be reused",
        ):
            self.assertFalse(
                module.ObjectAutomationMixin._auto_retryable_reason(
                    module.Paused(reason)
                )
            )
        self.assertFalse(
            module.ObjectAutomationMixin._auto_retryable_reason(
                RuntimeError("Stale force left")
            )
        )

    def test_uncertain_stop_and_bad_bag_do_not_retry(self):
        for mode in ("uncertain", "bag"):
            with tempfile.TemporaryDirectory() as tmp:
                n = self.runner(Path(tmp))
                n.fail_once = True
                if mode == "uncertain":

                    def cancel():
                        n._auto_uncertain_goal = True
                        return False

                    n._auto_cancel = cancel
                else:
                    n.bad_bag = True
                n._auto_run()
                self.assertEqual(n.waits, [])
                self.assertEqual(n._auto_job["status"], "paused")

    def test_blocked_retry_reason_is_saved_and_live_runtime_is_identified(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n.fail_once = True

            def cancel():
                n._auto_uncertain_goal = True
                return False

            n._auto_cancel = cancel
            n._auto_run()
            self.assertIn(
                "unconfirmed_gripper_goal",
                n._auto_job["automatic_retry_stop"]["causes"],
            )
            self.assertIn(
                "stop_unconfirmed", n._auto_job["automatic_retry_stop"]["causes"]
            )
            self.assertEqual(
                n._auto_job["automatic_retry_runtime"]["diagnostic_version"], 1
            )
            self.assertEqual(len(n._auto_job["automatic_retry_decisions"]), 1)

    def test_old_pause_reason_is_not_shown_as_current_while_running(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n._auto_job["pause_reason"] = "old transient fault"
            n._auto_run()
            self.assertNotIn("pause_reason", n._auto_job)
            self.assertEqual(n._auto_job["last_pause_reason"], "old transient fault")

    def test_init_failure_never_retries(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n._auto_init_position = lambda p: (_ for _ in ()).throw(
                module.Paused("Missing/stale/undecodable GelSight left")
            )
            n._auto_run()
            self.assertEqual(n.waits, [])

    def test_recovery_interruption_is_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n.fail_once = True
            n._auto_retry_wait = lambda *args: (_ for _ in ()).throw(
                module.Paused("Manual pause requested")
            )
            n._auto_run()
            self.assertEqual(n._auto_job["status"], "paused")
            self.assertEqual(
                n._auto_job["automatic_retries"][0]["status"], "manual_required"
            )

    def test_wait_requires_continuous_health_and_never_sends_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n._auto_manual = lambda: None
            clock = [0.0]
            observed = []

            def health(**kwargs):
                observed.append(clock[0])
                if 0.9 < clock[0] < 1.2:
                    raise module.Paused("Stale force left")

            n._auto_health = health
            n._auto_grip = lambda *args: self.fail("No motion allowed in recovery wait")
            policy = dict(wait_timeout_s=5.0, stable_s=2.0, minimum_wait_s=2.0)
            with patch.object(
                module.time, "monotonic", side_effect=lambda: clock[0]
            ), patch.object(
                module.time,
                "sleep",
                side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
            ):
                module.ObjectAutomationMixin._auto_retry_wait(
                    n, policy, dict(reason="Stale force left")
                )
            self.assertGreaterEqual(clock[0], 3.2)
            self.assertFalse(n._auto_active)
            self.assertGreater(len(observed), 30)

    def test_persistent_disconnect_times_out_without_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n._auto_manual = lambda: None
            n._auto_health = lambda **kwargs: (_ for _ in ()).throw(
                module.Paused("Missing/stale/undecodable GelSight left")
            )
            clock = [0.0]
            with patch.object(
                module.time, "monotonic", side_effect=lambda: clock[0]
            ), patch.object(
                module.time,
                "sleep",
                side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
            ):
                with self.assertRaisesRegex(module.Paused, "recovery timeout"):
                    module.ObjectAutomationMixin._auto_retry_wait(
                        n,
                        dict(wait_timeout_s=3.0, stable_s=2.0, minimum_wait_s=2.0),
                        dict(reason="image lost"),
                    )

    def test_force_fault_during_recovery_requires_manual_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            n._auto_manual = lambda: None
            n._auto_health = lambda **kwargs: (_ for _ in ()).throw(
                module.Paused("Force hard limit left")
            )
            with self.assertRaisesRegex(module.Paused, "Force hard limit"):
                module.ObjectAutomationMixin._auto_retry_wait(
                    n,
                    dict(wait_timeout_s=3.0, stable_s=2.0, minimum_wait_s=2.0),
                    dict(reason="image lost"),
                )
            self.assertFalse(n._auto_armed)

    def test_invalid_configuration_blocks_before_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            n = self.runner(Path(tmp))
            for settings in (
                dict(enabled="true"),
                dict(max_per_trial=-1),
                dict(stable_s=float("nan")),
                dict(unknown=True),
            ):
                n.config.raw["automatic_retry"] = settings
                with self.assertRaises(ValueError):
                    n._auto_retry_policy()
