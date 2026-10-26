"""Deterministic position/force simulation; never connects to ROS or hardware."""
import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'data_capture_tools'))
from data_capture_tools import force_press_worker as worker

spec = importlib.util.spec_from_file_location('step_probe', ROOT/'utils/diagnostics/force_stop_probe.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def plan():
    return dict(effective_width_m=.040, clearance_m=.002, max_depth_per_side_m=.004,
                threshold_n=.5, hard_limit_n=1., hold_seconds=4., target_confirm_ms=3.,
                close_ticks=20, max_motion_ms=2000, baud=115200, slave=9,
                trial_index=1, port='/dev/ttyUSB0', press_mode='stepped')


class Simulation:
    def __init__(self, force_per_count=.2, limit_delta=8, no_motion=False,
                 missing_samples=False, hard_during_pause=False):
        self.clock_ns = 10**12
        self.collection = worker.Collection(plan(), probe)
        self.state = probe.ForceState(.5, 1., 100., 'max', 3.)
        self.state.offsets = dict(left=10., right=20.)
        self.state.target_enabled = False
        self.start = self.collection.start_raw
        self.position = self.start
        self.limit = self.start+limit_delta
        self.target = self.start
        self.go = False
        self.commands, self.events = [], []
        self.force_per_count = force_per_count
        self.no_motion, self.missing_samples = no_motion, missing_samples
        self.hard_during_pause = hard_during_pause
        self.manual = threading.Event()
        self.collection.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=self.clock_ns)))
        self.feed()
        self.state.armed = True

    def feed(self):
        force = (self.position-self.start)*self.force_per_count
        if self.hard_during_pause and self.commands and not self.go:
            force = 1.2
        for side in ('left', 'right'):
            self.state.update(side, self.clock_ns, self.clock_ns, self.clock_ns,
                              [0, self.state.offsets[side]+force, 0, 0, 0, 0])

    def sleep(self, seconds):
        self.clock_ns += round(seconds*1e9)
        if not self.missing_samples:
            self.feed()

    def status(self):
        self.sleep(.001)
        return dict(g_act=1, g_sta=3, g_flt=0, g_gto=int(self.go),
                    g_obj=0, g_pr=self.target, g_po=self.position, g_cu=0)

    def command(self, target, speed, force, go):
        self.commands.append((target, speed, force, go))
        self.go = go
        if go:
            self.target = target
            if not self.no_motion:
                self.position = target
        self.sleep(.001)

    def event(self, name, **data):
        row = dict(event=name, monotonic_ns=self.clock_ns, **data)
        self.events.append(row)
        return row

    def run(self):
        clock = SimpleNamespace(monotonic=lambda: self.clock_ns/1e9,
                                monotonic_ns=lambda: self.clock_ns, sleep=self.sleep)
        with patch.object(worker, 'time', clock):
            initial = self.status()
            self.collection.run_stepped_press(self, self.status, self.event, self.state,
                                              self.manual, initial, self.limit)


class SteppedTests(unittest.TestCase):
    def test_full_probe_runs_steps_then_hold_and_release(self):
        spec = importlib.util.spec_from_file_location('steps_simulation', ROOT/'utils/diagnostics/test_force_stop_probe.py')
        tests = importlib.util.module_from_spec(spec); spec.loader.exec_module(tests)
        collection = worker.Collection(plan(), tests.probe)
        code, result, commands = tests.SimulatedRunTests().simulate('collection', collection=collection)
        self.assertEqual(code, 0, result)
        self.assertTrue(result['collection']['completed'])
        self.assertEqual(result['stop_trigger']['decision_method'], 'settled_window_median')
        self.assertEqual(len(result['collection']['steps']), 3)
        self.assertEqual(commands, [True, False]*3+[False, True, False])

    def test_lost_step_go_ack_uses_outer_stop_cleanup(self):
        spec = importlib.util.spec_from_file_location('steps_ack_simulation', ROOT/'utils/diagnostics/test_force_stop_probe.py')
        tests = importlib.util.module_from_spec(spec); spec.loader.exec_module(tests)
        class Hook:
            is_stepped = True
            def setup(self, node): pass
            def on_event(self, row): pass
            def on_status(self, row): pass
            def prepare(self, *args): pass
            def bound_target(self, target): return target
            def summary(self): return dict(completed=False)
            def run_stepped_press(self, port, status, event, state, manual, initial, target):
                port.command(target, 0, 0, True)
        code, result, commands = tests.SimulatedRunTests().simulate('ack_failure', collection=Hook())
        self.assertEqual(code, 2)
        self.assertEqual(commands, [True, False])
        self.assertTrue(result['stop_acknowledged'])

    def test_one_count_go_then_stop_then_new_sample_window_until_target(self):
        s = Simulation(); s.run()
        self.assertEqual(s.state.reason, 'force_threshold')
        self.assertEqual(s.position, s.start+3)
        self.assertLess(s.position, s.limit)
        self.assertEqual([x[3] for x in s.commands], [True, False]*3)
        for i, command in enumerate(s.commands[::2], 1):
            self.assertEqual(command[:3], (s.start+i, 0, 0))
        self.assertEqual(s.state.trigger['decision_method'], 'settled_window_median')
        self.assertAlmostEqual(s.state.trigger['measured_force_n']['left'], .6)
        for record in s.collection.step_records:
            self.assertGreaterEqual(record['measurement_end_ns']-record['stop_ack_ns'], 150_000_000)
            self.assertGreaterEqual(record['evidence']['left']['samples'], 10)

    def test_final_step_is_measured_and_can_succeed(self):
        s = Simulation(limit_delta=3); s.run()
        self.assertEqual(s.state.reason, 'force_threshold')

    def test_position_cap_aborts_without_extra_step(self):
        s = Simulation(force_per_count=.05, limit_delta=3); s.run()
        self.assertEqual(s.state.reason, 'position_limit_before_target')
        self.assertEqual(len(s.commands), 6)
        self.assertEqual(s.position, s.limit)

    def test_hard_limit_remains_immediate_during_motion(self):
        s = Simulation(force_per_count=1.2); s.run()
        self.assertEqual(s.state.reason, 'force_hard_limit')
        self.assertEqual(len([c for c in s.commands if c[3]]), 1)

    def test_hard_limit_remains_active_while_paused(self):
        s = Simulation(hard_during_pause=True); s.run()
        self.assertEqual(s.state.reason, 'force_hard_limit')
        self.assertEqual([c[3] for c in s.commands], [True, False])

    def test_no_motion_times_out_instead_of_skipping_single_count(self):
        s = Simulation(no_motion=True); s.run()
        self.assertEqual(s.state.reason, 'step_motion_timeout')
        self.assertEqual(len(s.commands), 1)

    def test_manual_stop_prevents_first_go(self):
        s = Simulation(); s.manual.set()
        with self.assertRaises(KeyboardInterrupt): s.run()
        self.assertEqual(s.commands, [])

    def test_missing_updates_abort_during_pause(self):
        s = Simulation(missing_samples=True)
        with self.assertRaisesRegex(RuntimeError, 'Stale force'): s.run()
        self.assertEqual(len([c for c in s.commands if c[3]]), 1)

    def test_total_timeout_is_bounded(self):
        s = Simulation(force_per_count=.001, limit_delta=20)
        s.collection.plan['step_total_timeout_ms'] = 1000
        s.run()
        self.assertEqual(s.state.reason, 'stepped_total_timeout')
        self.assertLessEqual(len([c for c in s.commands if c[3]]), 7)

    def test_invalid_step_settings_rejected(self):
        for key, value in [('step_counts', 0), ('step_counts', 4), ('step_settle_ms', 0),
                           ('step_sample_ms', 1), ('step_timeout_ms', float('nan')),
                           ('press_mode', 'unknown')]:
            p = plan(); p[key] = value
            with self.assertRaises(ValueError): worker.validate_plan(p)


if __name__ == '__main__':
    unittest.main()
