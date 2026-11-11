"""Snapshot warm-up, optional-failure and process ownership regressions. No hardware."""
import importlib.util
import io
from contextlib import redirect_stderr, redirect_stdout
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

REPO = Path(os.environ.get('CONTEXT_SNAPSHOT_TEST_REPO', str(Path(__file__).resolve().parents[2])))
OVERLAY = Path(os.environ.get('CONTEXT_SNAPSHOT_OVERLAY', str(REPO)))
sys.path.insert(0, str(REPO / 'data_capture_tools'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


snapshot = load('data_capture_tools.context_snapshot', OVERLAY/'data_capture_tools/data_capture_tools/context_snapshot.py')
automation_path = OVERLAY/'data_capture_tools/data_capture_tools/object_automation.py'
if not automation_path.exists():
    automation_path = REPO/'data_capture_tools/data_capture_tools/object_automation.py'
automation = load('data_capture_tools._snapshot_automation_test', automation_path)


class Process:
    pid = 654321
    returncode = None
    def poll(self): return self.returncode
    def send_signal(self, sig): self.returncode = 0
    def wait(self, timeout=None):
        if self.returncode is None: raise subprocess.TimeoutExpired('camera', timeout)
        return self.returncode


class CaptureTests(unittest.TestCase):
    def test_command_selects_only_wrist_rgb_in_separate_namespace(self):
        command = snapshot.camera_command('/driver', 'ac_context_123', '143122066280')
        self.assertIn('serial_no:=_143122066280', command)
        self.assertIn('enable_depth:=false', command)
        self.assertIn('__ns:=/ac_context_123', command)
        self.assertNotIn('/camera_wrist', ' '.join(command))
    def test_missing_dependencies_yields_failure_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(sys.modules, {'rclpy': None}):
                result = snapshot.capture(Path(tmp)/'snapshot', snapshot.DEFAULT_SERIAL)
            self.assertEqual(result['status'], 'failed')
            self.assertTrue((Path(tmp)/'snapshot/metadata.json').is_file())
            self.assertFalse((Path(tmp)/'snapshot/wrist.jpg').exists())
    def test_stop_escalates_only_owned_process(self):
        process = Process()
        self.assertEqual(snapshot.stop_camera(process), 0)
    def test_stop_uses_kill_if_camera_ignores_shutdown(self):
        seen = []
        process = Process()
        def send(sig):
            seen.append(sig)
            if sig == signal.SIGKILL:
                process.returncode = -9
        process.send_signal = send
        self.assertEqual(snapshot.stop_camera(process), -9)
        self.assertEqual(seen, [signal.SIGINT, signal.SIGTERM, signal.SIGKILL])
    def test_warmup_then_save_and_close_camera(self):
        if cv2 is None or np is None:
            self.skipTest('OpenCV/NumPy needed')
        callbacks = {}
        cleanup = []
        node = NS(create_subscription=lambda typ,topic,cb,qos: callbacks.setdefault(topic, cb),
                  destroy_node=lambda: cleanup.append('node'))
        context = NS(ok=lambda: True, shutdown=lambda: cleanup.append('context'))
        message = NS(header=NS(stamp=NS(sec=42,nanosec=123),frame_id='wrist'))
        process = Process()
        ticks = iter(n*.5 for n in range(100))
        def spin(timeout_sec):
            callback = next(cb for topic,cb in callbacks.items() if topic.endswith('image_raw'))
            callback(message)
        def executor_factory(*, context):
            self.assertIs(context, expected_context)
            return NS(add_node=lambda value: self.assertIs(value, node), spin_once=spin,
                      shutdown=lambda **kw: cleanup.append('executor'))
        expected_context = context
        modules = {'rclpy': NS(init=lambda **kwargs: None, create_node=lambda *a,**kw: node,
                              spin_once=lambda **kw: self.fail('Global spin must not be used')),
            'rclpy.executors': NS(SingleThreadedExecutor=executor_factory),
            'rclpy.context': NS(Context=lambda: context),
            'rclpy.signals': NS(SignalHandlerOptions=NS(NO=0)),
            'rclpy.qos': NS(qos_profile_sensor_data=None),
            'cv_bridge': NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda *a,**kw: np.zeros((8,8,3),dtype=np.uint8))),
            'sensor_msgs': NS(), 'sensor_msgs.msg': NS(Image=object,CameraInfo=object),
            'ament_index_python': NS()}
        with tempfile.TemporaryDirectory() as tmp:
            executable = Path(tmp)/'lib/realsense2_camera/realsense2_camera_node'
            executable.parent.mkdir(parents=True);executable.touch()
            modules['ament_index_python.packages'] = NS(get_package_prefix=lambda name: tmp)
            with patch.dict(sys.modules, modules), patch.object(snapshot.subprocess, 'Popen', return_value=process), \
                    patch.object(snapshot.time, 'monotonic', side_effect=lambda: next(ticks)):
                result = snapshot.capture(Path(tmp)/'snapshot', snapshot.DEFAULT_SERIAL)
            self.assertEqual(result['status'], 'saved', result)
            self.assertEqual(result['discarded_frames'], 2)
            self.assertTrue(result['camera_stop_confirmed'])
            self.assertEqual(process.returncode, 0)
            self.assertEqual(cleanup, ['executor', 'node', 'context'])
            self.assertIsNotNone(cv2.imread(str(Path(tmp)/'snapshot/wrist.jpg')))
    def test_cli_prints_original_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            error_output = io.StringIO()
            with patch.object(snapshot, 'capture', return_value=dict(status='failed',error='RuntimeError: original failure')), \
                    redirect_stdout(io.StringIO()), redirect_stderr(error_output):
                result = snapshot.main(['--out', str(Path(tmp)/'snapshot')])
            self.assertEqual(result, 1)
            self.assertIn('RuntimeError: original failure', error_output.getvalue())
    def test_existing_image_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp);(path/'wrist.jpg').write_bytes(b'original')
            with self.assertRaises(FileExistsError):snapshot.capture(path, snapshot.DEFAULT_SERIAL)
            self.assertEqual((path/'wrist.jpg').read_bytes(), b'original')


class Runner(automation.ObjectAutomationMixin):
    def __init__(self, root):
        self.config = NS(raw={})
        self._auto_job_path = root/'job.json'
        self._auto_job = {'job_id':'job'}
        self._stop_event = NS(is_set=lambda: False)
        self.messages = []
    def get_logger(self): return NS(info=self.messages.append, warn=self.messages.append)
    def _auto_manual(self): pass
    def _auto_save(self):
        self._auto_job_path.write_text(json.dumps(self._auto_job))


class IntegrationTests(unittest.TestCase):
    def test_success_then_resume_uses_same_photo(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = Runner(Path(tmp))
            def launch(command, **kwargs):
                directory = Path(command[command.index('--out')+1])
                (directory/'wrist.jpg').write_bytes(b'photo')
                (directory/'metadata.json').write_text(json.dumps(dict(status='saved',camera_stop_confirmed=True)))
                return NS(pid=654321,returncode=0,poll=lambda:0,wait=lambda **kw:0)
            with patch.object(automation.subprocess,'Popen',side_effect=launch) as start, \
                    patch.object(automation.os,'killpg',side_effect=ProcessLookupError):
                runner._auto_capture_context();runner._auto_capture_context()
            self.assertEqual(start.call_count,1)
            self.assertEqual(runner._auto_job['context_snapshot']['status'],'saved')
            self.assertEqual(len(runner._auto_job['context_snapshot_attempts']),1)
    def test_failure_is_recorded_without_raising_or_changing_trials(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = Runner(Path(tmp))
            with patch.object(automation.subprocess,'Popen',side_effect=OSError('camera missing')):
                runner._auto_capture_context()
            self.assertEqual(runner._auto_job['context_snapshot']['status'],'failed')
            self.assertIn('camera missing',runner._auto_job['context_snapshot']['error'])
            self.assertTrue(any('collection will continue' in m for m in runner.messages))
    def test_disabled_does_not_start_camera(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = Runner(Path(tmp));runner.config.raw={'context_snapshot':{'enabled':False}}
            with patch.object(automation.subprocess,'Popen') as start:runner._auto_capture_context()
            start.assert_not_called()
    def test_ac_go_invokes_photo_before_automatic_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = Runner(Path(tmp))
            runner.config = NS(output_root=Path(tmp),task_name='object1',raw={
                'automatic_collection':dict(depths_mm=[2.],loading_speeds=[.1],repeats=1,positions=['base'])})
            calls = []
            runner.create_publisher = lambda *args: None
            runner._auto_save = lambda: calls.append('save')
            runner._auto_capture_context = lambda: calls.append('photo')
            runner._auto_run = lambda: calls.append('run')
            modules = {'robotiq_2f_gripper_msgs': NS(),
                       'robotiq_2f_gripper_msgs.msg': NS(RobotActionEvent=object)}
            with patch.dict(sys.modules,modules), patch.object(automation.sys,'stdin',
                    NS(isatty=lambda:True,readline=lambda:'GO\n')):
                runner._execute_object_auto()
            self.assertEqual(calls,['save','photo','run'])
    def test_user_pause_cleans_up_owned_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = Runner(Path(tmp))
            runner._auto_manual = lambda: (_ for _ in ()).throw(automation.Paused('manual'))
            with patch.object(automation.subprocess,'Popen',return_value=NS(pid=654321,poll=lambda:None,wait=lambda **kw:0)), \
                    patch.object(automation.os,'killpg') as stop:
                with self.assertRaises(automation.Paused):runner._auto_capture_context()
            stop.assert_any_call(654321, signal.SIGTERM)
            stop.assert_any_call(654321, signal.SIGKILL)
            self.assertEqual(runner._auto_job['context_snapshot']['status'],'failed')


if __name__ == '__main__':unittest.main()
