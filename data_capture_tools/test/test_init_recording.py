import collections
import ast
import gzip
import importlib.util
import json
import queue
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('init_recording', ROOT/'data_capture_tools/data_capture_tools/init_recording.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


class FakeNode(module.InitRecordingMixin):
    def __init__(self, root, fail=False, error=False, stale=False, driver_change=False):
        self._init_record_lock = threading.RLock()
        self._active_init_log = self._last_successful_init = self._init_contact = None
        self._init_latest = {}
        self._init_startup = dict(startup_id='driver-A', activation_performed=True)
        self._init_driver_changed = False
        self._init_node_run_id = 'manager-A'
        self._init_record_root = root
        self._init_reference_id = 'gauge20'
        self._init_reference_width_mm = 20.
        self._init_mount_id = 'custom-1'
        self._effective_object_width_m = .123
        self._calibrated_target_m = .100
        self._saved_base_joints = [1]
        self.config = SimpleNamespace(raw={'push_depth': .004})
        self._force_monitor = SimpleNamespace(_history_left=collections.deque([10.2]*20),
            _history_right=collections.deque([20.2]*20), _offset_left=10., _offset_right=20.)
        self.fail, self.error, self.stale, self.driver_change = fail, error, stale, driver_change
        self.moves = 0

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns()))

    def get_logger(self):
        return SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

    def _run_init_motion_sequence(self):
        self.moves += 1
        if self.error: raise RuntimeError('simulated init exception')
        if self.fail: return False
        ros = time.time_ns()
        self._init_observe('gripper_status', header_ns=ros, status=dict(g_po=189,g_pr=189,g_flt=0))
        for side, fy in [('left', 10.2), ('right', 20.2)]:
            self._init_observe('force_'+side, header_ns=ros, wrench=[0,fy,0,0,0,0])
        self._init_observe('width', estimated_width_mm=21.67)
        if self.stale:
            self._init_latest['gripper_status']['receive_monotonic_ns'] -= 1_000_000_000
        self._init_record_tare()
        self._mark_init_contact(.02167)
        # Simulate the legacy automatic reopen after contact capture.
        self._init_observe('gripper_status', header_ns=time.time_ns(), status=dict(g_po=100,g_pr=100,g_flt=0))
        self._effective_object_width_m = .02167
        self._calibrated_target_m = .01367
        if self.driver_change:
            self._init_startup_callback(SimpleNamespace(data=json.dumps(dict(startup_id='driver-B'))))
        return True


class InitRecordingTests(unittest.TestCase):
    def test_stream_is_valid_gzip_and_close_flushes_all_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = module.InitLog(tmp, {})
            for i in range(1000): log.emit(dict(kind='force_left', sequence=i))
            result = log.close(True, {})
            with gzip.open(log.path/'samples.jsonl.gz','rt') as f: rows=[json.loads(x) for x in f]
            self.assertEqual([r['sequence'] for r in rows], list(range(1000)))
            self.assertTrue(result['recording_complete'])
            self.assertEqual(result['row_counts'], {'force_left':1000})
            log.emit(dict(kind='late'))
            self.assertEqual(log.written, 1000)

    def test_queue_overflow_is_reported_not_silently_lost(self):
        with tempfile.TemporaryDirectory() as tmp:
            log=module.InitLog(tmp, {}, queue_size=1)
            with patch.object(log.queue, 'put_nowait', side_effect=queue.Full):
                log.emit(dict(kind='one'))
            self.assertEqual(log.dropped, 1)
            result=log.close(False,None)
            self.assertFalse(result['recording_complete'])

    def test_cannot_open_stream_blocks_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp))
            with patch.object(module.gzip, 'open', side_effect=OSError('disk error')):
                self.assertFalse(node._run_init_sequence())
            self.assertEqual(node.moves,0)
            summary=json.loads(next(Path(tmp).glob('*/summary.json')).read_text())
            self.assertEqual(summary['status'],'failed_before_motion')
            self.assertIn('disk error',summary['writer_error'])

    def test_contact_is_before_reopen_and_copied_into_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp)/'records')
            self.assertTrue(node._run_init_sequence())
            source=node._last_successful_init
            result=json.loads((source/'summary.json').read_text())
            self.assertEqual(result['contact']['sources']['gripper_status']['status']['g_po'],189)
            self.assertEqual(node._init_latest['gripper_status']['status']['g_po'],100)
            self.assertTrue(result['metadata']['driver_initialization']['activation_performed'])
            self.assertEqual(result['metadata']['reference_width_mm'],20.)
            self.assertAlmostEqual(result['contact']['force_monitor']['calibrated_n']['left'],.2)
            session=Path(tmp)/'session';session.mkdir();node._attach_init_record(session)
            ref=json.loads((session/'init_reference.json').read_text())
            self.assertTrue(ref['available'])
            self.assertEqual((session/ref['samples']).read_bytes(),(source/'samples.jsonl.gz').read_bytes())
            node._attach_init_record(session)  # repeated session setup is safe for identical init

    def test_failed_init_invalidates_previous_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp));self.assertTrue(node._run_init_sequence())
            old=node._last_successful_init
            node.fail=True;self.assertFalse(node._run_init_sequence())
            self.assertIsNone(node._last_successful_init)
            self.assertIsNone(node._calibrated_target_m)
            self.assertTrue(old.exists())
            self.assertEqual(len(list(Path(tmp).glob('*/summary.json'))),2)

    def test_exception_preserves_failed_record_and_propagates(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp),error=True)
            with self.assertRaisesRegex(RuntimeError,'simulated'): node._run_init_sequence()
            result=json.loads(next(Path(tmp).glob('*/summary.json')).read_text())
            self.assertFalse(result['motion_success']);self.assertFalse(result['valid_for_collection'])
            self.assertIn('simulated',result['error'])

    def test_stale_contact_is_saved_but_not_reused(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp),stale=True)
            self.assertFalse(node._run_init_sequence())
            result=json.loads(next(Path(tmp).glob('*/summary.json')).read_text())
            self.assertTrue(result['motion_success']);self.assertFalse(result['contact']['usable'])

    def test_driver_change_during_init_invalidates_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp),driver_change=True)
            self.assertFalse(node._run_init_sequence())
            result=json.loads(next(Path(tmp).glob('*/summary.json')).read_text())
            self.assertTrue(result['driver_changed_during_init'])

    def test_driver_restart_after_success_requires_new_init(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp));node._run_init_sequence()
            node._init_startup_callback(SimpleNamespace(data=json.dumps(dict(startup_id='driver-A'))))
            self.assertIsNotNone(node._last_successful_init)
            node._init_startup_callback(SimpleNamespace(data=json.dumps(dict(startup_id='driver-B'))))
            self.assertIsNone(node._last_successful_init)
            self.assertIsNone(node._effective_object_width_m)

    def test_bad_output_path_fails_before_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'file';root.write_text('keep')
            node=FakeNode(root)
            self.assertFalse(node._run_init_sequence())
            self.assertEqual(node.moves,0)

    def test_checksum_rejects_modified_init_stream(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp)/'records');node._run_init_sequence()
            (node._last_successful_init/'samples.jsonl.gz').write_bytes(b'changed')
            session=Path(tmp)/'session';session.mkdir()
            with self.assertRaisesRegex(RuntimeError,'checksum'):node._attach_init_record(session)

    def test_missing_init_is_explicit_in_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=FakeNode(Path(tmp)/'records');node._attach_init_record(Path(tmp))
            self.assertFalse(json.loads((Path(tmp)/'init_reference.json').read_text())['available'])


class DriverRecordTests(unittest.TestCase):
    def test_cpp_initialization_json_for_activation_and_reuse(self):
        source=(ROOT/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_hardware/src/robotiq_2f_gripper_node.cpp').read_text()
        start=source.index('std::string make_initialization_record(')
        end=source.index('\n}\n}',start)+2
        function=source[start:end]
        prefix='#include <cstdint>\n#include <string>\n#include <sstream>\n#include <iomanip>\n#include <iostream>\n'
        main='''int main(){
std::cout << make_initialization_record(1,2,3,"/dev/ttyUSB0",115200,9,false,false,true,true) << "\\n";
std::cout << make_initialization_record(4,5,6,"/dev/ttyUSB0",115200,9,false,true,false,true) << "\\n";
}'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'record.cpp';p.write_text(prefix+function+main);binary=Path(tmp)/'record'
            subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror',str(p),'-o',str(binary)],check=True)
            rows=[json.loads(x) for x in subprocess.check_output([str(binary)],text=True).splitlines()]
        self.assertTrue(rows[0]['activation_performed'])
        self.assertFalse(rows[1]['activation_performed'])
        self.assertEqual(rows[0]['startup_id'],'1-2')
        self.assertEqual(rows[1]['startup_id'],'4-5')


class MonitorHookTests(unittest.TestCase):
    def test_existing_twenty_sample_mean_and_tare_keep_same_behavior(self):
        source=(ROOT/'data_capture_tools/data_capture_tools/data_capture_node_10times.py').read_text()
        tree=ast.parse(source)
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ForceSensorMonitor')
        subset=ast.Module(body=[cls], type_ignores=[])
        namespace=dict(collections=collections,time=time,QoSProfile=lambda **kw:kw,
                       ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=0),HistoryPolicy=SimpleNamespace(KEEP_LAST=0),
                       WrenchStamped=object)
        # Annotations are postponed so this isolated class needs no ROS imports.
        exec('from __future__ import annotations\n'+ast.unparse(subset),namespace)
        captured=[]
        node=SimpleNamespace(create_subscription=lambda *a:None,
                             _init_force_callback=lambda side,msg:captured.append(side),
                             _init_record_tare=lambda:captured.append('tare'),
                             get_logger=lambda:SimpleNamespace(info=lambda *a:None))
        monitor=namespace['ForceSensorMonitor'](node)
        for i in range(30):
            monitor._left_callback(SimpleNamespace(wrench=SimpleNamespace(force=SimpleNamespace(y=float(i)))))
            monitor._right_callback(SimpleNamespace(wrench=SimpleNamespace(force=SimpleNamespace(y=float(i+10)))))
        self.assertEqual(len(monitor._history_left),20)
        self.assertEqual(monitor.get_calibrated_left(),19.5)
        monitor.tare()
        self.assertEqual(monitor.get_calibrated_left(),0.)
        self.assertEqual(monitor.get_calibrated_right(),0.)
        self.assertEqual(captured.count('left'),30)
        self.assertEqual(captured[-1],'tare')


if __name__=='__main__': unittest.main()
