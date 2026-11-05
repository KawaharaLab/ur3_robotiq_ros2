"""Local-copy integrity, bounded I/O failure and unchanged ROS payload tests."""
import ast
import json
from pathlib import Path
import tempfile
import threading
import time
import types
from typing import Optional
import unittest

from ros2_gelsight_package.local_acquisition_records import LocalAcquisitionWriter

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2_gelsight_package'


class WrappedFile:
    def __init__(self, real, gate=None, entered=None, fail=False):
        self.real,self.gate,self.entered,self.fail=real,gate,entered,fail
    def writelines(self, lines):
        if self.entered:self.entered.set()
        if self.gate:self.gate.wait(3.)
        if self.fail:raise OSError('simulated disk full')
        return self.real.writelines(lines)
    def __getattr__(self, name):return getattr(self.real,name)


def writer(root, **kw):
    return LocalAcquisitionWriter(root, dict(node='camera_left',stream_id='unique',
        image_topic='gelsight/left/image_raw'), **kw)


class WriterTests(unittest.TestCase):
    def test_exact_payload_and_final_counters(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=writer(tmp)
            payload=json.dumps(dict(header_ns=1793697560130911119,frame_seq=7))
            for _ in range(10):self.assertTrue(w.submit('frame',payload,[1,2,3]))
            s=w.close()
            self.assertTrue(s['clean_close']);self.assertTrue(s['capture_thread_stopped'])
            self.assertEqual([s[k] for k in ('accepted_records','written_records','flushed_records')],[10]*3)
            self.assertEqual(s['dropped_records'],0);self.assertIsNone(s['error'])
            lines=[json.loads(v) for v in w.path.read_text().splitlines()]
            self.assertEqual(lines[0]['kind'],'start')
            self.assertEqual(lines[1]['record'],json.loads(payload))
            self.assertEqual(lines[1]['enqueue_clock'],[1,2,3])
            self.assertTrue(json.loads(w.status_path.read_text())['closed'])

    def test_queue_overflow_drops_without_waiting_for_disk(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=writer(tmp,queue_size=2);gate=threading.Event();entered=threading.Event()
            w._file=WrappedFile(w._file,gate,entered)
            try:
                self.assertTrue(w.submit('frame','{}',[1,2,3]));self.assertTrue(entered.wait(1.))
                self.assertTrue(w.submit('frame','{}',[1,2,3]))
                self.assertTrue(w.submit('frame','{}',[1,2,3]))
                self.assertFalse(w.submit('frame','{}',[1,2,3]))
                self.assertEqual(w.snapshot()['dropped_records'],1)
            finally:
                gate.set();s=w.close()
            self.assertEqual(s['written_records'],3)

    def test_disk_failure_does_not_make_submit_raise_or_retry_forever(self):
        with tempfile.TemporaryDirectory() as tmp:
            failures=[];w=writer(tmp,on_error=failures.append);entered=threading.Event()
            w._file=WrappedFile(w._file,entered=entered,fail=True)
            self.assertTrue(w.submit('frame','{}',[1,2,3]));self.assertTrue(entered.wait(1.))
            w._thread.join(1.);s=w.close()
            self.assertIn('simulated disk full',s['error']);self.assertFalse(s['clean_close'])
            self.assertFalse(w.submit('frame','{}',[1,2,3]))
            self.assertTrue(failures);self.assertEqual(w.snapshot()['rejected_records'],1)

    def test_slow_close_has_bounded_wait_and_reports_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=writer(tmp);gate=threading.Event();entered=threading.Event()
            w._file=WrappedFile(w._file,gate,entered)
            try:
                w.submit('frame','{}',[1,2,3]);self.assertTrue(entered.wait(1.))
                s=w.close(timeout=.01,capture_thread_stopped=False)
                self.assertTrue(s['close_timed_out']);self.assertFalse(s['closed'])
                self.assertFalse(s['capture_thread_stopped'])
            finally:
                gate.set();w.close()

    def test_periodic_flush_visible_before_shutdown(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=writer(tmp,flush_seconds=.02)
            try:
                w.submit('frame','{}',[1,2,3])
                deadline=time.monotonic()+1.
                while time.monotonic()<deadline:
                    s=json.loads(w.status_path.read_text())
                    if s['flushed_records']==1:break
                    time.sleep(.01)
                self.assertEqual(s['flushed_records'],1)
                self.assertEqual(len(w.path.read_text().splitlines()),2)
            finally:w.close()

    def test_filename_unique_and_option_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=writer(tmp)
            try:
                with self.assertRaises(FileExistsError):writer(tmp)
            finally:w.close()
            with self.assertRaises(ValueError):writer(tmp,queue_size=0)


class PublisherCopyTests(unittest.TestCase):
    def publisher(self, fail=False):
        tree=ast.parse((PACKAGE/'ros2_gelsight_publisher.py').read_text())
        cls=next(v for v in tree.body if isinstance(v,ast.ClassDef) and v.name=='GelSightMiniPublisher')
        env=dict(Node=object,ConfigModel=object,Optional=Optional,json=json,
                 cv2=types.SimpleNamespace(__version__='test'),
                 String=lambda **kw:types.SimpleNamespace(**kw))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'publisher','exec'),env)
        n=env['GelSightMiniPublisher'].__new__(env['GelSightMiniPublisher'])
        calls=[]
        def publish(msg):
            calls.append(('ros',msg.data))
            if fail:raise RuntimeError('simulated ROS error')
        n._acquisition_publisher=types.SimpleNamespace(publish=publish)
        n._acquisition_stream_id='stream';n._acquisition_backend='V4L2'
        n._acquisition_publish_errors=0;n._topic_name='gelsight/left/image_raw';n._frame_id='left'
        n._cam_stream=types.SimpleNamespace(frame_count=7,last_acquisition_timing={'read_end':[1,2,3]})
        n.get_clock=lambda:types.SimpleNamespace(clock_type=1)
        n.get_parameter=lambda name:types.SimpleNamespace(value=False)
        n.get_logger=lambda:types.SimpleNamespace(error=lambda *a,**kw:None)
        clock=iter([[10,20,30],[40,50,60],[70,80,90]])
        n._acquisition_clock=lambda:next(clock)
        n._local_acquisition_writer=types.SimpleNamespace(submit=lambda *a:calls.append(('local',a)))
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1793697560,nanosec=130911119)))
        n._publish_acquisition_record(msg,True)
        return n,calls

    def test_local_before_ros_preserves_identical_payload_and_separate_result(self):
        n,calls=self.publisher()
        self.assertEqual([c[0] for c in calls],['local','ros','local'])
        self.assertEqual(calls[0][1][0],'frame')
        self.assertEqual(calls[0][1][1],calls[1][1])
        result=json.loads(calls[2][1][1])
        self.assertTrue(result['success']);self.assertEqual(result['publish_start'],[40,50,60])
        self.assertEqual(result['publish_end'],[70,80,90])
        self.assertEqual(result['header_ns'],json.loads(calls[1][1])['header_ns'])

    def test_ros_failure_keeps_local_record(self):
        n,calls=self.publisher(fail=True)
        self.assertEqual(calls[0][1][0],'frame')
        self.assertFalse(json.loads(calls[-1][1][1])['success'])
        self.assertEqual(n._acquisition_publish_errors,1)


class ComparisonTests(unittest.TestCase):
    def test_missing_bag_timing_recovered_from_local_and_delivery_delay_separated(self):
        import importlib.util
        import sqlite3
        import struct
        helper=Path(__file__).resolve().parents[2]/'utils/diagnostics/compare_gelsight_local_bag.py'
        spec=importlib.util.spec_from_file_location('local_compare_test',helper)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        class CDRBuilder:
            def __init__(self):self.b=bytearray()
            def number(self,fmt,n,value):
                self.b.extend(b'\0'*((-len(self.b))%n));self.b.extend(struct.pack('<'+fmt,value))
            def string(self,value):
                b=value.encode()+b'\0';self.number('I',4,len(b));self.b.extend(b)
            def header(self,seconds):self.number('i',4,seconds);self.number('I',4,0);self.string('left')
            def finish(self):return b'\0\1\0\0'+self.b
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);db=root/'test.db3';local=root/'local';local.mkdir()
            c=sqlite3.connect(str(db));c.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
            c.executemany('INSERT INTO topics VALUES(?,?,?)',[(1,'/gelsight/left/image_raw/compressed','sensor_msgs/msg/CompressedImage'),(2,'/gelsight_mini_publisher_1/acquisition_timing','std_msgs/msg/String')])
            for sec in (2,3):
                b=CDRBuilder();b.header(sec);c.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(1,sec*10**9+1_000_000,b.finish()))
            timing=dict(schema_version=1,stream_id='stream',kind='gelsight_frame',header_ns=2*10**9,frame_seq=1,record_publish_errors_total=0)
            b=CDRBuilder();b.string(json.dumps(timing));c.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(2,7*10**9,b.finish()));c.commit();c.close()
            lines=[dict(kind='start',metadata=dict(stream_id='stream',image_topic='gelsight/left/image_raw'))]
            for sec in (2,3):lines.append(dict(kind='frame',record=dict(timing,header_ns=sec*10**9),enqueue_clock=[1,sec*10**9+1_000_000,2]))
            lines.append(dict(kind='publish_result',record=dict(header_ns=2*10**9,publish_start=[100,1,101],publish_end=[20100,2,20101],success=True)))
            (local/'left_stream.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in lines))
            result=module.compare(db,local)['topics']['/gelsight/left/image_raw/compressed']
            self.assertEqual(result['matched_local_frames'],2)
            self.assertEqual(result['bag_images_without_local'],0)
            self.assertEqual(result['bag_timing_missing_but_local_present'],1)
            self.assertEqual(result['bag_timing_after_local_enqueue_ms']['median'],4999.)
            self.assertEqual(result['publish_calls']['median'],.02)


class ArgumentTests(unittest.TestCase):
    def test_directory_is_opt_in_and_ros_remapping_arguments_preserved(self):
        import argparse
        import sys
        from unittest.mock import patch
        tree=ast.parse((PACKAGE/'ros2_gelsight_publisher.py').read_text())
        statements=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('_str2bool','parse_arguments')]
        env={'argparse':argparse}
        exec(compile(ast.Module(body=statements,type_ignores=[]),'arguments','exec'),env)
        with patch.object(sys,'argv',['publisher']):
            args,remaining=env['parse_arguments']()
            self.assertEqual(args.acquisition_record_dir,'')
            self.assertFalse(args.acquisition_records)
        with patch.object(sys,'argv',['publisher','--acquisition-records','true','--acquisition-record-dir','/tmp/local','--ros-args','-r','__node:=left']):
            args,remaining=env['parse_arguments']()
            self.assertTrue(args.acquisition_records)
            self.assertEqual(args.acquisition_record_dir,'/tmp/local')
            self.assertEqual(remaining,['--ros-args','-r','__node:=left'])


if __name__=='__main__':unittest.main()
