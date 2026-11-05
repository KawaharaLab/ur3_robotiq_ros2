"""Collector/receiver lifecycle and recorder hooks without ROS or robot hardware."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from data_capture_tools.gelsight_local_recording import GelsightLocalRecordingClient, STATUS_TOPICS
from ros2_gelsight_package.local_recording_control import RecordingLeaseController

ROOT=Path(__file__).resolve().parents[1]

class String:
    def __init__(self,**kw):self.__dict__.update(kw)

class Node:
    def __init__(self):self.messages=[];self.callbacks={};self.logs=[];self.route=None
    def create_publisher(self,*a):return types.SimpleNamespace(publish=self.publish)
    def publish(self,msg):
        self.messages.append(json.loads(msg.data))
        if self.route:self.route(json.loads(msg.data))
    def create_subscription(self,kind,topic,callback,*a):self.callbacks[topic]=callback;return callback
    def create_timer(self,*a):return types.SimpleNamespace()
    def get_logger(self):return types.SimpleNamespace(info=self.logs.append,warning=self.logs.append)

class ClientTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.patch=patch.dict(sys.modules,{'std_msgs':types.ModuleType('std_msgs'),'std_msgs.msg':types.SimpleNamespace(String=String)})
        self.patch.start();self.addCleanup(self.patch.stop)
        self.node=Node();self.client=GelsightLocalRecordingClient(self.node)
        self.controllers=[]
        for side in ('left','right'):
            self.controllers.append(RecordingLeaseController(lambda s=side:dict(node=s,stream_id='stream_'+s,image_topic='gelsight/'+s+'/image_raw')))
        def route(message):
            for topic,controller in zip(STATUS_TOPICS,self.controllers):
                status=controller.handle(message)
                self.node.callbacks[topic](String(data=json.dumps(status)))
        self.node.route=route
        self.addCleanup(lambda:self.client.stop(wait=False))
        self.addCleanup(lambda:[c.close() for c in self.controllers])
        self.root=Path(self.tmp.name)

    def test_start_stop_real_writers_and_session_manifest(self):
        self.assertTrue(self.client.start(self.root,timeout=.1))
        self.client.attach_process(types.SimpleNamespace(poll=lambda:None))
        for c in self.controllers:c.writer().submit('frame','{"header_ns":1}',[1,2,3])
        self.client._heartbeat()
        self.client.stop()
        m=json.loads((self.root/'gelsight_local_recording.json').read_text())
        self.assertEqual(m['phase'],'stopped');self.assertEqual(len(m['closed_nodes']),2)
        self.assertEqual(len(list((self.root/'gelsight_acquisition_records').glob('*.jsonl'))),2)
        self.assertTrue(all(c.status()['last_closed']['writer']['closed'] for c in self.controllers))

    def test_recorder_exit_automatically_stops_without_main_thread(self):
        self.assertTrue(self.client.start(self.root,timeout=.1))
        self.client.attach_process(types.SimpleNamespace(poll=lambda:0));self.client._heartbeat()
        self.assertIsNone(self.client._request)
        self.assertTrue(all(c.writer() is None for c in self.controllers))

    def test_no_acknowledgement_is_warning_not_exception(self):
        self.node.route=None
        self.assertFalse(self.client.start(self.root,timeout=0))
        self.assertTrue(self.node.logs);self.assertEqual(json.loads((self.root/'gelsight_local_recording.json').read_text())['phase'],'not_ready')

    def test_late_old_heartbeat_cannot_stop_new_session(self):
        self.assertTrue(self.client.start(self.root,timeout=.1));old=self.client._request['session_id'];self.client.stop()
        newroot=self.root/'new';newroot.mkdir()
        self.assertTrue(self.client.start(newroot,timeout=.1))
        self.client.stop(wait=False,expected_session=old)
        self.assertIsNotNone(self.client._request)

    def test_invalid_status_is_ignored_and_malformed_writer_does_not_crash(self):
        self.client._status(STATUS_TOPICS[0],String(data='[]'))
        self.assertEqual(self.client._statuses,{})
        self.client._status(STATUS_TOPICS[0],String(data='{"schema_version":1,"active":true,"writer":null}'))
        self.assertEqual(self.client._matched({'owner_id':'x','session_id':'y'}),{})

    def test_stop_then_old_start_does_not_reopen_files(self):
        self.assertTrue(self.client.start(self.root,timeout=.1));start=dict(self.node.messages[-1]);self.client.stop()
        self.node.route(start)
        self.assertTrue(all(c.writer() is None for c in self.controllers))


class RecorderHookTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree=ast.parse((ROOT/'data_capture_tools/data_capture_node_10times.py').read_text())
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DataCaptureNode')
        env=dict(Path=Path,os=os,subprocess=subprocess)
        for name in ('_start_rosbag','_stop_rosbag'):
            f=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name==name)
            exec(compile(ast.Module(body=[f],type_ignores=[]),'manager','exec'),env)
        cls.env=env

    def node(self,root,events):
        (root/'logs').mkdir()
        local=types.SimpleNamespace(start=lambda path:events.append('local_start'),attach_process=lambda p:events.append('attach'),stop=lambda *a,**kw:events.append('local_stop'))
        return types.SimpleNamespace(_bag_uri=root/'bag',_session_dir=root,_gelsight_local_client=local,
            config=types.SimpleNamespace(bag=types.SimpleNamespace(storage='sqlite3',compression_mode='file',compression_format='zstd'),topics=[]),
            get_logger=lambda:types.SimpleNamespace(info=lambda *a:None,warning=lambda *a:None),
            _check_closed_bag_quality=lambda:events.append('qa'),_discard_fast=False)

    def test_start_before_spawn_and_stop_before_qa(self):
        with tempfile.TemporaryDirectory() as tmp:
            events=[];n=self.node(Path(tmp),events);process=types.SimpleNamespace(poll=lambda:0)
            def spawn(*a,**kw):events.append('spawn');return process
            with patch.object(subprocess,'Popen',side_effect=spawn):self.env['_start_rosbag'](n)
            self.env['_stop_rosbag'](n)
            self.assertEqual(events,['local_start','spawn','attach','local_stop','qa'])

    def test_spawn_failure_releases_optional_writers(self):
        with tempfile.TemporaryDirectory() as tmp:
            events=[];n=self.node(Path(tmp),events)
            with patch.object(subprocess,'Popen',side_effect=OSError('spawn failed')):
                with self.assertRaises(OSError):self.env['_start_rosbag'](n)
            self.assertEqual(events,['local_start','local_stop'])
            n._bag_stdout.close();n._bag_stderr.close()


if __name__=='__main__':unittest.main()
