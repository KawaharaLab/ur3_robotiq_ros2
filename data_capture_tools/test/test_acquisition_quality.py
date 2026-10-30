"""No ROS/hardware required; real SQLite/CDR bags and advisory integration."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
REPO = Path(os.environ.get('ACQUISITION_QA_TEST_REPO', str(ROOT.parent)))
# This fallback is only used while testing a staged overlay before application.
sys.path.insert(0, str(ROOT))
if not (ROOT/'data_capture_tools/object_plan.py').exists():
    sys.path.insert(0, str(REPO/'data_capture_tools'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


qa = load('data_capture_tools.acquisition_quality', ROOT/'data_capture_tools/acquisition_quality.py')


class Encoder:
    def __init__(self):
        self.b = bytearray()
    def num(self, fmt, value, align=4):
        self.b.extend(b'\0' * ((-len(self.b)) % align))
        self.b.extend(struct.pack('<'+fmt, value))
    def string(self, value):
        raw = value.encode()+b'\0'
        self.num('I', len(raw)); self.b.extend(raw)
    def header(self, h):
        self.num('i', h//10**9); self.num('I', h % 10**9); self.string('sensor')
    def bytes(self):
        return b'\0\1\0\0'+self.b


def stamped(h, force=False):
    e = Encoder(); e.header(h)
    if force:
        for _ in range(6):
            e.num('d', 0., 8)
    return e.bytes()


def string(value):
    e = Encoder(); e.string(json.dumps(value)); return e.bytes()


def event(h, kind='START', mode='auto_depth_speed', action=False):
    e = Encoder(); e.header(h); e.string(kind); e.num('i', 1)
    if action:
        e.num('i', 2); e.string('gripper')
    else:
        e.string(mode)
    return e.bytes()


class Fixture:
    def __init__(self):
        self.types = dict(zip(qa.SENSORS, qa.TYPES))
        self.types.update({t: 'std_msgs/msg/String' for t in qa.TIMING})
        self.types[qa.TRIAL] = 'robotiq_2f_gripper_msgs/msg/TrialEvent'
        self.types[qa.ACTION] = 'robotiq_2f_gripper_msgs/msg/RobotActionEvent'
        self.rows = []
        for side in range(2):
            for seq in range(11):
                h = 1_000_000_000+seq*50_000_000
                self.rows.append((qa.SENSORS[side], h+1_000_000, stamped(h)))
                r = dict(schema_version=1, kind='gelsight_frame', stream_id='cam'+str(side),
                         frame_seq=seq, header_ns=h, record_publish_errors_total=0)
                self.rows.append((qa.TIMING[side], h+1_100_000, string(r)))
        for side in range(2):
            for seq in range(501):
                h = 1_000_000_000+seq*1_000_000
                self.rows.append((qa.SENSORS[side+2], h+100_000, stamped(h, True)))
                r = dict(schema_version=1, kind='mms101_reads', stream_id='ft'+str(side), batch_seq=seq,
                         dropped_samples_total=0, reads=[dict(byte_offset=seq*25, bytes_hex=(b'\0\x17'+b'\0'*23).hex(),
                                                            samples=[[seq,h,seq*25]])])
                self.rows.append((qa.TIMING[side+2], h+1_000_000, string(r)))
        self.rows += [(qa.TRIAL, 1_100_000_000, event(1_100_000_000)),
                      (qa.TRIAL, 1_400_000_000, event(1_400_000_000, 'END'))]
        for h, name in [(1_110_000_000,'auto_goal_sent'), (1_120_000_000,'auto_action_succeeded'),
                        (1_130_000_000,'auto_dwell_start'), (1_350_000_000,'auto_dwell_end')]:
            self.rows.append((qa.ACTION,h,event(h,name,action=True)))

    def write(self, path, rows=None):
        con = sqlite3.connect(path)
        con.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);'
                          'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        ids = {name: i for i,name in enumerate(self.types,1)}
        con.executemany('INSERT INTO topics VALUES(?,?,?)', [(i,name,self.types[name]) for name,i in ids.items()])
        con.executemany('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',
                        [(ids[t],bt,b) for t,bt,b in (self.rows if rows is None else rows)])
        con.commit(); con.close()


class AuditTests(unittest.TestCase):
    def run_fixture(self, transform=lambda r:r):
        with tempfile.TemporaryDirectory() as tmp:
            f = Fixture(); f.rows = transform(f.rows); p = Path(tmp)/'bag.db3'; f.write(p)
            before = p.read_bytes(); result = qa.audit_bag(p)
            self.assertEqual(p.read_bytes(), before)
            return result

    def test_clean_headers_and_event_pairs(self):
        r = self.run_fixture()
        self.assertEqual(r['status'], 'ok')
        self.assertEqual(r['mapping'][qa.SENSORS[0]]['matched'], 11)
        self.assertEqual(r['mapping'][qa.SENSORS[2]]['matched'], 501)

    def test_missing_image_timing_inside_trial_is_review(self):
        r = self.run_fixture(lambda rows:[x for x in rows if not (x[0]==qa.TIMING[0] and x[1]==1_200_000_000+1_100_000)])
        self.assertEqual(r['status'], 'needs_review')
        self.assertEqual(r['mapping'][qa.SENSORS[0]]['per_trial'][0]['sensor_without_timing'], 1)

    def test_same_missing_information_outside_trial_is_warning(self):
        r = self.run_fixture(lambda rows:[x for x in rows if not (x[0]==qa.TIMING[0] and x[1]==1_000_000_000+1_100_000)])
        self.assertEqual(r['status'], 'warning')
        self.assertEqual(r['mapping'][qa.SENSORS[0]]['per_trial'][0]['sensor_without_timing'], 0)

    def test_backlog_detected_even_when_counts_match(self):
        r = self.run_fixture(lambda rows:[(t,bt+5_000_000_000 if t==qa.TIMING[0] else bt,b) for t,bt,b in rows])
        self.assertEqual(r['status'], 'needs_review')
        self.assertEqual(r['mapping'][qa.SENSORS[0]]['sensor_without_timing'], 0)
        self.assertTrue(any(i['code']=='header_to_bag_delay_in_trial' for i in r['issues']))

    def test_force_loss_with_raw_packet_is_only_warning(self):
        r = self.run_fixture(lambda rows:[x for x in rows if not (x[0]==qa.SENSORS[2] and x[1]==1_201_000_000+100_000)])
        self.assertEqual(r['status'], 'warning')
        trial = r['mapping'][qa.SENSORS[2]]['per_trial'][0]
        self.assertEqual(trial['force_samples_recoverable_from_raw'], 1)

    def test_nonfinite_and_duplicate_headers(self):
        def change(rows):
            rows.append(next(x for x in rows if x[0]==qa.SENSORS[0]))
            e = Encoder(); e.header(1_201_000_000)
            for _ in range(6):e.num('d',float('nan'),8)
            return [(t,bt,e.bytes() if t==qa.SENSORS[2] and bt==1_201_000_000+100_000 else b) for t,bt,b in rows]
        r=self.run_fixture(change)
        self.assertEqual(r['status'],'needs_review')
        self.assertTrue(any(i['code']=='nonfinite_force' for i in r['issues']))

    def test_aborted_trial_not_classified_as_complete_trial(self):
        def change(rows):
            rows=[x for x in rows if not (x[0]==qa.TRIAL and x[1]==1_400_000_000)]
            rows.append((qa.TRIAL,1_400_000_000,event(1_400_000_000,'ABORT',mode='automatic')))
            return rows
        r=self.run_fixture(change)
        self.assertEqual(r['trials'][0]['outcome'],'ABORT')
        self.assertEqual(r['trials'][0]['mode'],'auto_depth_speed')
        self.assertNotEqual(r['status'],'needs_review')

    def test_missing_action_pair(self):
        r=self.run_fixture(lambda rows:[x for x in rows if not (x[0]==qa.ACTION and x[1]==1_350_000_000)])
        self.assertEqual(r['status'],'needs_review')

    def test_corrupt_database_report_is_not_checked(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.db3';p.write_text('broken')
            out=Path(tmp)/'report.json';r=qa.check_and_save(p,out)
            self.assertEqual(r['status'],'not_checked')
            self.assertEqual(json.loads(out.read_text())['status'],'not_checked')

    def test_parse_error_does_not_silently_pass(self):
        r=self.run_fixture(lambda rows:[(t,bt,b'bad' if t==qa.TIMING[0] else b) for t,bt,b in rows])
        self.assertEqual(r['status'],'needs_review')
        self.assertTrue(any(i['code']=='parse_error' for i in r['issues']))

    def test_missing_beginning_of_sensor_stream_is_review(self):
        r=self.run_fixture(lambda rows:[x for x in rows if not (x[0]==qa.SENSORS[2] and x[1]<1_200_000_000)])
        self.assertEqual(r['status'],'needs_review')
        self.assertTrue(any(i['code']=='sensor_coverage_short_at_trial_boundary' for i in r['issues']))

    def test_session_context_is_advisory_and_init_conflict_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'a.db3';Fixture().write(p)
            (root/'collection_provenance.json').write_text(json.dumps(dict(init_id='init-A',effective_config={})))
            (root/'init_reference.json').write_text(json.dumps(dict(init_id='init-B',available=False)))
            r=qa.check_and_save(p,root/'quality.json',session=root)
            self.assertEqual(r['status'],'needs_review')
            self.assertTrue(any(i['code']=='conflicting_session_init_ids' for i in r['issues']))

    def test_split_bags_are_all_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);f=Fixture()
            f.write(root/'a.db3',[x for x in f.rows if x[1]<1_250_000_000])
            f.write(root/'b.db3',[x for x in f.rows if x[1]>=1_250_000_000])
            r=qa.audit_bag(root)
            self.assertEqual(r['status'],'ok')
            self.assertEqual(r['mapping'][qa.SENSORS[0]]['matched'],11)

    def test_metadata_selects_all_splits_and_ignores_stale_database(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);f=Fixture()
            f.write(root/'a.db3',[x for x in f.rows if x[1]<1_250_000_000])
            f.write(root/'b.db3',[x for x in f.rows if x[1]>=1_250_000_000])
            (root/'stale.db3').write_text('not part of this bag')
            (root/'metadata.yaml').write_text('fixture metadata')
            metadata=dict(rosbag2_bagfile_information=dict(storage_identifier='sqlite3', relative_file_paths=['a.db3','b.db3']))
            with patch.dict(sys.modules, {'yaml':types.SimpleNamespace(safe_load=lambda text:metadata)}):
                r=qa.audit_bag(root)
            self.assertEqual(r['status'],'ok')
            self.assertEqual(r['mapping'][qa.SENSORS[0]]['matched'],11)

    def test_metadata_missing_database_is_not_checked(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'metadata.yaml').write_text('fixture metadata')
            metadata=dict(rosbag2_bagfile_information=dict(storage_identifier='sqlite3', relative_file_paths=['missing.db3']))
            with patch.dict(sys.modules, {'yaml':types.SimpleNamespace(safe_load=lambda text:metadata)}):
                r=qa.check_and_save(root,root/'report.json')
            self.assertEqual(r['status'],'not_checked')

    def test_compressed_database_and_cleanup(self):
        import shutil
        if not shutil.which('zstd'):self.skipTest('zstd unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'a.db3';Fixture().write(p)
            subprocess.run(['zstd','-q',str(p),'-o',str(root/'a.db3.zstd')],check=True)
            p.unlink()
            self.assertEqual(qa.audit_bag(root)['status'],'ok')
            self.assertEqual(sorted(x.name for x in root.iterdir()),['a.db3.zstd'])


class IntegrationTests(unittest.TestCase):
    def test_quality_does_not_repeat_or_pause_completed_trials(self):
        fixture_path = ROOT/'test/test_object_automation.py'
        if not fixture_path.exists():fixture_path=REPO/'data_capture_tools/test/test_object_automation.py'
        fixture=load('quality_runner_fixture',fixture_path)
        runner=load('data_capture_tools.quality_runner_under_test', ROOT/'data_capture_tools/object_automation.py')
        with tempfile.TemporaryDirectory() as tmp:
            n=fixture.FakeRunner(Path(tmp))
            n._stop_rosbag=lambda:setattr(n,'_last_bag_quality',dict(status='needs_review',report='quality.json'))
            runner.ObjectAutomationMixin._auto_run(n)
            self.assertEqual(n._auto_job['status'],'complete')
            self.assertEqual(n.attempt_calls,8)
            for p in n._auto_job['plan']['positions']:
                for t in p['trials']:
                    self.assertEqual(t['status'],'complete')
                    self.assertEqual(t['attempts'][0]['quality_check']['status'],'needs_review')

    def test_common_stop_runs_once_and_checker_failure_does_not_raise(self):
        source=ROOT/'data_capture_tools/data_capture_node_10times.py'
        cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='DataCaptureNode')
        methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in ('_stop_rosbag','_check_closed_bag_quality')]
        ns=dict(__package__='data_capture_tools',Path=Path,signal=signal,subprocess=subprocess)
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),ns)
        with tempfile.TemporaryDirectory() as tmp:
            n=types.SimpleNamespace(_bag_uri=Path(tmp)/'bag',_session_dir=Path(tmp),
                                    _bag_process=types.SimpleNamespace(poll=lambda:0),_bag_stdout=None,_bag_stderr=None,
                                    get_logger=lambda:types.SimpleNamespace(info=lambda *a:None,warning=lambda *a:None))
            n._check_closed_bag_quality=types.MethodType(ns['_check_closed_bag_quality'],n)
            with patch.object(qa,'check_and_save',side_effect=OSError('report write failed')) as check:
                ns['_stop_rosbag'](n);ns['_stop_rosbag'](n)
                self.assertEqual(check.call_count,1)
                self.assertEqual(n._last_bag_quality['status'],'not_checked')


if __name__=='__main__':
    unittest.main()
