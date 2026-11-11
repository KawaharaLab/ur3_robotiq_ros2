import importlib.util
import json
from pathlib import Path
import sqlite3
import struct
import sys
import tempfile
import unittest
from collections import Counter
from contextlib import redirect_stdout
import io
import os
try:
    import cv2
    import numpy as np
    import yaml
    HAS_IMAGE_DEPENDENCIES = True
except ImportError:
    HAS_IMAGE_DEPENDENCIES = False

repo = Path(os.environ.get('OBJECT_ACQUISITION_CHECK_REPO', str(Path(__file__).resolve().parents[2])))
sys.path[:0] = [str(repo/'data_capture_tools'), str(repo/'utils/diagnostics')]
path = Path(os.environ.get('OBJECT_ACQUISITION_CHECK_SCRIPT',
    str(repo/'utils/diagnostics/check_object_acquisition.py')))
spec = importlib.util.spec_from_file_location('strict_checker', path)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)
from data_capture_tools.object_plan import build_plan


class Encoder:
    def __init__(self): self.data = bytearray()
    def number(self, fmt, value, align=4):
        self.data.extend(b'\0' * ((-len(self.data)) % align))
        self.data.extend(struct.pack('<'+fmt, value))
    def string(self, value):
        raw = value.encode()+b'\0'
        self.number('I', len(raw)); self.data.extend(raw)
    def header(self, h):
        self.number('i', h//10**9); self.number('I', h%10**9); self.string('sensor')
    def result(self): return b'\0\1\0\0' + self.data


def event(h, name, action=False):
    e=Encoder();e.header(h);e.string(name);e.number('i',1)
    if action: e.number('i',1)
    else: e.string('auto_depth_speed')
    return e.result()


def string_record(record):
    e=Encoder();e.string(json.dumps(record));return e.result()


def fixture(root, change=None):
    session=root/'session';bag=session/'bag'/'bag1';bag.mkdir(parents=True)
    rows=[];local={};start=1_000_000_000
    types=dict(zip(checker.qa.SENSORS,checker.qa.TYPES))
    types.update({t:'std_msgs/msg/String' for t in checker.qa.TIMING})
    types[checker.qa.TRIAL]='robotiq_2f_gripper_msgs/msg/TrialEvent'
    types[checker.qa.ACTION]='robotiq_2f_gripper_msgs/msg/RobotActionEvent'
    jpeg=cv2.imencode('.jpg',np.zeros((8,8,3),np.uint8))[1].tobytes()
    for side in range(2):
        local[side]=[]
        for seq in range(40):
            h=start+seq*53_333_333
            e=Encoder();e.header(h);e.string('jpeg');e.number('I',len(jpeg));e.data.extend(jpeg)
            rows.append([checker.qa.SENSORS[side],h+100_000,e.result()])
            r=dict(schema_version=1,kind='gelsight_frame',stream_id='cam'+str(side),frame_seq=seq,
                   header_ns=h,record_publish_errors_total=0,backend='V4L2',opencv_version='test',
                   clock_type='ROS_TIME', stages=dict(backend_pos_msec=None,
                   read_start=[h-1000]*3,read_end=[h-500]*3,preprocess_end=[h]*3))
            rows.append([checker.qa.TIMING[side],h+200_000,string_record(r)])
            local[side].append(r)
        for seq in range(2100):
            h=start+seq*1_000_000
            e=Encoder();e.header(h)
            for _ in range(6):e.number('d',0.,8)
            rows.append([checker.qa.SENSORS[side+2],h+100_000,e.result()])
            r=dict(schema_version=1,kind='mms101_reads',stream_id='ft'+str(side),batch_seq=seq,
                dropped_reads_total=0,dropped_samples_total=0,failed_publish_reads_total=0,
                failed_publish_samples_total=0,force_scale=.001,torque_scale=.000001,
                reads=[dict(read_seq=seq,byte_offset=seq*25,bytes_hex=(b'\0\x17'+b'\0'*23).hex(),
                    read_start=[h-1000]*3,read_end=[h-500]*3,samples=[[seq,h,seq*25]])])
            rows.append([checker.qa.TIMING[side+2],h+200_000,string_record(r)])
    lo,hi=1_100_000_000,2_900_000_000
    rows.extend([[checker.qa.TRIAL,lo,event(lo,'START')],[checker.qa.TRIAL,hi,event(hi,'END')]])
    # The preparation action is outside the START/END trial window.
    for n in range(2):
        for j,name in enumerate(['auto_goal_sent','auto_action_succeeded','auto_dwell_start','auto_dwell_end']):
            h=lo+10_000_000+n*100_000_000+j*10_000_000
            rows.append([checker.qa.ACTION,h,event(h,name,True)])
    if change:rows=change(rows)
    connection=sqlite3.connect(bag/'test.db3')
    connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);'
        'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    ids={t:i for i,t in enumerate(types,1)}
    connection.executemany('INSERT INTO topics VALUES(?,?,?)',[(ids[t],t,v) for t,v in types.items()])
    connection.executemany('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',
        [(ids[t],h,bytes(p)) for t,h,p in rows]);connection.commit();connection.close()
    counts=Counter(t for t,h,p in rows)
    metadata=dict(storage_identifier='sqlite3',relative_file_paths=['test.db3'],message_count=len(rows),
        topics_with_message_count=[dict(topic_metadata=dict(name=t,type=types[t]),message_count=counts[t]) for t in types])
    (bag/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=metadata)))
    sidecar=session/'gelsight_acquisition_records';sidecar.mkdir()
    for side,records in local.items():
        path=sidecar/f'camera{side}.jsonl'
        with path.open('w') as out:
            out.write(json.dumps(dict(metadata=dict(image_topic=checker.qa.SENSORS[side].removesuffix('/compressed'),stream_id='cam'+str(side))))+'\n')
            for r in records:
                out.write(json.dumps(dict(kind='frame',record=r,enqueue_clock=[r['header_ns']+100_000]*3))+'\n')
        path.with_suffix('.status.json').write_text(json.dumps(dict(clean_close=True,error=None,
            dropped_records=0,rejected_records=0,pending_records=0,
            accepted_records=len(records),written_records=len(records),flushed_records=len(records))))
    (session/'collection_provenance.json').write_text(json.dumps(dict(init_id='init1',effective_config=dict(x=1))))
    (session/'init_reference.json').write_text(json.dumps(dict(init_id='init1',available=True,summary='init.json',samples='samples.json')))
    (session/'init.json').write_text('{}');(session/'samples.json').write_text('{}')
    (session/'trial_conditions.jsonl').write_text(json.dumps(dict(init_id='init1'))+'\n')
    plan=build_plan(dict(depths_mm=[2.],loading_speeds=[0.],positions=['base'],repeats=1))
    row=plan['positions'][0]['trials'][0]
    row.update(status='complete',attempts=[dict(status='complete',session=str(session),init_id='init1',
        start_ros_ns=lo-1,end_ros_ns=hi+1,tare=dict(left=0.,right=0.),
        sensor_stream_ids=dict(gelsight_left='cam0',gelsight_right='cam1',mms_left='ft0',mms_right='ft1'))])
    job=dict(status='complete',job_id='test',plan=plan)
    progress=root/'progress.json';progress.write_text(json.dumps(job))
    return progress,session


@unittest.skipUnless(HAS_IMAGE_DEPENDENCIES, 'OpenCV, NumPy and PyYAML required')
class StrictTests(unittest.TestCase):
    def run_case(self,change=None,post=None,skip=False):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);p,s=fixture(root,change)
            if post:post(p,s)
            before={str(f):f.read_bytes() for f in root.rglob('*') if f.is_file()}
            args=['--progress',str(p),'--out',str(root/'result')]
            if skip:args+=['--skip-image-decode']
            with redirect_stdout(io.StringIO()):code=checker.main(args)
            summary=json.loads((root/'result/summary.json').read_text())
            for f,b in before.items():self.assertEqual(Path(f).read_bytes(),b)
            return code,summary
    def test_clean_pass(self):
        code,r=self.run_case();self.assertEqual(code,0,r);self.assertEqual(r['status'],'PASS')
    def test_missing_in_window_action_is_review(self):
        def change(rows):
            removed = False
            result = []
            for row in rows:
                if row[0] == checker.qa.ACTION and not removed:
                    removed = True
                    continue
                result.append(row)
            return result
        code,r=self.run_case(change)
        self.assertEqual(code,1,r)
        self.assertTrue(any(i['code']=='planned_action_count_mismatch' for i in r['sessions'][0]['issues']))
    def test_local_repairs_missing_and_delayed_timing(self):
        def change(rows):return [[t,h+(5_000_000_000 if t==checker.qa.TIMING[0] else 0),p]
            for t,h,p in rows if not (t==checker.qa.TIMING[1] and h==1_533_333_330+200_000)]
        code,r=self.run_case(change);self.assertEqual(code,0,r);self.assertEqual(r['status'],'PASS_WITH_NOTES')
    def test_raw_packet_repairs_missing_wrench(self):
        code,r=self.run_case(lambda rows:[row for row in rows if not (row[0]==checker.qa.SENSORS[2] and row[1]==1_500_100_000)])
        self.assertEqual(code,0,r);self.assertEqual(r['status'],'PASS_WITH_NOTES')
    def test_unrecoverable_gap_review(self):
        code,r=self.run_case(lambda rows:[row for row in rows if not (row[0] in (checker.qa.SENSORS[2],checker.qa.TIMING[2]) and 1_500_000_000<row[1]<1_510_000_000)])
        self.assertEqual(code,1,r)
    def test_one_irrecoverable_sample_is_review(self):
        code,r=self.run_case(lambda rows:[row for row in rows if not (row[0] in (checker.qa.SENSORS[2],checker.qa.TIMING[2]) and row[1] in (1_500_100_000,1_500_200_000))])
        self.assertEqual(code,1,r)
    def test_large_wrench_gap_fully_recoverable_is_note(self):
        code,r=self.run_case(lambda rows:[row for row in rows if not (row[0]==checker.qa.SENSORS[2] and 1_500_000_000<row[1]<1_530_000_000)])
        self.assertEqual(code,0,r)
        self.assertEqual(r['status'],'PASS_WITH_NOTES')
    def test_consistent_sensor_spike_is_review(self):
        def change(rows):
            for row in rows:
                if row[0]==checker.qa.SENSORS[2] and row[1]==1_500_100_000:
                    e=Encoder();e.header(1_500_000_000)
                    for v in (0.,3.,0.,0.,0.,0.):e.number('d',v,8)
                    row[2]=e.result()
                if row[0]==checker.qa.TIMING[2] and row[1]==1_500_200_000:
                    record=json.loads(checker.qa.CDR(row[2]).string())
                    raw=bytearray.fromhex(record['reads'][0]['bytes_hex']);raw[7:10]=(3000).to_bytes(3,'big',signed=True)
                    record['reads'][0]['bytes_hex']=raw.hex();row[2]=string_record(record)
            return rows
        code,r=self.run_case(change)
        self.assertEqual(code,1,r)
        self.assertTrue(any(i['code']=='isolated_force_spike' for i in r['sessions'][0]['issues']))
    def test_corrupt_jpeg_fail(self):
        def change(rows):
            for row in rows:
                if row[0]==checker.qa.SENSORS[0]:row[2]=row[2][:-1]+b'!';break
            return rows
        code,r=self.run_case(change);self.assertEqual(code,2,r)
    def test_raw_value_mismatch_fail(self):
        def change(rows):
            for row in rows:
                if row[0]==checker.qa.SENSORS[2] and row[1]==1_500_100_000:
                    row[2]=row[2][:-8]+struct.pack('<d',1.)
            return rows
        code,r=self.run_case(change);self.assertEqual(code,2,r)
    def test_skip_decode_cannot_pass(self):
        code,r=self.run_case(skip=True);self.assertEqual(code,1,r)
    def test_metadata_mismatch_fail(self):
        def post(p,s):
            path=s/'bag/bag1/metadata.yaml';v=yaml.safe_load(path.read_text());v['rosbag2_bagfile_information']['message_count']+=1;path.write_text(yaml.safe_dump(v))
        code,r=self.run_case(post=post);self.assertEqual(code,2,r)
    def test_missing_session_fail(self):
        def post(p,s):(s/'bag/bag1/metadata.yaml').unlink()
        code,r=self.run_case(post=post);self.assertEqual(code,2,r)
    def test_missing_reference_review(self):
        code,r=self.run_case(post=lambda p,s:(s/'init_reference.json').unlink());self.assertEqual(code,1,r)
    def test_missing_local_and_bag_timing_review(self):
        def change(rows):return [row for row in rows if row[0]!=checker.qa.TIMING[0]]
        def post(p,s):
            for f in (s/'gelsight_acquisition_records').glob('camera0.*'):f.unlink()
        code,r=self.run_case(change,post);self.assertEqual(code,1,r)


if __name__=='__main__':unittest.main()
