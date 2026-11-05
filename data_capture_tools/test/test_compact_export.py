"""Read-only miniature SQLite bags: exact JPEGs, event matching and failure reports."""
from contextlib import contextmanager
import csv
import io
import json
from pathlib import Path
import sqlite3
import struct
import tempfile
import unittest
from unittest.mock import patch
from data_capture_tools import compact_export as module

class Encoder:
    def __init__(self):self.data=bytearray()
    def number(self,fmt,align,value):
        self.data.extend(b'\0'*((-len(self.data))%align));self.data.extend(struct.pack('<'+fmt,value));return self
    def string(self,value):
        b=value.encode()+b'\0';self.number('I',4,len(b));self.data.extend(b);return self
    def header(self,ns):return self.number('i',4,ns//10**9).number('I',4,ns%10**9).string('frame')
    def finish(self):return b'\x00\x01\x00\x00'+self.data

@contextmanager
def fake_databases(bag):yield iter([Path(bag)/'source.db3'])

class CompactTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.session=self.root/'source_session';self.bag=self.session/'bag/session_test';self.bag.mkdir(parents=True)
        (self.bag/'metadata.yaml').write_text('synthetic metadata')
        self.connection=sqlite3.connect(self.bag/'source.db3')
        self.connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        topics=[(module.EVENT,'robotiq_2f_gripper_msgs/msg/TrialEvent')]+[(t,'sensor_msgs/msg/CompressedImage') for t in module.GS]+[(t,'geometry_msgs/msg/WrenchStamped') for t in module.FORCE]
        for i,(t,typ) in enumerate(topics,1):self.connection.execute('INSERT INTO topics VALUES(?,?,?)',(i,t,typ))
        self.ids={t:i for i,(t,_) in enumerate(topics,1)};self.jpeg=b'\xff\xd8exact-original-jpeg-test-bytes\xff\xd9'
        self.event(100,'START');self.event(150,'ABORT');self.event(200,'START');self.event(400,'END')
        for ns in [120,210,220,230,410]:
            for topic in module.GS:
                e=Encoder().header(ns).string('jpeg').number('I',4,len(self.jpeg));e.data.extend(self.jpeg);self.message(topic,ns,e.finish())
            for topic in module.FORCE:
                e=Encoder().header(ns)
                for v in [0.,31. if ns==220 else -1.,0.,0.,0.,0.]:e.number('d',8,v)
                self.message(topic,ns,e.finish())
        self.connection.commit();self.connection.close()
        a=dict(status='complete',session=str(self.session),start_ros_ns=190,end_ros_ns=450,tare={'left':.5,'right':2.})
        self.row=dict(id='base_0001',kind='run',depth_mm=4.,speed=0.,status='complete',attempts=[dict(a,status='aborted',start_ros_ns=90,end_ros_ns=160),a])
        self.job=dict(job_id='test',plan={'positions':[{'id':'base','trials':[self.row]}]})
        self.selected=module.selected_attempts(self.job)[str(self.session.resolve())]
        self.progress=self.root/'job.json';self.progress.write_text(json.dumps(self.job))
        p=patch.object(module,'databases',fake_databases);p.start();self.addCleanup(p.stop)
    def message(self,topic,ns,payload):self.connection.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(self.ids[topic],ns+1000,payload))
    def event(self,ns,event):self.message(module.EVENT,ns,Encoder().header(ns).string(event).number('i',4,1).string('auto_depth_speed').finish())
    def test_preview_no_images_and_exact_counts(self):
        out=self.root/'preview'
        self.assertEqual(module.main(['--progress',str(self.progress),'--out',str(out)]),0)
        self.assertEqual(sorted(p.name for p in out.iterdir()),['report.json','source_progress.json'])
        r=json.loads((out/'report.json').read_text());self.assertEqual(r['original_jpeg_bytes'],6*len(self.jpeg))
    def test_jpeg_bytes_exact_and_full_rate_force_spike_is_retained(self):
        out=self.root/'write';r=module.export_session(self.session,self.selected,out,True)
        self.assertEqual(r['status'],'complete');self.assertEqual(len(list(out.rglob('*.jpg'))),6)
        self.assertTrue(all(p.read_bytes()==self.jpeg for p in out.rglob('*.jpg')))
        with (out/'force_right.csv').open() as f:rows=list(csv.DictReader(f))
        self.assertEqual(len(rows),3);self.assertEqual([int(r['source_stamp_ns']) for r in rows],[210,220,230])
        self.assertEqual(float(rows[1]['baseline_relative_Fy']),29.)
        self.assertTrue(all(r['trial_id']=='base_0001' for r in rows))
        with (out/'image_index.csv').open() as f:images=list(csv.DictReader(f))
        self.assertEqual(int(images[0]['bag_receive_ns'])-int(images[0]['source_stamp_ns']),1000)
    def test_preview_csv_byte_estimate_matches_export(self):
        preview=module.export_session(self.session,self.selected,self.root/'unused',False)
        written=module.export_session(self.session,self.selected,self.root/'written',True)
        self.assertEqual(preview['csv_bytes'],written['csv_bytes'])
    def test_no_source_changes_and_no_overwrite(self):
        before=module.fingerprint(self.bag/'source.db3');out=self.root/'write'
        module.export_session(self.session,self.selected,out,True)
        self.assertEqual(before,module.fingerprint(self.bag/'source.db3'))
        with self.assertRaises(FileExistsError):module.export_session(self.session,self.selected,out,True)
    def test_missing_end_is_not_silently_accepted(self):
        con=sqlite3.connect(self.bag/'source.db3');con.execute('DELETE FROM messages WHERE topic_id=1 AND timestamp=1400');con.commit();con.close()
        out=self.root/'bad';self.assertEqual(module.main(['--progress',str(self.progress),'--out',str(out),'--write']),1)
        self.assertIn('No unique START/END',json.loads((out/'report.json').read_text())['sessions'][0]['error'])
    def test_corrupt_database_reports_failure_not_complete(self):
        (self.bag/'source.db3').write_bytes(b'invalid sqlite')
        out=self.root/'bad';self.assertEqual(module.main(['--progress',str(self.progress),'--out',str(out)]),1)
        self.assertEqual(json.loads((out/'report.json').read_text())['status'],'incomplete')
    def test_missing_force_topic_marks_session_incomplete(self):
        con=sqlite3.connect(self.bag/'source.db3');con.execute('DELETE FROM messages WHERE topic_id=?',(self.ids[module.FORCE[1]],));con.commit();con.close()
        r=module.export_session(self.session,self.selected,self.root/'none',False)
        self.assertEqual(r['status'],'incomplete');self.assertEqual(r['missing_trial_topics'][0]['topic'],module.FORCE[1])
    def test_non_jpeg_does_not_get_transcoded(self):
        e=Encoder().header(210).string('png').number('I',4,4);e.data.extend(b'PNG!')
        con=sqlite3.connect(self.bag/'source.db3');con.execute('UPDATE messages SET data=? WHERE topic_id=2 AND timestamp=1210',(e.finish(),));con.commit();con.close()
        with self.assertRaisesRegex(ValueError,'Expected JPEG'):module.export_session(self.session,self.selected,self.root/'bad',False)
    def test_duplicate_headers_keep_distinct_messages(self):
        con=sqlite3.connect(self.bag/'source.db3')
        con.execute('INSERT INTO messages(topic_id,timestamp,data) SELECT topic_id,timestamp,data FROM messages WHERE topic_id=2 AND timestamp=1210')
        con.commit();con.close()
        out=self.root/'duplicates';module.export_session(self.session,self.selected,out,True)
        self.assertEqual(len(list((out/'images/left').glob('*.jpg'))),4)
    def test_compressed_failure_does_not_export_a_readable_prefix(self):
        @contextmanager
        def truncated(bag):
            def iterate():
                yield self.bag/'source.db3'
                raise RuntimeError('zstd: premature end')
            yield iterate()
        with patch.object(module,'databases',truncated):
            out=self.root/'truncated'
            self.assertEqual(module.main(['--progress',str(self.progress),'--out',str(out),'--write']),1)
        self.assertFalse(list(out.rglob('*.jpg')))
        r=json.loads((out/'report.json').read_text())
        self.assertIn('premature end',r['sessions'][0]['error'])
        self.assertEqual(r['sessions'][0]['affected_trials'],['base_0001'])
    def test_sidecar_preservation(self):
        folder=self.session/'gelsight_acquisition_records';folder.mkdir();(folder/'left.jsonl').write_text('original timing\n')
        r=module.export_session(self.session,self.selected,self.root/'copied',True)
        self.assertEqual((self.root/'copied/session_records/gelsight_acquisition_records/left.jsonl').read_text(),'original timing\n')
        self.assertEqual(r['session_sidecar_bytes'],len('original timing\n'))

if __name__=='__main__':unittest.main()
