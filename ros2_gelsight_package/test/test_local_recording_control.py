"""Recording leases must gate only optional files, never camera acquisition."""
from pathlib import Path
import tempfile
import unittest
from ros2_gelsight_package.local_recording_control import RecordingLeaseController


class FakeWriter:
    def __init__(self, directory, metadata, **kw):
        self.directory,self.metadata=directory,metadata;self.closed=False;self.close_calls=[]
    def snapshot(self):return dict(accepting=not self.closed,closed=self.closed,error=None,close_timed_out=False,path=self.directory+'/test.jsonl')
    def close(self, **kw):self.closed=True;self.close_calls.append(kw);return self.snapshot()


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.writers=[];self.errors=[]
        def factory(*a,**kw):
            w=FakeWriter(*a,**kw);self.writers.append(w);return w
        self.controller=RecordingLeaseController(lambda:dict(node='left',stream_id='stream'),
            on_error=self.errors.append,writer_factory=factory,clock=lambda:self.now,hostname='workstation')
        self.request=dict(schema_version=1,hostname='workstation',owner_id='owner',session_id='session',
            operation='start',sent_monotonic_ns=100_000_000_000,lease_seconds=10.,directory='/tmp/session/local')

    def test_standby_creates_no_writer_and_start_creates_one(self):
        self.controller.tick();self.assertIsNone(self.controller.writer());self.assertEqual(self.writers,[])
        s=self.controller.handle(self.request)
        self.assertTrue(s['active']);self.assertIs(self.controller.writer(),self.writers[0])
        self.assertEqual(self.writers[0].metadata['recording_session_id'],'session')

    def test_heartbeat_renews_without_reopening(self):
        self.controller.handle(self.request);self.now=108.
        self.controller.handle(dict(self.request,sent_monotonic_ns=108_000_000_000))
        self.now=111.;self.controller.tick()
        self.assertIsNotNone(self.controller.writer());self.assertEqual(len(self.writers),1)
        self.now=118.;self.controller.tick();self.assertIsNone(self.controller.writer())

    def test_capture_gate_expires_even_before_timer_runs(self):
        self.controller.handle(self.request);self.now=110.1
        self.assertIsNone(self.controller.writer());self.assertFalse(self.writers[0].closed)
        self.controller.tick();self.assertTrue(self.writers[0].closed)
        self.assertEqual(self.controller.status()['last_closed']['reason'],'lease_expired')

    def test_stop_and_delayed_heartbeat_cannot_restart_session(self):
        self.controller.handle(self.request)
        self.controller.handle(dict(self.request,operation='stop'))
        self.controller.handle(self.request)
        self.assertIsNone(self.controller.writer());self.assertEqual(len(self.writers),1)
        self.assertTrue(self.writers[0].close_calls[0]['capture_source_detached'])

    def test_expired_start_does_not_open_file(self):
        self.now=111.;self.controller.handle(self.request)
        self.assertEqual(self.writers,[])

    def test_other_owner_cannot_stop_or_replace_current_writer(self):
        self.controller.handle(self.request)
        self.controller.handle(dict(self.request,owner_id='other',operation='stop'))
        self.assertIsNotNone(self.controller.writer())
        self.controller.handle(dict(self.request,owner_id='other',session_id='other'))
        self.assertIsNotNone(self.controller.writer());self.assertEqual(len(self.writers),1)
        self.assertTrue(self.errors)

    def test_directory_change_rejected_but_current_recording_kept(self):
        self.controller.handle(self.request)
        self.controller.handle(dict(self.request,directory='/tmp/changed'))
        self.assertEqual(len(self.writers),1);self.assertEqual(self.controller.status()['directory'],self.request['directory'])

    def test_bad_host_relative_path_and_future_clock_do_not_open_files(self):
        for change in (dict(hostname='other'),dict(directory='relative'),dict(sent_monotonic_ns=200_000_000_000),dict(lease_seconds=100)):
            with self.subTest(change=change):
                self.controller.handle(dict(self.request,**change));self.assertEqual(self.writers,[])

    def test_new_session_after_stop_is_allowed(self):
        self.controller.handle(self.request);self.controller.handle(dict(self.request,operation='stop'))
        self.controller.handle(dict(self.request,session_id='new',directory='/tmp/session2/local'))
        self.assertEqual(len(self.writers),2);self.assertTrue(self.controller.status()['active'])

    def test_disk_failure_reported_without_throwing(self):
        def fail(*a,**kw):raise OSError('unwritable destination')
        self.controller._factory=fail
        s=self.controller.handle(self.request)
        self.assertFalse(s['active']);self.assertIn('unwritable',s['error'])

    def test_new_start_waits_for_stuck_previous_writer(self):
        self.controller.handle(self.request);old=self.writers[0]
        old.close=lambda **kw:dict(accepting=False,closed=False,error=None,close_timed_out=True)
        old.snapshot=lambda:dict(accepting=False,closed=False,error=None,close_timed_out=True)
        self.controller.handle(dict(self.request,operation='stop'))
        s=self.controller.handle(dict(self.request,session_id='new',directory='/tmp/session2/local'))
        self.assertFalse(s['active']);self.assertEqual(len(self.writers),1)

    def test_real_files_are_only_created_during_lease(self):
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'session'/'local'
            controller=RecordingLeaseController(lambda:dict(node='left',stream_id='real',image_topic='gelsight/left/image_raw'),clock=lambda:self.now,hostname='workstation')
            self.assertFalse(target.exists())
            controller.handle(dict(self.request,directory=str(target)))
            w=controller.writer();w.submit('frame','{"header_ns":123}',[1,2,3])
            controller.handle(dict(self.request,operation='stop'))
            s=controller.status()['last_closed']['writer']
            self.assertTrue(s['closed']);self.assertTrue(s['capture_source_detached'])
            self.assertEqual(s['written_records'],1);self.assertFalse(s['capture_thread_stopped'])
            size=Path(s['path']).stat().st_size
            controller.tick();self.assertEqual(Path(s['path']).stat().st_size,size)


if __name__=='__main__':unittest.main()
