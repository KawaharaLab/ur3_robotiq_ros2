"""No ROS/robot required: dwell guards, bounded moves, full probe hooks and outcomes."""
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'data_capture_tools'))

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value

probe=module('probe_collection_test',ROOT/'utils/diagnostics/force_stop_probe.py')
old_tests=module('probe_existing_tests',ROOT/'utils/diagnostics/test_force_stop_probe.py')
from data_capture_tools.force_press_worker import Collection, validate_plan, width_to_raw
from data_capture_tools.force_press_collection import ForcePressCollectionMixin


def plan():
    return dict(effective_width_m=.040,clearance_m=.002,max_depth_per_side_m=.004,
                threshold_n=.5,hard_limit_n=1.,hold_seconds=4.,target_confirm_ms=5.,
                close_ticks=20,max_motion_ms=2000,baud=115200,slave=9,
                trial_index=1,port='/dev/ttyUSB0')


class DwellTests(unittest.TestCase):
    def state(self):
        state=probe.ForceState(.5,1.,100.,'max',5.)
        state.offsets={'left':10.,'right':9.}
        self.update(state,0,0.)
        state.armed=True
        return state
    def update(self,state,ms,value,side='left'):
        stamp=1_000_000_000+round(ms*1e6)
        # Real source and receive time both advance; the static offset is large.
        state.update(side,stamp,stamp,stamp,[0,state.offsets[side]+value,0,0,0,0])
        if side=='left':state.update('right',stamp,stamp,stamp,[0,9.,0,0,0,0])
    def test_10N_offset_and_three_sample_noise_do_not_stop(self):
        s=self.state()
        for ms,value in enumerate([0.,.65,.62,.64,0.,0.,0.,0.]):self.update(s,ms,value)
        self.assertIsNone(s.reason)
    def test_sustained_threshold_stops_after_five_ms_on_same_side(self):
        s=self.state()
        for ms in range(1,6):self.update(s,ms,.6)
        self.assertIsNone(s.reason)
        self.update(s,6,.6)
        self.assertEqual(s.reason,'force_threshold')
    def test_other_side_and_repeated_checks_cannot_fake_duration(self):
        s=self.state();self.update(s,1,.6)
        for ms in range(2,9):
            self.update(s,ms,0.,side='right');s.check(1_000_000_000+ms*1_000_000)
        self.assertIsNone(s.reason)
    def test_hysteresis_preserves_real_contact_but_requires_target_at_confirmation(self):
        s=self.state()
        for ms,value in enumerate([.6,.49,.48,.47,.49,.49],start=1):self.update(s,ms,value)
        self.assertIsNone(s.reason)
        self.update(s,7,.51)
        self.assertEqual(s.reason,'force_threshold')
    def test_gap_resets_confirmation_and_hard_limit_remains_immediate(self):
        s=self.state();self.update(s,1,.6);self.update(s,30,.6)
        self.assertIsNone(s.reason)
        self.update(s,31,1.1)
        self.assertEqual(s.reason,'force_hard_limit')
    def test_invalid_and_stale_are_not_delayed_by_confirmation(self):
        s=self.state();self.update(s,1,float('nan'));self.assertEqual(s.reason,'invalid_force_value')
        s=self.state();s.check(1_200_000_000);self.assertEqual(s.reason,'stale_force')


class PlanTests(unittest.TestCase):
    def test_width_caps_match_existing_driver_and_clamp_by_both_limits(self):
        p=plan();start,minimum=validate_plan(p)
        self.assertAlmostEqual(start,.042);self.assertAlmostEqual(minimum,.032)
        c=Collection(p,probe)
        self.assertEqual(c.bound_target(180),min(180,width_to_raw(.032)))
        self.assertEqual(width_to_raw(.040),int(-1399.78*.040**2-1328.11*.040+218.384))
    def test_nan_inverted_force_and_unbounded_plan_are_rejected(self):
        for field,value in [('threshold_n',math.nan),('hard_limit_n',.4),('close_ticks',21),
                            ('max_depth_per_side_m',.020),('clearance_m',0.),('baud',100000),('port','anything')]:
            p=plan();p[field]=value
            with self.assertRaises(ValueError,msg=field):validate_plan(p)


class MoveTests(unittest.TestCase):
    def setup_move(self,lost_ack=False):
        c=Collection(plan(),probe)
        c.check_health=lambda *a:None
        state=types.SimpleNamespace(check=lambda now:None)
        class Port:
            def __init__(self):self.position=150;self.go=0;self.commands=[]
            def status(self):
                return dict(g_act=1,g_sta=3,g_flt=0,g_gto=self.go,g_obj=3,g_po=self.position,g_pr=self.position)
            def command(self,target,speed,force,go):
                self.commands.append((target,go));self.go=int(go)
                if go:
                    self.position=target
                    if lost_ack:raise TimeoutError('lost GO ACK')
        p=Port();events=[]
        event=lambda name,**kw:events.append(name)
        return c,state,p,event,events
    def test_prepare_sends_stop_even_when_go_ack_is_lost(self):
        c,s,p,event,events=self.setup_move(True)
        with self.assertRaises(TimeoutError):c.move(p,p.status,event,s,threading.Event(),155,'prepare',True)
        self.assertEqual(p.commands,[(155,True),(150,False)])
    def test_prepare_has_bounded_target_and_stop_echo(self):
        c,s,p,event,events=self.setup_move()
        c.move(p,p.status,event,s,threading.Event(),155,'prepare',True)
        self.assertEqual(p.commands,[(155,True),(150,False)])
        self.assertIn('prepare_complete',events)
    def test_failure_or_manual_stop_never_automatically_releases(self):
        c,s,p,event,events=self.setup_move()
        for reason in ['force_hard_limit','motion_timeout','position_or_hardware_contact_limit','manual_enter']:
            s.reason=reason
            with self.assertRaises(RuntimeError):c.finish(p,p.status,event,s,threading.Event())
        self.assertEqual(p.commands,[])
    def test_success_releases_only_after_hold_and_marks_completion(self):
        c,s,p,event,events=self.setup_move();s.reason='force_threshold';s.lock=threading.Lock()
        c.finish(p,p.status,event,s,threading.Event())
        self.assertEqual(p.commands[0],(c.start_raw,True))
        self.assertTrue(c.completed);self.assertEqual(events[-1],'collection_complete')
    def test_force_health_rejects_stale_data_even_after_threshold_latched(self):
        c=Collection(plan(),probe);s=probe.ForceState(.5,1.,100.,'max')
        old=time.monotonic_ns()-200_000_000
        for side in ('left','right'):s.update(side,1,old,1,[0.,0.,0.,0.,0.,0.])
        s.reason='force_threshold'
        with self.assertRaisesRegex(RuntimeError,'Stale'):c.check_health(s,threading.Event())


class HookTests(unittest.TestCase):
    def test_live_but_suspended_normal_driver_is_allowed_only_after_service_confirmation(self):
        p=plan();p['normal_driver_suspended']=True
        collection=Collection(p,probe)
        code,result,commands=old_tests.SimulatedRunTests().simulate('collection_suspended',collection=collection)
        self.assertEqual(code,0,result);self.assertTrue(result['collection']['completed'])
        self.assertEqual(commands,[True,False,True,False])
    def test_live_normal_driver_without_service_confirmation_is_rejected(self):
        collection=Collection(plan(),probe)
        code,result,commands=old_tests.SimulatedRunTests().simulate('collection_suspended',collection=collection)
        self.assertEqual(code,2);self.assertEqual(commands,[])
        self.assertIn('Normal gripper status publisher is active',result['error'])
    def test_collection_without_motion_authorization_cannot_prepare(self):
        class Hook:
            def setup(self,*a):pass
            def on_event(self,*a):pass
            def on_status(self,*a):pass
            def prepare(self,*a):raise AssertionError('must not prepare')
            def summary(self):return {}
        case=old_tests.SimulatedRunTests()
        code,result,commands=case.simulate('readonly',collection=Hook())
        self.assertEqual(code,2);self.assertEqual(commands,[])
        self.assertIn('explicit motion authorization',result['error'])
    def test_full_collection_with_simulated_force_and_serial_prepares_stops_holds_releases(self):
        collection=Collection(plan(),probe)
        case=old_tests.SimulatedRunTests()
        code,result,commands=case.simulate('collection',collection=collection)
        self.assertEqual(code,0,result)
        self.assertTrue(result['collection']['prepared'])
        self.assertTrue(result['collection']['completed'])
        self.assertEqual(result['stop_trigger']['reason'],'force_threshold')
        self.assertEqual(result['stop_trigger']['target_confirm_ms'],5.)
        self.assertEqual(commands,[True,False,True,False])
    def test_full_probe_calls_collection_prepare_hold_release_and_saves_metadata(self):
        class Hook:
            def __init__(self):self.events=[];self.prepared=False;self.finished=False
            def setup(self,node):pass
            def on_event(self,row):self.events.append(row['event'])
            def on_status(self,row):pass
            def prepare(self,*a):self.prepared=True
            def bound_target(self,target):return target
            def check_health(self,*a):pass
            def finish(self,*a):self.finished=True
            def summary(self):return dict(completed=self.finished)
        hook=Hook()
        case=old_tests.SimulatedRunTests()
        code,result,commands=case.simulate('force',collection=hook)
        self.assertEqual(code,0,result);self.assertTrue(hook.prepared);self.assertTrue(hook.finished)
        self.assertIn('baseline_start',hook.events);self.assertIn('hold_start',hook.events)
        self.assertTrue(result['collection']['completed']);self.assertEqual(commands,[True,False])


class ManagerTests(unittest.TestCase):
    def test_failed_trial_is_aborted_and_not_retried_with_sidecars_preserved(self):
        class Input(io.StringIO):
            def isatty(self):return True
        class Event:
            def is_set(self):return False
            def wait(self,*a):return False
        class Manager(ForcePressCollectionMixin):
            def __init__(self,root):
                self._effective_object_width_m=.040
                self._force_press_settings={k:v for k,v in plan().items() if k not in ('effective_width_m','trial_index')}
                self._stop_event=Event();self._bag_process=None;self.events=[];self.calls=0
                self._force_press_handoff_active=False
                self._force_press_control_client=types.SimpleNamespace(wait_for_service=lambda **kw:True)
                self.borrowed=0;self.restored=0
                self.config=types.SimpleNamespace(topics=[types.SimpleNamespace(name='/force_press_event')])
                self.root=root;self._session_dir=None
                self._marker_pub=types.SimpleNamespace(publish=lambda msg:None)
            def _borrow_force_press_control(self,base):self.borrowed+=1
            def _restore_force_press_control(self):self.restored+=1;return True
            def _check_sensors_health(self):return True
            def count_publishers(self,*a):return 0
            def _read_positive_int(self,*a):return 3
            def get_logger(self):return types.SimpleNamespace(info=lambda *a:None,error=lambda *a:None,warn=lambda *a:None)
            def _prepare_force_press_session(self):self._session_dir=self.root
            def _start_rosbag(self):self._bag_process=types.SimpleNamespace(poll=lambda:None)
            def _stop_rosbag(self):self._bag_process=None
            def _stop_force_press_worker(self):pass
            def _publish_trial_event(self,event,index,*a):self.events.append((event,index))
            def _run_force_press_worker(self,plan_path,output):
                self.calls+=1;output.mkdir()
                success=self.calls==1
                (output/'summary.json').write_text(json.dumps(dict(error=None if success else 'target not achieved',
                    stop_acknowledged=True,stop_trigger=dict(reason='force_threshold' if success else 'motion_timeout'),
                    collection=dict(completed=success,event_publish_errors=0))))
                return 0 if success else 2
        message=types.ModuleType('std_msgs.msg');message.String=lambda **kw:kw
        with tempfile.TemporaryDirectory() as temp:
            m=Manager(Path(temp))
            with patch.dict('sys.modules',{'std_msgs.msg':message}),patch('sys.stdin',Input('GO\n')):
                m._execute_force_press_loop()
            self.assertEqual(m.calls,2)
            self.assertEqual((m.borrowed,m.restored),(1,1))
            self.assertEqual(m.events,[('START',1),('END',1),('START',2),('ABORT',2)])
            metadata=json.loads((Path(temp)/'force_press_metadata.json').read_text())
            self.assertEqual([r['success'] for r in metadata['actual_condition_order']],[True,False])
            self.assertTrue((Path(temp)/'force_press/trial_0002/summary.json').exists())


if __name__=='__main__':unittest.main()
