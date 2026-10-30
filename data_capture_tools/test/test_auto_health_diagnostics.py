"""Guard ordering tests without ROS; thresholds and fail-closed play unchanged."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
REPO=Path(os.environ.get('ACQUISITION_QA_TEST_REPO',str(ROOT.parent)))
sys.path.insert(0,str(REPO/'data_capture_tools'))


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module;spec.loader.exec_module(module)
    return module


module=load('data_capture_tools.health_diagnostics_under_test',ROOT/'data_capture_tools/object_automation.py')
fixture=load('health_diagnostics_fixture',REPO/'data_capture_tools/test/test_object_automation.py')


class GuardOrderingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.node=fixture.HealthTests().node(Path(self.tmp.name))
        self.disk=patch.object(module.shutil,'disk_usage',return_value=types.SimpleNamespace(free=10*1024**3))
        self.disk.start();self.addCleanup(self.disk.stop)

    def test_samples_arriving_during_manual_disk_checks_are_not_future(self):
        n=self.node;clock={'now':time.time_ns()}
        n.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=clock['now']))
        def arrivals():
            clock['now']+=20_000_000
            for side in ('left','right'):
                n._init_latest['force_'+side]['header_ns']=clock['now']
                n._auto_images[side]['header_ns']=clock['now']
        n._auto_manual=arrivals
        module.ObjectAutomationMixin._auto_health(n)

    def test_inputs_replaced_after_snapshot_do_not_change_this_check(self):
        n=self.node;now=time.time_ns()
        for side in ('left','right'):
            n._init_latest['force_'+side]['header_ns']=now
            n._auto_images[side]['header_ns']=now
        def sample_clock():
            # A callback can replace the latest slot just after now is sampled.
            n._auto_images['right']=dict(n._auto_images['right'],header_ns=now+20_000_000)
            return types.SimpleNamespace(nanoseconds=now)
        n.get_clock=lambda:types.SimpleNamespace(now=sample_clock)
        module.ObjectAutomationMixin._auto_health(n)

    def test_actual_old_force_header_still_stops_with_numeric_diagnostics(self):
        n=self.node;n._init_latest['force_left']['header_ns']-=400_000_000
        with self.assertRaisesRegex(module.Paused,'Invalid force header age left: header_age_ms=.*receive_age_ms=.*allowed_ms='):
            module.ObjectAutomationMixin._auto_health(n)

    def test_actual_future_header_still_stops(self):
        n=self.node;n._init_latest['force_left']['header_ns']+=500_000_000
        with self.assertRaisesRegex(module.Paused,'Invalid force header age left'):
            module.ObjectAutomationMixin._auto_health(n)

    def test_receive_stall_still_stops(self):
        n=self.node;n._init_latest['force_left']['receive_monotonic_ns']-=500_000_000
        with self.assertRaisesRegex(module.Paused,'Stale force left: receive_age_ms='):
            module.ObjectAutomationMixin._auto_health(n)


class DashboardRunner(module.ObjectAutomationMixin):
    def __init__(self,late_running=True,active=True):
        self.clock=0.;self.calls=[];self.late_running=late_running;self.active=active
        self._auto_program=(False,0.)
        self._auto_job={'plan':{'settings':{'auto_play':True}}}
        self._auto_play=types.SimpleNamespace(wait_for_service=lambda **kw:True,
                                              call_async=lambda req:self.calls.append('play') or 'play')
        self._auto_controller_list=types.SimpleNamespace(wait_for_service=lambda **kw:True,
                                              call_async=lambda req:self.calls.append('controllers') or 'controllers')
        self.log=[]
    def _auto_health(self,**kw):
        if self.late_running and self.clock>=.5:self._auto_program=(True,self.clock)
    def _auto_future(self,which,*args,**kw):
        if which=='play':return types.SimpleNamespace(success=False,message='Failed to execute: play')
        return types.SimpleNamespace(controller=[types.SimpleNamespace(name='scaled_joint_trajectory_controller',state='active' if self.active else 'inactive')])
    def get_logger(self):return types.SimpleNamespace(warning=self.log.append,info=self.log.append)


class DashboardTests(unittest.TestCase):
    def run_fake(self,n,call):
        srv=types.ModuleType('std_srvs.srv');srv.Trigger=types.SimpleNamespace(Request=lambda:object())
        ctrl=types.ModuleType('controller_manager_msgs.srv');ctrl.ListControllers=types.SimpleNamespace(Request=lambda:object())
        def sleep(seconds):n.clock+=seconds
        with patch.dict(sys.modules,{'std_srvs':types.ModuleType('std_srvs'),'std_srvs.srv':srv,
                                     'controller_manager_msgs':types.ModuleType('controller_manager_msgs'),'controller_manager_msgs.srv':ctrl}), \
             patch.object(module.time,'monotonic',side_effect=lambda:n.clock), \
             patch.object(module.time,'sleep',side_effect=sleep):
            return call()

    def test_rejected_play_without_readiness_includes_response_and_never_retries(self):
        n=DashboardRunner(late_running=False)
        with self.assertRaisesRegex(module.Paused,'Dashboard play/start readiness failed: Failed to execute: play'):
            self.run_fake(n,n._auto_program_ready)
        self.assertEqual(n.calls.count('play'),1)

    def test_negative_response_followed_by_real_readiness_continues_without_replay(self):
        n=DashboardRunner()
        self.run_fake(n,n._auto_program_ready)
        self.assertEqual(n.calls.count('play'),1)
        self.assertTrue(n._auto_program[0])
        self.assertTrue(any('readiness confirmed' in x for x in n.log))

    def test_running_program_checks_controller_without_play(self):
        n=DashboardRunner();n._auto_program=(True,0.)
        self.run_fake(n,n._auto_program_ready)
        self.assertEqual(n.calls,['controllers'])

    def test_inactive_controller_blocks_init_without_switching(self):
        n=DashboardRunner(active=False)
        with self.assertRaisesRegex(module.Paused,'readiness failed'):
            self.run_fake(n,n._auto_program_ready)
        self.assertEqual(n.calls.count('play'),1)
        self.assertTrue(set(n.calls)<=set(['play','controllers']))

    def test_old_running_notification_cannot_confirm_a_new_play(self):
        n=DashboardRunner(late_running=False);n._auto_program=(True,-1.)
        self.assertFalse(self.run_fake(n,lambda:n._auto_wait_program_controller(program_after=0.)))
        self.assertEqual(n.calls,[])


if __name__=='__main__':unittest.main()
