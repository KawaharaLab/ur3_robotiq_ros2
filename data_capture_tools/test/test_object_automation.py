"""No hardware/ROS required. Schedule, recovery, health and action ownership tests."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from data_capture_tools.object_plan import build_plan,steps_for_trial,recover_progress,validate_saved_plan
from data_capture_tools.object_automation import ObjectAutomationMixin,Paused,GuardPublisher


def settings():
    return dict(depths_mm=[1.,2.,3.],loading_speeds=[.02,.1,.2],repeats=2)


class PlanTests(unittest.TestCase):
    def test_full_factorial_equal_coverage_and_reproducible_order(self):
        a=build_plan(settings());self.assertEqual(a,build_plan(settings()))
        self.assertEqual(sum(len(p['trials']) for p in a['positions']),72)
        self.assertEqual(a['positions'][0]['id'],'base')
        self.assertEqual({(p['lateral_m'],p['vertical_m']) for p in a['positions']},{(0.,0.),(.01,0.),(-.01,0.),(0.,.01)})
        for p in a['positions']:
            pairs=[(r['depth_mm'],r['speed']) for r in p['trials']]
            self.assertEqual(len(set(pairs)),9)
            self.assertTrue(all(pairs.count(x)==2 for x in set(pairs)))
        validate_saved_plan(a)
    def test_optional_ak_aur_gives_104_trials(self):
        s=settings();s['release_speeds']=[.02,.2]
        s['knead_conditions']=[dict(shallow_mm=1.,deep_mm=3.,deep_hold_s=t,shallow_hold_s=t) for t in (.1,.5)]
        p=build_plan(s);self.assertEqual(sum(len(p['trials']) for p in p['positions']),104)
    def test_invalid_inputs_block_before_plan(self):
        for key,val in [('depths_mm',None),('loading_speeds',[]),('depths_mm',[float('nan')]),('loading_speeds',[2.]),('repeats',True),('hard_limit_n',0.),('offset_m',.03),('auto_init','false')]:
            s=settings();s[key]=val
            with self.assertRaises(ValueError):build_plan(s)
    def test_saved_plan_tampering_and_progress_preservation(self):
        p=build_plan(settings());p['positions'][0]['reference']={'init_path':'a'}
        p['positions'][0]['trials'][0]['status']='complete';validate_saved_plan(p)
        p['positions'][0]['trials'][1]['speed']=.9
        with self.assertRaises(ValueError):validate_saved_plan(p)
    def test_crash_recovery_retries_unverified_but_never_repeats_committed_trials(self):
        job={'plan':build_plan(settings())};rows=job['plan']['positions'][0]['trials']
        rows[0]['status']='complete'
        rows[1].update(status='awaiting_bag',attempts=[dict(status='awaiting_bag')])
        rows[2].update(status='running',attempts=[dict(status='running')])
        recover_progress(job)
        self.assertEqual(rows[0]['status'],'complete')
        for row in rows[1:3]:
            self.assertEqual(row['status'],'pending');self.assertEqual(row['attempts'][-1]['status'],'interrupted')
    def test_goal_values_match_legacy_run_and_knead(self):
        row=dict(kind='run',depth_mm=2.,speed=.1,hold_s=4.,release_speed=.2)
        a=steps_for_trial(row,.040)
        self.assertEqual([(round(x['width_m'],6),x['speed'],x['force'],x['dwell_s']) for x in a],[(.140,.3,.1,1.),(.036,.1,.5,4.),(.140,.2,.1,1.)])
        k=dict(kind='knead',shallow_mm=1.,deep_mm=3.,cycles=5,loading_speed=.1,unloading_speed=.2,deep_hold_s=.1,shallow_hold_s=.5)
        a=steps_for_trial(k,.04);self.assertEqual(len(a),13)
        self.assertTrue(all(x['width_m']<.14 for x in a[1:-1]))
        with self.assertRaises(ValueError):steps_for_trial(row,.003)


class FakeRunner(ObjectAutomationMixin):
    def __init__(self,root):
        self.config=NS(output_root=root,raw={});self._auto_job_path=root/'progress.json'
        s=settings();s.update(depths_mm=[1.],loading_speeds=[.1],repeats=2,batch_size=2)
        self._auto_job=dict(job_id='test',plan=build_plan(s));self._auto_reference=None
        self._session_id_override='';self._auto_action=NS(wait_for_server=lambda **k:True)
        self._init_startup=dict(startup_id='driver',cancel_stops_motion=True)
        self._auto_goal=None;self._auto_uncertain_goal=False;self._auto_streams={}
        self._auto_event_pub=None;self._stop_event=threading.Event()
        self.init_calls=0;self.attempt_calls=0;self.fail_once=False;self.bad_bag=False
        self._effective_object_width_m=.04;self._last_successful_init=Path('init-A')
    def get_logger(self):return NS(info=lambda *a:None,warn=lambda *a:None,error=lambda *a:None)
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=time.time_ns()))
    def _auto_await_ready(self):pass
    def _auto_init_position(self,p):self.init_calls+=1;p['reference']=dict(init_path='init-A')
    def _auto_check_reference(self,r):pass
    def _auto_start_bag(self,p):
        self._session_dir=self.config.output_root/('session'+str(self.attempt_calls));self._session_dir.mkdir(exist_ok=True)
        self._auto_recording=True
    def _auto_execute_trial(self,row,attempt):
        self.attempt_calls+=1
        if self.fail_once and self.attempt_calls==2:raise Paused('Stale force left')
        row['status']='awaiting_bag';attempt['status']='awaiting_bag'
    def _stop_rosbag(self):pass
    def _auto_verify_bag(self):
        if self.bad_bag:raise Paused('missing topic')
        return {'checked':True}
    def _publish_trial_event(self,*a):pass


class RecoveryTests(unittest.TestCase):
    def test_sensor_pause_resumes_same_position_without_reinit(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=FakeRunner(Path(tmp));n.fail_once=True;n._auto_run()
            self.assertEqual(n._auto_job['status'],'paused');self.assertEqual(n.init_calls,1)
            first=n._auto_job['plan']['positions'][0]['trials']
            self.assertEqual(first[0]['status'],'complete');self.assertEqual(first[1]['status'],'pending')
            recover_progress(n._auto_job);n._auto_run()
            self.assertEqual(n._auto_job['status'],'complete');self.assertEqual(n.init_calls,4)
            self.assertEqual(len(first[0]['attempts']),1);self.assertEqual(len(first[1]['attempts']),2)
    def test_bag_failure_does_not_commit_trials(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=FakeRunner(Path(tmp));n.bad_bag=True;n._auto_run()
            rows=n._auto_job['plan']['positions'][0]['trials']
            self.assertTrue(all(r['status']=='pending' for r in rows))
            self.assertEqual(rows[0]['attempts'][0]['status'],'recording_unverified')
    def test_publish_guard_prevents_following_init_motion_after_abort(self):
        sent=[];node=NS(_auto_abort_reason=None);p=GuardPublisher(node,NS(publish=sent.append))
        p.publish('first');node._auto_abort_reason='sensor lost'
        with self.assertRaises(Paused):p.publish('next')
        self.assertEqual(sent,['first'])


class HealthTests(unittest.TestCase):
    def node(self,root):
        n=FakeRunner(root);n._auto_job['plan']['settings']['require_timing']=False
        n._auto_robot_mode=7;n._auto_safety_mode=1
        n._auto_armed=True;n._auto_force_fault=None;n._auto_abort_reason=None;n._auto_manual=lambda:None
        n._auto_reference={'joints':[0.]*6,'driver_id':'driver'};n._current_joints=[0.]*6
        n._force_monitor=NS(_offset_left=10.,_offset_right=20.)
        n._init_record_lock=threading.RLock();n._init_latest={};n._auto_images={}
        now=time.monotonic_ns();ros=time.time_ns()
        for side,offset in [('left',10.),('right',20.)]:
            n._init_latest['force_'+side]=dict(receive_monotonic_ns=now,header_ns=ros,wrench=[0.,offset,0,0,0,0])
            n._auto_images[side]=dict(at=now/1e9,header_ns=ros,valid=True)
        for name in ('gripper_status','arm_joint_state'):
            n._init_latest[name]=dict(receive_monotonic_ns=now,status=dict(g_flt=0))
        return n
    def test_raw_force_20N_not_offset_20N_is_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp));n._auto_health()
            n._init_latest['force_right']['wrench'][1]=40.1
            with self.assertRaisesRegex(Paused,'Force hard limit'):n._auto_health()
            n._auto_health(check_force=False)
    def test_stale_nonfinite_image_failure_arm_move_and_driver_restart(self):
        for mode in ('stale','nan','jpeg','arm','driver','safety','robot'):
            with tempfile.TemporaryDirectory() as tmp:
                n=self.node(Path(tmp))
                if mode=='stale':n._init_latest['force_left']['receive_monotonic_ns']-=10**9
                if mode=='nan':n._init_latest['force_left']['wrench'][1]=math.nan
                if mode=='jpeg':n._auto_images['left']['valid']=False
                if mode=='arm':n._current_joints[0]=.1
                if mode=='driver':n._init_startup['startup_id']='new'
                if mode=='safety':n._auto_safety_mode=3
                if mode=='robot':n._auto_robot_mode=5
                with self.assertRaises(Paused):n._auto_health()
    def test_single_force_spike_remains_latched_after_latest_sample_returns_to_normal(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp));n._auto_active=True
            n._auto_note_force('left',[0.,31.,0.,0.,0.,0.],12345)
            n._auto_note_force('left',[0.,10.,0.,0.,0.,0.])
            with self.assertRaisesRegex(Paused,'Force hard limit'):n._auto_health()
            self.assertEqual(n._auto_force_fault_record['header_ns'],12345)
            n._auto_armed=False;n._auto_health(check_force=False)

    def test_stream_restart_requires_manual_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            n=self.node(Path(tmp));n._auto_job['plan']['settings']['require_timing']=True
            names=('mms_left','mms_right','gelsight_left','gelsight_right')
            n._auto_streams={k:('A',time.monotonic()) for k in names};n._auto_expected_streams={k:'A' for k in names}
            n._auto_health();n._auto_streams['mms_left']=('B',time.monotonic())
            with self.assertRaisesRegex(Paused,'restarted'):n._auto_health()
            n._auto_expected_streams={};n._auto_health()


class NativeStopTests(unittest.TestCase):
    def test_actual_stop_packet_and_echo_logic_preserve_activation(self):
        import subprocess
        root=Path(__file__).resolve().parents[2]
        source=(root/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_interfaces/src/default_driver.cpp').read_text()
        method=source[source.index('void DefaultDriver::stop_motion()'):source.index('uint8_t DefaultDriver::get_gripper_position()')]
        cpp=r'''
#include <cassert>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>
constexpr int kActionRequestRegister=0x03e8,kWriteResponseSize=8;
struct DriverException:std::runtime_error{using std::runtime_error::runtime_error;};
enum class ActionStatus {STOPPED,MOVING};
struct DefaultDriver {
 struct {uint8_t g_po=181;} status_registers_;
 uint8_t commanded_gripper_speed_=7,commanded_gripper_force_=9;
 ActionStatus action_status_=ActionStatus::MOVING;
 bool no_ack=false;int reads=0;std::vector<uint16_t> packet;
 std::vector<uint8_t> create_write_command(int,std::vector<uint16_t> data){packet=data;return {};}
 std::vector<uint8_t> send(std::vector<uint8_t>,int){return no_ack?std::vector<uint8_t>{}:std::vector<uint8_t>{1};}
 void read_status(){if(++reads>=2)action_status_=ActionStatus::STOPPED;}
 void stop_motion();
};
'''+method+r'''
int main(){DefaultDriver d;d.stop_motion();assert(d.packet.size()==3);
 assert(d.packet[0]==0x0100 && d.packet[1]==181 && d.packet[2]==0x0709);
 assert(d.reads==2);DefaultDriver lost;lost.no_ack=true;bool failed=false;
 try{lost.stop_motion();}catch(const DriverException&){failed=true;}
 assert(failed && lost.reads==0);}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'stop.cpp';p.write_text(cpp);binary=Path(tmp)/'stop'
            subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror',str(p),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
    def test_actual_sequencer_stop_cancels_both_clients_before_busy_filter(self):
        import subprocess
        root=Path(__file__).resolve().parents[2]
        source=(root/'gripper_client/src/gripper_client_node.cpp').read_text()
        start=source.index('        if (msg->data == "stop")');end=source.index('        if (action_in_progress_)',start)
        branch=source[start:end]
        self.assertEqual(source.count('if (generation != sequence_generation_) return;'),3)
        cpp=r'''
#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
namespace std_msgs {namespace msg {struct Int32{int data;};}}
struct Client{int cancellations=0;void async_cancel_all_goals(){++cancellations;}};
struct Timer{bool canceled=false;void cancel(){canceled=true;}};
struct Pub{int phase=0;void publish(std_msgs::msg::Int32 m){phase=m.data;}};
struct Msg{std::string data;};
struct Node{uint64_t sequence_generation_=0;bool action_in_progress_=true;
 std::shared_ptr<Timer> timer_=std::make_shared<Timer>();
 std::shared_ptr<Client> arm_client_=std::make_shared<Client>(),gripper_client_=std::make_shared<Client>();
 std::shared_ptr<Pub> phase_pub_=std::make_shared<Pub>();
 void command(std::shared_ptr<Msg> msg){
'''+branch+r'''
 }
};
int main(){Node n;n.command(std::make_shared<Msg>(Msg{"stop"}));
 assert(n.sequence_generation_==1 && !n.action_in_progress_ && n.timer_->canceled);
 assert(n.arm_client_->cancellations==1 && n.gripper_client_->cancellations==1 && n.phase_pub_->phase==-1);}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'sequence.cpp';p.write_text(cpp);binary=Path(tmp)/'sequence'
            subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror',str(p),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)


class ActionOwnershipTests(unittest.TestCase):
    def node(self,handle):
        from concurrent.futures import Future
        n=ObjectAutomationMixin();n._auto_goal=None;n._auto_uncertain_goal=False;n._auto_armed=False
        n._auto_trial_index=1;n._auto_health=lambda **kw:None;n._auto_event=lambda *args:None
        n.get_logger=lambda:NS(error=lambda *args:None)
        n._auto_action=NS(send_goal_async=lambda goal:handle)
        n._auto_action_type=NS(Goal=lambda:NS())
        return n
    def completed(self,value):
        from concurrent.futures import Future
        f=Future();f.set_result(value);return f
    def test_success_and_rejection(self):
        terminal=self.completed(NS(status=4,result=NS(success=True)))
        h=NS(accepted=True,get_result_async=lambda:terminal)
        n=self.node(self.completed(h));n._auto_grip(.04,.1,.5,2);self.assertIsNone(n._auto_goal)
        n=self.node(self.completed(NS(accepted=False)))
        with self.assertRaisesRegex(Paused,'rejected'):n._auto_grip(.04,.1,.5,2)
    def test_late_accepted_goal_is_canceled_and_unknown_state_only_clears_on_confirmed_stop(self):
        from concurrent.futures import Future
        for status in (5,6):
            pending=Future();n=self.node(pending);original=n._auto_future
            def wait(future,*args,**kwargs):
                if future is pending:raise Paused('sensor lost while awaiting acceptance')
                return original(future,*args,**kwargs)
            n._auto_future=wait
            with self.assertRaises(Paused):n._auto_grip(.04,.1,.5,2)
            self.assertTrue(n._auto_uncertain_goal)
            canceled=[]
            h=NS(accepted=True,cancel_goal_async=lambda:canceled.append(True),
                 get_result_async=lambda:self.completed(NS(status=status,result=NS(success=False))))
            pending.set_result(h)
            self.assertEqual(canceled,[True]);self.assertEqual(n._auto_uncertain_goal,status!=5)
    def test_cancel_acceptance_alone_is_not_a_stop_confirmation(self):
        n=self.node(None)
        h=NS(cancel_goal_async=lambda:self.completed(NS()),
             get_result_async=lambda:self.completed(NS(status=6,result=NS(success=False))))
        n._auto_goal=h
        self.assertFalse(n._auto_cancel());self.assertTrue(n._auto_uncertain_goal)


if __name__=='__main__':unittest.main()
