"""No robot required: force guards, Modbus wire bytes/CRC, and simulated full runs."""
import importlib.util
import io
import json
import os
from pathlib import Path
import pty
import select
import struct
import tempfile
import threading
import time
import tty
import types
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('probe',Path(__file__).with_name('force_stop_probe.py'))
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


class GuardTests(unittest.TestCase):
    def state(self,metric='max'):
        s=probe.ForceState(.5,1.,100.,metric);s.offsets={'left':1.,'right':2.}
        s.update('left',1_000_000_000,10,1_000_000_000,[0,1.,0,0,0,0])
        s.update('right',1_000_000_000,10,1_000_000_000,[0,2.,0,0,0,0]);s.armed=True
        return s
    def test_raw_threshold_is_latched_without_20_sample_average(self):
        s=self.state();s.update('left',1_000_000_000,11,1_000_000_000,[0,1.6,0,0,0,0])
        self.assertEqual(s.reason,'force_threshold')
        s.update('left',1_000_000_000,12,1_000_000_000,[0,1.,0,0,0,0])
        self.assertEqual(s.reason,'force_threshold')
    def test_hard_limit_on_either_side_overrides_mean(self):
        s=self.state('mean');s.update('left',1_000_000_000,11,1_000_000_000,[0,2.1,0,0,0,0])
        self.assertEqual(s.reason,'force_hard_limit')
    def test_stale_and_backlogged_samples_stop(self):
        s=self.state();self.assertEqual(s.check(100_000_011),'stale_force')
        s=self.state();s.update('left',1,11,1_000_000_000,[0,1.,0,0,0,0])
        self.assertEqual(s.reason,'invalid_force_timestamp')
    def test_invalid_force_and_missing_side_stop(self):
        s=self.state();s.update('left',1_000_000_000,11,1_000_000_000,[0,float('nan'),0,0,0,0])
        self.assertEqual(s.reason,'invalid_force_value')
        self.assertIsNone(probe.json_safe(s.logs[-1])['wrench'][1])
        s=self.state();s.latest.pop('right');self.assertEqual(s.check(11),'missing_force')
    def test_manual_stop_from_same_thread_lock_is_not_deadlocked(self):
        s=self.state()
        with s.lock:s.request_stop('manual_signal')
        self.assertEqual(s.reason,'manual_signal')


class HandoffTimeoutTests(unittest.TestCase):
    def status(self, fault=0):
        return dict(g_act=1, g_sta=3, g_flt=fault, g_gto=0, g_obj=0, g_po=3)
    def test_only_transient_09_is_read_until_clear(self):
        events=[];reads=[]
        def read():
            reads.append(1)
            return self.status(9 if len(reads)==1 else 0)
        value=probe.recover_handoff_timeout(self.status(9),read,lambda *a,**kw:events.append(a[0]),lambda:False)
        self.assertEqual(value['g_flt'],0)
        self.assertEqual(len(reads),2)
        self.assertEqual(events,['handoff_communication_timeout_observed','handoff_communication_timeout_cleared'])
    def test_persistent_09_aborts_without_bypass(self):
        with self.assertRaisesRegex(RuntimeError,'gFLT=0x09'):
            probe.recover_handoff_timeout(self.status(9),lambda:self.status(9),lambda *a,**kw:None,lambda:False,timeout_s=.01)
    def test_other_fault_and_inactive_abort_without_retry(self):
        for value in [self.status(8), dict(self.status(9),g_act=0)]:
            with self.assertRaises(RuntimeError):
                probe.recover_handoff_timeout(value,lambda:self.fail('must not retry'),lambda *a,**kw:None,lambda:False)
    def test_moving_handoff_aborts(self):
        with self.assertRaisesRegex(RuntimeError,'already moving'):
            probe.recover_handoff_timeout(dict(self.status(9),g_gto=1),lambda:self.fail('must not retry'),lambda *a,**kw:None,lambda:False)


class WireTests(unittest.TestCase):
    def test_crc_reference_and_stop_has_no_reset_or_release_bits(self):
        self.assertEqual(probe.crc16(b'123456789'),0x4b37)
        p=probe.motion_pdu(123,0,0,False)
        self.assertEqual(p[:6],bytes.fromhex('1003e8000306'))
        self.assertEqual(p[6:],bytes.fromhex('0100007b0000'))
        self.assertEqual(probe.motion_pdu(123,0,0,True)[6],9)
    def port(self):
        master,slave=pty.openpty();tty.setraw(slave);os.set_blocking(slave,False)
        p=probe.Port.__new__(probe.Port);p.fd=slave;p.exclusive=False;p.slave=9;p.timeout_s=.04
        return p,master
    def test_status_and_stop_roundtrip_with_simulated_serial_device(self):
        p,master=self.port();requests=[];failures=[]
        def device():
            try:
                for length in [8,15]:
                    data=b''
                    while len(data)<length:
                        if not select.select([master],[],[],1)[0]:raise TimeoutError('test device')
                        data+=os.read(master,length-len(data))
                    requests.append(data)
                    self.assertEqual(probe.crc16(data[:-2]),int.from_bytes(data[-2:],'little'))
                    if data[1]==3:
                        response=probe.frame(9,bytes.fromhex('0306f900007b7b00'))
                    else:response=probe.frame(9,data[1:6])
                    os.write(master,response)
            except BaseException as e:failures.append(e)
        thread=threading.Thread(target=device);thread.start()
        try:
            self.assertEqual(p.status()['g_po'],123)
            p.command(123,0,0,False)
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive());self.assertFalse(failures,failures)
            self.assertEqual(requests[1][7],1)
        finally:p.close();os.close(master)
    def test_no_reply_is_bounded_and_not_retried(self):
        p,master=self.port();begin=time.monotonic()
        try:
            with self.assertRaises(TimeoutError):p.status()
            self.assertLess(time.monotonic()-begin,.5)
            data=os.read(master,100)
            self.assertEqual(len(data),8)
        finally:p.close();os.close(master)


class SimulatedRunTests(unittest.TestCase):
    def simulate(self,mode,collection=None):
        callbacks=[];stopped=threading.Event();controller={'moving':False,'commands':[],'active':True,'last_status':None,'status_gaps':[],'status_reads':0,
                    'position':getattr(collection,'start_raw',50),'target':getattr(collection,'start_raw',50)}
        class Node:
            def __init__(self,*a):pass
            def get_parameter(self,*a):return types.SimpleNamespace(value=False)
            def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=time.time_ns()))
            def create_subscription(self,typ,topic,callback,qos):callbacks.append((topic,callback));return object()
            def create_publisher(self,*a):return types.SimpleNamespace(publish=lambda msg:None)
            def count_publishers(self,t):return int(mode=='collection_suspended')
            def destroy_node(self):pass
        class Executor:
            def add_node(self,n):pass
            def spin(self):
                while not stopped.is_set():
                    ns=time.time_ns();value=.6 if controller['moving'] and mode=='force' else 0.
                    if mode in ('collection','collection_suspended'):value=max(0.,min(.8,(controller['position']-collection.start_raw)*.2))
                    for topic,callback in callbacks:
                        v=types.SimpleNamespace(x=0.,y=value,z=0.)
                        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)),wrench=types.SimpleNamespace(force=v,torque=v))
                        callback(msg)
                    time.sleep(.001)
            def shutdown(self,**kw):stopped.set()
        class Port:
            def __init__(self,*a):pass
            def status(self):
                now=time.monotonic();last=controller['last_status']
                if last is not None:controller['status_gaps'].append(now-last)
                controller['last_status']=now;controller['status_reads']+=1
                fault=9 if mode=='watchdog' and (last is None or now-last>1.) else 0
                if mode in ('collection','collection_suspended') and controller['moving']:
                    controller['position']+=(controller['target']>controller['position'])-(controller['target']<controller['position'])
                position=controller['position'] if mode in ('collection','collection_suspended') else 50
                target=controller['target'] if mode in ('collection','collection_suspended') else 50
                return {'g_act':int(controller['active']),'g_sta':3 if controller['active'] else 0,'g_flt':fault,
                        'g_gto':int(controller['moving']),'g_obj':0 if controller['moving'] and position!=target else 3,
                        'g_po':position,'g_pr':target,'g_cu':0}
            def command(self,pos,speed,force,go):
                controller['commands'].append(go);controller['moving']=go
                if mode in ('collection','collection_suspended'):controller['target']=pos if go else controller['target']
                if mode=='manual' and go: os.write(master,b'\n')
                if mode=='ack_failure' and go:raise TimeoutError('simulated lost ACK')
            def close(self):pass
        rcl=types.ModuleType('rclpy');rcl.init=lambda **kw:None;rcl.shutdown=lambda:None
        modules={'rclpy':rcl}
        for name,values in {'rclpy.node':{'Node':Node},'rclpy.executors':{'SingleThreadedExecutor':Executor},
            'rclpy.signals':{'SignalHandlerOptions':types.SimpleNamespace(NO=0)},
            'rclpy.qos':{'QoSProfile':lambda **kw:kw,'ReliabilityPolicy':types.SimpleNamespace(BEST_EFFORT=0)},
            'geometry_msgs.msg':{'WrenchStamped':object},
            'std_msgs.msg':{'String':lambda **kw:types.SimpleNamespace(**kw),'Float32':lambda **kw:types.SimpleNamespace(**kw)},
            'robotiq_2f_gripper_msgs.msg':{'GripperStatus':lambda:types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0,nanosec=0)))}}.items():
            m=types.ModuleType(name);m.__dict__.update(values);modules[name]=m
        with tempfile.TemporaryDirectory() as temp:
            a=types.SimpleNamespace(port='/dev/test',baud=115200,slave=9,out=Path(temp)/'result',enable_motion=mode!='readonly',
                close_ticks=4,speed_byte=0,force_byte=0,threshold_n=.5,hard_limit_n=1.,force_metric='max',
                stale_ms=100.,serial_timeout_ms=30.,max_motion_ms=500.,stop_after_ms=20. if mode in ('timed','watchdog') else None,hold_seconds=0.,target_confirm_ms=5. if mode in ('collection','collection_suspended') else 0.)
            master,slave=pty.openpty()
            with os.fdopen(slave,'r',buffering=1) as stdin:
                go_writer=None
                if mode=='watchdog':
                    def delayed_go():
                        time.sleep(1.2);os.write(master,b'GO\n')
                    go_writer=threading.Thread(target=delayed_go);go_writer.start()
                else:os.write(master,b'GO\n')
                with patch.dict('sys.modules',modules),patch.object(probe,'Port',Port),patch.object(probe.sys,'stdin',stdin),patch('sys.stdout',new=io.StringIO()):
                    code=probe.run(a,collection=collection)
                if go_writer:go_writer.join(timeout=2)
                if mode=='watchdog':
                    self.assertGreater(controller['status_reads'],20)
                    self.assertLess(max(controller['status_gaps']),.5)
                result=json.loads((a.out/'summary.json').read_text())
            os.close(master)
            return code,result,controller['commands']
    def test_watchdog_clears_and_reads_continue_during_go_wait_and_baseline(self):
        code,r,commands=self.simulate('watchdog');self.assertEqual(code,0,r)
        self.assertEqual(commands,[True,False]);self.assertEqual(r['stop_trigger']['reason'],'timed_stop')
    def test_readonly_sends_no_motion_or_stop(self):
        code,r,commands=self.simulate('readonly');self.assertEqual(code,0);self.assertEqual(commands,[])
    def test_threshold_stops_and_records_latency(self):
        code,r,commands=self.simulate('force');self.assertEqual(code,0,r)
        self.assertEqual(commands,[True,False]);self.assertEqual(r['stop_trigger']['reason'],'force_threshold')
        self.assertIsNotNone(r['trigger_to_stop_write_start_ms'])
    def test_no_contact_timed_stop(self):
        code,r,commands=self.simulate('timed');self.assertEqual(code,0,r)
        self.assertEqual(commands,[True,False]);self.assertEqual(r['stop_trigger']['reason'],'timed_stop')
    def test_enter_stop_is_observed_even_with_zero_hold(self):
        code,r,commands=self.simulate('manual');self.assertEqual(code,0,r)
        self.assertEqual(commands,[True,False]);self.assertEqual(r['stop_trigger']['reason'],'manual_enter')
        self.assertTrue(r['manual_stop_requested'])
    def test_command_ack_timeout_still_attempts_hardware_stop(self):
        code,r,commands=self.simulate('ack_failure');self.assertEqual(code,2)
        self.assertEqual(commands,[True,False]);self.assertTrue(r['stop_acknowledged'])



class HandoffTests(unittest.TestCase):
    def test_default_shutdown_still_deactivates_and_opt_in_only_disconnects(self):
        import subprocess
        root=Path(__file__).resolve().parents[2]
        source=(root/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_hardware/src/robotiq_2f_gripper_node.cpp').read_text()
        start=source.index('GripperNode::~GripperNode()');brace=source.index('{',start);depth=1;end=brace+1
        while depth:
            depth += (source[end]=='{')-(source[end]=='}');end+=1
        destructor=source[start:end]
        prefix='''#include <cassert>
#define RCLCPP_WARN(...) ((void)0)
struct Driver {int off=0,disconnects=0;void deactivate(){++off;}void disconnect(){++disconnects;}};
struct GripperNode {bool fake_hardware_=false;bool deactivate_on_shutdown_=true;bool serial_leased_=false;Driver* driver_;~GripperNode();};
'''
        suffix='''int main(){
Driver a;{GripperNode n;n.driver_=&a;}assert(a.off==1 && a.disconnects==1);
Driver b;{GripperNode n;n.driver_=&b;n.deactivate_on_shutdown_=false;}assert(b.off==0 && b.disconnects==1);
Driver c;{GripperNode n;n.driver_=&c;n.fake_hardware_=true;}assert(c.off==0 && c.disconnects==0);
Driver d;{GripperNode n;n.driver_=&d;n.serial_leased_=true;}assert(d.off==0 && d.disconnects==0);
}'''
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'handoff.cpp';p.write_text(prefix+destructor+suffix)
            subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror',str(p),'-o',str(Path(temp)/'handoff')],check=True)
            subprocess.run([str(Path(temp)/'handoff')],check=True)

if __name__=='__main__':unittest.main()
