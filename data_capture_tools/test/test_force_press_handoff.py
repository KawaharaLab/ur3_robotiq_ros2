"""Automatic serial handoff: real C++ service bodies, RPC failure and recovery guards."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import types
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'data_capture_tools'))
from data_capture_tools.force_press_collection import ForcePressCollectionMixin


def extract_function(text,name):
    start=text.index(name);brace=text.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]


class CppHandoffTests(unittest.TestCase):
    def test_service_rejects_busy_and_faulted_handoffs_and_restores_without_motion(self):
        source=(ROOT/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_hardware/src/robotiq_2f_gripper_node.cpp').read_text()
        function=extract_function(source,'void GripperNode::set_force_press_control(')
        prefix=r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
namespace std_srvs {namespace srv {struct SetBool {
struct Request {bool data=false;};struct Response {bool success=false;std::string message;};
};}}
struct Driver {
 struct Status {int g_flt=0,g_po=100;};
 bool connected=true,active=true,moving=false,open_fails=false,close_fails=false;
 int reads=0,opens=0,closes=0,timeout_reads=0;
 bool connect(){++opens;if(open_fails)throw std::runtime_error("busy port");connected=true;return true;}
 void disconnect(){++closes;if(close_fails)throw std::runtime_error("close failed");connected=false;}
 bool is_gripper_active(){return active;}
 bool motion_command_in_progress()const{return moving;}
 Status read_status(){++reads;auto s=status;if(timeout_reads>0){--timeout_reads;s.g_flt=9;}return s;}
 Status get_last_status()const{return status;}
 Status status;
};
struct GripperNode {
 bool fake_hardware_=false;std::atomic<bool> running_{false},serial_leased_{false};
 Driver *driver_;std::string serial_port_="/dev/ttyUSB0";int baudrate_=115200,slave_address_=9;
 double gripper_position_=0;
 double convertToMeters(int){return .040;}
 void set_force_press_control(const std::shared_ptr<std_srvs::srv::SetBool::Request>,std::shared_ptr<std_srvs::srv::SetBool::Response>);
};
'''
        suffix=r'''
int main(){
 Driver d;GripperNode n;n.driver_=&d;
 auto req=std::make_shared<std_srvs::srv::SetBool::Request>();
 auto res=std::make_shared<std_srvs::srv::SetBool::Response>();req->data=true;
 n.running_=true;n.set_force_press_control(req,res);assert(!res->success && d.reads==0 && d.closes==0);
 n.running_=false;d.moving=true;n.set_force_press_control(req,res);assert(!res->success && d.closes==0);
 d.moving=false;d.status.g_flt=8;n.set_force_press_control(req,res);assert(!res->success && d.closes==0);
 d.status.g_flt=0;n.set_force_press_control(req,res);assert(res->success && n.serial_leased_ && !d.connected);
 assert(res->message.find("\"request_acquired\":true")!=std::string::npos);
 int reads=d.reads;n.set_force_press_control(req,res);assert(!res->success && d.reads==reads);
 req->data=false;d.open_fails=true;n.set_force_press_control(req,res);assert(!res->success && n.serial_leased_);
 d.open_fails=false;d.timeout_reads=1;n.set_force_press_control(req,res);assert(res->success && !n.serial_leased_ && d.connected);
 int opens=d.opens;n.set_force_press_control(req,res);assert(res->success && d.opens==opens);
 req->data=true;d.close_fails=true;n.set_force_press_control(req,res);assert(!res->success && n.serial_leased_);
 assert(res->message.find("\"request_acquired\":true")!=std::string::npos);
}
'''
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'handoff.cpp';path.write_text(prefix+function+suffix)
            binary=Path(temp)/'handoff'
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',str(path),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_cached_motion_uses_both_go_and_object_status(self):
        source=(ROOT/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_interfaces/src/default_driver.cpp').read_text()
        function=extract_function(source,'bool DefaultDriver::motion_command_in_progress() const')
        prefix='''#include <cassert>
enum class ActionStatus{STOPPED,MOVING};enum class ObjectDetectionStatus{MOVING,DONE};
struct DefaultDriver{ActionStatus action_status_;ObjectDetectionStatus object_detection_status_;bool motion_command_in_progress()const;};
'''
        suffix='''int main(){DefaultDriver d{ActionStatus::STOPPED,ObjectDetectionStatus::MOVING};assert(!d.motion_command_in_progress());
d.action_status_=ActionStatus::MOVING;assert(d.motion_command_in_progress());d.object_detection_status_=ObjectDetectionStatus::DONE;assert(!d.motion_command_in_progress());}'''
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'motion.cpp';path.write_text(prefix+function+suffix);binary=Path(temp)/'motion'
            subprocess.run(['c++','-std=c++17',str(path),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_paused_driver_blocks_all_normal_serial_entry_points(self):
        source=(ROOT/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_hardware/src/robotiq_2f_gripper_node.cpp').read_text()
        for name in ['GripperNode::handle_move_goal(', 'GripperNode::update_joint_state_callback()',
                     'GripperNode::update_object_grasped_callback()', 'GripperNode::gripper_command_callback(',
                     'GripperNode::gripper_binary_command_callback(']:
            body=extract_function(source,name)
            self.assertIn('running_ || serial_leased_',body,name)
        cancel=extract_function(source,'GripperNode::handle_cancel_move(')
        self.assertNotIn('running_ = false',cancel)


class PythonHandoffTests(unittest.TestCase):
    def manager(self):
        m=ForcePressCollectionMixin()
        m._force_press_handoff_active=False;m._force_press_process=None;m._force_press_lock=threading.Lock()
        m.get_logger=lambda:types.SimpleNamespace(info=lambda *a:None,error=lambda *a:None)
        return m
    def test_actual_driver_serial_settings_are_used_and_returned(self):
        m=self.manager();calls=[]
        def rpc(enabled,**kw):
            calls.append(enabled)
            return types.SimpleNamespace(success=True),dict(request_acquired=enabled,leased=enabled,port='/dev/ttyUSB3',baud=115200,slave=9)
        m._request_force_press_control=rpc
        base=dict(effective_width_m=.040,clearance_m=.002,max_depth_per_side_m=.004,
                  threshold_n=.5,hard_limit_n=1.,hold_seconds=4.,target_confirm_ms=5.,close_ticks=20,
                  max_motion_ms=2000,baud=115200,slave=9,trial_index=1,port='/dev/ttyUSB0')
        m._borrow_force_press_control(base)
        self.assertEqual(base['port'],'/dev/ttyUSB3');self.assertTrue(base['normal_driver_suspended'])
        self.assertTrue(m._restore_force_press_control());self.assertFalse(m._force_press_handoff_active)
        self.assertEqual(calls,[True,False])
    def test_busy_response_does_not_take_or_return_another_lease(self):
        m=self.manager();calls=[]
        def rpc(enabled,**kw):
            calls.append(enabled)
            return types.SimpleNamespace(success=False),dict(request_acquired=False,leased=True,reason='busy')
        m._request_force_press_control=rpc
        with self.assertRaises(RuntimeError):m._borrow_force_press_control({})
        self.assertTrue(m._restore_force_press_control());self.assertEqual(calls,[True])
    def test_lost_response_blocks_normal_control_until_recovery(self):
        m=self.manager()
        def lost(*a,**kw):raise TimeoutError('lost')
        m._request_force_press_control=lost
        with self.assertRaises(TimeoutError):m._borrow_force_press_control({})
        self.assertTrue(m._force_press_handoff_active)
        self.assertFalse(m._force_press_normal_control_ready('ap'))
    def test_worker_must_close_port_before_return(self):
        m=self.manager();m._force_press_handoff_active=True;m._force_press_process=types.SimpleNamespace(poll=lambda:None)
        m._request_force_press_control=lambda *a,**kw:self.fail('must not reopen while worker alive')
        self.assertFalse(m._restore_force_press_control())


if __name__=='__main__':unittest.main()
