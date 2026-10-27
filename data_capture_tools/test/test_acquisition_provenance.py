import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'data_capture_tools'))
from data_capture_tools.acquisition_provenance import read_capture_measurements, read_measurements, runtime_provenance, legacy_conversion_description
from data_capture_tools.force_press_worker import raw_to_width_m
spec=importlib.util.spec_from_file_location('init_fixture',Path(__file__).with_name('test_init_recording.py'))
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)


class MeasurementTests(unittest.TestCase):
    def test_yaml_identity_and_dimensions_are_independent_of_motion(self):
        cfg = SimpleNamespace(task_name='object1', source_path='/capture.yaml', raw=dict(
            target_diameter=.010, push_depth=.004,
            specimen_metadata=dict(specimen_width_mm=12.4, clamp_gap_mm=8.2,
                                   width_measurement_state='before clamping', notes='reused')))
        result = read_capture_measurements(cfg)
        self.assertEqual(result['values']['specimen_id'], 'object1')
        self.assertEqual(result['specimen_id_source'], 'task_name')
        self.assertEqual(result['values']['specimen_width_mm'], 12.4)
        self.assertEqual(result['values']['clamp_gap_mm'], 8.2)
        self.assertEqual(cfg.raw['target_diameter'], .010)
        self.assertNotIn('specimen_id', cfg.raw['specimen_metadata'])
        self.assertEqual(result['source'], 'loaded_capture_yaml')

    def test_omitted_measurements_and_explicit_identity_are_supported(self):
        cfg = SimpleNamespace(task_name='object1', raw={})
        result = read_capture_measurements(cfg)
        self.assertEqual(result['values'], {'specimen_id':'object1'})
        self.assertIn('clamp_gap_mm', result['missing_fields'])
        cfg.raw['specimen_metadata'] = dict(specimen_id='sample-A')
        self.assertEqual(read_capture_measurements(cfg)['values']['specimen_id'], 'sample-A')

    def test_yaml_input_validation_and_legacy_json_opt_in(self):
        cfg = SimpleNamespace(task_name='object1', raw={})
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'specimen_metadata.json'
            p.write_text(json.dumps(dict(specimen_id='stale',clamp_gap_mm=20)))
            self.assertEqual(read_capture_measurements(cfg)['values']['specimen_id'], 'object1')
            self.assertEqual(read_capture_measurements(cfg,p)['values']['specimen_id'], 'stale')
            cfg.raw['specimen_metadata'] = dict(clamp_gap_mm=3)
            result = read_capture_measurements(cfg,p)
            self.assertEqual(result['values']['clamp_gap_mm'],20)
            self.assertIn('source_file_sha256',result)
            with self.assertRaisesRegex(ValueError,'does not exist'):
                read_capture_measurements(cfg,Path(tmp)/'missing.json')
        for bad in ([], 'bad', dict(clamp_gap_mm=True),dict(width_measurement_state=42)):
            cfg.raw['specimen_metadata'] = bad
            with self.assertRaises(ValueError):read_capture_measurements(cfg)

    def test_unknown_is_not_zero_and_all_fields_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'metadata.json'
            self.assertFalse(read_measurements(p)['available'])
            p.write_text(json.dumps(dict(schema_version=1,clamp_gap_mm=None,specimen_width_mm=10.2,
                                        measurement_uncertainty_mm=.2,notes='approximate',extra='preserved')))
            r=read_measurements(p)
            self.assertIsNone(r['values']['clamp_gap_mm'])
            self.assertEqual(r['values']['extra'],'preserved')
            self.assertEqual(r['values']['measurement_uncertainty_mm'],.2)
            self.assertEqual(len(r['sha256']),64)

    def test_physical_changes_have_new_id_but_notes_do_not(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.json';value=dict(specimen_id='A',clamp_gap_mm=20.,clamp_insertion_depth_mm=5.)
            p.write_text(json.dumps(value));a=read_measurements(p)
            value['notes']='updated';p.write_text(json.dumps(value));b=read_measurements(p)
            self.assertEqual(a['physical_condition_id'],b['physical_condition_id'])
            self.assertNotEqual(a['sha256'],b['sha256'])
            value['clamp_insertion_depth_mm']=6.;p.write_text(json.dumps(value));c=read_measurements(p)
            self.assertNotEqual(a['physical_condition_id'],c['physical_condition_id'])

    def test_malformed_numeric_values_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.json'
            for value in [-1,True,'10',float('nan')]:
                p.write_text(json.dumps(dict(clamp_gap_mm=value)))
                with self.assertRaises(ValueError):read_measurements(p)

    def test_runtime_environment_is_allowlisted_and_conversion_is_explicit(self):
        r=runtime_provenance()
        self.assertEqual(set(r['ros_environment']),{'ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION'})
        self.assertIn('init_recording.py',r['runtime_python_source_sha256'])
        self.assertEqual(legacy_conversion_description()['width_to_count']['rounding'],'truncate toward zero')


class SessionTests(unittest.TestCase):
    def test_yaml_snapshot_saved_with_init_and_session_and_no_measured_dimensions_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            node=fixture.FakeNode(root/'init')
            node.config.raw.update(task_name='object1', specimen_metadata=dict(
                specimen_width_mm=None,clamp_gap_mm=None,notes='previously used; count unknown'))
            self.assertTrue(node._run_init_sequence())
            summary=json.loads((node._last_successful_init/'summary.json').read_text())
            self.assertEqual(summary['metadata']['measurements']['values']['specimen_id'],'object1')
            session=root/'session';session.mkdir();node._attach_init_record(session)
            saved=json.loads((session/'collection_provenance.json').read_text())
            self.assertEqual(saved['measurements'],summary['metadata']['measurements'])
            node._session_dir=session
            node._record_trial_provenance('START',1,'depth_sweep',.020,.1)
            row=json.loads((session/'trial_conditions.jsonl').read_text())
            self.assertEqual(row['mode'],'depth_sweep')
            self.assertEqual(row['marker_target_width_m'],.020)
            self.assertEqual(row['init_id'],summary['init_id'])

    def test_selection_and_final_reference_are_separate_without_changing_motion_acceptance(self):
        class Node(fixture.FakeNode):
            def _run_init_motion_sequence(self):
                super()._run_init_motion_sequence()
                now=self.get_clock().now().nanoseconds
                self._init_observe('gripper_status',header_ns=now,status=dict(g_po=194,g_pr=194,g_flt=0))
                self._mark_init_selection(dict(w=raw_to_width_m(194),f_l=.1,f_r=.1,pos=0,diff=0))
                self._init_observe('centering_result',final_balanced_contact=False)
                self._init_observe('gripper_status',header_ns=now,status=dict(g_po=193,g_pr=193,g_flt=0))
                self._mark_init_contact(raw_to_width_m(193))
                return True
        with tempfile.TemporaryDirectory() as tmp:
            node=Node(Path(tmp)/'init');self.assertTrue(node._run_init_sequence())
            s=json.loads((node._last_successful_init/'summary.json').read_text())
            self.assertEqual(s['selected_contact']['observed_count'],194)
            self.assertTrue(s['selected_contact']['status_width_consistent'])
            self.assertEqual(s['selected_to_final_count_delta'],-1)
            self.assertTrue(s['record_quality_valid'])
            self.assertFalse(s['contact_criteria_met'])
            session=Path(tmp)/'session';session.mkdir();node._attach_init_record(session)
            p=json.loads((session/'collection_provenance.json').read_text())
            self.assertEqual(p['selected_contact_count'],194)
            self.assertEqual(p['final_reference_count'],193)

    def test_measurements_reloaded_each_init_and_changed_mount_requires_reinit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'m.json';p.write_text(json.dumps(dict(specimen_id='A',clamp_gap_mm=10.)))
            node=fixture.FakeNode(root/'init');node._specimen_metadata_path=p
            self.assertTrue(node._run_init_sequence())
            p.write_text(json.dumps(dict(specimen_id='B',clamp_gap_mm=20.)))
            session=root/'session';session.mkdir()
            with self.assertRaisesRegex(RuntimeError,'changed since init'):node._attach_init_record(session)
            self.assertTrue(node._run_init_sequence());node._attach_init_record(session)
            result=json.loads((session/'collection_provenance.json').read_text())
            self.assertEqual(result['measurements']['values']['clamp_gap_mm'],20.)

    def test_invalid_metadata_blocks_init_before_movement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'m.json';p.write_text('{broken')
            node=fixture.FakeNode(root/'init');node._specimen_metadata_path=p
            self.assertFalse(node._run_init_sequence());self.assertEqual(node.moves,0)

    def test_missing_recording_topic_blocks_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=fixture.FakeNode(Path(tmp)/'init');node.config.topics=[]
            with self.assertRaisesRegex(RuntimeError,'provenance topics'):node._attach_init_record(Path(tmp))

    def test_trial_marker_does_not_claim_open_position_is_indentation(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=fixture.FakeNode(Path(tmp)/'init');node._run_init_sequence()
            node._session_dir=Path(tmp)
            node._record_trial_provenance('START',1,'push',.020,.1)
            row=json.loads((Path(tmp)/'trial_conditions.jsonl').read_text())
            self.assertEqual(row['observed_status_at_marker']['status']['g_po'],100)
            self.assertIn('may be open',row['note'])
            self.assertEqual(row['init_id'],node._last_successful_init.name)

    def test_sensor_metadata_retains_identity_but_not_duplicate_raw_packet_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=fixture.FakeNode(Path(tmp))
            report=dict(stream_id='force-A',force_scale=.001,torque_scale=.00001,
                        configured_interval_us=1000,reads=[{'bytes_hex':'ab'*1000}])
            node._init_sensor_metadata_callback('mms_left','/timing',SimpleNamespace(data=json.dumps(report)))
            item=node._sensor_metadata_snapshot()['sensor_metadata_mms_left']
            self.assertEqual(item['metadata']['force_scale'],.001)
            self.assertNotIn('reads',item['metadata'])
            report['stream_id']='force-B'
            node._init_sensor_metadata_callback('mms_left','/timing',SimpleNamespace(data=json.dumps(report)))
            self.assertEqual(node._sensor_metadata_snapshot()['sensor_metadata_mms_left']['metadata']['stream_id'],'force-B')


class DriverTests(unittest.TestCase):
    def test_actual_cpp_wrapper_preserves_command_on_telemetry_failure_and_throws_write_error(self):
        src=(ROOT/'external/robotiq_2f_gripper_ros2/robotiq_2f_gripper_hardware/src/robotiq_2f_gripper_node.cpp').read_text()
        start=src.index('std::string make_command_record(');end=src.index('return out.str();\n}',start)+len('return out.str();\n}')
        helper=src[start:end]
        start=src.index('void GripperNode::send_recorded_position(');end=src.index('\nGripperNode::~GripperNode()',start)
        method=src[start:end]
        prefix=r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#define RCLCPP_ERROR(...) ((void)0)
namespace std_msgs {namespace msg {struct String {std::string data;};}}
struct Publisher {bool fail=false;std::vector<std::string> rows;
 void publish(const std_msgs::msg::String &s){if(fail)throw std::runtime_error("publish");rows.push_back(s.data);}};
struct Driver {bool fail=false;int writes=0,position=-1;
 void set_gripper_position(int p){++writes;position=p;if(fail)throw std::runtime_error("write");}};
struct GripperNode {std::atomic<uint64_t> command_sequence_{0};std::string startup_id_="A";
 std::shared_ptr<Publisher> command_event_publisher_=std::make_shared<Publisher>();
 std::unique_ptr<Driver> driver_=std::make_unique<Driver>();
 struct Stamp {int64_t nanoseconds(){return 100;}};Stamp now(){return {};}
 int convertToGripperSystem(double v){return static_cast<int>(v*255);}
 void send_recorded_position(uint8_t,double,double,double,const std::string &);
};
'''
        main=r'''
int main(){GripperNode n;
 n.send_recorded_position(193,.020,.1,.5,"action");assert(n.driver_->writes==1 && n.driver_->position==193);
 assert(n.command_event_publisher_->rows.size()==2);
 assert(n.command_event_publisher_->rows[0].find("write_start")!=std::string::npos);
 assert(n.command_event_publisher_->rows[1].find("write_return")!=std::string::npos);
 n.driver_->fail=true;bool threw=false;
 try{n.send_recorded_position(194,.019,0,0,"action");}catch(const std::runtime_error&){threw=true;}
 assert(threw && n.driver_->writes==2);
 assert(n.command_event_publisher_->rows.back().find("write_exception")!=std::string::npos);
 n.driver_->fail=false;n.command_event_publisher_->fail=true;
 n.send_recorded_position(195,.018,0,0,"action");assert(n.driver_->writes==3 && n.driver_->position==195);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'test.cpp';p.write_text(prefix+helper+method+main);binary=Path(tmp)/'test'
            subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror',str(p),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
        self.assertEqual(src.count('send_recorded_position(convertToGripperSystemPosition(position)'),2)


if __name__=='__main__':unittest.main()
