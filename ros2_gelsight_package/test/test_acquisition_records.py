import ast
import json
from pathlib import Path
import sys
import types
import unittest

PACKAGE = Path(__file__).resolve().parents[1]/'ros2_gelsight_package'
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'utils/diagnostics'))
from verify_acquisition_timing import validate, SENSORS, TIMING


def cls(path, name, environment):
    tree=ast.parse(path.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),environment)
    return environment[name]


class RecordsTests(unittest.TestCase):
    def test_camera_stages_bracket_actual_read_and_transform(self):
        order=[]
        samples=iter([[1,101,2],[3,103,4],[5,105,6]])
        def clock():
            order.append('clock');return next(samples)
        def read():order.append('read');return 'pixels'
        def transform(a,b):order.append('transform');return a
        env={'Optional':object,'MatLike':object,'Camera':object,
             'cv2':types.SimpleNamespace(COLOR_BGR2RGB=1,CAP_PROP_POS_MSEC=2,cvtColor=transform),
             'time':types.SimpleNamespace(time=lambda:1.), 'crop_and_resize':lambda **kw:kw['image'],
             'log_message':lambda x:None}
        env['Optional']=__import__('typing').Optional
        mini=cls(PACKAGE/'utilities/gelsightmini.py','GelSightMini',env)(acquisition_clock=clock)
        mini.camera=types.SimpleNamespace(read_frame=read,cap=types.SimpleNamespace(get=lambda p:1234.5))
        self.assertEqual(mini.update(0),'pixels')
        self.assertEqual(order,['clock','read','clock','transform','clock'])
        self.assertEqual(mini.last_acquisition_timing['read_end'],[3,103,4])
        self.assertEqual(mini.last_acquisition_timing['backend_pos_msec'],1234.5)

    def test_frame_mapping_and_json_preserve_ns_integers(self):
        env={'Node':object,'ConfigModel':object,'Optional':__import__('typing').Optional,
             'cv2':types.SimpleNamespace(__version__='test'),'json':json,
             'String':lambda **kw:types.SimpleNamespace(**kw)}
        publisher=cls(PACKAGE/'ros2_gelsight_publisher.py','GelSightMiniPublisher',env).__new__(
            cls(PACKAGE/'ros2_gelsight_publisher.py','GelSightMiniPublisher',env))
        output=[]
        publisher._acquisition_publisher=types.SimpleNamespace(publish=lambda m:output.append(json.loads(m.data)))
        publisher._acquisition_stream_id='stream';publisher._acquisition_backend='V4L2'
        publisher._acquisition_publish_errors=0;publisher._frame_id='left';publisher._topic_name='image'
        publisher._cam_stream=types.SimpleNamespace(frame_count=7,last_acquisition_timing={'read_start':[1,2,3]})
        publisher.get_clock=lambda:types.SimpleNamespace(clock_type=1)
        publisher.get_parameter=lambda name:types.SimpleNamespace(value=False)
        stamp=types.SimpleNamespace(sec=1792054592,nanosec=915155240)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp))
        publisher._publish_acquisition_record(msg,True)
        self.assertEqual(output[0]['header_ns'],1792054592915155240)
        self.assertEqual(output[0]['frame_seq'],7)
        self.assertTrue(output[0]['compressed_published'])

    def test_validator_detects_wrong_values_and_reassembles_split_packet(self):
        packet=bytearray(25);packet[:2]=b'\x00\x17'
        for p in [4,7,10,13,16,19]:packet[p:p+3]=(-1234).to_bytes(3,'big',signed=True)
        h=1792054592915155240
        sensors={t:[{'header_ns':h}] for t in SENSORS}
        records=[]
        for i,t in enumerate(TIMING[:2]):
            records.append((t,{'schema_version':1,'kind':'gelsight_frame','stream_id':'s',
                'frame_seq':1,'header_ns':h,'stages':{'read_start':[1,h,2],'read_end':[3,h,4],
                'preprocess_end':[5,h,6],'backend_pos_msec':None},'backend':'V4L2',
                'record_publish_errors_total':0,'opencv_version':'test','clock_type':1}))
        for i,t in enumerate(TIMING[2:]):
            sensors[SENSORS[i+2]][0]['wrench']=[-1234*.001]*3+[-1234*.00001]*3
            records.append((t,{'schema_version':1,'kind':'mms101_reads','stream_id':'s','batch_seq':1,
                'dropped_reads_total':0,'dropped_samples_total':0,'failed_publish_reads_total':0,
                'failed_publish_samples_total':0,'force_scale':.001,'torque_scale':.00001,'reads':[
                {'read_seq':1,'byte_offset':0,'read_start':[1,h,2],'read_end':[3,h,4],
                 'bytes_hex':packet[:10].hex(),'samples':[]},
                {'read_seq':2,'byte_offset':10,'read_start':[5,h,6],'read_end':[7,h,8],
                 'bytes_hex':packet[10:].hex(),'samples':[[1,h,0]]}]}))
        result=validate(records,sensors)
        self.assertTrue(result['schema_and_mapping_checks_passed'],result)
        self.assertEqual(result['topics'][TIMING[2]]['raw_packet_value_comparisons'],1)
        sensors[SENSORS[2]][0]['wrench'][0]=999
        self.assertFalse(validate(records,sensors)['schema_and_mapping_checks_passed'])


if __name__=='__main__': unittest.main()
