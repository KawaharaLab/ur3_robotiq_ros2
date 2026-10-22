"""Experimental collection worker using the same exclusive-port probe as the WS test.

The manager authorizes one plan, records the bag, and forwards Enter/SIGINT.
This child owns serial throughout preparation, pressing, holding and release.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace


def load_probe():
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('data_capture_tools'))/'diagnostics/force_stop_probe.py'
    spec = importlib.util.spec_from_file_location('collection_stop_probe', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def width_to_raw(width_m):
    """Same empirical 2F-140 conversion as the existing driver; not indentation calibration."""
    if not math.isfinite(width_m) or not .001 <= width_m <= .142:
        raise ValueError('width must be within 1..142 mm')
    value = (-1399.78*width_m**2 - 1328.11*width_m + 218.384
             if width_m >= .015 else 18315*width_m**2 - 1849.82*width_m + 223.626)
    return max(0, min(226, int(value)))


def raw_to_width_m(value):
    """Same raw-position-to-width estimate used by the normal driver."""
    if value <= 200:
        return -3.84615e-7*value**2 - 5.67622e-4*value + .142692
    if value <= 226:
        return 8.92857e-6*value**2 - 4.38036e-3*value + .533911
    return 0.


def validate_plan(plan):
    numeric = ('effective_width_m', 'clearance_m', 'max_depth_per_side_m',
               'threshold_n', 'hard_limit_n', 'hold_seconds', 'target_confirm_ms')
    for name in numeric:
        if type(plan.get(name)) not in (int, float) or not math.isfinite(plan[name]):
            raise ValueError(f'invalid {name}')
    if not .003 <= plan['effective_width_m'] <= .138: raise ValueError('effective width outside supported range')
    if not .001 <= plan['clearance_m'] <= .005: raise ValueError('clearance must be 1..5 mm')
    if not 0 < plan['max_depth_per_side_m'] <= .010: raise ValueError('depth per side must be >0..10 mm')
    if not 0 < plan['threshold_n'] < plan['hard_limit_n'] <= 5: raise ValueError('require 0 < target < hard limit <= 5 N')
    if not 0 <= plan['hold_seconds'] <= 10: raise ValueError('hold must be 0..10 seconds')
    if not 0 <= plan['target_confirm_ms'] <= 20: raise ValueError('confirmation must be 0..20 ms')
    for name, lo, hi in [('close_ticks',1,20), ('max_motion_ms',1,10000), ('baud',9600,115200), ('slave',1,247), ('trial_index',1,100000)]:
        if type(plan.get(name)) is not int or not lo <= plan[name] <= hi: raise ValueError(f'invalid {name}')
    if plan['baud'] not in (9600,19200,38400,57600,115200): raise ValueError('unsupported baud')
    if not isinstance(plan.get('port'), str) or not plan['port'].startswith('/dev/'):
        raise ValueError('explicit /dev serial port required')
    start = plan['effective_width_m'] + plan['clearance_m']
    minimum = max(.001, plan['effective_width_m'] - 2*plan['max_depth_per_side_m'])
    width_to_raw(start); width_to_raw(minimum)
    if width_to_raw(start) >= width_to_raw(minimum): raise ValueError('no calibrated closing range')
    return start, minimum


class Collection:
    def __init__(self, plan, probe):
        self.plan, self.probe = plan, probe
        self.start_width, self.minimum_width = validate_plan(plan)
        self.start_raw, self.minimum_raw = width_to_raw(self.start_width), width_to_raw(self.minimum_width)
        self.completed = False
        self.event_publish_errors = 0
        self.node = None
        self.status_publisher = None
        self._prepared = False
        self.auxiliary_motion_commanded = False

    def setup(self, node):
        from std_msgs.msg import String
        self.message_type = String
        self.node = node
        self.publisher = node.create_publisher(String, '/force_press_event', 100)

    def on_event(self, row):
        value = dict(row, schema_version=1, trial_index=self.plan['trial_index'], action='force_press_hold')
        # File log remains authoritative if ROS event publication fails.
        try:
            self.publisher.publish(self.message_type(data=json.dumps(self.probe.json_safe(value), allow_nan=False)))
        except Exception:
            self.event_publish_errors += 1

    def on_status(self, row):
        ros_ns=self.node.get_clock().now().nanoseconds
        self.on_event(dict(event='status', monotonic_ns=row['response_monotonic_ns'], ros_ns=ros_ns, **row))
        if self.status_publisher is not None:
            message=self.status_type()
            message.header.stamp.sec=ros_ns//1_000_000_000
            message.header.stamp.nanosec=ros_ns%1_000_000_000
            for field in ('g_obj','g_flt','g_pr','g_po','g_cu'):setattr(message,field,row[field])
            message.read_duration_ms=(row['response_monotonic_ns']-row['request_monotonic_ns'])/1e6
            try:
                self.status_publisher.publish(message)
                self.width_publisher.publish(self.width_type(data=raw_to_width_m(row['g_po'])*1000.))
            except Exception:
                self.event_publish_errors+=1

    def setup_gripper_publishers(self):
        # Only after the probe has checked the normal owner's absence.
        from robotiq_2f_gripper_msgs.msg import GripperStatus
        from std_msgs.msg import Float32
        self.status_type,self.width_type=GripperStatus,Float32
        self.status_publisher=self.node.create_publisher(GripperStatus,'/robotiq_2f_gripper/status',10)
        self.width_publisher=self.node.create_publisher(Float32,'/robotiq_2f_gripper/finger_distance_mm',10)

    def summary(self):
        return dict(plan=self.plan, start_width_m=self.start_width, minimum_width_m=self.minimum_width,
                    start_raw=self.start_raw, minimum_raw=self.minimum_raw,
                    prepared=self._prepared, completed=self.completed,
                    auxiliary_motion_commanded=self.auxiliary_motion_commanded,
                    event_publish_errors=self.event_publish_errors, target_confirmation_reset_ratio=.8,
                    note='Width/depth limits use the existing empirical gripper conversion. Force is baseline-relative Fy; hold is position hold, not force regulation.')

    def check_health(self, state, manual_stop):
        if manual_stop.is_set(): raise KeyboardInterrupt('Manual collection stop')
        now = time.monotonic_ns()
        with state.lock:
            if set(state.latest) != {'left','right'}: raise RuntimeError('Missing force side')
            for row in state.latest.values():
                age = row['receive_ros_ns']-row['header_ns']
                if now-row['receive_monotonic_ns'] > state.stale_ns or not -10_000_000 <= age <= state.stale_ns:
                    raise RuntimeError('Stale force during collection')
                if not all(math.isfinite(v) for v in row['wrench']): raise RuntimeError('Invalid force during collection')

    def move(self, port, status, event, state, manual_stop, target, stage, force_guard):
        """Bounded preparatory/release move. STOP on all exits, including lost GO ACK."""
        initial = status()
        self.probe.require_ready_status(initial)
        if abs(initial['g_po']-target) <= 1:
            event(stage+'_already_at_position', target_raw=target)
            return
        if stage == 'prepare' and target-initial['g_po'] > 40:
            raise RuntimeError('Preparation would close >40 raw counts; check init/object placement')
        event(stage+'_write_start', target_raw=target)
        self.auxiliary_motion_commanded=True
        started = time.monotonic()
        failure = None
        try:
            port.command(target,0,0,True)
            while True:
                self.check_health(state, manual_stop)
                if force_guard and state.check(time.monotonic_ns()):
                    raise RuntimeError('Force guard during preparation: '+state.reason)
                value = status()
                if value['g_flt'] or value['g_act'] != 1 or value['g_sta'] != 3:
                    raise RuntimeError('Gripper fault during '+stage)
                if value['g_pr']==target and abs(value['g_po']-target)<=1:
                    event(stage+'_position_observed', g_po=value['g_po'])
                    break
                if value['g_obj'] in (1,2): raise RuntimeError('Hardware contact stop during '+stage)
                if time.monotonic()-started > 3: raise TimeoutError(stage+' move timeout')
                time.sleep(.02)
        except BaseException as exc:
            failure = exc
        # rGTO=0: this target byte does not command a reverse move.
        stop_error = None
        for attempt in range(2):
            try:
                port.command(initial['g_po'],0,0,False)
                event(stage+'_stop_ack', attempt=attempt+1)
                stop_error = None
                break
            except Exception as exc:
                stop_error = exc
                event(stage+'_stop_failed', attempt=attempt+1, error=str(exc))
        if stop_error is not None: raise stop_error
        if failure is not None: raise failure
        value = status()
        if value['g_gto'] or value['g_flt']: raise RuntimeError('No healthy STOP echo after '+stage)
        event(stage+'_complete', g_po=value['g_po'])

    def prepare(self, port, status, event, state, manual_stop):
        # Establish a preliminary tare at init's open position to guard the
        # approach to effective_width+clearance. Final tare is measured again.
        self.setup_gripper_publishers()
        event('collection_plan', **self.summary())
        begin = time.monotonic_ns()
        while time.monotonic_ns()-begin < 500_000_000:
            self.check_health(state, manual_stop)
            self.probe.require_ready_status(status())
            time.sleep(.05)
        with state.lock:
            rows=[r for r in state.logs if r['receive_monotonic_ns'] >= begin]
            for side in ('left','right'):
                values=[r['wrench'][1] for r in rows if r['side']==side]
                if len(values)<100 or not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Insufficient valid preparation baseline')
                state.offsets[side]=statistics.median(values)
            state.armed=True
        self.move(port,status,event,state,manual_stop,self.start_raw,'prepare',True)
        with state.lock:
            if state.reason: raise RuntimeError('Preparation force guard: '+state.reason)
            state.armed=False
            state.target_candidates.clear()
        begin=time.monotonic()
        while time.monotonic()-begin < .3:
            self.check_health(state,manual_stop)
            self.probe.require_ready_status(status())
            time.sleep(.05)
        self._prepared=True

    def bound_target(self, relative_target):
        return min(relative_target, self.minimum_raw)

    def finish(self, port, status, event, state, manual_stop):
        if state.reason != 'force_threshold':
            raise RuntimeError('Target force not achieved: '+str(state.reason))
        self.check_health(state, manual_stop)
        event('hold_complete')
        with state.lock: state.armed=False
        event('release_start')
        self.move(port,status,event,state,manual_stop,self.start_raw,'release',False)
        self.completed=True
        event('collection_complete')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--enable-motion',action='store_true',required=True)
    a=p.parse_args()
    if not sys.platform.startswith('linux'): p.error('Linux workstation only')
    plan=json.loads(a.plan.read_text())
    validate_plan(plan)
    probe=load_probe()
    collection=Collection(plan,probe)
    args=SimpleNamespace(port=plan['port'],baud=plan['baud'],slave=plan['slave'],out=a.out,
                         enable_motion=True,close_ticks=plan['close_ticks'],speed_byte=0,force_byte=0,
                         threshold_n=plan['threshold_n'],hard_limit_n=plan['hard_limit_n'],force_metric='max',
                         stale_ms=100.,serial_timeout_ms=30.,max_motion_ms=plan['max_motion_ms'],
                         stop_after_ms=None,hold_seconds=plan['hold_seconds'],target_confirm_ms=plan['target_confirm_ms'])
    return probe.run(args,collection=collection)


if __name__=='__main__':
    raise SystemExit(main())
