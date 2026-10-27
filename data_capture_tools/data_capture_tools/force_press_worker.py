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
    if plan.get('press_mode', 'continuous') not in ('continuous', 'stepped'):
        raise ValueError('unknown press mode')
    if plan.get('press_mode') == 'stepped':
        for name, default, lo, hi in [('step_counts',1,1,3), ('step_settle_ms',100,50,1000),
                                      ('step_sample_ms',50,20,200), ('step_timeout_ms',500,100,2000),
                                      ('step_total_timeout_ms',10000,1000,30000)]:
            value = plan.get(name, default)
            if type(value) is not int or not lo <= value <= hi:
                raise ValueError('invalid '+name)
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
        self.is_stepped = plan.get('press_mode', 'continuous') == 'stepped'
        self.step_records = []
        self.completed = False
        self.event_publish_errors = 0
        self.node = None
        self.status_publisher = None
        self._prepared = False
        self.auxiliary_motion_commanded = False
        self.startup_force_wait = None
        self.abort_release = None

    def allows_suspended_driver(self):
        # Manager sets this only after a successful service reply. Port's /proc
        # scan and TIOCEXCL remain mandatory; an open normal owner still fails.
        return self.plan.get('normal_driver_suspended') is True

    def setup(self, node):
        from std_msgs.msg import String
        self.message_type = String
        self.node = node
        self.publisher = node.create_publisher(String, '/force_press_event', 100)

    def on_event(self, row):
        value = dict(row, schema_version=1, trial_index=self.plan['trial_index'], action='force_press_step_hold' if self.is_stepped else 'force_press_hold')
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
        return dict(plan=self.plan, press_mode='stepped' if self.is_stepped else 'continuous',
                    steps=self.step_records, start_width_m=self.start_width, minimum_width_m=self.minimum_width,
                    start_raw=self.start_raw, minimum_raw=self.minimum_raw,
                    prepared=self._prepared, completed=self.completed,
                    auxiliary_motion_commanded=self.auxiliary_motion_commanded,
                    startup_force_wait=self.startup_force_wait, abort_release=self.abort_release,
                    event_publish_errors=self.event_publish_errors, target_confirmation_reset_ratio=.8,
                    note='Width/depth limits use the existing empirical gripper conversion. Force is baseline-relative Fy; hold is position hold, not force regulation.')

    def force_health_snapshot(self, state):
        snapshot={}
        with state.lock:
            now=time.monotonic_ns()
            for side in ('left','right'):
                row=state.latest.get(side)
                if row is None:
                    snapshot[side]={'present':False,'fresh':False}
                    continue
                age=row['receive_ros_ns']-row['header_ns']
                receive_age=now-row['receive_monotonic_ns']
                valid=all(math.isfinite(v) for v in row['wrench'])
                snapshot[side]=dict(present=True,valid_values=valid,
                    receive_age_ms=receive_age/1e6,header_age_at_receive_ms=age/1e6,
                    receive_monotonic_ns=row['receive_monotonic_ns'],header_ns=row['header_ns'],
                    fresh=0 <= receive_age <= state.stale_ns and -10_000_000 <= age <= state.stale_ns)
        return snapshot

    def check_health(self, state, manual_stop):
        if manual_stop.is_set(): raise KeyboardInterrupt('Manual collection stop')
        snapshot=self.force_health_snapshot(state)
        if any(value.get('present') and not value.get('valid_values') for value in snapshot.values()):
            raise RuntimeError('Invalid force during collection: '+json.dumps(snapshot))
        if not all(value['fresh'] for value in snapshot.values()):
            raise RuntimeError('Stale force during collection: '+json.dumps(snapshot))

    def wait_for_fresh_startup_force(self, status, event, state, manual_stop,
                                    timeout_s=2., stable_s=.1):
        """No motion: re-check subscription readiness after serial/DDS setup.
        This does not catch/retry a stale sample after any motion has begun.
        """
        begin=time.monotonic_ns()
        deadline=begin+round(timeout_s*1e9)
        stable_since=None
        first=self.force_health_snapshot(state)
        event('startup_force_wait_start', health=first, timeout_s=timeout_s, stable_s=stable_s)
        while True:
            if manual_stop.is_set(): raise KeyboardInterrupt('Manual stop while waiting for force readiness')
            self.probe.require_ready_status(status()) # keep serial alive, read only
            snapshot=self.force_health_snapshot(state)
            now=time.monotonic_ns()
            if any(value.get('present') and not value.get('valid_values') for value in snapshot.values()):
                raise RuntimeError('Invalid force before preparation: '+json.dumps(snapshot))
            ready=all(value['fresh'] and value.get('receive_monotonic_ns',0)>=begin for value in snapshot.values())
            if ready:
                if stable_since is None: stable_since=now
                if now-stable_since >= round(stable_s*1e9):
                    self.startup_force_wait=dict(wait_ms=(now-begin)/1e6,initial_health=first,ready_health=snapshot)
                    event('startup_force_ready', **self.startup_force_wait)
                    return
            else:
                stable_since=None
            if now>=deadline:
                self.startup_force_wait=dict(wait_ms=(now-begin)/1e6,initial_health=first,last_health=snapshot)
                event('startup_force_timeout', **self.startup_force_wait)
                raise RuntimeError('Fresh force not ready before preparation: '+json.dumps(snapshot))
            time.sleep(.01)

    def move(self, port, status, event, state, manual_stop, target, stage, force_guard):
        """Bounded preparatory/release move. STOP on all exits, including lost GO ACK."""
        initial = status()
        self.probe.require_ready_status(initial)
        tolerance = 0 if stage == 'abort_release' else 1
        if abs(initial['g_po']-target) <= tolerance:
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
                if value['g_pr']==target and abs(value['g_po']-target)<=tolerance:
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
        self.wait_for_fresh_startup_force(status,event,state,manual_stop)
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

    def run_stepped_press(self, port, status, event, state, manual_stop, initial, limit):
        """Bound each GO by position; evaluate target only after STOP and settling.

        The probe's outer exception handler always attempts STOP on failures here.
        ForceState remains armed for immediate hard-limit/invalid/stale stops.
        """
        deadline = time.monotonic() + self.plan.get('step_total_timeout_ms', 10000)/1000
        current = initial['g_po']
        state.target_enabled = False

        def guarded_status(stopped=False):
            self.check_health(state, manual_stop)
            reason = state.check(time.monotonic_ns())
            if reason:
                return None
            if time.monotonic() >= deadline:
                state.request_stop('stepped_total_timeout')
                return None
            value = status()
            if value['g_flt'] or value['g_act'] != 1 or value['g_sta'] != 3:
                raise RuntimeError('Gripper fault during stepped pressing')
            if value['g_po'] > limit:
                raise RuntimeError('Observed position exceeds stepped closing limit')
            if stopped and value['g_gto']:
                raise RuntimeError('Gripper GO unexpectedly active during step pause')
            self.check_health(state, manual_stop)
            if state.check(time.monotonic_ns()):
                return None
            return value

        while current < limit:
            value = guarded_status(stopped=True)
            if value is None:
                return
            current = value['g_po']
            if current >= limit:
                break
            target = min(limit, current + self.plan.get('step_counts', 1))
            number = len(self.step_records) + 1
            record = dict(step_index=number, start_raw=current, target_raw=target)
            self.step_records.append(record)  # incomplete moves survive exceptions
            record['write_start_ns'] = event('step_write_start', **record)['monotonic_ns']
            port.command(target, 0, 0, True)
            event('step_write_ack', step_index=number, target_raw=target)
            move_deadline = time.monotonic() + self.plan.get('step_timeout_ms', 500)/1000
            while True:
                value = guarded_status()
                if value is None:
                    return  # outer probe sends the terminal STOP
                if value['g_pr'] == target and value['g_po'] >= target:
                    break
                if value['g_obj'] in (1, 2):
                    state.request_stop('position_or_hardware_contact_limit')
                    return
                if time.monotonic() >= move_deadline:
                    state.request_stop('step_motion_timeout')
                    return
                time.sleep(.005)
            event('step_position_observed', step_index=number, g_po=value['g_po'])
            event('step_stop_write_start', step_index=number)
            port.command(current, 0, 0, False)  # rGTO=0, never a reverse GO
            record['stop_ack_ns'] = event('step_stop_write_ack', step_index=number)['monotonic_ns']
            value = guarded_status()
            if value is None:
                return
            if value['g_gto']:
                raise RuntimeError('No STOP echo between force steps')
            current = value['g_po']
            record['stopped_raw'] = current
            if current > limit:
                raise RuntimeError('Observed position exceeds stepped closing limit')
            settle_end = time.monotonic() + self.plan.get('step_settle_ms', 100)/1000
            while time.monotonic() < settle_end:
                if guarded_status(stopped=True) is None:
                    return
                time.sleep(.005)
            # Require newly acquired samples after settling, not queued pre-STOP data.
            begin = time.monotonic_ns()
            begin_ros = self.node.get_clock().now().nanoseconds
            window_ns = self.plan.get('step_sample_ms', 50)*1_000_000
            event('step_measure_start', step_index=number, sample_start_ros_ns=begin_ros)
            while time.monotonic_ns()-begin < window_ns:
                if guarded_status(stopped=True) is None:
                    return
                time.sleep(.005)
            measured, evidence = {}, {}
            with state.lock:
                if state.reason:
                    return
                for side in ('left', 'right'):
                    rows = [r for r in state.logs if r['side'] == side
                            and begin <= r['receive_monotonic_ns'] < begin+window_ns
                            and begin_ros <= r['header_ns'] < begin_ros+window_ns]
                    if len(rows) < 10 or rows[-1]['header_ns']-rows[0]['header_ns'] < window_ns//2:
                        raise RuntimeError('Insufficient newly acquired settled force: '+side)
                    measured[side] = abs(statistics.median(r['wrench'][1] for r in rows)-state.offsets[side])
                    evidence[side] = dict(samples=len(rows), first_header_ns=rows[0]['header_ns'],
                                          last_header_ns=rows[-1]['header_ns'])
                record.update(measured_force_n=measured, evidence=evidence,
                              sample_start_monotonic_ns=begin, sample_end_monotonic_ns=begin+window_ns,
                              sample_start_ros_ns=begin_ros, sample_end_ros_ns=begin_ros+window_ns,
                              measurement_end_ns=time.monotonic_ns())
                if max(measured.values()) >= self.plan['threshold_n']:
                    state._latch('force_threshold', time.monotonic_ns())
                    state.trigger.update(decision_method='settled_window_median',
                                         measured_force_n=measured, step_index=number,
                                         settled_evidence=evidence)
            event('step_measurement', **record)
            if state.reason:
                return
        # The final allowed step is measured before declaring range exhaustion.
        state.request_stop('position_limit_before_target')

    def release_after_hard_limit(self, port, status, event, state, manual_stop, stop_acknowledged):
        """Unload a failed press, without clearing its trigger or marking success.

        Only the collection worker opts in; standalone probes remain STOP-only.
        Confirm STOP and healthy telemetry before sending an opening-only GO.
        """
        if state.reason != 'force_hard_limit':
            return
        result = self.abort_release = dict(attempted=False, completed=False,
                                           target_raw=self.start_raw, reason=state.reason)
        try:
            if not self._prepared:
                raise RuntimeError('Press preparation incomplete; opening reference not established')
            if not stop_acknowledged:
                raise RuntimeError('STOP acknowledgment unavailable')
            self.check_health(state, manual_stop)
            value = status()
            self.probe.require_ready_status(value)
            if value['g_gto']:
                raise RuntimeError('STOP echo not confirmed before abort release')
            if self.start_raw > value['g_po']:
                raise RuntimeError('Abort release target would close gripper; opening refused')
            result['start_raw'] = value['g_po']
            event('abort_release_start', **result)
            result['attempted'] = True
            # The original hard-limit reason stays latched. Opening unloads the
            # sample; freshness, manual stop, serial/fault and timeout guards stay active.
            self.move(port, status, event, state, manual_stop,
                      self.start_raw, 'abort_release', False)
            result['completed'] = True
            event('abort_release_finished', **result)
        except BaseException as exc:
            result['error'] = type(exc).__name__+': '+str(exc)
            event('abort_release_failed', **result)

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
