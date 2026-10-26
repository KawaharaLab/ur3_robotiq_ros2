#!/usr/bin/env python3
"""Experimental MMS101-triggered Robotiq stop probe. Linux, ROS Humble, exclusive serial ownership.
No arm commands, activation or reset. Standalone CLI is read-only by default and
never reopens automatically. An explicitly authorized collection hook may prepare/release.
"""
import argparse
import collections
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import statistics
import struct
import sys
import termios
import threading
import time


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def frame(slave, pdu):
    data = bytes([slave]) + pdu
    return data + struct.pack('<H', crc16(data))


def motion_pdu(position, speed, force, go=True):
    if any(type(v) is not int or not 0 <= v <= 255 for v in (position, speed, force)):
        raise ValueError('position/speed/force must be register bytes')
    # rACT stays 1; rGTO is 0 for STOP. Never rATR or reset.
    payload = bytes([0x09 if go else 0x01, 0, 0, position, speed, force])
    return struct.pack('>BHHB', 0x10, 0x03e8, 3, 6) + payload


def decode_status(response):
    if len(response) != 11 or response[1:3] != b'\x03\x06':
        raise ValueError('invalid status response')
    s = response[3]
    return {'g_act': s & 1, 'g_gto': (s >> 3) & 1, 'g_sta': (s >> 4) & 3,
            'g_obj': (s >> 6) & 3, 'g_flt': response[5], 'g_pr': response[6],
            'g_po': response[7], 'g_cu': response[8]}


def require_ready_status(value):
    if value['g_act'] != 1 or value['g_sta'] != 3 or value['g_flt'] != 0:
        raise RuntimeError('Gripper must be activated and fault-free: '
                           f"gACT={value['g_act']}, gSTA={value['g_sta']}, gFLT=0x{value['g_flt']:02x}; "
                           'probe will not reset/activate it')
    if value['g_gto'] and value['g_obj'] == 0:
        raise RuntimeError('Gripper is already moving')


def recover_handoff_timeout(initial, read_status, event, canceled, timeout_s=1.):
    """Read only: allow a stale 0x09 echo to clear after exclusive serial handoff.
    No write, reset, activation, or ignoring persistent/other faults.
    """
    value = initial
    deadline = time.monotonic() + timeout_s
    if value['g_flt'] == 9:
        event('handoff_communication_timeout_observed', status=value)
    while value['g_act'] == 1 and value['g_sta'] == 3 and value['g_flt'] == 9:
        if value['g_gto'] and value['g_obj'] == 0:
            raise RuntimeError('Gripper is already moving')
        if canceled(): raise KeyboardInterrupt('Canceled during handoff status check')
        if time.monotonic() >= deadline: break
        time.sleep(.05)
        value = read_status()
    require_ready_status(value)
    if initial['g_flt'] == 9:
        event('handoff_communication_timeout_cleared', status=value)
    return value


class Port:
    """One request at a time, one bounded attempt. Does not share the standard node's port."""
    def __init__(self, name, baud, slave, timeout_s):
        self.name = str(Path(name).resolve(strict=True))
        self.fd = None
        self.exclusive = False
        self.slave, self.timeout_s = slave, timeout_s
        # TIOCEXCL prevents later opens, not already-open descriptors. Check those first.
        for proc in Path('/proc').glob('[0-9]*'):
            if proc.name == str(os.getpid()):
                continue
            try:
                for fd in (proc/'fd').iterdir():
                    try:
                        if os.path.realpath(fd) == self.name:
                            raise RuntimeError(f'{self.name} already open by PID {proc.name}; stop its node first')
                    except OSError:
                        pass
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                pass
        fd = os.open(self.name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.fd = fd
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            self.exclusive = True
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = getattr(termios, 'B'+str(baud))
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            try:
                if self.exclusive:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError:
                pass
            finally:
                os.close(fd)

    def exchange(self, pdu, response_size):
        request = frame(self.slave, pdu)
        deadline = time.monotonic() + self.timeout_s
        termios.tcflush(self.fd, termios.TCIFLUSH)
        sent = 0
        while sent < len(request):
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError('Modbus write timeout')
            try:
                n = os.write(self.fd, request[sent:])
                if n <= 0: raise OSError('zero-length serial write')
                sent += n
            except BlockingIOError:
                continue
        response = bytearray()
        while len(response) < response_size:
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
                raise TimeoutError('Modbus response timeout')
            try:
                chunk = os.read(self.fd, response_size-len(response))
            except BlockingIOError:
                continue
            if not chunk: continue
            response.extend(chunk)
            if len(response) >= 2 and response[1] & 0x80:
                response_size = 5
        if response[0] != self.slave or crc16(response[:-2]) != int.from_bytes(response[-2:], 'little'):
            raise ValueError('Modbus address/CRC mismatch')
        if response[1] & 0x80:
            raise RuntimeError(f'Modbus exception {response[2]}')
        if response[1] != pdu[0]:
            raise ValueError('Modbus function mismatch')
        if pdu[0] == 0x10 and response[2:6] != pdu[1:5]:
            raise ValueError('Modbus write echo mismatch')
        return bytes(response)

    def status(self):
        return decode_status(self.exchange(struct.pack('>BHH', 3, 0x07d0, 3), 11))

    def command(self, position, speed, force, go):
        return self.exchange(motion_pdu(position, speed, force, go), 8)


class ForceState:
    """Raw, tared Fy. Stop decision happens in the receiving callback; it is latched."""
    def __init__(self, threshold, hard_limit, stale_ms, metric, target_confirm_ms=0.):
        self.lock = threading.RLock()
        self.latest = {}
        self.offsets = {}
        self.logs = collections.deque(maxlen=200000)
        self.log_overflow = 0
        self.threshold, self.hard_limit = threshold, hard_limit
        self.stale_ns, self.metric = round(stale_ms*1e6), metric
        self.armed = False
        self.reason = None
        self.trigger = None
        self.target_confirm_ns = round(target_confirm_ms*1e6)
        self.target_candidates = {}
        self.target_enabled = True

    def update(self, side, header_ns, received_ns, ros_now_ns, wrench):
        with self.lock:
            if len(self.logs) == self.logs.maxlen: self.log_overflow += 1
            row = {'side': side, 'header_ns': header_ns, 'receive_monotonic_ns': received_ns,
                   'receive_ros_ns': ros_now_ns, 'wrench': list(wrench)}
            self.logs.append(row)
            self.latest[side] = row
            if self.armed:
                self._evaluate(received_ns)

    def values(self):
        return {s: abs(r['wrench'][1]-self.offsets.get(s, 0.0)) for s, r in self.latest.items()}

    def _latch(self, reason, now):
        if self.reason is None:
            self.reason = reason
            self.trigger = {'reason': reason, 'monotonic_ns': now,
                            'force_n': self.values(),
                            'target_confirm_ms': self.target_confirm_ns/1e6,
                            'target_confirmation_reset_ratio': .8,
                            'target_evidence': dict(self.target_candidates),
                            'source_headers_ns': {s:r['header_ns'] for s,r in self.latest.items()}}

    def _evaluate(self, now):
        if len(self.latest) != 2:
            self._latch('missing_force', now)
            return
        for row in self.latest.values():
            if now-row['receive_monotonic_ns'] > self.stale_ns:
                self._latch('stale_force', now)
                return
            age = row['receive_ros_ns']-row['header_ns']
            if age > self.stale_ns or age < -10_000_000:
                self._latch('invalid_force_timestamp', now)
                return
        values = self.values()
        if not all(math.isfinite(v) for v in values.values()):
            self._latch('invalid_force_value', now)
        elif max(values.values()) >= self.hard_limit:
            self._latch('force_hard_limit', now)
        elif self.target_enabled and self._target_confirmed(values):
            self._latch('force_threshold', now)

    def _target_confirmed(self, values):
        if not self.target_confirm_ns:
            return (max(values.values()) if self.metric == 'max' else sum(values.values())/2) >= self.threshold
        # Collection uses max: each side must supply its own consecutive evidence.
        # Other-side callbacks and main-loop checks cannot advance this duration.
        if self.metric != 'max': raise ValueError('Confirmation currently requires max metric')
        reached = False
        for side, row in self.latest.items():
            received, header = row['receive_monotonic_ns'], row['header_ns']
            old = self.target_candidates.get(side)
            # Hysteresis tolerates small threshold-edge fluctuations; a return
            # below 80% cancels the candidate. Confirmation itself still requires
            # the current sample to be at or above the target.
            if values[side] < self.threshold * (.8 if old is not None else 1.):
                self.target_candidates.pop(side, None)
                continue
            if old is None or received-old[2] > max(10_000_000, 2*self.target_confirm_ns) or header-old[3] > max(10_000_000, 2*self.target_confirm_ns) or header <= old[3] and received != old[2]:
                old = (received, header, received, header, 1)
            elif received != old[2]:
                old = (old[0], old[1], received, header, old[4]+1)
            self.target_candidates[side] = old
            reached |= (values[side] >= self.threshold and old[4] >= 3
                        and received-old[0] >= self.target_confirm_ns and header-old[1] >= self.target_confirm_ns)
        return reached

    def check(self, now):
        with self.lock:
            if self.armed: self._evaluate(now)
            return self.reason

    def request_stop(self, reason):
        with self.lock:
            self._latch(reason, time.monotonic_ns())


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [json_safe(v) for v in value]
    return value


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True, help='Same port used by the normal gripper node; stop that node first')
    p.add_argument('--baud', type=int, default=115200, choices=[9600,19200,38400,57600,115200])
    p.add_argument('--slave', type=int, default=9)
    p.add_argument('--out', type=Path, required=True, help='Use ./force_stop_run1 etc. No existing directory is overwritten')
    p.add_argument('--enable-motion', action='store_true', help='Without this, only status and force are read')
    p.add_argument('--close-ticks', type=int, default=4, help='Maximum additional raw gPO/rPR counts; NOT millimetres')
    p.add_argument('--speed-byte', type=int, default=0, help='0=hardware minimum speed, not zero speed')
    p.add_argument('--force-byte', type=int, default=0, help='Hardware minimum effort; not 0 N or a calibrated force limit')
    p.add_argument('--threshold-n', type=float, default=.5)
    p.add_argument('--hard-limit-n', type=float, default=1.)
    p.add_argument('--force-metric', choices=['max','mean'], default='max')
    p.add_argument('--stale-ms', type=float, default=100.)
    p.add_argument('--serial-timeout-ms', type=float, default=30.)
    p.add_argument('--max-motion-ms', type=float, default=1000.)
    p.add_argument('--stop-after-ms', type=float, default=None, help='No-contact timed stop test; still respects force limits')
    p.add_argument('--hold-seconds', type=float, default=4.)
    a = p.parse_args()
    for k in ['threshold_n','hard_limit_n','stale_ms','serial_timeout_ms','max_motion_ms','hold_seconds']:
        if not math.isfinite(getattr(a,k)): p.error(k+' must be finite')
    if not (0 < a.threshold_n < a.hard_limit_n): p.error('require 0 < threshold < hard limit')
    if not 1 <= a.close_ticks <= 20: p.error('--close-ticks must be 1..20')
    if not 0 <= a.speed_byte <= 255 or not 0 <= a.force_byte <= 255: p.error('register bytes must be 0..255')
    if not 1 <= a.slave <= 247: p.error('--slave must be 1..247')
    if not 5 <= a.serial_timeout_ms <= 100: p.error('serial timeout must be 5..100 ms')
    if not 10 <= a.stale_ms <= 200: p.error('stale-ms must be 10..200')
    if not 1 <= a.max_motion_ms <= 10000: p.error('max motion must be 1..10000 ms')
    if not 0 <= a.hold_seconds <= 10: p.error('hold-seconds must be 0..10')
    if a.stop_after_ms is not None and (not math.isfinite(a.stop_after_ms) or not 0 < a.stop_after_ms <= a.max_motion_ms):
        p.error('stop-after-ms must be positive and <= max-motion-ms')
    if not sys.platform.startswith('linux'): p.error('Linux workstation only')
    if a.enable_motion and not sys.stdin.isatty(): p.error('Motion requires an interactive terminal for GO/Enter stop')
    return a


def run(a, collection=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import WrenchStamped
    a.out.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('force_stop_probe')
    if node.get_parameter('use_sim_time').value:
        raise RuntimeError('Physical probe requires use_sim_time=false')
    state = ForceState(a.threshold_n, a.hard_limit_n, a.stale_ms, a.force_metric,
                       getattr(a, 'target_confirm_ms', 0.))
    if collection is not None: collection.setup(node)
    events, statuses = [], []
    stop_watch = threading.Event()
    manual_stop = threading.Event()
    def event(name, **data):
        row = {'event':name,'monotonic_ns':time.monotonic_ns(),'ros_ns':node.get_clock().now().nanoseconds,**data}
        events.append(row)
        if collection is not None: collection.on_event(row)
        return row
    def callback(side):
        def receive(msg):
            w = msg.wrench
            state.update(side, msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                         time.monotonic_ns(),node.get_clock().now().nanoseconds,
                         (w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z))
        return receive
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [node.create_subscription(WrenchStamped, '/force_torque/'+s, callback(s), qos) for s in ('left','right')]
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def request_manual(reason):
        manual_stop.set()
        state.request_stop(reason)
    previous_handlers = {}
    for sig in (signal.SIGINT,signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, lambda signum, stack: request_manual('manual_signal'))
    port = None
    motion_may_have_started = False
    stop_acknowledged = False
    initial_status = None
    error = None
    keyboard_thread = None
    exit_code = 0
    def confirm(text):
        print('Type '+text+': ',end='',flush=True)
        while not manual_stop.is_set():
            require_ready_status(status())
            if select.select([sys.stdin],[],[],.05)[0]: break
        if manual_stop.is_set(): raise KeyboardInterrupt('Canceled before command')
        return sys.stdin.readline().strip()==text
    def status():
        before = time.monotonic_ns()
        value = port.status()
        record = {'request_monotonic_ns':before,'response_monotonic_ns':time.monotonic_ns(),**value}
        statuses.append(record)
        if collection is not None: collection.on_status(record)
        return value
    def stop():
        nonlocal stop_acknowledged
        event('stop_write_start', reason=state.reason)
        port.command(initial_status['g_po'],a.speed_byte,a.force_byte,False)
        stop_acknowledged = True
        event('stop_write_ack')
    try:
        print('Waiting for force topics; do not press the sample during baseline.', flush=True)
        deadline = time.monotonic()+5
        while len(state.latest)<2 and time.monotonic()<deadline and state.reason is None:
            time.sleep(.01)
        if len(state.latest)<2: raise RuntimeError('Both force topics are required')
        # Discovery guard supplements the /proc and TIOCEXCL checks; does not replace operator exclusivity.
        owner_suspended = collection is not None and getattr(collection, 'allows_suspended_driver', lambda: False)()
        if node.count_publishers('/robotiq_2f_gripper/status') and not owner_suspended:
            raise RuntimeError('Normal gripper status publisher is active. Stop the gripper node first.')
        port = Port(a.port,a.baud,a.slave,a.serial_timeout_ms/1000)
        initial_status = status()
        event('handoff_initial_status', status=initial_status)
        initial_status = recover_handoff_timeout(initial_status, status, event, manual_stop.is_set)
        if collection is not None:
            if not a.enable_motion: raise RuntimeError('Collection requires explicit motion authorization')
            collection.prepare(port, status, event, state, manual_stop)
            initial_status = status()
            require_ready_status(initial_status)
            if getattr(collection, 'is_stepped', False):
                state.target_enabled = False  # hard/freshness guards remain active
        target = min(255,initial_status['g_po']+a.close_ticks)
        if collection is not None: target = collection.bound_target(target)
        if target==initial_status['g_po']: raise RuntimeError('No closing range remains')
        print(f'Current raw position={initial_status["g_po"]}; bounded target={target}; speed byte={a.speed_byte}.', flush=True)
        if not a.enable_motion:
            event('read_only_check',status=initial_status)
            print('READ ONLY: no motion/activation/stop command sent. Use --enable-motion only for a deliberate test.',flush=True)
            return 0
        print('Pressing starts only after GO. Enter stops after start; Ctrl+C also requests STOP. Hardware stop remains independent.',flush=True)
        if collection is None and not confirm('GO'):
            event('not_started');return 0
        if state.reason: raise RuntimeError('Stop was requested before start')
        begin = time.monotonic_ns()
        event('baseline_start')
        while time.monotonic_ns()-begin<1_000_000_000 and state.reason is None:
            require_ready_status(status())
            time.sleep(.05)
        with state.lock:
            rows = [r for r in state.logs if r['receive_monotonic_ns']>=begin]
            for side in ('left','right'):
                values = [r['wrench'][1] for r in rows if r['side']==side]
                if len(values)<100 or not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Insufficient valid baseline samples')
                state.offsets[side] = statistics.median(values)
            state.armed = True
            state._evaluate(time.monotonic_ns())
        if state.reason: raise RuntimeError('Force/timestamp check failed before motion: '+state.reason)
        event('baseline_ready', offsets=dict(state.offsets), target_confirm_ms=getattr(a, 'target_confirm_ms', 0.))
        keyboard_input = sys.stdin
        def keyboard():
            while not stop_watch.is_set():
                try:
                    if select.select([keyboard_input],[],[],.02)[0]:
                        keyboard_input.readline();request_manual('manual_enter');return
                except (OSError, ValueError):
                    if not stop_watch.is_set():request_manual('manual_input_closed')
                    return
        keyboard_thread = threading.Thread(target=keyboard,daemon=True)
        if collection is None: keyboard_thread.start()
        else: keyboard_thread = None
        if state.check(time.monotonic_ns()) is not None or manual_stop.is_set():
            event('not_started_after_guard');return 2
        if collection is not None and getattr(collection, 'is_stepped', False):
            motion_may_have_started = True  # includes lost per-step GO ACK
            event('motion_write_start', target_raw=target, press_mode='stepped')
            collection.run_stepped_press(port, status, event, state, manual_stop, initial_status, target)
        else:
            motion_event = event('motion_write_start',target_raw=target)
            motion_may_have_started = True  # even an ACK timeout can leave a command executing
            port.command(target,a.speed_byte,a.force_byte,True)
            started = motion_event['monotonic_ns']
            event('motion_write_ack')
            next_status = time.monotonic()+.05
            while state.check(time.monotonic_ns()) is None:
                elapsed_ms = (time.monotonic_ns()-started)/1e6
                if a.stop_after_ms is not None and elapsed_ms>=a.stop_after_ms:
                    state.request_stop('timed_stop');break
                if elapsed_ms>=a.max_motion_ms:
                    state.request_stop('motion_timeout');break
                if time.monotonic()>=next_status:
                    s = status()
                    next_status = time.monotonic()+.05
                    if s['g_flt']: state.request_stop('gripper_fault')
                    elif s['g_po']>=target or s['g_obj'] in (1,2): state.request_stop('position_or_hardware_contact_limit')
                time.sleep(.001)
        stop()
        # Observe controller state and quantized position. This is NOT a high-speed physical motion measurement.
        if collection is not None: event('hold_start', stop_reason=state.reason)
        hold_end = time.monotonic()+max(.2, a.hold_seconds if state.reason in ('force_threshold','timed_stop') else .2)
        clear_seen = False
        first_observation = True
        while first_observation or (time.monotonic()<hold_end and not manual_stop.is_set()):
            first_observation = False
            s = status()
            if not s['g_gto'] and not clear_seen:
                event('controller_gto_clear_observed',g_po=s['g_po']);clear_seen=True
            if s['g_flt']:event('post_stop_fault',fault=s['g_flt']);break
            if collection is not None: collection.check_health(state, manual_stop)
            time.sleep(.05)
        event('observation_end',controller_stop_echo_seen=clear_seen)
        if not clear_seen: raise RuntimeError('No gGTO=0 echo observed after stop')
        if collection is not None:
            if manual_stop.is_set(): raise KeyboardInterrupt('Manual stop during collection hold')
            collection.finish(port, status, event, state, manual_stop)
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        exit_code = 2
        state.request_stop('exception')
        event('exception',error=error)
    finally:
        if motion_may_have_started and (error is not None or not stop_acknowledged) and port is not None:
            # Bounded best effort; a disconnected cable cannot be stopped by software.
            for attempt in range(2):
                try:
                    stop();break
                except Exception as exc:
                    event('stop_attempt_failed',attempt=attempt+1,error=str(exc))
        stop_watch.set()
        if keyboard_thread is not None:keyboard_thread.join(timeout=.1)
        if port is not None:port.close()
        executor.shutdown(timeout_sec=1.)
        thread.join(timeout=1.)
        node.destroy_node()
        rclpy.shutdown()
        for sig,handler in previous_handlers.items():signal.signal(sig,handler)
        with state.lock:
            force_rows = list(state.logs);trigger=state.trigger
        summary = {'schema_version':1,'parameters':{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
                   'initial_status':initial_status,'motion_may_have_started':motion_may_have_started,
                   'stop_acknowledged':stop_acknowledged,'stop_trigger':trigger,'manual_stop_requested':manual_stop.is_set(),'error':error,
                   'force_log_overflow':state.log_overflow,'offsets':state.offsets,
                   'note':'Host callback/Modbus ACK/controller echo only; not calibrated physical stopping time. Raw register counts are not indentation mm. No automatic reopen.'}
        if collection is not None:
            summary['collection'] = collection.summary()
            summary['press_motion_may_have_started'] = motion_may_have_started
            summary['motion_may_have_started'] = motion_may_have_started or summary['collection'].get('auxiliary_motion_commanded', False)
            summary['note'] = 'Host callback/Modbus ACK/controller echo only; not physical stopping time. Collection may prepare and release after successful force stop and hold. Width conversion is empirical.'
            summary['post_trigger_peak_scope'] = 'All remaining received samples, including hold and release; use events to isolate pressing overshoot.'
        if trigger:
            first = next((e for e in events if e['event']=='stop_write_start'),None)
            ack = next((e for e in events if e['event']=='stop_write_ack'),None)
            summary['trigger_to_stop_write_start_ms'] = (first['monotonic_ns']-trigger['monotonic_ns'])/1e6 if first else None
            summary['trigger_to_stop_ack_ms'] = (ack['monotonic_ns']-trigger['monotonic_ns'])/1e6 if ack else None
            if state.offsets:
                for side in ('left','right'):
                    values=[abs(r['wrench'][1]-state.offsets[side]) for r in force_rows if r['side']==side and r['receive_monotonic_ns']>=trigger['monotonic_ns']]
                    summary['post_trigger_peak_'+side+'_n']=max(values) if values else None
        (a.out/'summary.json').write_text(json.dumps(json_safe(summary),indent=2,allow_nan=False)+'\n')
        for name,rows in [('force.jsonl',force_rows),('events.jsonl',events),('status.jsonl',statuses)]:
            with (a.out/name).open('w') as f:
                for row in rows:f.write(json.dumps(json_safe(row),allow_nan=False)+'\n')
        print(f'Output: {a.out.resolve()}\nStop acknowledged: {stop_acknowledged}; reason: {state.reason}; error: {error}',flush=True)
    return exit_code


if __name__=='__main__':
    raise SystemExit(run(arguments()))
