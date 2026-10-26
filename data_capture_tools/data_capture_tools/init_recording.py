"""Init provenance and bounded asynchronous logging; no robot commands in this module."""
import collections
from datetime import datetime
import gzip
import hashlib
import json
import math
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid


def safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    return value


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(safe(value), indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    temporary.replace(path)


class InitLog:
    def __init__(self, root, metadata, queue_size=20000):
        self.init_id = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:12]
        self.path = Path(root)/self.init_id
        self.path.mkdir(parents=True, exist_ok=False)  # fail before any init movement
        self.metadata = metadata
        self.lock = threading.Lock()
        self.queue = queue.Queue(maxsize=queue_size)
        self.accepting = True
        self.dropped = 0
        self.written = 0
        self.error = None
        self.counts = collections.Counter()
        self.ready = threading.Event()
        write_json(self.path/'summary.json', dict(schema_version=1, init_id=self.init_id,
                   status='in_progress', motion_success=None, recording_complete=False,
                   valid_for_collection=False, metadata=metadata))
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()
        if not self.ready.wait(timeout=2.) or self.error is not None:
            self.accepting = False
            self.queue.put_nowait(None)
            write_json(self.path/'summary.json', dict(schema_version=1, init_id=self.init_id,
                       status='failed_before_motion', motion_success=False, recording_complete=False,
                       valid_for_collection=False, writer_error=self.error or 'Writer open timed out',
                       metadata=metadata))
            raise RuntimeError('Init stream could not be opened: '+str(self.error))

    def emit(self, row):
        with self.lock:
            if not self.accepting:
                return
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                self.dropped += 1

    def _write(self):
        try:
            with gzip.open(self.path/'samples.jsonl.gz', 'wt', encoding='utf-8', compresslevel=3) as f:
                self.ready.set()
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    f.write(json.dumps(safe(row), allow_nan=False)+'\n')
                    self.written += 1
                    self.counts[row['kind']] += 1
        except Exception as e:
            self.error = repr(e)
            self.ready.set()

    def close(self, success, contact, error=None):
        with self.lock:
            self.accepting = False
        try:
            self.queue.put(None, timeout=5.)
        except queue.Full:
            self.error = self.error or 'Writer queue did not drain'
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            self.error = self.error or 'Writer did not finish'
        complete = self.error is None and self.dropped == 0
        result = dict(schema_version=1, init_id=self.init_id, status='finished', motion_success=bool(success),
                      recording_complete=complete, error=error, writer_error=self.error,
                      dropped_log_rows=self.dropped, written_rows=self.written,
                      row_counts=dict(self.counts), contact=contact, metadata=self.metadata,
                      note='Observed callbacks only; not a complete 1000 Hz stream or physical synchronization. '
                           'Contact snapshots contain asynchronous latest samples; ages are explicit. '
                           'Widths use the unchanged legacy approximation, not calibrated GelSight surface distance.')
        if not self.thread.is_alive() and (self.path/'samples.jsonl.gz').exists():
            result['samples_sha256'] = hashlib.sha256((self.path/'samples.jsonl.gz').read_bytes()).hexdigest()
        write_json(self.path/'summary.json', result)
        return result


class InitRecordingMixin:
    def _setup_init_recording(self):
        from robotiq_2f_gripper_msgs.msg import GripperStatus
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self._init_record_lock = threading.RLock()
        self._active_init_log = None
        self._last_successful_init = None
        self._init_contact = None
        self._init_latest = {}
        self._init_startup = None
        self._init_driver_changed = False
        self._init_node_run_id = uuid.uuid4().hex
        self._init_record_root = Path(self.declare_parameter('init_record_dir', str(Path.cwd()/'init_records')).value).expanduser().resolve()
        self._init_reference_id = self.declare_parameter('init_reference_id', '').value
        self._init_reference_width_mm = float(self.declare_parameter('init_reference_width_mm', 0.).value)
        self._init_mount_id = self.declare_parameter('init_mount_id', '').value
        if not math.isfinite(self._init_reference_width_mm) or self._init_reference_width_mm < 0:
            raise ValueError('init_reference_width_mm must be finite and nonnegative; 0 means unknown')
        self._init_status_sub = self.create_subscription(GripperStatus, '/robotiq_2f_gripper/status', self._init_status_callback, 100)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._init_startup_sub = self.create_subscription(String, '/robotiq_2f_gripper/initialization', self._init_startup_callback, qos)
        self._init_command_sub = self.create_subscription(String, '/robot_cmd',
            lambda msg: self._init_observe('robot_command', command=msg.data), 100)

    def _init_observe(self, kind, **data):
        row = dict(kind=kind, receive_monotonic_ns=time.monotonic_ns(),
                   receive_ros_ns=self.get_clock().now().nanoseconds, **data)
        with self._init_record_lock:
            self._init_latest[kind] = row
            if self._active_init_log is not None:
                self._active_init_log.emit(row)

    def _init_force_callback(self, side, msg):
        w = msg.wrench
        self._init_observe('force_'+side,
                           header_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                           wrench=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z])

    def _init_status_callback(self, msg):
        self._init_observe('gripper_status',
                           header_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                           status={k:int(getattr(msg,k)) for k in ('g_pr','g_po','g_obj','g_flt','g_cu')},
                           read_duration_ms=float(msg.read_duration_ms))

    def _invalidate_init(self):
        self._last_successful_init = None
        self._effective_object_width_m = None
        self._calibrated_target_m = None
        self._saved_base_joints = None

    def _init_startup_callback(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value.get('startup_id'), str):
                raise ValueError('missing startup_id')
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warn('Invalid gripper initialization record; activation history unknown.')
            return
        with self._init_record_lock:
            previous = self._init_startup
            self._init_startup = value
            changed = previous is not None and previous['startup_id'] != value['startup_id']
            # Late discovery after init cannot establish that the old result belongs to this driver.
            late = previous is None and (self._active_init_log is not None or self._last_successful_init is not None)
            if changed or late:
                self._init_driver_changed = True
                self._invalidate_init()
                self.get_logger().warn('Gripper driver startup changed/discovered after init began; run init again.')
        self._init_observe('driver_initialization', data=value)

    def _init_force_snapshot(self):
        m = self._force_monitor
        left, right = tuple(m._history_left), tuple(m._history_right)
        means = {side:sum(values)/len(values) if values else None
                 for side, values in (('left',left),('right',right))}
        offsets = dict(left=m._offset_left, right=m._offset_right)
        return dict(method='absolute difference of trailing 20-sample Fy mean and tare',
                    offsets_n=offsets, trailing_mean_fy_n=means,
                    samples=dict(left=len(left),right=len(right)),
                    calibrated_n={s:abs(v-offsets[s]) if v is not None else None for s,v in means.items()})

    def _init_record_tare(self):
        self._init_observe('tare', force_monitor=self._init_force_snapshot())

    def _mark_init_contact(self, effective_width_m):
        with self._init_record_lock:
            now = time.monotonic_ns()
            sources = {}
            for kind in ('gripper_status','force_left','force_right','width','centering_result'):
                row = self._init_latest.get(kind)
                if row is not None:
                    sources[kind] = dict(row, receive_age_ms=(now-row['receive_monotonic_ns'])/1e6)
                    if 'header_ns' in row:
                        sources[kind]['header_age_at_receive_ms'] = (row['receive_ros_ns']-row['header_ns'])/1e6
            required = ('gripper_status','force_left','force_right')
            warnings = []
            for kind in required:
                row = sources.get(kind)
                if row is None or not 0 <= row['receive_age_ms'] <= 250:
                    warnings.append(kind+': missing or older than 250 ms')
                elif not -10 <= row['header_age_at_receive_ms'] <= 250:
                    warnings.append(kind+': source timestamp outside accepted range')
                if row is not None and kind.startswith('force_') and not all(math.isfinite(v) for v in row['wrench']):
                    warnings.append(kind+': nonfinite wrench')
            if sources.get('gripper_status',{}).get('status',{}).get('g_flt',0):
                warnings.append('gripper reports fault')
            monitor = self._init_force_snapshot()
            if any(v is None or not math.isfinite(v) for v in monitor['calibrated_n'].values()):
                warnings.append('invalid force monitor snapshot')
            self._init_contact = dict(captured_monotonic_ns=now,
                                      captured_ros_ns=self.get_clock().now().nanoseconds,
                                      effective_width_m=effective_width_m,
                                      sources=sources, force_monitor=monitor,
                                      usable=not warnings, warnings=warnings)
            self._init_observe('contact_before_reopen', snapshot=self._init_contact)

    def _run_init_sequence(self):
        with self._init_record_lock:
            self._invalidate_init()
            self._init_contact = None
            self._init_driver_changed = False
            startup = self._init_startup
            metadata = dict(node_run_id=self._init_node_run_id, reference_id=self._init_reference_id,
                            reference_width_mm=self._init_reference_width_mm or None,
                            mount_id=self._init_mount_id, driver_initialization=startup,
                            activation_history_known=startup is not None,
                            config=self.config.raw,
                            conversion_model='legacy_piecewise_polynomial_v1_uncalibrated',
                            observer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
            manager_source = Path(__file__).with_name('data_capture_node_10times.py')
            if manager_source.exists():
                metadata['manager_sha256'] = hashlib.sha256(manager_source.read_bytes()).hexdigest()
            try:
                log = InitLog(self._init_record_root, metadata)
            except Exception as e:
                self.get_logger().error('Cannot create init record; no motion started: '+repr(e))
                return False
            self._active_init_log = log
            self._init_observe('init_start')
        self.get_logger().info('Init record: '+str(log.path))
        success, error = False, None
        try:
            success = bool(self._run_init_motion_sequence())
        except BaseException as e:
            error = repr(e)
            raise
        finally:
            with self._init_record_lock:
                valid = success and not self._init_driver_changed and self._init_contact is not None and self._init_contact['usable']
                log.emit(dict(kind='init_end', receive_monotonic_ns=time.monotonic_ns(),
                              receive_ros_ns=self.get_clock().now().nanoseconds, motion_success=success))
                self._active_init_log = None
            try:
                result = log.close(success, self._init_contact, error)
                with self._init_record_lock:
                    result['valid_for_collection'] = bool(valid and not self._init_driver_changed and result['recording_complete'])
                    result['driver_changed_during_init'] = self._init_driver_changed
                    write_json(log.path/'summary.json', result)
                    if result['valid_for_collection']:
                        self._last_successful_init = log.path
                    else:
                        self._invalidate_init()
                        self.get_logger().error('Init result not reusable; inspect '+str(log.path/'summary.json'))
            except Exception as e:
                self._invalidate_init()
                self.get_logger().error('Could not finalize init record; run init again: '+repr(e))
        return success and self._last_successful_init == log.path

    def _attach_init_record(self, session_dir):
        with self._init_record_lock:
            source = self._last_successful_init
            if source is None:
                write_json(session_dir/'init_reference.json', dict(schema_version=1, available=False,
                                                                  reason='No valid recorded init in this manager run'))
                return
            summary = json.loads((source/'summary.json').read_text())
            if not summary.get('valid_for_collection'):
                raise RuntimeError('Recorded init is not valid for collection')
            if hashlib.sha256((source/'samples.jsonl.gz').read_bytes()).hexdigest() != summary['samples_sha256']:
                raise RuntimeError('Init stream checksum changed')
            relative = Path('init')/source.name
            reference_path = session_dir/'init_reference.json'
            if reference_path.exists():
                previous = json.loads(reference_path.read_text())
                if previous.get('available') and previous.get('init_id') != summary['init_id']:
                    raise RuntimeError('Existing session refers to a different init; use a new session')
            destination = session_dir/relative
            destination.parent.mkdir(exist_ok=True)
            if not destination.exists():
                shutil.copytree(source, destination)
            elif (destination/'summary.json').read_bytes() != (source/'summary.json').read_bytes():
                raise RuntimeError('Conflicting init copy in session')
            if hashlib.sha256((destination/'samples.jsonl.gz').read_bytes()).hexdigest() != summary['samples_sha256']:
                raise RuntimeError('Copied init stream checksum mismatch')
            write_json(session_dir/'init_reference.json', dict(schema_version=1, available=True,
                                                               init_id=summary['init_id'], summary=str(relative/'summary.json'),
                                                               samples=str(relative/'samples.jsonl.gz')))
