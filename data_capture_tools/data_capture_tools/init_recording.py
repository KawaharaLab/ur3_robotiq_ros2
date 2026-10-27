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

from .acquisition_provenance import read_capture_measurements, runtime_provenance, legacy_conversion_description, effective_capture_config
from .force_press_worker import raw_to_width_m, width_to_raw


def safe(value):
    if isinstance(value, Path):
        return str(value)
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
        metadata_path = self.declare_parameter('specimen_metadata_file', '').value
        self._specimen_metadata_path = Path(metadata_path).expanduser().resolve() if metadata_path else None
        self._init_selected_point = None
        if not math.isfinite(self._init_reference_width_mm) or self._init_reference_width_mm < 0:
            raise ValueError('init_reference_width_mm must be finite and nonnegative; 0 means unknown')
        self._init_status_sub = self.create_subscription(GripperStatus, '/robotiq_2f_gripper/status', self._init_status_callback, 100)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._init_startup_sub = self.create_subscription(String, '/robotiq_2f_gripper/initialization', self._init_startup_callback, qos)
        self._init_driver_command_sub = self.create_subscription(String, '/robotiq_2f_gripper/command_event',
            self._init_driver_command_callback, 100)
        self._init_command_sub = self.create_subscription(String, '/robot_cmd',
            lambda msg: self._init_observe('robot_command', command=msg.data), 100)
        self._init_sensor_metadata_subs = []
        for name, topic in (
            ('mms_left','/mms101_driver_1/acquisition_timing'),
            ('mms_right','/mms101_driver_2/acquisition_timing'),
            ('gelsight_left','/gelsight_mini_publisher_1/acquisition_timing'),
            ('gelsight_right','/gelsight_mini_publisher_2/acquisition_timing')):
            self._init_sensor_metadata_subs.append(self.create_subscription(String, topic,
                lambda msg, name=name, topic=topic: self._init_sensor_metadata_callback(name, topic, msg), 10))

    def _init_observe(self, kind, record=True, **data):
        row = dict(kind=kind, receive_monotonic_ns=time.monotonic_ns(),
                   receive_ros_ns=self.get_clock().now().nanoseconds, **data)
        with self._init_record_lock:
            self._init_latest[kind] = row
            if self._active_init_log is not None and record:
                self._active_init_log.emit(row)

    def _init_sensor_metadata_callback(self, name, topic, msg):
        try:
            report = json.loads(msg.data)
            if not isinstance(report.get('stream_id'), str):
                return
            fields = ('schema_version','kind','stream_id','node','port','frame_id','image_topic',
                      'clock_type','use_sim_time','host_monotonic_clock','configured_interval_us',
                      'force_scale','torque_scale','backend','opencv_version')
            metadata = {k:report.get(k) for k in fields if k in report}
            metadata['topic'] = topic
            key = 'sensor_metadata_'+name
            with self._init_record_lock:
                old = self._init_latest.get(key, {}).get('metadata')
                self._init_observe(key, record=old != metadata, metadata=metadata)
        except (ValueError, TypeError, AttributeError):
            self._init_observe('malformed_sensor_metadata', topic=topic)

    def _sensor_metadata_snapshot(self):
        with self._init_record_lock:
            now = time.monotonic_ns()
            return {k:dict(self._init_latest[k],receive_age_ms=(now-self._init_latest[k]['receive_monotonic_ns'])/1e6)
                    if k in self._init_latest else None
                    for k in ('sensor_metadata_mms_left','sensor_metadata_mms_right',
                              'sensor_metadata_gelsight_left','sensor_metadata_gelsight_right')}

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

    def _init_driver_command_callback(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('command event must be an object')
            self._init_observe('driver_command', data=value)
        except (ValueError, TypeError):
            self._init_observe('malformed_driver_command', raw=msg.data)

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

    def _init_state_snapshot(self, effective_width_m):
        with self._init_record_lock:
            now = time.monotonic_ns()
            sources = {}
            for kind in ('gripper_status','force_left','force_right','width','centering_result','centering_geometry','arm_joint_state','driver_command'):
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
            snapshot = dict(captured_monotonic_ns=now,
                                      captured_ros_ns=self.get_clock().now().nanoseconds,
                                      effective_width_m=effective_width_m,
                                      sources=sources, force_monitor=monitor,
                                      usable=not warnings, warnings=warnings)
            return snapshot

    def _mark_init_selection(self, best_state):
        snapshot = self._init_state_snapshot(best_state['w'])
        snapshot['selection_values'] = dict(best_state)
        status = snapshot['sources'].get('gripper_status', {}).get('status', {})
        count = status.get('g_po')
        snapshot['observed_count'] = count
        if count is not None:
            snapshot['reported_width_minus_count_estimate_mm'] = (best_state['w']-raw_to_width_m(count))*1000
            snapshot['status_width_consistent'] = abs(snapshot['reported_width_minus_count_estimate_mm']) < .01
        try:
            snapshot['legacy_restore_request_count_estimate'] = width_to_raw(best_state['w'])
        except ValueError:
            snapshot['legacy_restore_request_count_estimate'] = None
        self._init_selected_point = snapshot
        self._init_observe('best_contact_selected', snapshot=snapshot)

    def _mark_init_contact(self, effective_width_m):
        self._init_contact = self._init_state_snapshot(effective_width_m)
        self._init_observe('contact_before_reopen', snapshot=self._init_contact)

    def _run_init_sequence(self):
        # File parsing and checkout queries happen before motion and outside sensor locks.
        try:
            measurements = read_capture_measurements(self.config, getattr(self, '_specimen_metadata_path', None))
            provenance = runtime_provenance(getattr(self.config, 'source_path', None))
        except Exception as e:
            with self._init_record_lock:
                self._invalidate_init()
            self.get_logger().error('Cannot read specimen metadata; no motion started: '+repr(e))
            return False
        with self._init_record_lock:
            self._invalidate_init()
            self._init_contact = None
            self._init_selected_point = None
            self._init_latest.pop('centering_result', None)
            self._init_latest.pop('centering_geometry', None)
            self._init_driver_changed = False
            startup = self._init_startup
            metadata = dict(measurements=measurements, runtime=provenance,
                            sensor_metadata=self._sensor_metadata_snapshot(),
                            conversion=legacy_conversion_description(),
                            node_run_id=self._init_node_run_id, reference_id=self._init_reference_id,
                            reference_width_mm=self._init_reference_width_mm or None,
                            mount_id=self._init_mount_id, driver_initialization=startup,
                            activation_history_known=startup is not None,
                            config=self.config.raw,
                            effective_config=effective_capture_config(self.config),
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
                    result['schema_version'] = 2
                    result['record_quality_valid'] = result['valid_for_collection']
                    result['contact_criteria_met'] = (self._init_contact or {}).get('sources', {}).get('centering_result', {}).get('final_balanced_contact')
                    result['selected_contact'] = self._init_selected_point
                    result['sensor_metadata_at_end'] = self._sensor_metadata_snapshot()
                    selected_count = (self._init_selected_point or {}).get('observed_count')
                    final_count = (self._init_contact or {}).get('sources', {}).get('gripper_status', {}).get('status', {}).get('g_po')
                    result['selected_to_final_count_delta'] = final_count-selected_count if selected_count is not None and final_count is not None else None
                    result['validity_policy'] = 'valid_for_collection is a legacy alias for record_quality_valid; contact_criteria_met is reported separately and does not change init motion acceptance.'
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
        measurements = read_capture_measurements(self.config, getattr(self, '_specimen_metadata_path', None))
        provenance = runtime_provenance(getattr(self.config, 'source_path', None))
        topics = getattr(self.config, 'topics', None)
        if topics is not None:
            required = {'/robotiq_2f_gripper/status', '/robotiq_2f_gripper/command_event',
                        '/robot_action_event', '/force_torque/left', '/force_torque/right'}
            missing = required - {t.name for t in topics}
            if missing:
                raise RuntimeError('Config must record provenance topics: '+', '.join(sorted(missing)))
            if not self._init_startup or self._init_startup.get('command_event_topic') != '/robotiq_2f_gripper/command_event':
                raise RuntimeError('Updated gripper command-event publisher required; restart the rebuilt gripper driver')
        with self._init_record_lock:
            source = self._last_successful_init
            summary = json.loads((source/'summary.json').read_text()) if source is not None else None
            init_measurements = (summary or {}).get('metadata', {}).get('measurements', {})
            if source is not None and init_measurements.get('physical_condition_id') != measurements.get('physical_condition_id'):
                raise RuntimeError('Specimen/mount measurements changed since init; run init again before collection')
            session_metadata = dict(schema_version=1, measurements=measurements, runtime=provenance,
                                    sensor_metadata=self._sensor_metadata_snapshot(),
                                    config=self.config.raw, effective_config=effective_capture_config(self.config),
                                    conversion=legacy_conversion_description(),
                                    gripper_initialization=self._init_startup,
                                    init_id=summary['init_id'] if summary else None,
                                    selected_contact_count=(summary or {}).get('selected_contact', {}).get('observed_count') if (summary or {}).get('selected_contact') else None,
                                    selected_contact_snapshot_usable=(summary or {}).get('selected_contact', {}).get('usable') if (summary or {}).get('selected_contact') else None,
                                    final_reference_count=(summary or {}).get('contact', {}).get('sources', {}).get('gripper_status', {}).get('status', {}).get('g_po') if (summary or {}).get('contact') else None,
                                    record_quality_valid=(summary or {}).get('record_quality_valid', (summary or {}).get('valid_for_collection')),
                                    contact_criteria_met=(summary or {}).get('contact_criteria_met'),
                                    definitions=dict(count='Quantized position register, not physical indentation',
                                                     estimated_width_m='Legacy conversion, not calibrated contact-surface gap',
                                                     push_depth_m='Nominal per-side parameter; not measured isolated specimen strain',
                                                     measured_dimensions='User-entered, nullable; no automatic clamp measurement'))
            existing = session_dir/'collection_provenance.json'
            if existing.exists():
                old = json.loads(existing.read_text())
                if old.get('init_id') != session_metadata['init_id'] or old.get('measurements', {}).get('sha256') != measurements.get('sha256'):
                    raise RuntimeError('Existing session has different provenance; use a new session')
            else:
                write_json(existing, session_metadata)
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

    def _record_trial_provenance(self, event, index, mode, target_width_m, speed):
        if self._session_dir is None:
            return
        with self._init_record_lock:
            status = self._init_latest.get('gripper_status')
            selected = getattr(self, '_init_selected_point', None) if self._last_successful_init else None
            try:
                target_count = width_to_raw(target_width_m)
            except ValueError:
                target_count = None
            row = dict(schema_version=1, event=event, trial_index=index, mode=mode,
                       ros_ns=self.get_clock().now().nanoseconds, monotonic_ns=time.monotonic_ns(),
                       init_id=self._last_successful_init.name if self._last_successful_init else None,
                       marker_target_width_m=target_width_m, marker_speed_normalized=speed,
                       target_count_model_estimate=target_count, selected_contact_count=(selected or {}).get('observed_count'),
                       observed_status_at_marker=status,
                       note='Model estimate is not the actual sent target. Driver command_event contains sent register counts for the normal driver; af/afs commands are in force_press sidecars. Marker status may be open or in another phase.')
        # Called by the trial-control thread, never the high-rate sensor callbacks.
        with (self._session_dir/'trial_conditions.jsonl').open('a') as f:
            f.write(json.dumps(safe(row), allow_nan=False)+'\n')
