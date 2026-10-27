"""Force-triggered press/position-hold collection menu, isolated from legacy AP/init."""
import json
import select
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

from .force_press_worker import validate_plan


class ForcePressCollectionMixin:
    def _init_force_press(self):
        self._force_press_process = None
        self._force_press_handoff_active = False
        self._force_press_lock = threading.Lock()
        from std_srvs.srv import SetBool
        self._force_press_service_type = SetBool
        service_name = self.declare_parameter('force_press_control_service',
            '/robotiq_2f_gripper_node/set_force_press_control').value
        self._force_press_control_client = self.create_client(SetBool, service_name)
        defaults = dict(port='/dev/ttyUSB0', baud=115200, slave=9, threshold_n=.5,
                        hard_limit_n=1., hold_seconds=4., clearance_m=.002,
                        max_depth_per_side_m=float(self.config.push_depth),
                        close_ticks=20, max_motion_ms=2000, target_confirm_ms=5.,
                        step_counts=1, step_settle_ms=100, step_sample_ms=50,
                        step_timeout_ms=500, step_total_timeout_ms=10000)
        self._force_press_settings = {
            name: self.declare_parameter('force_press_'+name, default).value
            for name, default in defaults.items()
        }

    def _request_force_press_control(self, enabled, allow_shutdown=False):
        if not self._force_press_control_client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('Updated gripper driver service unavailable; keep the normal driver running')
        request = self._force_press_service_type.Request()
        request.data = enabled
        future = self._force_press_control_client.call_async(request)
        finished = threading.Event()
        future.add_done_callback(lambda result: finished.set())
        deadline = time.monotonic()+15.
        while not finished.wait(.05):
            if time.monotonic() >= deadline:
                raise TimeoutError('Serial handoff response timed out; control state is uncertain')
            if self._stop_event.is_set() and not allow_shutdown:
                raise RuntimeError('Shutdown during serial handoff; recovery will be attempted')
        response = future.result()
        if response is None: raise RuntimeError('No serial handoff response')
        payload = json.loads(response.message)
        return response, payload

    def _borrow_force_press_control(self, base):
        # Set before RPC so a lost response cannot silently permit ordinary AP.
        self._force_press_handoff_active=True
        response, payload = self._request_force_press_control(True)
        self._force_press_handoff_active=bool(payload.get('request_acquired', False))
        if not response.success or not payload.get('leased') or not self._force_press_handoff_active:
            raise RuntimeError('Could not borrow gripper control: '+payload.get('reason','invalid response'))
        base.update(port=payload['port'], baud=payload['baud'], slave=payload['slave'],
                    normal_driver_suspended=True, handoff_mode='automatic_service')
        validate_plan(base)
        self.get_logger().info('Gripper serial automatically borrowed: '+base['port'])

    def _restore_force_press_control(self):
        if not self._force_press_handoff_active: return True
        with self._force_press_lock:
            process=self._force_press_process
        if process is not None and process.poll() is None:
            self.get_logger().error('Force worker still running; normal control remains blocked')
            return False
        try:
            response, payload=self._request_force_press_control(False, allow_shutdown=True)
            if not response.success or payload.get('leased', True):
                raise RuntimeError(payload.get('reason','return not confirmed'))
            self._force_press_handoff_active=False
            self.get_logger().info('Normal gripper control automatically restored. AP/init may run normally.')
            return True
        except Exception as exc:
            self.get_logger().error('Normal control was not restored: '+str(exc))
            return False

    def _force_press_normal_control_ready(self, command):
        normal_commands={'i','s','ap','as','av','arp','ah','aur','ad','ak','ae'}
        if not self._force_press_handoff_active or command not in normal_commands: return True
        return self._restore_force_press_control()

    def _stop_force_press_worker(self):
        with self._force_press_lock:
            process = self._force_press_process
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=3.)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                # Killing the serial owner would prevent its best-effort STOP.
                self.get_logger().error('Force worker did not exit; use independent gripper stop. Worker left alive for STOP cleanup.')

    def _run_force_press_worker(self, plan_path, output):
        command = [sys.executable, '-m', 'data_capture_tools.force_press_worker',
                   '--plan', str(plan_path), '--out', str(output), '--enable-motion']
        with self._force_press_lock:
            if self._stop_event.is_set(): return None
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL)
            self._force_press_process = process
        self.get_logger().info('Force trial active. Enter aborts this trial and the remaining loop.')
        stop_sent = False
        next_health_check = time.monotonic()+.2
        try:
            while process.poll() is None:
                manual = False
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], .02)[0]:
                    sys.stdin.readline()
                    manual = True
                recorder_failed = self._bag_process is None or self._bag_process.poll() is not None
                sensors_failed = False
                if time.monotonic() >= next_health_check:
                    sensors_failed = not self._check_sensors_health()
                    next_health_check = time.monotonic()+.2
                if (manual or self._stop_event.is_set() or recorder_failed or sensors_failed) and not stop_sent:
                    try: process.send_signal(signal.SIGINT)
                    except ProcessLookupError: pass
                    stop_sent = True
                time.sleep(.01)
            return process.returncode
        finally:
            with self._force_press_lock:
                if process.poll() is not None: self._force_press_process = None

    def _prepare_force_press_session(self):
        # Reserve a fresh directory before the legacy helper copies config.
        # A fixed session_id must not overwrite a previous experiment.
        original=self._session_id_override
        timestamp=original or datetime.now().strftime('%Y%m%d_%H%M%S')
        destination=self.config.output_root/self.config.task_name/timestamp
        destination.mkdir(parents=True,exist_ok=False)
        self._session_id_override=timestamp
        try: self._prepare_session()
        finally: self._session_id_override=original

    def _execute_force_press_loop(self, stepped=False):
        from std_msgs.msg import String
        if not self._check_sensors_health(): return
        if self._effective_object_width_m is None:
            self.get_logger().error('Run init [i] first.')
            return
        if self._force_press_handoff_active and not self._restore_force_press_control(): return
        if not self._force_press_control_client.wait_for_service(timeout_sec=2.):
            self.get_logger().error('Start the updated normal gripper driver. No manual shutdown/handoff is required.')
            return
        if not sys.stdin.isatty():
            self.get_logger().error('af requires an interactive terminal.')
            return
        count = self._read_positive_int('Number of force-stop trials: ')
        if count is None: return
        base = dict(self._force_press_settings, effective_width_m=self._effective_object_width_m, trial_index=1)
        base['press_mode'] = 'stepped' if stepped else 'continuous'
        action = 'force_press_step_hold' if stepped else 'force_press_hold'
        experiment = 'afs' if stepped else 'af'
        try:
            start_width, minimum_width = validate_plan(base)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.get_logger().warn(
            f"{experiment}: target={base['threshold_n']} N (max of baseline-relative Fy), "
            f"raw hard limit={base['hard_limit_n']} N, confirm={base['target_confirm_ms']} ms, "
            f"start width={start_width*1000:.2f} mm, minimum width={minimum_width*1000:.2f} mm, "
            f"relative cap={base['close_ticks']} counts, hold={base['hold_seconds']} s. "
            'Preparation and release are automatic. Clear hands; Enter/Ctrl+C requests STOP. '
            'The probe does not regulate force during hold.'
        )
        if stepped:
            self.get_logger().warn(
                f"Stepped: {base['step_counts']} raw count(s) per move, STOP, "
                f"settle={base['step_settle_ms']} ms, median window={base['step_sample_ms']} ms; "
                'target uses settled force, not the continuous confirmation timer. Hard limit remains immediate.')
        print('Type GO to record and execute this force-stop series: ', end='', flush=True)
        if sys.stdin.readline().strip() != 'GO': return
        if not any(t.name == '/force_press_event' for t in self.config.topics):
            self.get_logger().error('Config must record /force_press_event (std_msgs/msg/String). No motion started.')
            return
        metadata = None
        try:
            self._borrow_force_press_control(base)
            for index in range(1, count+1):
                if self._stop_event.is_set() or not self._check_sensors_health(): break
                if (index-1) % 10 == 0:
                    if self._bag_process: self._stop_rosbag()
                    self._prepare_force_press_session()
                    metadata_path = self._session_dir/'force_press_metadata.json'
                    if metadata_path.exists(): raise RuntimeError('Existing force collection session; use a new session id')
                    (self._session_dir/'force_press').mkdir(exist_ok=False)
                    metadata = dict(schema_version=1, experiment=experiment, effective_object_width_m=self._effective_object_width_m,
                                    settings=base, actual_condition_order=[],
                                    note='Position hold after force-triggered stop; not constant-force feedback. Original bag and per-trial sidecars are required.')
                    metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
                    self._start_rosbag()
                    if self._stop_event.wait(3.5): break
                    if self._bag_process.poll() is not None: raise RuntimeError('Bag recorder exited; no force motion started')
                plan = dict(base, trial_index=index)
                directory = self._session_dir/'force_press'
                plan_path = directory/f'trial_{index:04d}_plan.json'
                output = directory/f'trial_{index:04d}'
                plan_path.write_text(json.dumps(plan,indent=2)+'\n')
                self._publish_trial_event('START',index,action,minimum_width,0.)
                self._marker_pub.publish(String(data=f'START,trial:{index},model:{action},target_force_n:{base["threshold_n"]}'))
                self._is_active_session=True
                code=self._run_force_press_worker(plan_path,output)
                self._is_active_session=False
                result_path=output/'summary.json'
                result=json.loads(result_path.read_text()) if result_path.exists() else {}
                success=(code==0 and result.get('error') is None and result.get('stop_acknowledged') is True
                         and (result.get('stop_trigger') or {}).get('reason')=='force_threshold'
                         and result.get('collection',{}).get('completed') is True
                         and result.get('collection',{}).get('event_publish_errors',1)==0)
                event='END' if success else 'ABORT'
                self._publish_trial_event(event,index,action,minimum_width,0.)
                self._marker_pub.publish(String(data=f'{event},trial:{index},model:{action}'))
                metadata['actual_condition_order'].append(dict(trial_index=index,plan=plan,success=success,
                                                             result=str(result_path.relative_to(self._session_dir))))
                metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
                self.get_logger().info(f'Force trial {index}: {event}; logs: {output}')
                if not success:
                    self.get_logger().error('Force series stopped; no automatic retry. Hard-limit abort release outcome is in summary.collection.abort_release; other failures remain STOP-only. Review summary before continuing.')
                    break
                if self._stop_event.wait(1.): break
        except Exception as exc:
            self.get_logger().error('Force collection stopped: '+str(exc))
        finally:
            self._stop_force_press_worker()
            self._restore_force_press_control()
            self._is_active_session=False
            if self._bag_process: self._stop_rosbag()
