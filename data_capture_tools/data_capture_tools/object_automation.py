"""Object-wide acquisition, monitored direct gripper actions and explicit resume.

Legacy i/ap/ad/ak paths are unchanged. This runner mirrors their goal values but
owns one gripper action at a time so cancellation cannot launch a queued phase.
"""
import copy
import json
import math
from pathlib import Path
import select
import shutil
import sys
import threading
import time
import uuid

from .object_plan import atomic_json, build_plan, digest, recover_progress, steps_for_trial, validate_saved_plan


class Paused(RuntimeError):
    pass


class GuardPublisher:
    def __init__(self,node,original):self.node,self.original=node,original
    def publish(self,msg):
        if self.node._auto_abort_reason:raise Paused(self.node._auto_abort_reason)
        self.original.publish(msg)


class ObjectAutomationMixin:
    def _setup_object_automation(self):
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger
        from rclpy.action import ActionClient
        from robotiq_2f_gripper_msgs.action import MoveTwoFingerGripper
        self._auto_action_type=MoveTwoFingerGripper
        self._auto_action=ActionClient(self,MoveTwoFingerGripper,'/robotiq_2f_gripper_action')
        self._auto_play=self.create_client(Trigger,'/dashboard_client/play')
        from controller_manager_msgs.srv import ListControllers
        self._auto_controller_list=self.create_client(ListControllers,'/controller_manager/list_controllers')
        self._auto_arm_controller_states={}
        self._auto_program=None;self._auto_robot_mode=None;self._auto_safety_mode=None
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        program_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._auto_program_sub=self.create_subscription(Bool,'/io_and_status_controller/robot_program_running',
            lambda m:setattr(self,'_auto_program',(bool(m.data),time.monotonic())),program_qos)
        from ur_dashboard_msgs.msg import RobotMode, SafetyMode
        self._auto_robot_sub=self.create_subscription(RobotMode,'/io_and_status_controller/robot_mode',
            lambda m:setattr(self,'_auto_robot_mode',m.mode),program_qos)
        self._auto_safety_sub=self.create_subscription(SafetyMode,'/io_and_status_controller/safety_mode',
            lambda m:setattr(self,'_auto_safety_mode',m.mode),program_qos)
        self._auto_job_path=None;self._auto_job=None;self._auto_goal=None
        self._auto_abort_reason=None;self._auto_init_offset=None
        self._auto_armed=False;self._auto_active=False;self._auto_force_fault=None
        self._auto_images={};self._auto_streams={};self._auto_timing_counts={}
        self._auto_reference=None;self._auto_uncertain_goal=False

    def _auto_note_image(self,side,msg):
        if not getattr(self,'_auto_active',False):return
        # Decode sparingly: a fresh header alone does not prove a valid JPEG.
        now=time.monotonic();old=self._auto_images.get(side,{})
        valid=old.get('valid',True)
        if now-old.get('decoded_at',0)>1.:
            import cv2
            import numpy as np
            try:valid=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR) is not None
            except Exception:valid=False
            decoded=now
        else:decoded=old.get('decoded_at',now)
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self._auto_images[side]=dict(at=now,header_ns=ns,valid=valid,decoded_at=decoded,
                                    count=old.get('count',0)+1)

    def _auto_note_force(self,side,wrench,header_ns=None):
        # Latch each received raw sample, not only the latest at the 20-ms monitor tick.
        if not getattr(self,'_auto_active',False) or not self._auto_armed:return
        if not all(math.isfinite(v) for v in wrench):
            self._auto_force_fault='Nonfinite force '+side
        elif abs(wrench[1]-getattr(self._force_monitor,'_offset_'+side))>=self._auto_job['plan']['settings']['hard_limit_n']:
            self._auto_force_fault='Force hard limit '+side
        if self._auto_force_fault and getattr(self,'_auto_force_fault_record',None) is None:
            self._auto_force_fault_record=dict(side=side,header_ns=header_ns,
                receive_monotonic_ns=time.monotonic_ns(),reason=self._auto_force_fault,
                baseline_n=getattr(self._force_monitor,'_offset_'+side),
                wrench=[v if math.isfinite(v) else None for v in wrench])

    def _auto_note_stream(self,name,stream_id):
        self._auto_streams[name]=(stream_id,time.monotonic())
        self._auto_timing_counts[name]=self._auto_timing_counts.get(name,0)+1

    def _auto_save(self):
        self._auto_job['updated_time_ns']=time.time_ns()
        atomic_json(self._auto_job_path,self._auto_job)

    def _auto_manual(self):
        if self._stop_event.is_set():raise Paused('Node shutdown requested')
        if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
            sys.stdin.readline();raise Paused('Manual pause requested')

    def _auto_health(self,*,check_pose=True,check_force=True,check_bag=False):
        settings=self._auto_job['plan']['settings']
        self._auto_manual()
        if check_force and self._auto_armed and self._auto_force_fault:raise Paused(self._auto_force_fault)
        if self._auto_abort_reason:raise Paused(self._auto_abort_reason)
        if self._auto_uncertain_goal:raise Paused("Unconfirmed gripper goal; wait for cancellation/result before resume")
        if self._auto_robot_mode!=7:raise Paused('UR robot mode is not RUNNING: '+str(self._auto_robot_mode))
        if self._auto_safety_mode not in (1,2):raise Paused('UR safety mode is not NORMAL/REDUCED: '+str(self._auto_safety_mode))
        if shutil.disk_usage(self.config.output_root).free < settings['min_free_gib']*1024**3:
            raise Paused('Insufficient free storage')
        # Snapshot all inputs before sampling 'now': callbacks can replace these
        # records while disk checks/manual-input checks take time. Reading a new
        # sample against an older 'now' can falsely report a future Header stamp.
        with self._init_record_lock:
            latest=copy.deepcopy({k:self._init_latest.get(k) for k in
                                  ('force_left','force_right','gripper_status','arm_joint_state')})
        images=dict(self._auto_images)
        streams=dict(self._auto_streams)
        now=time.monotonic();ros=self.get_clock().now().nanoseconds
        for side in ('left','right'):
            row=latest.get('force_'+side)
            receive_age=(now*1e9-row['receive_monotonic_ns'])/1e9 if row else None
            if receive_age is None or receive_age>settings['force_timeout_s']:
                raise Paused(f'Stale force {side}: receive_age_ms={receive_age*1000 if receive_age is not None else None}; limit_ms={settings["force_timeout_s"]*1000:.1f}')
            age=(ros-row['header_ns'])/1e9
            if not -.01<=age<=settings['force_timeout_s']:
                raise Paused(f'Invalid force header age {side}: header_age_ms={age*1000:.3f}; receive_age_ms={receive_age*1000:.3f}; allowed_ms=[-10,{settings["force_timeout_s"]*1000:.1f}]')
            if not all(math.isfinite(v) for v in row['wrench']):raise Paused('Nonfinite force '+side)
            if check_force and self._auto_armed:
                offset=getattr(self._force_monitor,'_offset_'+side)
                if abs(row['wrench'][1]-offset)>=settings['hard_limit_n']:raise Paused('Force hard limit '+side)
            image=images.get(side)
            if not image or now-image['at']>settings['image_timeout_s'] or not image['valid']:
                raise Paused('Missing/stale/undecodable GelSight '+side)
            image_age=(ros-image['header_ns'])/1e9
            if not -.01<=image_age<=settings['image_timeout_s']:
                raise Paused(f'Invalid image header age {side}: header_age_ms={image_age*1000:.3f}; receive_age_ms={(now-image["at"])*1000:.3f}; allowed_ms=[-10,{settings["image_timeout_s"]*1000:.1f}]')
        if settings['require_timing']:
            for name in ('mms_left','mms_right','gelsight_left','gelsight_right'):
                value=streams.get(name)
                if not value or now-value[1]>.8:raise Paused('Acquisition timing missing '+name)
                expected=getattr(self,'_auto_expected_streams',{}).get(name)
                if expected and value[0]!=expected:raise Paused('Sensor stream restarted '+name)
        status=latest.get('gripper_status');joints=latest.get('arm_joint_state')
        if not status or now*1e9-status['receive_monotonic_ns']>500_000_000:raise Paused('Stale gripper status')
        if status['status']['g_flt']:raise Paused('Gripper fault')
        if not joints or now*1e9-joints['receive_monotonic_ns']>500_000_000:raise Paused('Stale arm state')
        if not self._current_joints or not all(math.isfinite(v) for v in self._current_joints):raise Paused('Invalid arm joints')
        ref=self._auto_reference
        if ref:
            if not self._init_startup or self._init_startup['startup_id']!=ref['driver_id']:
                raise Paused('Gripper driver changed; init required')
            if check_pose and max(abs(a-b) for a,b in zip(self._current_joints,ref['joints']))>.015:
                raise Paused('Arm position changed; reference cannot be reused')
        if check_bag:
            if self._bag_process is None or self._bag_process.poll() is not None:raise Paused('Bag recorder exited')

    def _auto_await_ready(self):
        deadline=time.monotonic()+5.;stable=None;last='Sensors not ready'
        while time.monotonic()<deadline:
            try:
                self._auto_health(check_pose=False,check_force=False)
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>=1.:return
            except Paused as exc:
                last=str(exc);stable=None
                if 'Manual' in last or 'shutdown' in last:raise
            time.sleep(.05)
        raise Paused(last)

    def _auto_wait(self,seconds,**guards):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            self._auto_health(**guards);time.sleep(.02)

    def _auto_future(self,future,timeout,guard=True):
        deadline=time.monotonic()+timeout
        while not future.done():
            if guard:self._auto_health(check_pose=not getattr(self,'_auto_in_init',False),check_bag=getattr(self,'_auto_recording',False))
            if time.monotonic()>deadline:raise Paused('ROS request timeout')
            time.sleep(.02)
        return future.result()

    def _auto_cancel(self):
        goal=self._auto_goal
        if goal is None:return True
        try:
            result=self._auto_future(goal.cancel_goal_async(),2.,guard=False)
            # Require terminal result as well; cancel acceptance is not a STOP ACK.
            terminal=self._auto_future(goal.get_result_async(),3.,guard=False)
            self._auto_goal=None
            if terminal.status != 5 and not (terminal.status == 4 and terminal.result.success):
                self._auto_uncertain_goal=True
                raise Paused('Cancellation did not confirm STOP/completion')
            return True
        except Exception as exc:
            self._auto_uncertain_goal=True
            self.get_logger().error('Gripper cancellation unconfirmed: '+str(exc));return False

    def _auto_grip(self,width,speed,force,phase):
        goal=self._auto_action_type.Goal();goal.target_position=float(width);goal.target_speed=float(speed);goal.target_force=float(force)
        self._auto_health(check_bag=getattr(self,'_auto_recording',False))
        # Keep the pending send future: a lost/late response may still execute a GO.
        future=self._auto_action.send_goal_async(goal)
        try:
            handle=self._auto_future(future,3.)
        except BaseException:
            self._auto_uncertain_goal=True
            def cancel_late(f):
                try:
                    h=f.result()
                    if h and h.accepted:
                        h.cancel_goal_async()
                        def terminal_done(result_future):
                            try:
                                result=result_future.result()
                                if result.status==5 or (result.status==4 and result.result.success):
                                    self._auto_uncertain_goal=False
                            except Exception:pass
                        h.get_result_async().add_done_callback(terminal_done)
                    else:self._auto_uncertain_goal=False
                except Exception:pass
            future.add_done_callback(cancel_late)
            raise
        if not handle or not handle.accepted:raise Paused('Gripper goal rejected')
        self._auto_goal=handle
        self._auto_event('goal_sent',phase,width,speed,False)
        try:
            result=self._auto_future(handle.get_result_async(),20.)
            if result.status!=4 or not result.result.success:raise Paused('Gripper action failed')
            self._auto_goal=None
            self._auto_event('action_succeeded',phase,width,speed,True)
        except BaseException:
            self._auto_cancel();raise

    def _auto_event(self,event,phase,width,speed,success):
        from robotiq_2f_gripper_msgs.msg import RobotActionEvent
        msg=RobotActionEvent();msg.header.stamp=self.get_clock().now().to_msg()
        msg.event='auto_'+event;msg.phase_id=phase;msg.trial_index=getattr(self,'_auto_trial_index',0)
        msg.action_type='gripper';msg.target_position=float(width);msg.target_speed=float(speed);msg.success=success
        self._auto_event_pub.publish(msg)

    def _auto_check_reference(self,reference):
        self._auto_reference=reference
        self._auto_health(check_force=False)
        if not self._init_startup.get('cancel_stops_motion',False):raise Paused('Updated gripper STOP-on-cancel driver required')
        path=Path(reference['init_path']);summary=json.loads((path/'summary.json').read_text())
        if not summary.get('valid_for_collection'):raise Paused('Recorded init invalid')
        from hashlib import sha256
        if sha256((path/'samples.jsonl.gz').read_bytes()).hexdigest()!=summary['samples_sha256']:raise Paused('Init stream checksum mismatch')
        self._init_selected_point=summary.get('selected_contact')
        self._last_successful_init=path
        self._effective_object_width_m=reference['width_m'];self._calibrated_target_m=reference['target_m']
        self._saved_base_joints=list(reference['joints'])

    def _auto_wait_program_controller(self,program_after=None):
        """Observe actual readiness, without sending another play or switching controllers."""
        from controller_manager_msgs.srv import ListControllers
        deadline=time.monotonic()+5.
        self._auto_arm_controller_states={}
        while time.monotonic()<deadline:
            self._auto_health(check_pose=False,check_force=False)
            running=self._auto_program
            if running and running[0] and (program_after is None or running[1]>=program_after):
                if self._auto_controller_list.wait_for_service(timeout_sec=.1):
                    result=self._auto_future(self._auto_controller_list.call_async(ListControllers.Request()),2.,guard=False)
                    self._auto_arm_controller_states={c.name:c.state for c in result.controller} if result else {}
                    running=self._auto_program
                    if (self._auto_arm_controller_states.get('scaled_joint_trajectory_controller')=='active'
                            and running and running[0] and (program_after is None or running[1]>=program_after)):
                        return True
            time.sleep(.05)
        return False

    def _auto_program_ready(self):
        deadline=time.monotonic()+2.
        while self._auto_program is None and time.monotonic()<deadline:time.sleep(.05)
        if self._auto_program and self._auto_program[0]:
            if self._auto_wait_program_controller():return
            raise Paused('Robot program running but arm controller readiness not confirmed: '+str(self._auto_arm_controller_states))
        if not self._auto_job['plan']['settings']['auto_play']:raise Paused('Start External Control then resume')
        if not self._auto_play.wait_for_service(timeout_sec=2.):raise Paused('Dashboard play unavailable; start External Control manually')
        from std_srvs.srv import Trigger
        sent_at=time.monotonic()
        try:
            response=self._auto_future(self._auto_play.call_async(Trigger.Request()),5.,guard=False)
            accepted=bool(response is not None and response.success)
            detail=str(getattr(response,'message','No response'))
        except Exception as exc:
            accepted=False;detail=type(exc).__name__+': '+str(exc)
        if not accepted:
            self.get_logger().warning('Dashboard play response: '+detail+'; checking actual program/controller state without replay')
        # A rejected/timeout response can precede the actual connection by ~0.5s.
        # Never replay on that basis; require a new running notification and an
        # active trajectory controller before issuing the init trajectory.
        if self._auto_wait_program_controller(program_after=sent_at):
            if not accepted:self.get_logger().info('Robot program and arm controller readiness confirmed after Dashboard response')
            return
        raise Paused('Dashboard play/start readiness failed: '+detail+'; program='+str(self._auto_program)
                     +'; controllers='+str(self._auto_arm_controller_states)+'; inspect UR driver/pendant')

    def _auto_init_position(self,position):
        self._auto_reference=None;self._auto_armed=False;self._auto_abort_reason=None
        self._auto_expected_streams={}
        self._auto_wait(1.,check_pose=False,check_force=False)
        self._auto_program_ready()
        self._auto_init_offset=dict(lateral_m=position['lateral_m'],vertical_m=position['vertical_m'],position_id=position['id'])
        if not self._auto_job['plan']['settings']['auto_init']:
            raise Paused('Manual init required: use ai for planned position, then ar')
        self._auto_run_init()
        position['reference']=self._auto_capture_reference();self._auto_save()

    def _auto_capture_reference(self):
        if self._last_successful_init is None or self._effective_object_width_m is None:raise Paused('Init not completed')
        return dict(init_path=str(self._last_successful_init),width_m=self._effective_object_width_m,
                    target_m=self._calibrated_target_m,joints=list(self._saved_base_joints),
                    driver_id=self._init_startup['startup_id'])

    def _auto_run_init(self):
        self._auto_health(check_pose=False,check_force=False)
        original_cmd,original_script=self._cmd_pub,self._urscript_pub
        self._auto_in_init=True;self._auto_abort_reason=None
        self._cmd_pub=GuardPublisher(self,original_cmd);self._urscript_pub=GuardPublisher(self,original_script)
        done=threading.Event();begin=time.monotonic()
        def monitor():
            from std_msgs.msg import String
            while not done.wait(.05):
                try:
                    if time.monotonic()-begin>180.:raise Paused('Init time limit')
                    self._auto_health(check_pose=False,check_force=False)
                except BaseException as exc:
                    self._auto_abort_reason=str(exc)
                    original_cmd.publish(String(data='stop'))
                    original_script.publish(String(data='stopj(2.0)'))
                    self._sequence_failed_event.set();self._sequence_finished_event.set()
                    return
        watcher=threading.Thread(target=monitor,daemon=True);watcher.start()
        try:
            init_ok=self._run_init_sequence()
            if self._auto_abort_reason:raise Paused(self._auto_abort_reason)
            if not init_ok:raise Paused('Init failed')
        finally:
            done.set();watcher.join(timeout=1.)
            self._cmd_pub,self._urscript_pub=original_cmd,original_script
            self._auto_in_init=False

    def _auto_required_topics(self):
        names=['/force_torque/left','/force_torque/right','/gelsight/left/image_raw/compressed',
               '/gelsight/right/image_raw/compressed','/robotiq_2f_gripper/status',
               '/robotiq_2f_gripper/command_event','/robot_action_event','/trial_event']
        if self._auto_job['plan']['settings']['require_timing']:
            names += ['/mms101_driver_1/acquisition_timing','/mms101_driver_2/acquisition_timing',
                      '/gelsight_mini_publisher_1/acquisition_timing','/gelsight_mini_publisher_2/acquisition_timing']
        return names

    def _auto_verify_bag(self):
        import yaml
        metadata=self._bag_uri/'metadata.yaml'
        if self._bag_process.returncode not in (0,None):raise Paused('Bag closed with error '+str(self._bag_process.returncode))
        if not metadata.exists():raise Paused('Bag metadata missing')
        info=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        counts={x['topic_metadata']['name']:x['message_count'] for x in info['topics_with_message_count']}
        missing=[t for t in self._auto_required_topics() if counts.get(t,0)==0]
        if missing:raise Paused('Bag has no messages: '+', '.join(missing))
        logs='\n'.join(p.read_text(errors='replace') for p in (self._session_dir/'logs').glob('rosbag_*.log'))
        if '[ERROR]' in logs or 'Messages lost' in logs:raise Paused('Bag recorder reported errors/loss; review logs')
        return dict(topic_counts=counts,note='Aggregate bag counts plus live health checks; not proof of zero losses or physical synchronization')

    def _auto_start_bag(self,position):
        configured={t.name for t in self.config.topics}
        missing=set(self._auto_required_topics())-configured
        if missing:raise Paused('Missing configured topics '+str(sorted(missing)))
        self._session_id_override=self._auto_job['job_id']+'_'+position['id']+'_'+uuid.uuid4().hex[:8]
        self._prepare_session()
        self._start_rosbag();self._auto_recording=True
        self._auto_wait(3.5,check_force=False,check_bag=True)
        for topic in self._auto_required_topics():
            if not any('rosbag' in endpoint.node_name for endpoint in self.get_subscriptions_info_by_topic(topic)):
                raise Paused('Recorder subscription not discovered: '+topic)

    def _auto_execute_trial(self,row,attempt):
        # Open before tare. A sensor restart may have changed its zero but not geometry.
        self._auto_armed=False
        self._auto_grip(.140,.3,.1,1);self._auto_wait(1.,check_force=False,check_bag=True)
        self._force_monitor.tare()
        attempt['tare']=dict(left=self._force_monitor._offset_left,right=self._force_monitor._offset_right,
                             ros_ns=self.get_clock().now().nanoseconds)
        self._auto_save();self._auto_force_fault=None;self._auto_force_fault_record=None;self._auto_armed=True
        self._auto_expected_streams={k:v[0] for k,v in self._auto_streams.items()}
        mode='auto_knead' if row['kind']=='knead' else 'auto_'+row.get('variant','depth_speed')
        steps=steps_for_trial(row,self._effective_object_width_m)
        self._publish_trial_event('START',self._auto_trial_index,mode,steps[-2]['width_m'],steps[1]['speed'])
        # Pre-open above mirrors the first step of legacy run; do not issue it twice.
        for step in steps[1:]:
            self._auto_grip(step['width_m'],step['speed'],step['force'],step['phase'])
            self._auto_event('dwell_start',step['phase'],step['width_m'],step['speed'],True)
            self._auto_wait(step['dwell_s'],check_bag=True)
            self._auto_event('dwell_end',step['phase'],step['width_m'],step['speed'],True)
        self._auto_wait(self._auto_job['plan']['settings']['completion_settle_s'],check_bag=True)
        self._publish_trial_event('END',self._auto_trial_index,mode,steps[-2]['width_m'],steps[1]['speed'])
        self._auto_wait(self._auto_job['plan']['settings']['rest_s'],check_bag=True)
        attempt.update(status='awaiting_bag',end_ros_ns=self.get_clock().now().nanoseconds)
        row['status']='awaiting_bag';self._auto_save()

    def _auto_run(self):
        self._auto_active=True;self._auto_armed=False;self._auto_recording=False;self._auto_force_fault=None
        self._auto_abort_reason=None;self._auto_expected_streams={}
        original_override=self._session_id_override
        self.get_logger().warn('Automatic collection: Enter pauses. Do not move specimen/arm or remount sensors when resuming without init.')
        try:
            if not self._auto_action.wait_for_server(timeout_sec=3.):raise Paused('Gripper action server unavailable')
            self._auto_await_ready()
            if not self._init_startup or not self._init_startup.get('cancel_stops_motion',False):raise Paused('Rebuild/restart updated gripper driver before automated motion')
            if self._init_startup.get('fake_hardware',False):raise Paused('Real gripper hardware required')
            self._auto_job['status']='running';self._auto_save()
            for position in self._auto_job['plan']['positions']:
                pending=[t for t in position['trials'] if t['status']!='complete']
                if not pending:continue
                if position['reference'] is None:self._auto_init_position(position)
                self._auto_check_reference(position['reference'])
                # Every position's candidate widths are checked before trial motion.
                for row in pending:steps_for_trial(row,self._effective_object_width_m)
                while pending:
                    batch=pending[:self._auto_job['plan']['settings']['batch_size']];finished=[];failed=None
                    try:
                        self._auto_start_bag(position)
                        for row in batch:
                            self._auto_trial_index=position['trials'].index(row)+1
                            attempt=dict(status='running',session=str(self._session_dir),
                                         start_ros_ns=self.get_clock().now().nanoseconds,init_id=self._last_successful_init.name,
                                         sensor_stream_ids={k:v[0] for k,v in self._auto_streams.items()},
                                         position_id=position['id'],requested_offset=dict(lateral_m=position['lateral_m'],vertical_m=position['vertical_m']))
                            row['attempts'].append(attempt);row['status']='running';self._auto_save()
                            atomic_json(self._session_dir/'automatic_plan.json',self._auto_job)
                            try:self._auto_execute_trial(row,attempt);finished.append(row)
                            except BaseException as exc:
                                stopped=self._auto_cancel()
                                if str(exc).startswith('Force hard limit') and stopped:
                                    try:
                                        self._auto_armed=False
                                        self._auto_grip(.140,.3,.1,3)
                                        attempt['abort_release']='completed'
                                    except BaseException as release_error:
                                        attempt['abort_release']='failed: '+str(release_error)
                                attempt.update(status='aborted',reason=type(exc).__name__+': '+str(exc),
                                               force_trigger=getattr(self,'_auto_force_fault_record',None));row['status']='pending'
                                self._publish_trial_event('ABORT',self._auto_trial_index,'automatic',0.,0.)
                                raise
                    except BaseException as exc:failed=exc;self._auto_cancel()
                    finally:
                        if self._auto_recording:
                            self._stop_rosbag();self._auto_recording=False
                            # Keep quality separate from execution status, including ABORT attempts.
                            advisory=getattr(self,'_last_bag_quality',None)
                            if advisory:
                                self._auto_job.setdefault('quality_reports',{})[str(self._session_dir)]=dict(advisory)
                                for item in batch:
                                    for old_attempt in item.get('attempts',[]):
                                        if old_attempt.get('session')==str(self._session_dir):
                                            old_attempt['quality_check']=dict(advisory)
                            try:
                                quality=self._auto_verify_bag()
                                for row in finished:
                                    row['status']='complete';row['attempts'][-1].update(status='complete',bag_quality=quality)
                            except BaseException as exc:
                                for row in finished:row['status']='pending';row['attempts'][-1].update(status='recording_unverified',reason=str(exc))
                                if failed is None:failed=exc
                            self._auto_save()
                    if failed is not None:raise failed
                    pending=[t for t in position['trials'] if t['status']!='complete']
            self._auto_job['status']='complete';self._auto_save()
            self.get_logger().info('Object collection complete: '+str(self._auto_job_path))
        except BaseException as exc:
            self._auto_cancel();self._auto_job.update(status='paused',pause_reason=type(exc).__name__+': '+str(exc));self._auto_save()
            self.get_logger().error('Collection paused: '+str(exc)+'; resolve the reported cause, then ar. Progress: '+str(self._auto_job_path))
        finally:
            self._auto_active=False;self._auto_armed=False;self._auto_init_offset=None
            self._session_id_override=original_override

    def _execute_object_auto(self,resume=False):
        from robotiq_2f_gripper_msgs.msg import RobotActionEvent
        self._auto_event_pub=self.create_publisher(RobotActionEvent,'/robot_action_event',100)
        try:
            if not sys.stdin.isatty():raise ValueError('Interactive terminal required for GO/pause/resume')
            if resume:
                print('Progress JSON path (Enter = current): ',end='',flush=True);value=sys.stdin.readline().strip()
                path=Path(value).expanduser() if value else self._auto_job_path
                if path is None:raise ValueError('No current job; enter progress JSON path')
                job=json.loads(path.read_text())
                validate_saved_plan(job['plan'])
                if job['config_id']!=digest(self.config.raw):raise ValueError('Loaded YAML differs from original job; restore original settings')
                self._auto_job_path=path;self._auto_job=recover_progress(job)
                self._auto_reference=None
            else:
                plan=build_plan(self.config.raw.get('automatic_collection',{}))
                root=self.config.output_root/self.config.task_name/'automatic_jobs';root.mkdir(parents=True,exist_ok=True)
                job_id=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
                self._auto_job_path=root/(job_id+'.json')
                self._auto_job=dict(job_id=job_id,status='planned',config_id=digest(self.config.raw),plan=plan)
                self._auto_reference=None
            total=sum(len(p['trials']) for p in self._auto_job['plan']['positions'])
            completed=sum(t['status']=='complete' for p in self._auto_job['plan']['positions'] for t in p['trials'])
            print(f'Object={self.config.task_name}; trials={total}; complete={completed}; progress={self._auto_job_path}')
            print('Confirm specimen, arm and sensor mounting unchanged for resume. Type GO: ',end='',flush=True)
            if sys.stdin.readline().strip()!='GO':return
            self._auto_save();self._auto_run()
        except Exception as exc:self.get_logger().error('Automatic collection not started: '+str(exc))

    def _execute_planned_init(self):
        if not self._auto_job: self.get_logger().error('Start ac/ar first');return
        position=next((p for p in self._auto_job['plan']['positions'] if any(t['status']!='complete' for t in p['trials'])),None)
        if position is None:return
        self._auto_active=True;self._auto_reference=None;self._auto_abort_reason=None;self._auto_expected_streams={}
        self._auto_init_offset=dict(lateral_m=position['lateral_m'],vertical_m=position['vertical_m'],position_id=position['id'])
        try:
            self._auto_await_ready();self._auto_program_ready();self._auto_run_init()
            position['reference']=self._auto_capture_reference();self._auto_save()
            self.get_logger().info('Planned init saved. Use ar to continue without another init.')
        except Exception as exc:self.get_logger().error('Planned init failed: '+str(exc))
        finally:self._auto_active=False;self._auto_init_offset=None
