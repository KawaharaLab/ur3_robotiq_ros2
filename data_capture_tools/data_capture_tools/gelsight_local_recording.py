"""Advisory local-metadata leases tied to one collector-owned rosbag process."""
import json
from pathlib import Path
import socket
import threading
import time
import uuid

from .object_plan import atomic_json

CONTROL_TOPIC = '/gelsight/local_recording_control'
STATUS_TOPICS = ('/gelsight_mini_publisher_1/local_recording_status',
                 '/gelsight_mini_publisher_2/local_recording_status')


class GelsightLocalRecordingClient:
    def __init__(self, node):
        from std_msgs.msg import String
        self._string_type = String
        self._node = node
        self._publisher = node.create_publisher(String, CONTROL_TOPIC, 10)
        self._subscriptions = [node.create_subscription(String, topic,
            lambda msg, t=topic: self._status(t, msg), 10) for topic in STATUS_TOPICS]
        self._timer = node.create_timer(1., self._heartbeat)
        self._lock = threading.RLock()
        self._save_lock = threading.Lock()
        self._owner = uuid.uuid4().hex
        self._request = None
        self._last_request = None
        self._statuses = {}
        self._received_at = {}
        self._warned_inactive = False
        self._process = None
        self._starting_until = 0.
        self._manifest = None

    def _status(self, topic, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status,dict) or status.get('schema_version') != 1:return
            with self._lock:
                self._statuses[topic] = status
                self._received_at[topic] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _send(self, request, operation):
        message = dict(request, operation=operation, sent_monotonic_ns=time.monotonic_ns())
        self._publisher.publish(self._string_type(data=json.dumps(message)))

    def _heartbeat(self):
        try:self._heartbeat_once()
        except Exception as exc:
            self._node.get_logger().warning('Local recording heartbeat failed; its lease will expire: '+str(exc))

    def _heartbeat_once(self):
        with self._lock:
            request, process = self._request, self._process
            starting = time.monotonic() < self._starting_until
        if request is None:return
        if (process is not None and process.poll() is not None) or (process is None and not starting):
            self.stop(wait=False, expected_session=request["session_id"])
        else:
            with self._lock:
                if self._request is request:self._send(request, 'start')
            if process is not None and len(self._matched(request))<len(STATUS_TOPICS) and not self._warned_inactive:
                self._warned_inactive = True
                self._node.get_logger().warning('GelSight local copy/status no longer active on both nodes; primary bag continues; inspect local recording manifest.')

    def _matched(self, request, closed=False):
        with self._lock:
            statuses = dict(self._statuses)
            received = dict(self._received_at)
        result = {}
        for topic, status in statuses.items():
            candidate = status.get('last_closed') if closed else status
            if isinstance(candidate,dict) and candidate.get('owner_id') == request['owner_id'] and candidate.get('session_id') == request['session_id']:
                if closed or (time.monotonic()-received.get(topic,0.) <= 3. and candidate.get('active') and not candidate.get('error') and (candidate.get('writer') or {}).get('accepting')):
                    result[topic] = candidate
        return result

    def _save(self, request, phase):
        manifest = request.get('manifest_path')
        if manifest is None:return
        with self._lock:statuses = dict(self._statuses)
        try:
            with self._save_lock:
                atomic_json(Path(manifest), dict(schema_version=1, phase=phase, request=request,
                    statuses=statuses, ready_nodes=list(self._matched(request)),
                    closed_nodes=list(self._matched(request, closed=True)),
                    note='Advisory local copy only. The primary bag/motion is not retried or stopped by this result.'))
        except Exception as exc:
            self._node.get_logger().warning('Could not save local recording control status: '+str(exc))

    def start(self, session_dir, timeout=2.5):
        if self._request is not None:self.stop()
        root = Path(session_dir).resolve()
        request = dict(schema_version=1, hostname=socket.gethostname(), owner_id=self._owner,
            session_id=uuid.uuid4().hex, directory=str(root/'gelsight_acquisition_records'), lease_seconds=10.,
            manifest_path=str(root/'gelsight_local_recording.json'))
        with self._lock:
            self._manifest = root/'gelsight_local_recording.json'
            self._request = self._last_request = request
            self._process = None
            self._starting_until = time.monotonic()+10.
            self._warned_inactive = False
        self._send(request, 'start')
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline and len(self._matched(request)) < len(STATUS_TOPICS):
            time.sleep(.02)
        ready = len(self._matched(request)) == len(STATUS_TOPICS)
        self._save(request, 'ready' if ready else 'not_ready')
        if ready:
            self._node.get_logger().info('GelSight local recording ready: '+request['directory'])
        else:
            self._warned_inactive = True
            self._node.get_logger().warning('GelSight local recording not ready on both nodes; check rebuilt publishers and acquisition_records:=true. Primary bag collection continues.')
        return ready

    def attach_process(self, process):
        with self._lock:self._process = process;self._starting_until = 0.

    def stop(self, wait=True, expected_session=None):
        with self._lock:
            if expected_session is not None and (self._request is None or self._request["session_id"]!=expected_session):return
            request = self._request or self._last_request
            self._request = None
            self._process = None
            self._starting_until = 0.
        if request is None:return
        self._send(request, 'stop')
        if wait:
            deadline = time.monotonic()+2.5
            while time.monotonic()<deadline and len(self._matched(request,closed=True))<len(STATUS_TOPICS):
                time.sleep(.02)
        # Timer callbacks must not do filesystem I/O on the sensor executor.
        # The manager/helper's final stop(wait=True) persists the final manifest.
        if wait:
            self._save(request, 'stopped' if len(self._matched(request,closed=True))==len(STATUS_TOPICS) else 'stop_requested')
