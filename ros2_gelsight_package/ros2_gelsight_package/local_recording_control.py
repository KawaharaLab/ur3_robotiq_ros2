"""Host-local recording leases. Call handle/tick on the node's control thread.

The capture thread only reads writer(), which performs no file I/O or locking.
Opening and draining writers belongs to the control thread, never capture.
"""
from collections import deque
import math
from pathlib import Path
import socket
import time

from .local_acquisition_records import LocalAcquisitionWriter

CONTROL_TOPIC = '/gelsight/local_recording_control'


class RecordingLeaseController:
    def __init__(self, metadata_factory, on_error=None, writer_factory=LocalAcquisitionWriter,
                 clock=time.monotonic, hostname=None):
        self._metadata_factory = metadata_factory
        self._on_error = on_error
        self._factory = writer_factory
        self._clock = clock
        self._hostname = hostname or socket.gethostname()
        self._writer = None
        self._deadline = 0.
        self._request = None
        self._closed = None
        self._draining = None
        self._error = None
        self._retired = deque(maxlen=128)

    def _notify(self, message):
        if self._on_error:
            try:self._on_error(message)
            except Exception:pass

    def writer(self):
        # Even if the ROS control callback is delayed, never submit new data
        # after the lease deadline. Atomic pointer reads suffice for this gate.
        writer = self._writer
        return writer if self._clock() < self._deadline else None

    def status(self):
        request = self._request or {}
        writer = self._writer
        snapshot = writer.snapshot() if writer else None
        return dict(schema_version=1, mode='bag', enabled=True,
            active=writer is not None and self._clock() < self._deadline and bool(snapshot.get('accepting')),
            owner_id=request.get('owner_id'), session_id=request.get('session_id'),
            directory=request.get('directory'), writer=snapshot,
            last_closed=self._closed, error=self._error or (snapshot or {}).get('error'))

    def _close(self, reason, capture_thread_stopped=False):
        writer, request = self._writer, self._request
        # Disconnect capture first; never hold a capture lock across file I/O.
        self._writer = None
        self._deadline = 0.
        self._request = None
        if request:
            self._retired.append((request['owner_id'], request['session_id']))
        if writer:
            snapshot = writer.close(timeout=2., capture_thread_stopped=capture_thread_stopped, capture_source_detached=True)
            if snapshot["close_timed_out"]:self._draining = writer
            self._closed = dict(owner_id=request['owner_id'], session_id=request['session_id'],
                reason=reason, writer=snapshot)
            if snapshot['error'] or snapshot['close_timed_out']:
                self._error = snapshot['error'] or 'Local writer close timed out'
                self._notify(self._error)

    def tick(self):
        if self._draining is not None and self._draining.snapshot()['closed']:
            self._draining = None
        if self._writer is not None and self._clock() >= self._deadline:
            self._close('lease_expired')

    def handle(self, message):
        self.tick()
        try:
            if message.get('schema_version') != 1:
                raise ValueError('Unsupported local recording control schema')
            owner, session = message['owner_id'], message['session_id']
            if not isinstance(owner, str) or not owner or not isinstance(session, str) or not session:
                raise ValueError('Missing recording owner/session')
            key = (owner, session)
            if message.get('hostname') != self._hostname:
                raise ValueError('Local recording controller and collector must run on the same host')
            if message['operation'] == 'stop':
                if self._request and key == (self._request['owner_id'], self._request['session_id']):
                    self._close('collector_stop')
                elif key not in self._retired:
                    self._retired.append(key)
                return self.status()
            if message['operation'] != 'start':
                raise ValueError('Unknown local recording operation')
            if key in self._retired:
                return self.status()  # A queued heartbeat must not resurrect a stopped file.
            lease = message['lease_seconds']
            sent = message['sent_monotonic_ns']
            if type(lease) not in (int, float) or not math.isfinite(lease) or not 3 <= lease <= 30:
                raise ValueError('Invalid local recording lease')
            if type(sent) is not int or sent <= 0:
                raise ValueError('Invalid control timestamp')
            now = self._clock()
            if sent / 1e9 > now + 1.:
                raise ValueError('Control clock is not comparable to local monotonic clock')
            deadline = sent / 1e9 + lease
            if deadline <= now:
                return self.status()  # Receiving old control data does not renew a lease.
            directory = Path(message['directory'])
            if not directory.is_absolute():
                raise ValueError('Recording directory must be absolute')
            if self._request:
                current = (self._request['owner_id'], self._request['session_id'])
                if key != current:
                    raise ValueError('Another collector owns the local recording lease')
                if str(directory) != self._request['directory']:
                    raise ValueError('Directory changed during the same recording lease')
                self._deadline = max(self._deadline, deadline)
                return self.status()
            if self._draining is not None:
                raise ValueError('Previous local writer is still draining; no new writer started')
            metadata = dict(self._metadata_factory(), recording_owner_id=owner,
                            recording_session_id=session, lease_seconds=lease)
            writer = self._factory(str(directory), metadata, on_error=self._on_error)
            self._request = dict(owner_id=owner, session_id=session, directory=str(directory))
            self._deadline = deadline
            self._error = None
            self._writer = writer
        except Exception as exc:
            self._error = type(exc).__name__ + ': ' + str(exc)
            self._notify('Local recording control: ' + self._error)
        return self.status()

    def close(self, capture_thread_stopped=True):
        self._close('node_shutdown', capture_thread_stopped=capture_thread_stopped)
        return self.status()
