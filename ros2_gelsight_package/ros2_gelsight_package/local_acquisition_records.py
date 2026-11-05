"""Optional bounded JSONL copy of acquisition metadata, independent of DDS.

Only the writer thread touches files after startup. Submission never waits for
queue space or disk I/O. A successful submit is not a durable-write guarantee.
"""
import json
import os
from pathlib import Path
import queue
import re
import threading
import time


class LocalAcquisitionWriter:
    def __init__(self, directory, metadata, queue_size=256, flush_seconds=1.0,
                 on_error=None):
        if type(queue_size) is not int or queue_size < 1:
            raise ValueError('queue_size must be a positive integer')
        if not 0 < flush_seconds <= 60:
            raise ValueError('flush_seconds must be in (0,60]')
        root = Path(directory).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        node = re.sub(r'[^a-zA-Z0-9_.-]', '_', metadata['node'])
        stream = metadata['stream_id']
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', stream):
            raise ValueError('Invalid stream_id for local filename')
        self.path = root / (node + '_' + stream + '.jsonl')
        self.status_path = self.path.with_suffix('.status.json')
        self._file = self.path.open('x', encoding='utf-8', buffering=65536)
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._flush_seconds = flush_seconds
        self._on_error = on_error
        self._state = dict(local_schema_version=1, path=str(self.path),
            stream_id=stream, queue_capacity=queue_size, flush_seconds=flush_seconds,
            attempted_records=0, accepted_records=0, written_records=0,
            flushed_records=0, dropped_records=0, rejected_records=0,
            queue_high_watermark=0, error=None, accepting=True, closed=False,
            close_timed_out=False, clean_close=False, capture_thread_stopped=None,
            note='Counters cover frame and publish_result envelopes; data fsync only on clean close.')
        try:
            self._file.write(json.dumps(dict(local_schema_version=1, kind='start',
                metadata=metadata), ensure_ascii=False, allow_nan=False) + '\n')
            self._file.flush()
            self._write_status()
        except BaseException:
            self._file.close()
            raise
        self._thread = threading.Thread(target=self._run, name='gelsight-local-records', daemon=True)
        self._thread.start()

    def snapshot(self):
        with self._lock:
            result = dict(self._state)
        result['pending_records'] = self._queue.qsize()
        result['status_monotonic_ns'] = time.monotonic_ns()
        return result

    def submit(self, kind, payload_json, enqueue_clock):
        """Copy immutable serialized text and clock values; never wait for disk."""
        warning = None
        with self._lock:
            self._state['attempted_records'] += 1
            if not self._state['accepting']:
                self._state['rejected_records'] += 1
                return False
            try:
                self._queue.put_nowait((kind, payload_json, tuple(enqueue_clock)))
            except queue.Full:
                self._state['dropped_records'] += 1
                if self._state['dropped_records'] == 1:
                    warning = 'Local acquisition queue full; metadata dropped; image acquisition continues'
                accepted = False
            else:
                self._state['accepted_records'] += 1
                self._state['queue_high_watermark'] = max(
                    self._state['queue_high_watermark'], self._queue.qsize())
                accepted = True
        # No logging/I/O on the capture thread, including on overflow.
        if warning:
            with self._lock:
                self._overflow_warning = warning
        return accepted

    def _notify(self, message):
        if self._on_error is not None:
            try:
                self._on_error(message)
            except Exception:
                pass

    def _write_status(self):
        tmp = self.status_path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(self.snapshot(), ensure_ascii=False, allow_nan=False, indent=2) + '\n')
        os.replace(tmp, self.status_path)

    def _flush(self):
        self._file.flush()
        with self._lock:
            self._state['flushed_records'] = self._state['written_records']
        self._write_status()

    def _run(self):
        last_flush = time.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                batch = []
                try:
                    batch.append(self._queue.get(timeout=.1))
                except queue.Empty:
                    pass
                while batch and len(batch) < 32:
                    try:
                        batch.append(self._queue.get_nowait())
                    except queue.Empty:
                        break
                if batch:
                    lines = []
                    for kind, payload, clock in batch:
                        prefix = json.dumps(dict(local_schema_version=1, kind=kind,
                            enqueue_clock=clock, writer_monotonic_ns=time.monotonic_ns()),
                            separators=(',', ':'), allow_nan=False)
                        lines.append(prefix[:-1] + ',"record":' + payload + '}\n')
                    self._file.writelines(lines)
                    with self._lock:
                        self._state['written_records'] += len(batch)
                with self._lock:
                    warning = getattr(self, '_overflow_warning', None)
                    self._overflow_warning = None
                if warning:
                    self._notify(warning)
                if time.monotonic() - last_flush >= self._flush_seconds:
                    self._flush()
                    last_flush = time.monotonic()
            self._file.flush()
            os.fsync(self._file.fileno())
            with self._lock:
                self._state['flushed_records'] = self._state['written_records']
                self._state['clean_close'] = True
        except Exception as exc:
            with self._lock:
                self._state['error'] = type(exc).__name__ + ': ' + str(exc)
                self._state['accepting'] = False
            self._notify('Local acquisition writer failed; images continue: ' + str(exc))
        finally:
            try:
                self._file.close()
            except Exception as exc:
                with self._lock:
                    self._state['error'] = self._state['error'] or repr(exc)
                    self._state['clean_close'] = False
            with self._lock:
                self._state['accepting'] = False
                self._state['closed'] = True
            try:
                self._write_status()
            except Exception as exc:
                with self._lock:
                    self._state['error'] = self._state['error'] or repr(exc)
                self._notify('Could not save local acquisition status: ' + str(exc))

    def close(self, timeout=5.0, capture_thread_stopped=True):
        with self._lock:
            self._state['accepting'] = False
            self._state['capture_thread_stopped'] = bool(capture_thread_stopped)
        self._stop.set()
        self._thread.join(timeout=max(0., timeout))
        if self._thread.is_alive():
            with self._lock:
                self._state['close_timed_out'] = True
        return self.snapshot()
