"""Small, read-only, post-recording audit. Findings never authorize motion/STOP.

Only the known Header, String and TrialEvent CDR layouts are decoded. Image
pixels are not decoded. Bag receive times are never treated as exposure times.
"""
import argparse
import hashlib
from collections import Counter, defaultdict
from contextlib import contextmanager
import json
import math
from pathlib import Path
import shutil
import sqlite3
import struct
import subprocess
import tempfile
import time

from .object_plan import atomic_json

SENSORS = ('/gelsight/left/image_raw/compressed', '/gelsight/right/image_raw/compressed',
           '/force_torque/left', '/force_torque/right')
TIMING = ('/gelsight_mini_publisher_1/acquisition_timing', '/gelsight_mini_publisher_2/acquisition_timing',
          '/mms101_driver_1/acquisition_timing', '/mms101_driver_2/acquisition_timing')
TYPES = ('sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CompressedImage',
         'geometry_msgs/msg/WrenchStamped', 'geometry_msgs/msg/WrenchStamped')
TRIAL = '/trial_event'
ACTION = '/robot_action_event'


class CDR:
    """Bounds-checked reader for ROS2 CDR1, with either byte order."""
    def __init__(self, data):
        if data[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('Unsupported CDR encapsulation')
        self.order = '<' if data[1] else '>'
        self.data, self.pos = memoryview(data)[4:], 0

    def number(self, fmt, alignment):
        self.pos = (self.pos + alignment - 1) // alignment * alignment
        value = struct.unpack_from(self.order + fmt, self.data, self.pos)[0]
        self.pos += struct.calcsize(fmt)
        return value

    def string(self):
        n = self.number('I', 4)
        if n < 1 or self.pos + n > len(self.data) or self.data[self.pos + n - 1] != 0:
            raise ValueError('Invalid or truncated CDR string')
        value = bytes(self.data[self.pos:self.pos + n - 1]).decode('utf-8')
        self.pos += n
        return value

    def header(self):
        sec, ns = self.number('i', 4), self.number('I', 4)
        self.string()  # frame_id
        if sec < 0 or not 0 <= ns < 10**9 or sec * 10**9 + ns <= 0:
            raise ValueError('Invalid/zero Header stamp')
        return sec * 10**9 + ns


def stats(values):
    values = sorted(values)
    if not values:
        return {'count': 0}
    n = len(values)
    return dict(count=n, min=values[0], median=(values[(n-1)//2]+values[n//2])/2,
                p95=values[max(0, math.ceil(n*.95)-1)], max=values[-1])


@contextmanager
def databases(bag):
    """Follow metadata order, including split bags; never modify source files."""
    bag = Path(bag).expanduser().resolve()
    root = bag if bag.is_dir() else bag.parent
    if bag.is_file():
        paths = [bag]
    elif (root/'metadata.yaml').exists():
        import yaml
        info = yaml.safe_load((root/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        if info.get('storage_identifier') != 'sqlite3':
            raise ValueError('Lightweight QA supports sqlite3 bags only')
        paths = [root/p for p in info['relative_file_paths']]
        if len(paths) != len(set(paths)):
            raise ValueError('Duplicate database paths in metadata')
    else:
        paths = sorted(root.glob('*.db3')) + sorted(root.glob('*.db3.zstd'))
    if not paths:
        raise FileNotFoundError('No SQLite bag files')
    # Sequential decompression keeps temporary space limited to one split file.
    with tempfile.TemporaryDirectory(prefix='acquisition_qa_') as temp:
        def iterate():
            for i, path in enumerate(paths):
                if not path.is_file() or root not in path.resolve().parents:
                    raise ValueError('Missing/outside bag database: '+str(path))
                if path.name.endswith('.db3.zstd'):
                    if not shutil.which('zstd'):
                        raise RuntimeError('zstd executable required for file-compressed QA')
                    target = Path(temp)/(str(i)+'.db3')
                    with target.open('wb') as output:
                        subprocess.run(['zstd', '-d', '-c', str(path)], stdout=output,
                                       stderr=subprocess.PIPE, stdin=subprocess.DEVNULL, check=True)
                    yield target
                    target.unlink()
                elif path.suffix == '.db3':
                    yield path
                else:
                    raise ValueError('Unsupported bag file: '+str(path))
        yield iterate()


def _read(bag):
    data = {t: [] for t in SENSORS + TIMING + (TRIAL, ACTION)}
    counts, errors = Counter(), Counter()
    raw_headers = {t: set() for t in TIMING[2:]}
    raw_packets = {t: set() for t in TIMING[2:]}
    sequences = defaultdict(set)
    streams = defaultdict(set)
    producer_counters = defaultdict(dict)
    tails = {}
    with databases(bag) as files:
        for path in files:
            con = sqlite3.connect(path.as_uri()+'?mode=ro', uri=True)
            try:
                topics = {i: (name, typ) for i, name, typ in con.execute('SELECT id,name,type FROM topics')}
                selected = [i for i, (name, typ) in topics.items() if name in data]
                if not selected:
                    continue
                # Avoid loading JPEG payloads into Python. Parse Header only.
                query = '''SELECT m.topic_id,m.timestamp,
                    CASE WHEN t.type='sensor_msgs/msg/CompressedImage'
                    THEN substr(m.data,1,4096) ELSE m.data END
                    FROM messages m JOIN topics t ON t.id=m.topic_id
                    WHERE m.topic_id IN ('''+','.join('?' for _ in selected)+') ORDER BY m.timestamp,m.id'
                for i, bt, payload in con.execute(query, selected):
                    topic, typ = topics[i]
                    counts[topic] += 1
                    try:
                        d = CDR(payload)
                        if topic in SENSORS:
                            if typ != TYPES[SENSORS.index(topic)]:
                                raise ValueError('Unexpected sensor type')
                            h = d.header()
                            finite = True
                            if topic in SENSORS[2:]:
                                finite = all(math.isfinite(d.number('d', 8)) for _ in range(6))
                            data[topic].append((h, bt, finite))
                        elif topic == ACTION:
                            if typ != 'robotiq_2f_gripper_msgs/msg/RobotActionEvent':
                                raise ValueError('Unexpected action event type')
                            h = d.header()
                            event, index, phase = d.string(), d.number('i', 4), d.number('i', 4)
                            data[topic].append((h, bt, event, index, phase))
                        elif topic == TRIAL:
                            if typ != 'robotiq_2f_gripper_msgs/msg/TrialEvent':
                                raise ValueError('Unexpected trial event type')
                            h = d.header()
                            event, index, mode = d.string(), d.number('i', 4), d.string()
                            data[topic].append((h, bt, event, index, mode))
                        else:
                            if typ != 'std_msgs/msg/String':
                                raise ValueError('Unexpected timing type')
                            r = json.loads(d.string())
                            if r['schema_version'] != 1 or not isinstance(r['stream_id'], str):
                                raise ValueError('Unsupported timing schema')
                            stream = r['stream_id']
                            streams[topic].add(stream)
                            for key, value in r.items():
                                if key.endswith('_total') and isinstance(value, int):
                                    producer_counters[topic][key] = max(value, producer_counters[topic].get(key, 0))
                            if topic in TIMING[:2]:
                                if r['kind'] != 'gelsight_frame':
                                    raise ValueError('Wrong timing kind')
                                h = r['header_ns']
                                if type(h) is not int or h <= 0:
                                    raise ValueError('Invalid timing Header')
                                data[topic].append((h, bt, stream))
                                sequences[(topic, stream, 'frame_seq')].add(r['frame_seq'])
                            else:
                                if r['kind'] != 'mms101_reads':
                                    raise ValueError('Wrong timing kind')
                                sequences[(topic, stream, 'batch_seq')].add(r['batch_seq'])
                                for read in r['reads']:
                                    raw = bytes.fromhex(read['bytes_hex'])
                                    offset = read['byte_offset']
                                    end, tail = tails.get((topic, stream), (None, b''))
                                    prefix = tail if end == offset else b''
                                    merged, begin = prefix + raw, offset - len(prefix)
                                    for seq, h, pos in read['samples']:
                                        if type(h) is not int or h <= 0:
                                            raise ValueError('Invalid force sample Header')
                                        raw_headers[topic].add(h)
                                        data[topic].append((h, bt, stream))
                                        if begin <= pos and pos+25 <= begin+len(merged):
                                            packet = merged[pos-begin:pos-begin+25]
                                            if packet[:2] == b'\x00\x17':
                                                raw_packets[topic].add(h)
                                        sequences[(topic, stream, 'sample_seq')].add(seq)
                                    tails[(topic, stream)] = (offset+len(raw), merged[-24:])
                    except (ValueError, KeyError, TypeError, IndexError, struct.error) as exc:
                        errors[topic+': '+type(exc).__name__+': '+str(exc)[:100]] += 1
            finally:
                con.close()
    return data, counts, errors, raw_headers, raw_packets, sequences, streams, producer_counters


def audit_bag(bag, recorder_logs=None):
    started = time.monotonic()
    data, counts, errors, raw_headers, raw_packets, sequences, streams, counters = _read(bag)
    issues = []
    def issue(code, severity='warning', **details):
        issues.append(dict(code=code, severity=severity, **details))
    events = sorted(data[TRIAL], key=lambda r: r[0])
    windows, active = [], {}
    for h, bt, event, index, mode in events:
        key = (index, mode)
        if event == 'START':
            if key in active:
                issue('duplicate_trial_start', 'needs_review', trial_index=index, mode=mode)
            active[key] = h
        elif event in ('END', 'ABORT'):
            if key not in active:
                # Auto ABORT uses mode='automatic'; match only an unambiguous index.
                candidates = [k for k in active if k[0] == index] if event == 'ABORT' else []
                key = candidates[0] if len(candidates) == 1 else key
            begin = active.pop(key, None)
            if begin is None:
                issue('trial_end_without_start', trial_index=index, event=event)
            else:
                windows.append(dict(trial_index=index, mode=key[1], start_ns=begin,
                                    end_ns=h, outcome=event))
    for (index, mode), begin in active.items():
        windows.append(dict(trial_index=index, mode=mode, start_ns=begin, end_ns=None, outcome='INCOMPLETE'))
        issue('incomplete_trial', trial_index=index, mode=mode)
    complete = [w for w in windows if w['outcome'] == 'END']
    if not complete:
        issue('no_completed_trial_window')
    def inside(h):
        return any(w['start_ns'] <= h <= w['end_ns'] for w in complete)
    for w in windows:
        if w['end_ns'] is None:
            continue
        names = Counter(r[2] for r in data[ACTION]
                        if r[3] == w['trial_index'] and w['start_ns'] <= r[0] <= w['end_ns'])
        w['action_event_counts'] = dict(names)
        if w['outcome'] == 'END' and w['mode'].startswith('auto_'):
            for begin, end in (('auto_goal_sent', 'auto_action_succeeded'),
                               ('auto_dwell_start', 'auto_dwell_end')):
                if not names[begin] or names[begin] != names[end]:
                    issue('incomplete_action_event_pairs', 'needs_review',
                          trial_index=w['trial_index'], start_event=begin, end_event=end)
    topics = {}
    for t in SENSORS + TIMING:
        rows = data[t]
        headers = [r[0] for r in rows]
        # MMS samples arrive in batches; do not classify batching as a receive gap.
        differences = [b-a for a, b in zip(headers, headers[1:])]
        unique = sorted(set(headers))
        gap = 20_000_000 if t in SENSORS[2:] else 250_000_000
        gaps = [(a,b) for a,b in zip(unique, unique[1:]) if b-a > gap]
        delays = [(r[1]-r[0])/1e6 for r in rows]
        trial_delays = [(r[1]-r[0])/1e6 for r in rows if inside(r[0])]
        topics[t] = dict(messages=counts[t], parsed_headers=len(rows),
                         duplicate_headers=len(headers)-len(set(headers)),
                         backward_headers=sum(d < 0 for d in differences),
                         header_to_bag_ms=stats(delays), completed_trial_header_to_bag_ms=stats(trial_delays),
                         max_header_gap_ms=max([b-a for a,b in zip(unique,unique[1:])] or [0])/1e6,
                         large_header_gaps_in_completed_trials=sum(any(w['start_ns']<=b and a<=w['end_ns'] for w in complete) for a,b in gaps))
        if not rows:
            issue('topic_without_parsed_headers', 'needs_review' if t in SENSORS else 'warning', topic=t)
        if topics[t]['duplicate_headers'] or topics[t]['backward_headers']:
            trial_fault = any(n > 1 and inside(h) for h,n in Counter(headers).items()) or any(
                b < a and (inside(a) or inside(b)) for a,b in zip(headers,headers[1:]))
            issue('noncontinuous_headers', 'needs_review' if trial_fault else 'warning', topic=t)
        if t in SENSORS and any(not r[2] and inside(r[0]) for r in rows):
            issue('nonfinite_force', 'needs_review', topic=t)
        if t in SENSORS and topics[t]['large_header_gaps_in_completed_trials']:
            issue('large_sensor_gap_in_trial', 'needs_review', topic=t)
        if trial_delays and (stats(trial_delays)['p95'] > 250 or max(trial_delays) > 1000 or min(trial_delays) < -10):
            issue('header_to_bag_delay_in_trial', 'needs_review', topic=t)
    mapping = {}
    for sensor, timing in zip(SENSORS, TIMING):
        actual, expected = {r[0] for r in data[sensor]}, {r[0] for r in data[timing]}
        missing = actual-expected
        absent_sensor = expected-actual
        per_trial = []
        for w in windows:
            if w['end_ns'] is None:
                continue
            in_window = lambda hs: {h for h in hs if w['start_ns'] <= h <= w['end_ns']}
            ms, mt = in_window(missing), in_window(absent_sensor)
            recoverable = len(mt & raw_packets.get(timing, set()))
            sensor_stamps = sorted(in_window(actual))
            boundary_gap = (max(sensor_stamps[0]-w['start_ns'], w['end_ns']-sensor_stamps[-1], 0)/1e6
                            if sensor_stamps else None)
            per_trial.append(dict(trial_index=w['trial_index'], mode=w['mode'], outcome=w['outcome'],
                                  max_boundary_gap_ms=boundary_gap,
                                  sensor_header_to_bag_ms=stats([(r[1]-r[0])/1e6 for r in data[sensor] if w['start_ns']<=r[0]<=w['end_ns']]),
                                  timing_header_to_bag_ms=stats([(r[1]-r[0])/1e6 for r in data[timing] if w['start_ns']<=r[0]<=w['end_ns']]),
                                  sensor_samples=len(sensor_stamps), sensor_without_timing=len(ms),
                                  timing_without_sensor=len(mt), force_samples_recoverable_from_raw=recoverable))
            if w['outcome'] == 'END':
                tolerance_ms = 20 if sensor in SENSORS[2:] else 250
                if boundary_gap is not None and boundary_gap > tolerance_ms:
                    issue('sensor_coverage_short_at_trial_boundary', 'needs_review', topic=sensor,
                          trial_index=w['trial_index'], boundary_gap_ms=boundary_gap)
                if ms:
                    issue('missing_acquisition_timing_in_trial', 'needs_review', topic=sensor,
                          trial_index=w['trial_index'], count=len(ms))
                if mt:
                    issue('sensor_missing_but_timing_present',
                          'warning' if sensor in SENSORS[2:] and recoverable == len(mt) else 'needs_review',
                          topic=sensor, trial_index=w['trial_index'], count=len(mt), raw_recoverable=recoverable)
                if not sensor_stamps:
                    issue('trial_without_sensor_samples', 'needs_review', topic=sensor, trial_index=w['trial_index'])
        mapping[sensor] = dict(timing_topic=timing, matched=len(actual & expected),
                               sensor_without_timing=len(missing), timing_without_sensor=len(absent_sensor),
                               per_trial=per_trial)
        if missing or absent_sensor:
            issue('whole_bag_header_mismatch', topic=sensor, sensor_without_timing=len(missing),
                  timing_without_sensor=len(absent_sensor))
    for error, n in errors.items():
        issue('parse_error', 'needs_review', description=error, count=n)
    sequence_gaps = {}
    for (topic, stream, name), values in sequences.items():
        ordered = sorted(values)
        gaps = sum(b-a-1 for a,b in zip(ordered, ordered[1:]))
        sequence_gaps[topic+':'+stream+':'+name] = gaps
        if gaps:
            issue('producer_sequence_gaps', topic=topic, stream_id=stream, sequence=name, missing=gaps)
    for t, values in counters.items():
        if any(values.values()):
            issue('nonzero_cumulative_producer_counter', topic=t, counters=values,
                  note='Cumulative since producer start, not necessarily during this bag')
    log_findings = []
    if recorder_logs and Path(recorder_logs).is_dir():
        for path in sorted(Path(recorder_logs).glob('rosbag_*.log')):
            for line in path.read_text(errors='replace').splitlines():
                if '[ERROR]' in line or '[WARN]' in line or 'Messages lost' in line:
                    log_findings.append(line[:1000])
        if log_findings:
            issue('recorder_log_findings', count=len(log_findings))
    status = 'needs_review' if any(x['severity']=='needs_review' for x in issues) else 'warning' if issues else 'ok'
    return dict(schema_version=1, status=status, bag=str(Path(bag).resolve()),
                checked_time_ns=time.time_ns(), elapsed_seconds=time.monotonic()-started,
                window_basis='TrialEvent START to END; ABORT/incomplete kept separately; no guessed contact window',
                topics=topics, mapping=mapping, trials=windows, issues=issues,
                stream_ids={t: sorted(v) for t,v in streams.items()}, producer_sequence_gaps=sequence_gaps,
                producer_counters=dict(counters), recorder_log_findings=log_findings[:50],
                checks_not_performed=['physical synchronization/exposure calibration', 'image content/contact validity',
                                      'full raw-packet value comparison', 'motion geometry/force regulation'],
                warning_thresholds=dict(force_header_gap_ms=20, image_header_gap_ms=250,
                                        trial_delay_p95_ms=250, trial_delay_max_ms=1000, negative_delay_ms=-10),
                note='Post-recording quality only. Does not alter motion, resume state, timestamps or source bag.')


def check_and_save(bag, output, recorder_logs=None, session=None):
    """Always try to persist a status, including unsupported/corrupt bags."""
    try:
        report = audit_bag(bag, recorder_logs)
        if session is not None:
            session = Path(session)
            context = {}
            for name in ('collection_provenance.json', 'init_reference.json', 'automatic_plan.json', 'trial_conditions.jsonl'):
                path = session/name
                entry = dict(exists=path.is_file())
                if path.is_file():
                    try:
                        if name.endswith('.jsonl'):
                            rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
                            entry.update(rows=len(rows), init_ids=sorted({str(r['init_id']) for r in rows if r.get('init_id')}))
                        else:
                            value = json.loads(path.read_text())
                            entry.update(readable=True, init_id=value.get('init_id'))
                            if name == 'collection_provenance.json':
                                entry['effective_config_present'] = bool(value.get('effective_config'))
                            if name == 'init_reference.json':
                                entry['available'] = value.get('available')
                                if value.get('available'):
                                    for key in ('summary', 'samples'):
                                        rel = Path(value.get(key, ''))
                                        entry[key+'_exists'] = not rel.is_absolute() and '..' not in rel.parts and (session/rel).is_file()
                    except Exception as exc:
                        entry['error'] = str(exc)
                context[name] = entry
            report['session_context'] = context
            init_ids = {entry['init_id'] for entry in context.values() if entry.get('init_id')}
            init_ids.update(context['trial_conditions.jsonl'].get('init_ids', []))
            if len(init_ids) > 1:
                report['issues'].append(dict(code='conflicting_session_init_ids', severity='needs_review', init_ids=sorted(init_ids)))
                report['status'] = 'needs_review'
            for name in ('collection_provenance.json', 'init_reference.json', 'trial_conditions.jsonl'):
                entry = context[name]
                if not entry['exists'] or entry.get('error') or entry.get('available') is False or entry.get('effective_config_present') is False or any(entry.get(k) is False for k in ('summary_exists', 'samples_exists')):
                    report['issues'].append(dict(code='session_reference_incomplete', severity='warning', file=name))
            if report['issues'] and report['status'] == 'ok':
                report['status'] = 'warning'
    except Exception as exc:
        report = dict(schema_version=1, status='not_checked', bag=str(Path(bag).resolve()),
                      checked_time_ns=time.time_ns(), error=type(exc).__name__+': '+str(exc),
                      note='Quality check failed, not a data-quality pass. Source bag unchanged.')
    report['checker_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    atomic_json(Path(output), report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=Path('acquisition_quality.json'))
    parser.add_argument('--logs', type=Path)
    parser.add_argument('--session', type=Path, help='Optional session folder for provenance/init checks')
    args = parser.parse_args()
    report = check_and_save(args.bag, args.out, args.logs, args.session)
    print(json.dumps(dict(status=report['status'], report=str(args.out.resolve()),
                          issues=len(report.get('issues', []))), ensure_ascii=False))
    return 0 if report['status'] in ('ok', 'warning') else 1


if __name__ == '__main__':
    raise SystemExit(main())
