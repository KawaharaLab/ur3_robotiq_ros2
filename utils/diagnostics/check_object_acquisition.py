#!/usr/bin/env python3
"""Offline strict audit of all final attempts in an ac progress JSON.

Reads closed bags and local sidecars. Writes only a new report directory.
Exit 0: PASS/PASS_WITH_NOTES, 1: REVIEW, 2: FAIL or incomplete audit.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'data_capture_tools'))
from data_capture_tools import acquisition_quality as qa
from data_capture_tools.compact_export import selected_attempts, trial_windows, read_events, image_payload
from data_capture_tools.object_plan import atomic_json, validate_saved_plan
from compare_gelsight_local_bag import compare
from verify_acquisition_timing import validate

# Screening thresholds for this acquisition protocol, not instrument accuracy.
LIMITS = dict(image_gap_ms=100.0, force_gap_ms=5.0,
              image_boundary_ms=100.0, force_boundary_ms=10.0,
              image_rate_hz=[17.5, 20.0], force_rate_hz=[950.0, 1050.0],
              isolated_force_spike_n=2.5, neighbor_difference_n=0.5,
              image_delivery_p95_ms=50.0, image_delivery_max_ms=250.0,
              force_delivery_p95_ms=200.0, force_delivery_max_ms=500.0)
RANK = {'NOTE': 0, 'REVIEW': 1, 'FAIL': 2}


def finding(issues, level, code, **details):
    issues.append(dict(level=level, code=code, **details))


def status(issues):
    maximum = max((RANK[row['level']] for row in issues), default=-1)
    return ('PASS', 'PASS_WITH_NOTES', 'REVIEW', 'FAIL')[maximum + 1]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scan_payloads(bag, decode_images):
    """Check every DB and payload, retaining only force values and timing JSON."""
    sensors = {topic: [] for topic in qa.SENSORS}
    records, events, integrity = [], [], []
    counts, sizes = Counter(), {t: Counter() for t in qa.SENSORS[:2]}
    image_errors = []
    image_error_count = 0
    if decode_images:
        import cv2
        import numpy as np
    with qa.databases(bag) as files:
        for db in files:
            connection = sqlite3.connect(db.as_uri() + '?mode=ro', uri=True)
            try:
                checks = [row[0] for row in connection.execute('PRAGMA quick_check(1)')]
                if checks != ['ok']:
                    raise ValueError('SQLite quick_check failed: ' + repr(checks))
                integrity.append(dict(database=db.name, quick_check=checks))
                counts.update({name: n for name, n in connection.execute(
                    'SELECT t.name,COUNT(m.id) FROM topics t LEFT JOIN messages m '
                    'ON m.topic_id=t.id GROUP BY t.name')})
                events.extend(read_events(connection))
                query = ('SELECT t.name,t.type,m.timestamp,m.data FROM messages m '
                         'JOIN topics t ON t.id=m.topic_id WHERE t.name IN ('
                         + ','.join('?' for _ in qa.SENSORS + qa.TIMING)
                         + ') ORDER BY m.timestamp,m.id')
                for topic, typ, received, payload in connection.execute(query, qa.SENSORS + qa.TIMING):
                    reader = qa.CDR(payload)
                    if topic in qa.TIMING:
                        if typ != 'std_msgs/msg/String':
                            raise ValueError('Unexpected timing type: ' + topic)
                        records.append((topic, json.loads(reader.string())))
                        continue
                    if typ != qa.TYPES[qa.SENSORS.index(topic)]:
                        raise ValueError('Unexpected sensor type: ' + topic)
                    header = reader.header()
                    row = dict(header_ns=header)
                    if topic in qa.SENSORS[2:]:
                        row['wrench'] = [reader.number('d', 8) for _ in range(6)]
                    else:
                        try:
                            _, jpeg = image_payload(reader)
                            if not jpeg.endswith(b'\xff\xd9'):
                                raise ValueError('JPEG end marker missing')
                            if decode_images:
                                pixels = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if pixels is None or pixels.size == 0:
                                    raise ValueError('JPEG decoder returned no pixels')
                                sizes[topic][str(tuple(pixels.shape))] += 1
                        except Exception as exc:
                            image_error_count += 1
                            if len(image_errors) < 20:
                                image_errors.append(dict(topic=topic, header_ns=header, error=str(exc)))
                    sensors[topic].append(row)
            finally:
                connection.close()
    import yaml
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = Counter()
    for row in info['topics_with_message_count']:
        expected[row['topic_metadata']['name']] += row['message_count']
    differences = {name: dict(metadata=expected[name], actual=counts[name])
                   for name in expected.keys() | counts.keys() if expected[name] != counts[name]}
    if info['message_count'] != sum(counts.values()):
        differences['__total__'] = dict(metadata=info['message_count'], actual=sum(counts.values()))
    details = dict(databases=integrity, message_count_differences=differences,
                   message_counts=dict(counts), image_error_count=image_error_count,
                   image_error_examples=image_errors,
                   image_shapes={key: dict(value) for key, value in sizes.items()},
                   all_images_decoded=decode_images)
    return sensors, records, events, details


def local_records(comparison, issues):
    records = {topic: {} for topic in qa.TIMING[:2]}
    for entry in comparison['local_files']:
        topic = qa.TIMING[qa.SENSORS.index(entry['topic'])]
        state = entry.get('status', {})
        if (entry.get('malformed_lines') or entry.get('trailing_partial_line')
                or entry.get('status_missing') or entry.get('status_error')
                or state.get('clean_close') is not True or state.get('error')
                or any(state.get(key, 0) for key in
                       ('dropped_records', 'rejected_records', 'pending_records'))
                or not (state.get('accepted_records') == state.get('written_records')
                        == state.get('flushed_records'))):
            finding(issues, 'REVIEW', 'local_writer_not_clean', file=entry['path'], status=state)
        with Path(entry['path']).open() as stream:
            next(stream)
            for line in stream:
                try:
                    envelope = json.loads(line)
                    if envelope['kind'] == 'frame':
                        record = envelope['record']
                        if record['stream_id'] != entry['stream_id']:
                            raise ValueError('Local stream mismatch')
                        records[topic][record['header_ns']] = record
                except (ValueError, KeyError, TypeError) as exc:
                    finding(issues, 'REVIEW', 'invalid_local_record', file=entry['path'], error=str(exc))
    return records


def check_windows(windows, sensors, validation, issues, job_settings):
    """Restrict strict coverage/noise decisions to selected complete trials."""
    trial_reports = []
    ordered = {topic: sorted(rows, key=lambda row: row['header_ns']) for topic, rows in sensors.items()}
    headers = {topic: [row['header_ns'] for row in rows] for topic, rows in ordered.items()}
    for window in windows:
        trial_id = window['trial_id']
        lo, hi = window['start_ns'], window['end_ns']
        row_report = dict(trial_id=trial_id, position=window['position_id'], topics={})
        for i, topic in enumerate(qa.SENSORS):
            start, end = bisect_left(headers[topic], lo), bisect_right(headers[topic], hi)
            rows = ordered[topic][start:end]
            actual = [row['header_ns'] for row in rows]
            # Force packets are independently checked against Wrench values below.
            expected = validation['headers'][qa.TIMING[i]]
            expected_in_trial = sorted(h for h in expected if lo <= h <= hi)
            stamps = sorted(set(actual) | (set(expected_in_trial) if i >= 2 else set()))
            gaps = [(b-a)/1e6 for a, b in zip(stamps, stamps[1:])]
            boundary = max(stamps[0]-lo, hi-stamps[-1], 0)/1e6 if stamps else None
            rate = (len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps) > 1 else None
            missing_timing = set(actual) - expected
            missing_sensor = set(expected_in_trial) - set(actual)
            result = dict(sensor_samples=len(rows), effective_samples=len(stamps),
                          header_rate_hz=rate, max_gap_ms=max(gaps, default=0),
                          boundary_gap_ms=boundary, sensor_without_timing=len(missing_timing),
                          timing_without_sensor=len(missing_sensor))
            row_report['topics'][topic] = result
            gap_limit = LIMITS['image_gap_ms' if i < 2 else 'force_gap_ms']
            boundary_limit = LIMITS['image_boundary_ms' if i < 2 else 'force_boundary_ms']
            rate_lo, rate_hi = LIMITS['image_rate_hz' if i < 2 else 'force_rate_hz']
            if not rows or not stamps:
                finding(issues, 'FAIL', 'trial_sensor_missing', trial_id=trial_id, topic=topic)
            if result['max_gap_ms'] > gap_limit or boundary is None or boundary > boundary_limit:
                finding(issues, 'REVIEW', 'trial_coverage_gap', trial_id=trial_id, topic=topic, metrics=result)
            if rate is None or not rate_lo <= rate <= rate_hi:
                finding(issues, 'REVIEW', 'trial_rate_outside_range', trial_id=trial_id, topic=topic, hz=rate)
            if missing_timing:
                finding(issues, 'REVIEW', 'timing_unrecoverable', trial_id=trial_id, topic=topic,
                        count=len(missing_timing))
            if len(actual) != len(set(actual)):
                finding(issues, 'REVIEW', 'duplicate_sensor_header', trial_id=trial_id, topic=topic)
            if i < 2:
                if missing_sensor:
                    finding(issues, 'REVIEW', 'image_missing_but_timing_present', trial_id=trial_id,
                            topic=topic, count=len(missing_sensor))
                continue
            finite = [all(math.isfinite(v) for v in row['wrench']) for row in rows]
            if not all(finite):
                finding(issues, 'FAIL', 'nonfinite_force', trial_id=trial_id, topic=topic)
            spike_headers = [rows[n]['header_ns'] for n in range(1, len(rows)-1)
                             if all(finite[n-1:n+2])
                             and abs(rows[n-1]['wrench'][1]-rows[n+1]['wrench'][1]) < LIMITS['neighbor_difference_n']
                             and min(abs(rows[n]['wrench'][1]-rows[n-1]['wrench'][1]),
                                     abs(rows[n]['wrench'][1]-rows[n+1]['wrench'][1])) > LIMITS['isolated_force_spike_n']]
            if spike_headers:
                finding(issues, 'REVIEW', 'isolated_force_spike', trial_id=trial_id, topic=topic,
                        count=len(spike_headers), header_examples=spike_headers[:10])
            side = 'left' if i == 2 else 'right'
            baseline = window['attempt'].get('tare', {}).get(side)
            if baseline is None or not math.isfinite(baseline):
                finding(issues, 'REVIEW', 'missing_force_baseline', trial_id=trial_id, topic=topic)
            else:
                values = [abs(row['wrench'][1]-baseline) for row in rows if all(math.isfinite(v) for v in row['wrench'])]
                result['peak_absolute_baseline_relative_fy_n'] = max(values, default=0)
                if result['peak_absolute_baseline_relative_fy_n'] >= job_settings.get('hard_limit_n', 30.0):
                    finding(issues, 'REVIEW', 'force_limit_crossing_in_complete_trial', trial_id=trial_id,
                            topic=topic, peak_n=result['peak_absolute_baseline_relative_fy_n'])
        trial_reports.append(row_report)
    return trial_reports


def audit_session(session, selected, target, job_settings, decode_images):
    issues = []
    target.mkdir()
    metas = list((session / 'bag').rglob('metadata.yaml'))
    if len(metas) != 1:
        raise ValueError('Expected exactly one bag metadata.yaml: ' + str(session))
    bag = metas[0].parent
    source_files = [p for p in session.rglob('*') if p.is_file()]
    source_state = {str(p): (p.stat().st_size, p.stat().st_mtime_ns) for p in source_files}
    if any(p.name.endswith('-wal') and p.stat().st_size for p in source_files):
        raise ValueError('Nonempty SQLite WAL; stop recording and close the bag first')
    sensors, records, events, payload = scan_payloads(bag, decode_images)
    atomic_json(target / 'payload_integrity.json', payload)
    if payload['message_count_differences']:
        finding(issues, 'FAIL', 'metadata_message_count_mismatch', differences=payload['message_count_differences'])
    if payload['image_error_count']:
        finding(issues, 'FAIL', 'invalid_jpeg', count=payload['image_error_count'])
    if not decode_images:
        finding(issues, 'REVIEW', 'image_decode_not_checked')
    for topic, shapes in payload['image_shapes'].items():
        if len(shapes) != 1:
            finding(issues, 'REVIEW', 'image_shape_not_constant', topic=topic, shapes=shapes)
    windows = trial_windows(events, selected)
    quality = qa.check_and_save(bag, target / 'acquisition_quality.json', session / 'logs', session)
    comparison = compare(bag, session / 'gelsight_acquisition_records')
    atomic_json(target / 'gelsight_comparison.json', comparison)
    for warning in comparison['warnings']:
        finding(issues, 'REVIEW', 'local_comparison_warning', description=warning)
    if comparison['errors']:
        finding(issues, 'FAIL', 'bag_parse_errors', errors=comparison['errors'])
    local = local_records(comparison, issues)
    # Local frame metadata replaces the late DDS copy only for matching streams.
    combined = {topic: {} for topic in qa.TIMING[:2]}
    force_records = []
    for topic, record in records:
        if topic in combined:
            h = record['header_ns']
            if h in local[topic] and local[topic][h] != record:
                finding(issues, 'REVIEW', 'local_bag_record_disagrees', topic=topic, header_ns=h)
            combined[topic][h] = record
        else:
            force_records.append((topic, record))
    for topic in combined:
        combined[topic].update(local[topic])
    effective_records = [(topic, record) for topic, rows in combined.items()
                         for _, record in sorted(rows.items())] + force_records
    trial_sensors = {topic: [row for row in rows if any(
        w['start_ns'] <= row['header_ns'] <= w['end_ns'] for w in windows)]
        for topic, rows in sensors.items()}
    timing = validate(effective_records, trial_sensors)
    atomic_json(target / 'timing_validation.json', timing)
    if timing['errors_count']:
        finding(issues, 'REVIEW', 'timing_schema_or_value_errors', count=timing['errors_count'],
                examples=timing['errors_examples'])
    # Locate sequence holes and counter increments in the selected trial windows.
    sequences = {}
    counter_history = {}
    stream_headers = {topic: {} for topic in qa.TIMING}
    for topic, record in effective_records:
        sid = record['stream_id']
        if topic in qa.TIMING[:2]:
            samples = [(record['frame_seq'], record['header_ns'])]
        else:
            samples = [(seq, h) for read in record['reads'] for seq, h, _ in read['samples']]
        for seq, h in samples:
            sequences.setdefault((topic, sid), []).append((seq, h))
            stream_headers[topic].setdefault(sid, []).append(h)
        if samples:
            counter_history.setdefault((topic, sid), []).append((min(h for _, h in samples),
                {k: v for k, v in record.items() if k.endswith('_total') and type(v) is int}))
    for (topic, sid), samples in sequences.items():
        samples.sort()
        for (seq_a, h_a), (seq_b, h_b) in zip(samples, samples[1:]):
            if seq_b > seq_a + 1 and any(w['start_ns'] <= h_a <= h_b <= w['end_ns'] for w in windows):
                finding(issues, 'REVIEW', 'source_sequence_gap_in_trial', topic=topic, stream_id=sid,
                        missing=seq_b-seq_a-1, start_ns=h_a, end_ns=h_b)
    for (topic, sid), history in counter_history.items():
        history.sort(key=lambda row: row[0])
        for (h_a, previous), (h_b, current) in zip(history, history[1:]):
            increases = {k: v-previous[k] for k, v in current.items() if k in previous and v > previous[k]}
            if increases and any(w['start_ns'] <= h_b and h_a <= w['end_ns'] for w in windows):
                finding(issues, 'REVIEW', 'producer_counter_increased', topic=topic, stream_id=sid, increments=increases)
    # Build the same Header sets used by validate, without changing that existing checker.
    timing_headers = {topic: set(combined[topic]) for topic in combined}
    for topic in qa.TIMING[2:]:
        timing_headers[topic] = {h for t, r in force_records if t == topic
                                for read in r['reads'] for _, h, _ in read['samples']}
    for topic, row in comparison['topics'].items():
        if row['bag_timing_missing_but_local_present']:
            finding(issues, 'NOTE', 'bag_timing_covered_by_local', topic=topic,
                    count=row['bag_timing_missing_but_local_present'])
        if row['bag_images_without_local']:
            finding(issues, 'REVIEW', 'images_without_local_copy', topic=topic,
                    count=row['bag_images_without_local'])
        if row['publish_failure_records']:
            finding(issues, 'REVIEW', 'image_publish_failure', topic=topic, count=row['publish_failure_records'])
    for topic in qa.TIMING[2:]:
        row = timing['topics'][topic]
        actual_count = len(trial_sensors[qa.SENSORS[qa.TIMING.index(topic)]])
        if row['raw_packet_value_comparisons'] < actual_count:
            finding(issues, 'REVIEW', 'force_values_not_all_raw_verified', topic=topic,
                    verified=row['raw_packet_value_comparisons'], sensor_samples=actual_count)
        if row['raw_packet_value_mismatches']:
            finding(issues, 'FAIL', 'raw_wrench_value_mismatch', topic=topic,
                    count=row['raw_packet_value_mismatches'])
    trial_reports = check_windows(windows, sensors, dict(headers=timing_headers), issues, job_settings)
    for topic in qa.SENSORS:
        for row in quality.get('mapping', {}).get(topic, {}).get('per_trial', []):
            if row['outcome'] != 'END':
                continue
            delays = row['sensor_header_to_bag_ms']
            prefix = 'image' if topic in qa.SENSORS[:2] else 'force'
            if delays['count'] and (delays['p95'] > LIMITS[prefix + '_delivery_p95_ms']
                    or delays['max'] > LIMITS[prefix + '_delivery_max_ms'] or delays['min'] < -10):
                finding(issues, 'REVIEW', 'sensor_delivery_delay', topic=topic,
                        trial_index=row['trial_index'], header_to_bag_ms=delays)
    # Keep every existing finding; known redundant paths may explain it but never erase it.
    for issue in quality.get('issues', []):
        code, topic = issue['code'], issue.get('topic')
        covered = (topic in qa.SENSORS[:2] and comparison['topics'].get(topic, {}).get('bag_images_without_local') == 0)
        redundant = code == 'missing_acquisition_timing_in_trial' and covered
        redundant |= code == 'header_to_bag_delay_in_trial' and topic in qa.TIMING[:2] and bool(local.get(topic))
        redundant |= code == 'sensor_missing_but_timing_present' and issue.get('count') == issue.get('raw_recoverable')
        if code in ('large_sensor_gap_in_trial', 'sensor_coverage_short_at_trial_boundary') and topic in qa.SENSORS[2:]:
            covered_rows = [r['topics'][topic] for r in trial_reports]
            raw_complete = all(r['timing_without_sensor'] == r['force_samples_recoverable_from_raw']
                               for r in quality['mapping'][topic]['per_trial'] if r['outcome'] == 'END')
            redundant |= raw_complete and all(r['max_gap_ms'] <= LIMITS['force_gap_ms']
                and r['boundary_gap_ms'] is not None
                and r['boundary_gap_ms'] <= LIMITS['force_boundary_ms'] for r in covered_rows)
        level = 'NOTE' if redundant or issue['severity'] == 'warning' else 'REVIEW'
        if code == 'recorder_log_findings' and any('[ERROR]' in line for line in quality.get('recorder_log_findings', [])):
            level = 'REVIEW'
        if code in ('session_reference_incomplete', 'conflicting_session_init_ids'):
            level = 'REVIEW'
        finding(issues, level, 'existing_' + code, original=issue)
    if quality['status'] == 'not_checked':
        finding(issues, 'FAIL', 'existing_audit_failed', error=quality.get('error'))
    for window in windows:
        matches = [w for w in quality.get('trials', []) if w['trial_index'] == window['trial_index']
                   and w['start_ns'] == window['start_ns'] and w['end_ns'] == window['end_ns']]
        # Only the number of steps is needed; no calibrated width is inferred here.
        condition = window['conditions']
        expected = 3 if condition['kind'] == 'run' else 3 + 2 * condition['cycles']
        actions = matches[0].get('action_event_counts', {}) if len(matches) == 1 else {}
        if actions.get('auto_goal_sent') != expected or actions.get('auto_action_succeeded') != expected:
            finding(issues, 'REVIEW', 'planned_action_count_mismatch', trial_id=window['trial_id'],
                    expected=expected, actions=actions)
        stream_names = ('gelsight_left', 'gelsight_right', 'mms_left', 'mms_right')
        for topic, name in zip(qa.TIMING, stream_names):
            expected_id = window['attempt'].get('sensor_stream_ids', {}).get(name)
            actual_ids = {sid for sid, hs in stream_headers[topic].items()
                          if any(window['start_ns'] <= h <= window['end_ns'] for h in hs)}
            if not expected_id or actual_ids != {expected_id}:
                finding(issues, 'REVIEW', 'attempt_sensor_stream_mismatch', trial_id=window['trial_id'],
                        topic=topic, expected=expected_id, actual=sorted(actual_ids))
        if not window['attempt'].get('init_id'):
            finding(issues, 'REVIEW', 'attempt_init_id_missing', trial_id=window['trial_id'])
    final_state = {str(p): (p.stat().st_size, p.stat().st_mtime_ns)
                   for p in session.rglob('*') if p.is_file()}
    if final_state != source_state:
        finding(issues, 'FAIL', 'source_changed_during_audit')
    atomic_json(target / 'trials.json', trial_reports)
    return dict(session=str(session), status=status(issues), selected_trials=len(windows),
                issues=issues, report_directory=str(target), trial_ids=[w['trial_id'] for w in windows])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--progress', type=Path, required=True,
                        help='Progress JSON or automatic_jobs directory (latest modified JSON)')
    parser.add_argument('--out', type=Path, required=True, help='New report directory, never overwritten')
    parser.add_argument('--skip-image-decode', action='store_true',
                        help='Diagnostic shortcut only; result cannot pass strict audit')
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error('--out exists; choose a new directory')
    if args.progress.is_dir():
        candidates = sorted(args.progress.glob('*.json'), key=lambda p: (p.stat().st_mtime_ns, p.name))
        if not candidates:
            parser.error('No progress JSON files in directory')
        args.progress = candidates[-1]
    print('Progress: ' + str(args.progress.resolve()), flush=True)
    job_bytes = args.progress.read_bytes()
    job = json.loads(job_bytes)
    args.out.mkdir(parents=True)
    (args.out / 'progress_snapshot.json').write_bytes(job_bytes)
    issues, sessions = [], []
    started = time.monotonic()
    report = dict(schema_version=1, status='RUNNING', checked_time_ns=time.time_ns(),
                  progress=str(args.progress.resolve()),
                  progress_sha256=hashlib.sha256(job_bytes).hexdigest(), job_id=job.get('job_id'),
                  thresholds=LIMITS, sessions=sessions, issues=issues,
                  checker_sha256=digest(__file__),
                  dependency_sha256={Path(m.__file__).name: digest(m.__file__) for m in
                                     (qa, sys.modules[compare.__module__], sys.modules[validate.__module__],
                                      sys.modules[selected_attempts.__module__], sys.modules[validate_saved_plan.__module__])},
                  limitations=['No physical exposure/ADC synchronization calibration',
                               'No image contact/semantic validity or mechanical geometry assessment',
                               'Recoverable records are identified, not merged into training exports',
                               'Full image decode checks pixel availability; subtle visual defects need inspection'])
    atomic_json(args.out / 'summary.json', report)
    try:
        validate_saved_plan(job['plan'])
        rows = [row for p in job['plan']['positions'] for row in p['trials']]
        report['planned_trials'] = len(rows)
        report['complete_trials'] = sum(row['status'] == 'complete' for row in rows)
        report['positions'] = {p['id']: dict(planned=len(p['trials']),
            complete=sum(r['status'] == 'complete' for r in p['trials'])) for p in job['plan']['positions']}
        if job.get('status') != 'complete' or report['complete_trials'] != len(rows):
            finding(issues, 'FAIL', 'object_collection_incomplete', job_status=job.get('status'),
                    incomplete_ids=[r['id'] for r in rows if r['status'] != 'complete'])
        groups = selected_attempts(job)
        names = [Path(s).name for s in groups]
        if len(names) != len(set(names)):
            raise ValueError('Session basenames collide')
        for n, (session_name, selected) in enumerate(groups.items(), 1):
            session = Path(session_name)
            print(f'[{n}/{len(groups)}] Checking {session.name} ({len(selected)} trials)', flush=True)
            try:
                result = audit_session(session, selected, args.out / session.name,
                                       job['plan']['settings'], not args.skip_image_decode)
            except Exception as exc:
                result = dict(session=str(session), status='FAIL', selected_trials=len(selected),
                              issues=[dict(level='FAIL', code='session_audit_incomplete',
                                           error=type(exc).__name__ + ': ' + str(exc))])
            sessions.append(result)
            atomic_json(args.out / 'summary.json', report)
            print('  ' + result['status'], flush=True)
    except KeyboardInterrupt:
        finding(issues, 'FAIL', 'object_audit_interrupted')
    except Exception as exc:
        finding(issues, 'FAIL', 'object_audit_incomplete', error=type(exc).__name__ + ': ' + str(exc))
    if args.progress.read_bytes() != job_bytes:
        finding(issues, 'FAIL', 'progress_changed_during_audit')
    all_issues = issues + [issue for session in sessions for issue in session['issues']]
    report.update(status=status(all_issues), elapsed_seconds=time.monotonic()-started,
                  finding_counts=dict(Counter(row['level'] for row in all_issues)),
                  needs_investigation=[dict(session=s['session'], status=s['status'],
                      issues=[i for i in s['issues'] if i['level'] in ('FAIL', 'REVIEW')])
                      for s in sessions if s['status'] in ('FAIL', 'REVIEW')])
    atomic_json(args.out / 'summary.json', report)
    text = [f"Status: {report['status']}",
            f"Trials: {report.get('complete_trials', 0)}/{report.get('planned_trials', 0)}",
            f"Sessions: {len(sessions)}", f"Finding counts: {report['finding_counts']}",
            'PASS_WITH_NOTES may require local timing/raw force recovery before training export.']
    for issue in issues:
        text.append(json.dumps(issue, ensure_ascii=False))
    for session in report['needs_investigation']:
        text.append('\n' + session['session'])
        text.extend(json.dumps(i, ensure_ascii=False) for i in session['issues'])
    (args.out / 'summary.txt').write_text('\n'.join(text) + '\n')
    print('\n'.join(text[:5]))
    print('Report: ' + str((args.out / 'summary.json').resolve()))
    return 2 if report['status'] == 'FAIL' else 1 if report['status'] == 'REVIEW' else 0


if __name__ == '__main__':
    raise SystemExit(main())
