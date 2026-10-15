#!/usr/bin/env python3
"""Validate additional timing records without changing or aligning sensor data."""
import collections
import math
import statistics

NODES = ('gelsight_mini_publisher_1', 'gelsight_mini_publisher_2', 'mms101_driver_1', 'mms101_driver_2')
SENSORS = ('/gelsight/left/image_raw/compressed', '/gelsight/right/image_raw/compressed',
           '/force_torque/left', '/force_torque/right')
TIMING = tuple('/' + node + '/acquisition_timing' for node in NODES)


def stats(values):
    if not values:
        return {'count': 0}
    ordered = sorted(values)
    return {'count': len(values), 'median': statistics.median(ordered),
            'p95': ordered[round((len(ordered)-1)*.95)], 'min': ordered[0], 'max': ordered[-1]}


def validate(records, sensors):
    errors, examples = [], []
    headers = {t: set() for t in TIMING}
    counts = collections.Counter()
    metadata = {t: [] for t in TIMING}
    pending = {}
    candidates = {t: [] for t in TIMING[:2]}
    counters = {}
    packets = {t: {} for t in TIMING[2:]}
    sequence_gaps = collections.Counter()
    previous_sequences = {}

    def check(value, description):
        if not value:
            errors.append(description)

    def clock(value):
        check(isinstance(value, list) and len(value) == 3 and all(type(v) is int for v in value), 'invalid clock sample')
        if not isinstance(value, list) or len(value) != 3:
            raise ValueError('invalid clock sample')
        check(value[0] <= value[2], 'clock bracket reversed')
        return value

    def sequence(topic, stream, name, number):
        key = topic, stream, name
        old = previous_sequences.get(key)
        if old is not None:
            check(number > old, f'{topic}: {name} reversed/duplicate')
            sequence_gaps[topic + ':' + name] += max(0, number-old-1)
        previous_sequences[key] = number

    for topic, report in records:
        if topic not in headers:
            errors.append('unexpected timing topic: ' + topic)
            continue
        counts[topic] += 1
        try:
            check(report['schema_version'] == 1, 'unsupported schema')
            stream = report['stream_id']
            if report['kind'] == 'gelsight_frame':
                check(topic in TIMING[:2], 'wrong frame topic')
                sequence(topic, stream, 'frame_seq', report['frame_seq'])
                headers[topic].add(report['header_ns'])
                start = clock(report['stages']['read_start'])
                end = clock(report['stages']['read_end'])
                preprocess = clock(report['stages']['preprocess_end'])
                check(start[2] <= end[0] <= end[2] <= preprocess[0], 'frame stage order reversed')
                check(report['stages']['backend_pos_msec'] is None or math.isfinite(report['stages']['backend_pos_msec']), 'nonfinite backend timestamp')
                pos = report['stages']['backend_pos_msec']
                if report['backend'] == 'V4L2' and pos is not None:
                    candidates[topic].append((end[0]-round(pos*1e6))/1e6)
                counters[topic] = {'record_publish_errors_total': report['record_publish_errors_total']}
                metadata[topic].append({'backend': report['backend'], 'opencv_version': report['opencv_version'],
                                        'clock_type': report['clock_type']})
            elif report['kind'] == 'mms101_reads':
                check(topic in TIMING[2:], 'wrong serial topic')
                sequence(topic, stream, 'batch_seq', report['batch_seq'])
                counters[topic] = {k: report[k] for k in ('dropped_reads_total', 'dropped_samples_total',
                    'failed_publish_reads_total', 'failed_publish_samples_total')}
                for read in report['reads']:
                    sequence(topic, stream, 'read_seq', read['read_seq'])
                    start, end = clock(read['read_start']), clock(read['read_end'])
                    check(start[2] <= end[0], 'serial read stage order reversed')
                    raw = bytes.fromhex(read['bytes_hex'])
                    offset = read['byte_offset']
                    old_end, old_tail = pending.get((topic, stream), (-1, b''))
                    prefix = old_tail if old_end == offset else b''
                    merged = prefix + raw
                    begin = offset-len(prefix)
                    pending[topic, stream] = offset+len(raw), merged[-24:]
                    for seq, h, packet_offset in read['samples']:
                        sequence(topic, stream, 'sample_seq', seq)
                        headers[topic].add(h)
                        check(packet_offset+25 > offset and packet_offset+25 <= offset+len(raw), 'packet does not end in this read')
                        if begin <= packet_offset and packet_offset+25 <= begin+len(merged):
                            packet = merged[packet_offset-begin:packet_offset-begin+25]
                            check(packet[:2] == b'\x00\x17', 'raw packet header mismatch')
                            values = [int.from_bytes(packet[i:i+3], 'big', signed=True)*scale
                                      for i, scale in zip((4,7,10,13,16,19),
                                                          [report['force_scale']]*3+[report['torque_scale']]*3)]
                            packets[topic][h] = values
            else:
                errors.append('unknown record kind')
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append(f'{topic}: {exc}')
    results = {}
    for i, topic in enumerate(TIMING):
        rows = sensors.get(SENSORS[i], [])
        sensor_headers = {r['header_ns'] for r in rows}
        expected = headers[topic]
        common = sensor_headers & expected
        limits = (max(min(sensor_headers), min(expected)), min(max(sensor_headers), max(expected))) if expected and sensor_headers else None
        inside = lambda hs: {h for h in hs if limits and limits[0] <= h <= limits[1]}
        result = {'timing_messages': counts[topic], 'sensor_messages': len(rows),
                  'sensor_receive_hz': (len(rows)-1)*1e9/(rows[-1]['arrival_monotonic_ns']-rows[0]['arrival_monotonic_ns']) if len(rows)>1 and 'arrival_monotonic_ns' in rows[0] else None,
                  'matched_sensor_headers': len(common),
                  'sensor_headers_without_timing_in_common_range': len(inside(sensor_headers)-expected),
                  'timing_headers_without_sensor_in_common_range': len(inside(expected)-sensor_headers),
                  'counters': counters.get(topic, {})}
        check(bool(common), topic + ': no matched sensor headers')
        if i < 2:
            result['backend_timestamp_to_read_return_ms_candidate_only'] = stats(candidates[topic])
            result['backend_metadata'] = list({tuple(sorted(m.items())): m for m in metadata[topic]}.values())
        else:
            mismatches = 0
            verified = 0
            for row in rows:
                if row['header_ns'] in packets[topic] and 'wrench' in row:
                    verified += 1
                    if row['wrench'] != packets[topic][row['header_ns']]:
                        mismatches += 1
                        if len(examples) < 5: examples.append({'topic': topic, 'header_ns': row['header_ns']})
            result['raw_packet_value_comparisons'] = verified
            result['raw_packet_value_mismatches'] = mismatches
            check(mismatches == 0, topic + ': packet values differ from WrenchStamped')
        results[topic] = result
    return {'schema_version': 1, 'schema_and_mapping_checks_passed': not errors,
            'errors_count': len(errors), 'errors_examples': errors[:20], 'topics': results,
            'sequence_gaps': dict(sequence_gaps), 'value_mismatch_examples': examples,
            'note': 'Independent subscribers can lose messages; common-range gaps are reported, not repaired. Backend timestamps are uncalibrated candidates, not proof of physical synchronization.'}
