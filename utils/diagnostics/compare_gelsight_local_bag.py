#!/usr/bin/env python3
"""Compare a passive local metadata copy with a closed bag. No robot commands."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'data_capture_tools'))
from data_capture_tools.acquisition_quality import _read, stats, SENSORS, TIMING


def compare(bag, local_dir):
    data, counts, errors, _, _, _, streams, _ = _read(bag)
    result = dict(schema_version=1, bag=str(Path(bag).resolve()),
        local_dir=str(Path(local_dir).resolve()), topics={}, local_files=[],
        errors=dict(errors), warnings=[],
        note='Host delivery/processing comparison only; not physical exposure or force timing calibration. '
             'Open JSONL files can have up to one flush interval of pending records.')
    local = {t: {} for t in SENSORS[:2]}
    calls = {t: {} for t in SENSORS[:2]}
    for path in sorted(Path(local_dir).glob('*.jsonl')):
        with path.open(encoding='utf-8') as stream:
            try:
                start = json.loads(next(stream))
                metadata = start['metadata']
                topic = '/' + metadata['image_topic'].strip('/') + '/compressed'
                sid = metadata['stream_id']
            except (StopIteration, ValueError, KeyError, TypeError) as exc:
                result['warnings'].append(str(path)+': bad start record: '+repr(exc))
                continue
            if topic not in local:
                continue
            index = SENSORS.index(topic)
            # Keep a fallback when a bag has images but no timing topic at all.
            if streams.get(TIMING[index]) and sid not in streams[TIMING[index]]:
                continue
            entry = dict(path=str(path.resolve()), stream_id=sid, topic=topic,
                frame_records=0, malformed_lines=0, trailing_partial_line=False)
            for line in stream:
                try:
                    envelope = json.loads(line)
                    record = envelope['record']
                    h = record['header_ns']
                    if envelope['kind'] == 'frame':
                        if record['stream_id'] != sid:
                            raise ValueError('Stream ID differs from start record')
                        entry['frame_records'] += 1
                        if h in local[topic]:
                            result['warnings'].append(str(path)+': duplicate local header '+str(h))
                        local[topic][h] = envelope
                    elif envelope['kind'] == 'publish_result':
                        calls[topic][h] = record
                except (ValueError, KeyError, TypeError) as exc:
                    if not line.endswith('\n'):
                        entry['trailing_partial_line'] = True
                    else:
                        entry['malformed_lines'] += 1
            status_path = path.with_suffix('.status.json')
            if status_path.is_file():
                try:
                    entry['status'] = json.loads(status_path.read_text())
                except ValueError as exc:
                    entry['status_error'] = str(exc)
            else:
                entry['status_missing'] = True
            result['local_files'].append(entry)
    for i, topic in enumerate(SENSORS[:2]):
        sensor = data[topic]
        if not sensor:
            result['warnings'].append('No bag images: '+topic)
            continue
        images = {r[0]: r[1] for r in sensor}
        timing = {r[0]: r[1] for r in data[TIMING[i]]}
        hs = set(images);lh = set(local[topic]);th = set(timing)
        publish = [calls[topic][h] for h in hs & set(calls[topic])]
        row = dict(images=len(sensor),matched_local_frames=len(hs & lh),
            bag_images_without_local=len(hs-lh),bag_images_without_bag_timing=len(hs-th),
            bag_timing_missing_but_local_present=len((hs-th)&lh),
            local_enqueue_to_header_ms=stats([(local[topic][h]['enqueue_clock'][1]-h)/1e6 for h in hs&lh]),
            bag_timing_to_header_ms=stats([(timing[h]-h)/1e6 for h in hs&th]),
            bag_timing_after_local_enqueue_ms=stats([(timing[h]-local[topic][h]['enqueue_clock'][1])/1e6 for h in hs&th&lh]),
            publish_calls=stats([(v['publish_end'][0]-v['publish_start'][0])/1e6 for v in publish]),
            publish_failure_records=sum(not v['success'] for v in publish),
            image_header_to_bag_ms=stats([(bt-h)/1e6 for h,bt in images.items()]),
            image_header_hz=(len(hs)-1)/((max(hs)-min(hs))/1e9) if len(hs)>1 else None)
        result['topics'][topic] = row
    if not result['local_files']:
        result['warnings'].append('No matching local files; confirm directory and recording option')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--local-dir', type=Path, required=True)
    p.add_argument('--out', type=Path, default=Path('gelsight_local_comparison.json'))
    a = p.parse_args()
    if not a.local_dir.is_dir():p.error('--local-dir does not exist')
    if a.out.exists():p.error('--out already exists; choose a new filename')
    report = compare(a.bag, a.local_dir)
    with a.out.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2);stream.write('\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('Report:', a.out.resolve())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
