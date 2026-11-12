"""Conservative ac export: preview by default; original JPEG bytes and full-rate force.

Source bags and progress never change. Only final completed attempts are selected.
One image file per selected message; trial membership is an index, not a copy.
"""
import argparse
import csv
from collections import Counter, defaultdict
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3
import sys

from .acquisition_quality import CDR, databases
from .object_plan import atomic_json

GS = ('/gelsight/left/image_raw/compressed', '/gelsight/right/image_raw/compressed')
FORCE = ('/force_torque/left', '/force_torque/right')
EVENT = '/trial_event'
ACTION = '/robot_action_event'
WIDTH = '/robotiq_2f_gripper/finger_distance_mm'
STATUS = '/robotiq_2f_gripper/status'
AUXILIARY_FIELDS = {
    'robot_action_event.csv': ['trial_id','source_stamp_ns','bag_receive_ns','frame_id',
        'event','trial_index','phase_id','action_type','target_position','target_speed','success'],
    'gripper_width.csv': ['trial_id','source_stamp_ns','bag_receive_ns','frame_id','width_mm'],
    'gripper_status.csv': ['trial_id','source_stamp_ns','bag_receive_ns','frame_id',
        'g_obj','g_flt','g_pr','g_po','g_cu','read_duration_ms'],
}
IMAGE_FIELDS = ['trial_id','topic','source_stamp_ns','bag_receive_ns','frame_id','database_index',
                'message_id','format','jpeg_bytes','artifact_path']
FORCE_FIELDS = ['trial_id','source_stamp_ns','bag_receive_ns','frame_id','database_index','message_id',
                'Fx','Fy','Fz','Tx','Ty','Tz','baseline_Fy','baseline_relative_Fy']
SIDECARS = ('gelsight_acquisition_records','gelsight_local_recording.json',
            'acquisition_quality.json','collection_provenance.json','init_reference.json',
            'trial_conditions.jsonl','automatic_plan.json')


def fingerprint(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def selected_attempts(job):
    groups=defaultdict(list)
    ids=set()
    for position in job['plan']['positions']:
        for index,row in enumerate(position['trials'],1):
            if row['status']!='complete':continue
            if row['id'] in ids:raise ValueError('Duplicate trial ID')
            ids.add(row['id'])
            attempts=[a for a in row.get('attempts',[]) if a.get('status')=='complete']
            if len(attempts)!=1:raise ValueError('Expected exactly one final complete attempt: '+row['id'])
            a=attempts[0]
            if a['end_ros_ns']<=a['start_ros_ns']:raise ValueError('Invalid attempt bounds')
            groups[str(Path(a['session']).resolve())].append(dict(
                trial_id=row['id'],trial_index=index,position_id=position['id'],
                conditions={k:v for k,v in row.items() if k not in ('status','attempts')},
                attempt=a))
    if not groups:raise ValueError('No completed attempts in progress')
    return groups


def read_events(connection):
    events=[]
    query='SELECT m.id,m.timestamp,m.data,t.type FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? ORDER BY m.timestamp,m.id'
    for mid,bt,payload,typ in connection.execute(query,(EVENT,)):
        if typ!='robotiq_2f_gripper_msgs/msg/TrialEvent':raise ValueError('Unexpected trial event type')
        r=CDR(payload);h=r.header();event=r.string();index=r.number('i',4);mode=r.string()
        events.append(dict(source_stamp_ns=h,bag_receive_ns=bt,event=event,trial_index=index,mode=mode))
    return events


def trial_windows(events,selected):
    pairs=[];active=None
    for e in sorted(events,key=lambda e:e['source_stamp_ns']):
        if e['event']=='START':active=e
        elif e['event'] in ('END','ABORT'):
            if active and e['trial_index']==active['trial_index']:
                if e['event']=='END':pairs.append((active,e))
                active=None
    windows=[]
    for item in selected:
        a=item['attempt'];matches=[(s,e) for s,e in pairs if
            s['trial_index']==item['trial_index'] and
            a['start_ros_ns']<=s['source_stamp_ns']<e['source_stamp_ns']<=a['end_ros_ns']]
        if len(matches)!=1:raise ValueError('No unique START/END pair for '+item['trial_id'])
        s,e=matches[0];windows.append(dict(item,start_ns=s['source_stamp_ns'],end_ns=e['source_stamp_ns'],mode=s['mode']))
    windows.sort(key=lambda w:w['start_ns'])
    for a,b in zip(windows,windows[1:]):
        if a['end_ns']>=b['start_ns']:raise ValueError('Overlapping trial windows')
    return windows


def sensor_header(reader):
    sec=reader.number('i',4);ns=reader.number('I',4);frame=reader.string()
    if sec<0 or not 0<=ns<1_000_000_000 or sec*10**9+ns<=0:raise ValueError('Invalid sensor Header')
    return sec*10**9+ns,frame


def image_payload(reader):
    fmt=reader.string();length=reader.number('I',4)
    if length<=0 or reader.pos+length>len(reader.data):raise ValueError('Truncated compressed image')
    value=bytes(reader.data[reader.pos:reader.pos+length])
    if not ('jpeg' in fmt.lower() or 'jpg' in fmt.lower()) or not value.startswith(b'\xff\xd8'):
        raise ValueError('Expected JPEG; no automatic transcoding is permitted')
    return fmt,value


def auxiliary_payload(topic, typ, reader, received):
    """Export actual events/status. Headerless jaw width keeps bag time explicit."""
    if topic == WIDTH:
        formats = {'std_msgs/msg/Float32': ('f', 4), 'std_msgs/msg/Float64': ('d', 8)}
        if typ not in formats:
            raise ValueError('Unexpected jaw-width type')
        return 'gripper_width.csv', received, dict(source_stamp_ns='', frame_id='',
                                                   width_mm=reader.number(*formats[typ]))
    header, frame_id = sensor_header(reader)
    values = dict(source_stamp_ns=header, frame_id=frame_id)
    if topic == ACTION:
        if typ != 'robotiq_2f_gripper_msgs/msg/RobotActionEvent':
            raise ValueError('Unexpected action event type')
        values.update(event=reader.string(), trial_index=reader.number('i', 4),
                      phase_id=reader.number('i', 4), action_type=reader.string(),
                      target_position=reader.number('d', 8), target_speed=reader.number('d', 8),
                      success=bool(reader.number('B', 1)))
        return 'robot_action_event.csv', header, values
    if typ != 'robotiq_2f_gripper_msgs/msg/GripperStatus':
        raise ValueError('Unexpected gripper status type')
    for field in ('g_obj', 'g_flt', 'g_pr', 'g_po', 'g_cu'):
        values[field] = reader.number('B', 1)
    values['read_duration_ms'] = reader.number('f', 4)
    return 'gripper_status.csv', header, values


def copy_sidecars(session,target,write):
    size=0;found=[]
    for name in SIDECARS:
        src=session/name
        files=sorted(src.rglob('*')) if src.is_dir() else [src]
        for path in files:
            if not path.is_file():continue
            if path.is_symlink() or session.resolve() not in path.resolve().parents:
                raise ValueError('Refusing outside/symlink sidecar: '+str(path))
            rel=path.relative_to(session);found.append(str(rel));size+=path.stat().st_size
            if write:
                dest=target/'session_records'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,dest)
    return size,found


def export_session(session,selected,target,write=False):
    if not selected:raise ValueError('No selected completed trials')
    metas=list((session/'bag').rglob('metadata.yaml'))
    if len(metas)!=1:raise ValueError('Expected one bag metadata.yaml in '+str(session))
    bag=metas[0].parent;events=[];counts=Counter();nonfinite=Counter();jpeg_bytes=0
    auxiliary_counts = Counter()
    # One split database is decompressed at a time. No JPEG decoding or resampling.
    with databases(bag) as files:
        # Keep paths, not open cursors: decompressed split files are removed when advancing.
        # Events across all splits are gathered in the first pass.
        for db in files:
            con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
            try:events.extend(read_events(con))
            finally:con.close()
    windows=trial_windows(events,selected)
    writers={}
    class ByteCounter:
        def __init__(self):self.bytes=0
        def write(self,value):self.bytes+=len(value.encode('utf-8'));return len(value)
    csv_fields = {'image_index.csv': IMAGE_FIELDS, 'force_left.csv': FORCE_FIELDS,
                  'force_right.csv': FORCE_FIELDS}
    csv_fields.update(AUXILIARY_FIELDS)
    estimates = {key: ByteCounter() for key in csv_fields}
    if write:
        target.mkdir(parents=True,exist_ok=False)
        atomic_json(target/'manifest.json',dict(status='in_progress',source_bag=str(bag)))
    with ExitStack() as stack:
        for key, fields in csv_fields.items():
            stream=stack.enter_context((target/key).open('x',newline='')) if write else estimates[key]
            writers[key]=csv.writer(stream);writers[key].writerow(fields)
        with databases(bag) as files:
            for dbindex,db in enumerate(files):
                con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
                try:
                    selected_topics = GS + FORCE + (ACTION, WIDTH, STATUS)
                    query = ('SELECT m.id,m.timestamp,t.name,t.type,m.data FROM messages m '
                             'JOIN topics t ON m.topic_id=t.id WHERE t.name IN ('
                             + ','.join('?' for _ in selected_topics) + ') ORDER BY m.timestamp,m.id')
                    for mid,bt,topic,typ,payload in con.execute(query,selected_topics):
                        if topic in (ACTION, WIDTH, STATUS):
                            key, header, values = auxiliary_payload(topic, typ, CDR(payload), bt)
                            window = next((w for w in windows if max(w['attempt']['start_ros_ns'],
                                w['start_ns']-500_000_000) <= header <= w['end_ns']), None)
                            if window is not None:
                                values.update(trial_id=window['trial_id'], bag_receive_ns=bt)
                                writers[key].writerow([values[field] for field in csv_fields[key]])
                                auxiliary_counts[window['trial_id']+'|'+topic] += 1
                            continue
                        expected='sensor_msgs/msg/CompressedImage' if topic in GS else 'geometry_msgs/msg/WrenchStamped'
                        if typ!=expected:raise ValueError('Unexpected type: '+topic)
                        reader=CDR(payload);h,frame_id=sensor_header(reader)
                        window=next((w for w in windows if w['start_ns']<=h<=w['end_ns']),None)
                        if window is None:continue
                        counts[window['trial_id']+'|'+topic]+=1
                        if topic in GS:
                            fmt,jpeg=image_payload(reader);jpeg_bytes+=len(jpeg)
                            side='left' if topic==GS[0] else 'right'
                            rel=Path('images')/side/f'{h}_{dbindex}_{mid}.jpg'
                            if write:
                                path=target/rel;path.parent.mkdir(parents=True,exist_ok=True)
                                with path.open('xb') as f:f.write(jpeg)
                            writers['image_index.csv'].writerow([window['trial_id'],topic,h,bt,frame_id,dbindex,mid,fmt,len(jpeg),str(rel)])
                        else:
                            values=[reader.number('d',8) for _ in range(6)]
                            if not all(math.isfinite(v) for v in values):nonfinite[topic]+=1
                            side='left' if topic==FORCE[0] else 'right'
                            baseline=window['attempt'].get('tare',{}).get(side)
                            relative=values[1]-baseline if baseline is not None else ''
                            writers['force_'+side+'.csv'].writerow([window['trial_id'],h,bt,frame_id,dbindex,mid,*values,baseline,relative])
                finally:con.close()
    missing=[dict(trial_id=w['trial_id'],topic=t) for w in windows for t in GS+FORCE if not counts[w['trial_id']+'|'+t]]
    sidecar_bytes,sidecars=copy_sidecars(session,target,write)
    result=dict(schema_version=1,status='incomplete' if missing else 'complete',source_bag=str(bag),
        source_metadata_sha256=fingerprint(metas[0]),exporter_sha256=fingerprint(__file__),
        selection='Final complete attempts; TrialEvent START/END publisher headers; no contact inference or padding',
        windows=windows,message_counts=dict(counts),missing_trial_topics=missing,nonfinite_force_samples=dict(nonfinite),
        auxiliary_schema_version=1, auxiliary_csvs=list(AUXILIARY_FIELDS),
        auxiliary_message_counts=dict(auxiliary_counts),
        auxiliary_window_basis='Up to 0.5 seconds before START, bounded by attempt start, through END',
        original_jpeg_bytes=jpeg_bytes,session_sidecar_bytes=sidecar_bytes,sidecars=sidecars,
        csv_bytes=sum((target/k).stat().st_size for k in estimates) if write else sum(v.bytes for v in estimates.values()),
        image_policy='Exact JPEG payload; no decoding, resizing, transcoding or downsampling',
        timestamp_correction_applied=False,
        note='Export completeness is not sensor synchronization or source-bag integrity certification. Other topics and pre/post-trial samples remain in the original bag.')
    if write:
        atomic_json(target/'trial_events.json',events)
        result['artifact_sizes']={str(p.relative_to(target)):p.stat().st_size for p in target.rglob('*') if p.is_file() and p.name!='manifest.json'}
        atomic_json(target/'manifest.json',result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--progress',required=True,type=Path,help='ac automatic_jobs progress JSON')
    parser.add_argument('--out',required=True,type=Path,help='New output directory; never overwrite')
    parser.add_argument('--write',action='store_true',help='Actually write JPEG/CSV; absent means preview only')
    parser.add_argument('--session',action='append',help='Optional exact session basename; repeatable')
    args=parser.parse_args(argv)
    if args.out.exists():parser.error('--out exists; choose a new directory')
    source=args.progress.expanduser().resolve();out=args.out.expanduser().resolve()
    job=json.loads(source.read_text());groups=selected_attempts(job)
    if args.session:
        names=set(args.session);groups={s:v for s,v in groups.items() if Path(s).name in names}
        if {Path(s).name for s in groups}!=names:parser.error('Requested session has no final completed attempts')
    # Prevent writing exports into a source session/bag tree.
    for s in groups:
        if Path(s)==out or Path(s) in out.parents:parser.error('Output must be outside source sessions')
    out.mkdir(parents=True,exist_ok=False)
    report=dict(schema_version=1,mode='write' if args.write else 'preview',progress=str(source),
        progress_sha256=fingerprint(source),job_id=job.get('job_id'),sessions=[],
        expected_completed_trials=sum(len(v) for v in groups.values()))
    shutil.copy2(source,out/'source_progress.json')
    atomic_json(out/'report.json',dict(report,status='in_progress'))
    for session,selected in groups.items():
        session=Path(session);target=out/session.name
        print(('Exporting: ' if args.write else 'Previewing: ')+session.name,flush=True)
        try:result=export_session(session,selected,target,args.write)
        except Exception as exc:
            result=dict(status='failed',error=type(exc).__name__+': '+str(exc),source_session=str(session),
                        affected_trials=[r['trial_id'] for r in selected])
            if target.is_dir():atomic_json(target/'manifest.json',result)
        report['sessions'].append(result)
        atomic_json(out/'report.json',dict(report,status='in_progress'))
    report['status']='complete' if all(r['status']=='complete' for r in report['sessions']) else 'incomplete'
    report['original_jpeg_bytes']=sum(r.get('original_jpeg_bytes',0) for r in report['sessions'])
    report['csv_bytes']=sum(r.get('csv_bytes',0) for r in report['sessions'])
    report['session_sidecar_bytes']=sum(r.get('session_sidecar_bytes',0) for r in report['sessions'])
    report['estimated_payload_bytes']=report['original_jpeg_bytes']+report['csv_bytes']+report['session_sidecar_bytes']
    report['exported_or_previewed_trials']=sum(len(r.get('windows',[])) for r in report['sessions'] if r['status']=='complete')
    report['failed_or_incomplete_sessions']=sum(r['status']!='complete' for r in report['sessions'])
    report['note']='Preview reads compressed bags but writes no images/force CSV. JPEG bytes are exact; CSV, indexes, sidecars and filesystem overhead add capacity. Failed sessions are never counted as exported.'
    atomic_json(out/'report.json',report)
    print('Report:',out/'report.json',flush=True)
    print('JPEG payload total GiB:',round(report['original_jpeg_bytes']/1024**3,3))
    print('JPEG + CSV + session records GiB:',round(report['estimated_payload_bytes']/1024**3,3))
    print('Complete trials:',report['exported_or_previewed_trials'],'/',report['expected_completed_trials'])
    return 0 if report['status']=='complete' else 1


if __name__=='__main__':raise SystemExit(main())
