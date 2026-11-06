#!/usr/bin/env python3
"""Read-only source-bag salvage probe. Recovered candidates are never certified trials.

Keeps partial decompression and SQLite recovery outputs under --out only. No
source metadata/progress changes, robot commands, or learning export are made.
"""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'data_capture_tools'))
from data_capture_tools.acquisition_quality import CDR
from data_capture_tools.compact_export import GS,FORCE,EVENT,selected_attempts,sensor_header,image_payload
from data_capture_tools.object_plan import atomic_json


def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda:source.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def bounded_capture(command,output,log,max_bytes):
    """Preserve readable prefix even on decompressor/recovery failure; bounded disk use."""
    count=0;limited=False
    with output.open('xb') as target,log.open('x') as errors:
        process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=errors)
        try:
            while True:
                block=process.stdout.read(1024*1024)
                if not block:break
                if count+len(block)>max_bytes:
                    limited=True;process.kill();break
                target.write(block);count+=len(block)
            process.stdout.close();code=process.wait()
        finally:
            if process.poll() is None:process.kill();process.wait()
    return dict(command=command,returncode=code,output_bytes=count,limit_exceeded=limited,
                stderr=log.read_text(errors='replace')[-8000:])


def inspect_database(database,selected):
    result=dict(path=str(database),status='unreadable',topic_counts={},events=[],trials=[],
                query_completed=False,parse_errors={},warnings=[])
    connection=None
    states={item['trial_id']:dict(item=item,events=[],topics={t:dict(messages=0,
        valid_payloads=0,_headers=[],first_header_ns=None,last_header_ns=None,max_header_gap_ms=0.) for t in GS+FORCE}) for item in selected}
    try:
        connection=sqlite3.connect(database.resolve().as_uri()+'?mode=ro&immutable=1',uri=True)
        connection.execute('PRAGMA query_only=ON')
        try:result['quick_check']=[r[0] for r in connection.execute('PRAGMA quick_check(5)')]
        except sqlite3.Error as exc:result['quick_check_error']=str(exc)
        schema={name:sql for name,sql in connection.execute("SELECT name,sql FROM sqlite_master WHERE type='table'")}
        result['tables']=list(schema)
        for table,columns in [('topics',{'id','name','type'}),('messages',{'id','topic_id','timestamp','data'})]:
            if table not in schema:raise ValueError('Missing '+table+' table')
            actual={r[1] for r in connection.execute('PRAGMA table_info('+table+')')}
            if not columns<=actual:raise ValueError('Unrecognized columns: '+table)
        topics={i:(name,typ) for i,name,typ in connection.execute('SELECT id,name,type FROM topics')}
        result['topics']={str(k):list(v) for k,v in topics.items()}
        counts=Counter();errors=Counter();previous={};event_list=[]
        # Scan the table directly: a damaged timestamp index must not hide readable rows.
        cursor=connection.execute('SELECT id,topic_id,timestamp,data FROM messages NOT INDEXED')
        result['status']='partial_read'
        try:
            for mid,tid,bt,payload in cursor:
                topic,typ=topics.get(tid,('unknown:'+str(tid),''));counts[topic]+=1
                if topic not in GS+FORCE+(EVENT,):continue
                try:
                    r=CDR(payload);h,frame=sensor_header(r)
                    if topic==EVENT:
                        if typ!='robotiq_2f_gripper_msgs/msg/TrialEvent':raise ValueError('Unexpected event type')
                        e=dict(header_ns=h,bag_receive_ns=bt,event=r.string(),trial_index=r.number('i',4),mode=r.string())
                        event_list.append(e)
                        for state in states.values():
                            item=state['item'];a=item['attempt']
                            if item['trial_index']==e['trial_index'] and a['start_ros_ns']<=h<=a['end_ros_ns']:state['events'].append(e)
                        continue
                    if topic in GS:
                        if typ!='sensor_msgs/msg/CompressedImage':raise ValueError('Unexpected image type')
                        fmt,jpeg=image_payload(r)
                        # Basic JPEG completeness only; not a full pixel-decoding validation.
                        valid=jpeg.endswith(b'\xff\xd9')
                    else:
                        if typ!='geometry_msgs/msg/WrenchStamped':raise ValueError('Unexpected force type')
                        values=[r.number('d',8) for _ in range(6)];valid=all(math.isfinite(v) for v in values)
                    for state in states.values():
                        a=state['item']['attempt']
                        # Attempt envelope is intentionally used until original paired events are known.
                        if not a['start_ros_ns']<=h<=a['end_ros_ns']:continue
                        row=state['topics'][topic];row['messages']+=1;row['valid_payloads']+=int(valid);row['_headers'].append((h,valid))
                        if row['first_header_ns'] is None or h<row['first_header_ns']:row['first_header_ns']=h
                        row['last_header_ns']=max(h,row['last_header_ns'] or h)
                        key=(state['item']['trial_id'],topic)
                        if key in previous and h>previous[key]:row['max_header_gap_ms']=max(row['max_header_gap_ms'],(h-previous[key])/1e6)
                        previous[key]=h
                except Exception as exc:errors[topic+': '+type(exc).__name__+': '+str(exc)]+=1
            result['query_completed']=True;result['status']='scan_completed'
        except sqlite3.Error as exc:result['scan_error']=str(exc)
        result['topic_counts']=dict(counts);result['parse_errors']=dict(errors);result['events']=event_list
    except Exception as exc:result['error']=type(exc).__name__+': '+str(exc)
    finally:
        if connection is not None:connection.close()
    for trial_id,state in states.items():
        es=sorted(state['events'],key=lambda e:e['header_ns']);starts=[e for e in es if e['event']=='START'];ends=[e for e in es if e['event']=='END']
        paired=len(starts)==len(ends)==1 and starts[0]['header_ns']<ends[0]['header_ns'] and not any(e['event']=='ABORT' for e in es)
        paired_topics={}
        for topic,row in state['topics'].items():
            headers=row.pop('_headers')
            if not paired:continue
            hs=sorted(h for h,valid in headers if valid and starts[0]['header_ns']<=h<=ends[0]['header_ns'])
            paired_topics[topic]=dict(valid_messages=len(hs),unique_headers=len(set(hs)),
                first_header_ns=hs[0] if hs else None,last_header_ns=hs[-1] if hs else None,
                start_boundary_gap_ms=(hs[0]-starts[0]['header_ns'])/1e6 if hs else None,
                end_boundary_gap_ms=(ends[0]['header_ns']-hs[-1])/1e6 if hs else None,
                max_header_gap_ms=max(((b-a)/1e6 for a,b in zip(hs,hs[1:])),default=None))
        present=paired and all(v['valid_messages']>0 for v in paired_topics.values())
        result['trials'].append(dict(trial_id=trial_id,conditions=state['item']['conditions'],
            attempt_start_ns=state['item']['attempt']['start_ros_ns'],attempt_end_ns=state['item']['attempt']['end_ros_ns'],
            paired_start_end=paired,events=es,topics=state['topics'],paired_interval_topics=paired_topics,
            assessment='candidate_requires_validation' if paired and present else 'partial_or_not_found',
            note='Counts use the wider attempt envelope, not a final learning interval. No claim of all original messages surviving.'))
    return result


def probe_session(session,selected,target,max_bytes):
    target.mkdir();sources=sorted((session/'bag').rglob('*.db3.zstd'))
    if len(sources)!=1:raise ValueError('This diagnostic expects one compressed SQLite file per selected session')
    source=sources[0];before=dict(bytes=source.stat().st_size,sha256=sha256(source))
    result=dict(source_session=str(session),source=str(source),source_before=before,
                affected_trials=[s['trial_id'] for s in selected],recovery_is_not_certification=True)
    raw=target/'decompressed_prefix.partial.db3'
    result['decompression']=bounded_capture(['zstd','-d','-c',str(source)],raw,target/'zstd_stderr.log',max_bytes)
    if raw.stat().st_size:
        result['prefix_scan']=inspect_database(raw,selected)
        sqlite=shutil.which('sqlite3')
        if sqlite and not result['decompression']['limit_exceeded']:
            help_result=subprocess.run([sqlite,'-batch',':memory:','.help recover'],stdin=subprocess.DEVNULL,capture_output=True,text=True)
            result['sqlite_recover_help']=(help_result.stdout+help_result.stderr)[-2000:]
            if '.recover' in help_result.stdout:
                sql=target/'recovery.sql'
                result['sqlite_recover']=bounded_capture([sqlite,'-batch',str(raw),'.recover --ignore-freelist'],sql,target/'recover_stderr.log',max_bytes*2)
                if not result['sqlite_recover']['limit_exceeded']:
                    recovered=target/'recovered_candidate.db3'
                    with sql.open('rb') as stream,(target/'import_stdout.log').open('x') as out,(target/'import_stderr.log').open('x') as err:
                        proc=subprocess.run([sqlite,'-batch',str(recovered)],stdin=stream,stdout=out,stderr=err)
                    result['sqlite_import']=dict(returncode=proc.returncode,stderr=(target/'import_stderr.log').read_text(errors='replace')[-4000:])
                    if recovered.exists():result['recovered_scan']=inspect_database(recovered,selected)
            else:result['sqlite_recover_unavailable']='sqlite3 shell has no .recover support'
        else:result['sqlite_recover_unavailable']='sqlite3 command missing or decompression size cap reached'
    try:
        import yaml
        metas=list((session/'bag').rglob('metadata.yaml'))
        info=yaml.safe_load(metas[0].read_text())['rosbag2_bagfile_information']
        expected={x['topic_metadata']['name']:x['message_count'] for x in info['topics_with_message_count']}
        result['metadata_topic_counts']=expected
        for key in ('prefix_scan','recovered_scan'):
            if key in result:
                scan=result[key];observed=scan.get('topic_counts',{})
                scan['metadata_count_differences']={t:dict(expected=n,observed=observed.get(t,0),difference=observed.get(t,0)-n) for t,n in expected.items() if observed.get(t,0)!=n}
    except Exception as exc:result['metadata_comparison_error']=type(exc).__name__+': '+str(exc)
    result['source_after']=dict(bytes=source.stat().st_size,sha256=sha256(source))
    result['source_unchanged']=result['source_before']==result['source_after']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--progress',required=True,type=Path);p.add_argument('--out',required=True,type=Path)
    p.add_argument('--session',action='append',required=True,help='Exact session basename; repeatable')
    p.add_argument('--max-prefix-gib',type=float,default=2.)
    a=p.parse_args()
    if not shutil.which('zstd'):p.error('zstd command is required')
    if not .1<=a.max_prefix_gib<=8:p.error('--max-prefix-gib must be between 0.1 and 8')
    out=a.out.resolve()
    if out.exists():p.error('--out exists; choose a new directory')
    groups=selected_attempts(json.loads(a.progress.read_text()));names=set(a.session)
    chosen={s:rows for s,rows in groups.items() if Path(s).name in names}
    if {Path(s).name for s in chosen}!=names:p.error('Session not found among completed attempts')
    if any(Path(s)==out or Path(s) in out.parents for s in chosen):p.error('Output must be outside original sessions')
    out.mkdir(parents=True)
    report=dict(schema_version=1,progress=str(a.progress.resolve()),progress_sha256=sha256(a.progress),sessions=[],
        note='Recovery candidates only. Original bags and progress are unchanged. A clean recovered SQLite database does not prove all original messages survived.')
    for session,rows in chosen.items():
        print('Checking:',Path(session).name,flush=True)
        try:r=probe_session(Path(session),rows,out/Path(session).name,int(a.max_prefix_gib*1024**3))
        except Exception as exc:r=dict(source_session=session,error=type(exc).__name__+': '+str(exc),affected_trials=[x['trial_id'] for x in rows])
        report['sessions'].append(r);atomic_json(out/'report.json',report)
    print('Send only:',out/'report.json')
    print('Keep the local partial/recovered files for follow-up. Do not use them as certified training data yet.')
    return 0


if __name__=='__main__':raise SystemExit(main())
