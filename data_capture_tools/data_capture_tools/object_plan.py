"""Deterministic acquisition plans and crash-safe progress (no ROS dependencies)."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import random
import time


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False, default=str).encode()).hexdigest()


def atomic_json(path, value):
    path=Path(path); temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as f:
        json.dump(value,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(temporary,path)


def build_plan(settings):
    """Freeze the whole object schedule before motion. Never silently choose depths."""
    s=copy.deepcopy(settings)
    def number(value,name,lo,hi):
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError(name+' outside '+str((lo,hi)))
    depths=s.get('depths_mm');speeds=s.get('loading_speeds')
    if not isinstance(depths,list) or not depths or not isinstance(speeds,list) or not speeds:
        raise ValueError('Set automatic_collection.depths_mm and loading_speeds before collection')
    for d in depths:number(d,'depths_mm',.01,10.)
    for v in speeds:number(v,'loading_speeds',0.,1.)
    if len(set(depths))!=len(depths) or len(set(speeds))!=len(speeds):raise ValueError('Duplicate conditions')
    for name,default in [('repeats',2),('seed',20261028),('batch_size',10)]:
        s.setdefault(name,default)
        if type(s[name]) is not int or s[name]<1:raise ValueError('Invalid '+name)
    if s['repeats']>100 or s['batch_size']>10:raise ValueError('Too many repeats or bag batch size >10')
    for name,default,lo,hi in [('hold_s',4.,0.,10.),('rest_s',.8,.1,30.),('completion_settle_s',1.5,0.,5.),
                              ('hard_limit_n',20.,.1,20.),('offset_m',.01,0.,.02),
                              ('force_timeout_s',.25,.05,1.),('image_timeout_s',.5,.15,2.),
                              ('min_free_gib',5.,.1,1000.)]:
        s.setdefault(name,default);number(s[name],name,lo,hi)
    for name in ('require_timing','auto_init','auto_play'):
        s.setdefault(name,True)
        if type(s[name]) is not bool:raise ValueError(name+' must be boolean')
    s.setdefault('release_speed',.3);number(s['release_speed'],'release_speed',0.,1.)
    s.setdefault('release_speeds',[])
    if not isinstance(s['release_speeds'],list):raise ValueError('release_speeds must be a list')
    for v in s['release_speeds']:number(v,'release_speeds',0.,1.)
    s.setdefault('knead_conditions',[])
    if not isinstance(s['knead_conditions'],list):raise ValueError('knead_conditions must be a list')
    for k in s['knead_conditions']:
        for name in ('shallow_mm','deep_mm'):number(k.get(name),name,.01,10.)
        if k['shallow_mm']>=k['deep_mm']:raise ValueError('knead requires shallow < deep')
        if type(k.get('cycles',5)) is not int or not 1<=k.get('cycles',5)<=50:raise ValueError('Invalid cycles')
        for name,default,hi in [('loading_speed',.1,1.),('unloading_speed',.1,1.),('deep_hold_s',.1,10.),('shallow_hold_s',.1,10.)]:
            k.setdefault(name,default);number(k[name],name,0.,hi)
        k.setdefault('cycles',5)
    positions=[dict(id='base',lateral_m=0.,vertical_m=0.),
               dict(id='front',lateral_m=s['offset_m'],vertical_m=0.),
               dict(id='back',lateral_m=-s['offset_m'],vertical_m=0.),
               dict(id='up',lateral_m=0.,vertical_m=s['offset_m'])]
    rng=random.Random(s['seed']);others=positions[1:];rng.shuffle(others);positions=positions[:1]+others
    result=[]
    for p in positions:
        rows=[]
        for repeat in range(s['repeats']):
            round_rows=[dict(kind='run',depth_mm=d,speed=v,hold_s=s['hold_s'],release_speed=s['release_speed']) for d in depths for v in speeds]
            middle=sorted(depths)[len(depths)//2];speed=speeds[len(speeds)//2]
            round_rows.extend(dict(kind='run',depth_mm=middle,speed=speed,hold_s=s['hold_s'],release_speed=v,variant='aur') for v in s['release_speeds'])
            round_rows.extend(dict(kind='knead',**k) for k in s['knead_conditions'])
            rng.shuffle(round_rows)
            for row in round_rows:
                row.update(id=p['id']+'_'+str(len(rows)+1).zfill(4),repeat=repeat+1,status='pending',attempts=[]);rows.append(row)
        result.append(dict(**p,reference=None,trials=rows))
    return dict(schema_version=1,settings=s,positions=result,plan_id=digest(dict(settings=s,positions=result)))


def steps_for_trial(row,width):
    """Mirror existing run/knead goal values and dwell times, with raw-count logging upstream."""
    def step(w,s,f,t,phase):
        if not .001<=w<=.142:raise ValueError('Target width outside 1..142 mm')
        return dict(width_m=w,speed=s,force=f,dwell_s=t,phase=phase)
    result=[step(.140,.3,.1,1.,1)]
    if row['kind']=='run':
        result.append(step(width-2*row['depth_mm']/1000,row['speed'],.5,row['hold_s'],2))
    else:
        shallow=width-2*row['shallow_mm']/1000;deep=width-2*row['deep_mm']/1000
        result.append(step(shallow,row['loading_speed'],.5,0.,101))
        for i in range(row['cycles']):
            result.extend([step(deep,row['loading_speed'],.5,row['deep_hold_s'],200+2*i),
                           step(shallow,row['unloading_speed'],.5,row['shallow_hold_s'],201+2*i)])
    result.append(step(.140,row.get('release_speed',.3),.1,1.,3))
    return result


def recover_progress(job):
    """Incomplete or unverified attempts are preserved and retried, never called complete."""
    for p in job['plan']['positions']:
        for t in p['trials']:
            if t['status']!='complete':
                if t['attempts'] and t['attempts'][-1].get('status') in ('running','awaiting_bag'):
                    t['attempts'][-1].update(status='interrupted',interrupted_time_ns=time.time_ns())
                t['status']='pending'
    job['status']='paused'
    return job


def validate_saved_plan(plan):
    expected=build_plan(plan['settings'])
    if plan.get('plan_id')!=expected['plan_id']:raise ValueError('Plan fingerprint changed')
    if len(plan['positions'])!=len(expected['positions']):raise ValueError('Position count changed')
    for actual,reference in zip(plan['positions'],expected['positions']):
        for key in ('id','lateral_m','vertical_m'):
            if actual[key]!=reference[key]:raise ValueError('Position changed')
        if len(actual['trials'])!=len(reference['trials']):raise ValueError('Trial count changed')
        for a,b in zip(actual['trials'],reference['trials']):
            if {k:v for k,v in a.items() if k not in ('status','attempts')}!={k:v for k,v in b.items() if k not in ('status','attempts')}:
                raise ValueError('Trial condition changed')
