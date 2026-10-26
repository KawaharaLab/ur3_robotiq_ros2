#!/usr/bin/env python3
"""Read init_records summaries only. Never connects to ROS or commands hardware."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path('./init_records'))
    p.add_argument('--out',type=Path,default=Path('./init_width_check_run1.json'))
    a=p.parse_args()
    if a.out.exists():p.error('Output exists; choose a new --out')
    files=sorted(a.root.glob('*/summary.json'))
    if not files:p.error('No init summaries under '+str(a.root))
    records=[];groups=collections.defaultdict(list)
    for f in files:
        s=json.loads(f.read_text());contact=s.get('contact') or {};metadata=s.get('metadata') or {}
        status=contact.get('sources',{}).get('gripper_status',{})
        startup=metadata.get('driver_initialization') or {}
        config=metadata.get('config') or {}
        settings={k:config.get(k) for k in ('target_diameter','contact_threshold','contact_margin',
                                            'centering_step_grip','centering_step_arm','safe_margin')}
        settings_id=hashlib.sha256(json.dumps(settings,sort_keys=True).encode()).hexdigest()[:16]
        row=dict(init_id=s['init_id'],file=str(f.resolve()),status=s.get('status'),
                 motion_success=s.get('motion_success'),valid_for_collection=s.get('valid_for_collection',False),
                 recording_complete=s.get('recording_complete'),error=s.get('error'),
                 writer_error=s.get('writer_error'),dropped_log_rows=s.get('dropped_log_rows'),
                 node_run_id=metadata.get('node_run_id'),driver_startup_id=startup.get('startup_id'),
                 activation_performed=startup.get('activation_performed'),
                 reference_id=metadata.get('reference_id'),reference_width_mm=metadata.get('reference_width_mm'),
                 mount_id=metadata.get('mount_id'),conversion_model=metadata.get('conversion_model'),
                 init_settings=settings,init_settings_id=settings_id,
                 g_po=status.get('status',{}).get('g_po'),g_pr=status.get('status',{}).get('g_pr'),
                 status_header_ns=status.get('header_ns'),status_age_ms=status.get('receive_age_ms'),
                 effective_width_m=contact.get('effective_width_m'),
                 force_monitor=contact.get('force_monitor'),contact_warnings=contact.get('warnings'),
                 centering_result=contact.get('sources',{}).get('centering_result'))
        records.append(row)
        if row['valid_for_collection'] and row['g_po'] is not None:
            driver_key=row['driver_startup_id'] or ('unknown:'+str(row['node_run_id']))
            groups[(driver_key,row['reference_id'],row['reference_width_mm'],row['mount_id'],settings_id)].append(row['g_po'])
        print(s['init_id'],'valid='+str(row['valid_for_collection']),'gPO='+str(row['g_po']),
              'activation='+str(row['activation_performed']))
    result=dict(schema_version=1,records=records,
                groups=[dict(driver_startup_id=k[0],reference_id=k[1],reference_width_mm=k[2],mount_id=k[3],init_settings_id=k[4],
                             repeats=len(v),contact_g_po=v,median_count=statistics.median(v),
                             min_count=min(v),max_count=max(v)) for k,v in groups.items()],
                note='Grouped by driver startup and reference, not proof of a physical width calibration. '
                     'Compare force/tare and centering results as well as count.')
    with a.out.open('x') as f:f.write(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print('Send:',a.out.resolve())


if __name__=='__main__':main()
