#!/usr/bin/env python3
"""Validate/preview whole-object schedule. Does not import ROS or send commands."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'data_capture_tools'))
from data_capture_tools.object_plan import build_plan,atomic_json


def main():
    import yaml
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path('object_plan_preview.json'))
    a=p.parse_args();config=yaml.safe_load(a.config.read_text());plan=build_plan(config.get('automatic_collection',{}))
    out=dict(task_name=config.get('task_name'),plan=plan,total_trials=sum(len(p['trials']) for p in plan['positions']))
    atomic_json(a.out,out)
    print(json.dumps(dict(task_name=out['task_name'],total_trials=out['total_trials'],
                         positions=[dict(id=p['id'],lateral_m=p['lateral_m'],vertical_m=p['vertical_m'],trials=len(p['trials'])) for p in plan['positions']],
                         hard_limit_n=plan['settings']['hard_limit_n'],out=str(a.out.resolve())),indent=2))

if __name__=='__main__':main()
