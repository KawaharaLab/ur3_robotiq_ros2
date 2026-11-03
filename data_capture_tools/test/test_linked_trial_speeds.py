"""Frozen speed conditions and actual runner goals; no ROS or hardware required."""
import copy
import json
from types import SimpleNamespace as NS
import unittest

from data_capture_tools.object_plan import build_plan, steps_for_trial, validate_saved_plan
from data_capture_tools.object_automation import ObjectAutomationMixin


def production_settings():
    return dict(link_trial_speeds=True, depths_mm=[2.,3.,4.,5.,6.],
                loading_speeds=[0.,.1,.2], repeats=2, release_speeds=[],
                knead_conditions=[dict(shallow_mm=2.,deep_mm=depth,cycles=5,
                    loading_speed=speed,deep_hold_s=hold,shallow_hold_s=hold)
                    for depth in (4.,6.) for speed in (0.,.1) for hold in (.1,.5)])


class LinkedSpeedTests(unittest.TestCase):
    def test_184_trial_plan_and_all_movement_speeds(self):
        plan=build_plan(production_settings())
        self.assertEqual(sum(len(p['trials']) for p in plan['positions']),184)
        for position in plan['positions']:
            self.assertEqual(sum(r['kind']=='run' for r in position['trials']),30)
            self.assertEqual(sum(r['kind']=='knead' for r in position['trials']),16)
            for row in position['trials']:
                speed=row['speed'] if row['kind']=='run' else row['loading_speed']
                steps=steps_for_trial(row,.04)
                self.assertTrue(all(step['speed']==speed for step in steps))
                self.assertEqual(steps[0]['width_m'],.140)
                self.assertEqual(steps[-1]['width_m'],.140)
        validate_saved_plan(json.loads(json.dumps(plan)))

    def test_explicit_equal_knead_speed_is_accepted(self):
        s=production_settings()
        for k in s['knead_conditions']:k['unloading_speed']=k['loading_speed']
        validate_saved_plan(build_plan(s))

    def test_conflicting_conditions_rejected_before_motion(self):
        for change in ('bool','aur','knead_return','knead_release'):
            s=production_settings()
            if change=='bool':s['link_trial_speeds']='true'
            if change=='aur':s['release_speeds']=[.2]
            if change=='knead_return':s['knead_conditions'][0]['unloading_speed']=.2
            if change=='knead_release':s['knead_conditions'][0]['release_speed']=.2
            with self.subTest(change=change),self.assertRaises(ValueError):build_plan(s)

    def test_legacy_plan_fingerprint_and_independent_speeds_unchanged(self):
        s=dict(depths_mm=[1.,2.,3.],loading_speeds=[.02,.1,.2],repeats=2)
        plan=build_plan(s)
        self.assertEqual(plan['plan_id'],'49441473984fd2184af9d54b5952cb0571b48dea98f2e7c13d46919675e25afb')
        self.assertNotIn('link_trial_speeds',plan['settings'])
        self.assertNotIn('preparation_speed',plan['positions'][0]['trials'][0])
        validate_saved_plan(json.loads(json.dumps(plan)))
        s.update(link_trial_speeds=False,release_speed=.3,release_speeds=[.02,.2],
                 knead_conditions=[dict(shallow_mm=1.,deep_mm=3.,loading_speed=.1,unloading_speed=.2)])
        plan=build_plan(s)
        row=next(r for r in plan['positions'][0]['trials'] if r['kind']=='knead')
        steps=steps_for_trial(row,.04)
        self.assertEqual([steps[i]['speed'] for i in (0,1,2,3,-1)],[.3,.1,.1,.2,.3])
        aur=next(r for r in plan['positions'][0]['trials'] if r.get('variant')=='aur')
        self.assertEqual(steps_for_trial(aur,.04)[-1]['speed'],aur['release_speed'])


class ActualRunnerSpeedTests(unittest.TestCase):
    def execute(self,row,width=.04):
        calls=[]
        monitor=NS(_offset_left=1.,_offset_right=2.,tare=lambda:calls.append(('tare',)))
        n=NS(_effective_object_width_m=width,_force_monitor=monitor,
             _auto_streams={},_auto_trial_index=1,
             _auto_job={'plan':{'settings':{'completion_settle_s':1.5,'rest_s':.8}}},
             get_clock=lambda:NS(now=lambda:NS(nanoseconds=123)),
             _auto_grip=lambda *args:calls.append(('goal',)+args),
             _auto_wait=lambda *args,**kw:None,_auto_save=lambda:None,
             _publish_trial_event=lambda *args:calls.append(('event',)+args),
             _auto_event=lambda *args:None)
        attempt={}
        ObjectAutomationMixin._auto_execute_trial(n,row,attempt)
        self.assertEqual(row['status'],'awaiting_bag')
        self.assertEqual(attempt['tare']['left'],1.)
        self.assertEqual(calls[1],('tare',))
        return calls

    def test_runner_uses_planned_preopen_not_hardcoded_point_three(self):
        plan=build_plan(production_settings())
        for kind in ('run','knead'):
            for speed in (0.,.1):
                row=copy.deepcopy(next(r for r in plan['positions'][0]['trials']
                      if r['kind']==kind and r.get('speed',r.get('loading_speed'))==speed))
                with self.subTest(kind=kind,speed=speed):
                    calls=self.execute(row)
                    goals=[c for c in calls if c[0]=='goal']
                    self.assertTrue(all(c[2]==speed for c in goals))
                    self.assertEqual(len(goals),len(steps_for_trial(row,.04)))
                    self.assertEqual(goals[-1][1],.140)

    def test_runner_keeps_legacy_preparation_and_release(self):
        calls=self.execute(dict(kind='run',depth_mm=2.,speed=.1,hold_s=4.,release_speed=.2))
        self.assertEqual([c[2] for c in calls if c[0]=='goal'],[.3,.1,.2])

    def test_invalid_width_blocks_even_preparation(self):
        n=NS(_effective_object_width_m=.003)
        row=dict(kind='run',depth_mm=2.,speed=.1,hold_s=4.,release_speed=.1)
        with self.assertRaises(ValueError):ObjectAutomationMixin._auto_execute_trial(n,row,{})


if __name__=='__main__':unittest.main()
