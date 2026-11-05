"""Synthetic sample sequences plus the observed real spike; no ROS/hardware."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from data_capture_tools.object_automation import ObjectAutomationMixin
from data_capture_tools.object_plan import build_plan,validate_saved_plan

class ConfirmationTests(unittest.TestCase):
    def node(self,confirm=5.):
        n=NS(_auto_active=True,_auto_armed=True,_auto_force_fault=None,
             _force_monitor=NS(_offset_left=.3911,_offset_right=2.5658),
             _auto_job={'plan':{'settings':{'hard_limit_n':30.,'force_limit_confirm_ms':confirm}}})
        return n
    def sample(self,n,ms,value,side='right'):
        baseline=getattr(n._force_monitor,'_offset_'+side)
        ObjectAutomationMixin._auto_note_force(n,side,[0.,baseline+value,0.,0.,0.,0.],1_000_000_000+int(ms*1e6))
    def test_real_observed_spike_does_not_latch_at_old_20N_limit(self):
        n=self.node();n._auto_job['plan']['settings']['hard_limit_n']=20.
        for ms,value in [(-.965725,-1.2738),(0.,26.6992),(1.036102,-1.2038)]:self.sample(n,ms,value)
        self.assertIsNone(n._auto_force_fault)
    def test_continuous_negative_force_triggers_at_five_ms(self):
        n=self.node()
        for ms in range(5):self.sample(n,ms,-31.)
        self.assertIsNone(n._auto_force_fault)
        self.sample(n,5,-31.)
        self.assertEqual(n._auto_force_fault,'Force hard limit right')
        self.assertEqual(n._auto_force_fault_record['confirmation']['samples'],6)
    def test_interrupted_and_independent_side_sequences_do_not_accumulate(self):
        n=self.node()
        for ms in range(15):
            self.sample(n,ms,31. if ms%2 else 0.,'right')
            self.sample(n,ms,0. if ms%2 else -31.,'left')
        self.assertIsNone(n._auto_force_fault)
    def test_duplicate_backwards_or_large_gap_resets_evidence(self):
        for sequence in [[0,1,1,2,3,4,5],[0,1,.5,1,2,3,4],[0,1,25,26,27,28,29]]:
            n=self.node()
            for ms in sequence:self.sample(n,ms,31.)
            self.assertIsNone(n._auto_force_fault)
    def test_batched_callbacks_use_producer_times(self):
        n=self.node()
        for ms in range(6):self.sample(n,ms,31.)
        self.assertIsNotNone(n._auto_force_fault)
    def test_nonfinite_is_immediate_and_first_side_is_preserved(self):
        n=self.node();self.sample(n,0,float('nan'),'left');self.sample(n,1,40.,'right')
        self.assertEqual(n._auto_force_fault,'Nonfinite force left')
        self.assertEqual(n._auto_force_fault_record['side'],'left')
    def test_legacy_absent_setting_remains_immediate(self):
        n=self.node();del n._auto_job['plan']['settings']['force_limit_confirm_ms'];self.sample(n,0,31.)
        self.assertIsNotNone(n._auto_force_fault)
    def test_new_and_old_plan_fingerprints_survive(self):
        base=dict(depths_mm=[4.],loading_speeds=[0.],hard_limit_n=30.)
        old=build_plan(base);self.assertNotIn('force_limit_confirm_ms',old['settings']);validate_saved_plan(old)
        new=build_plan(dict(base,force_limit_confirm_ms=5.));validate_saved_plan(new)
        for value in [-1.,21.,float('nan'),True]:
            with self.assertRaises(ValueError):build_plan(dict(base,force_limit_confirm_ms=value))
        with self.assertRaises(ValueError):build_plan(dict(base,hard_limit_n=30.01))

if __name__=='__main__':unittest.main()
