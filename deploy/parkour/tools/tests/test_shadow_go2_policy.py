from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import yaml

TOOLS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TOOLS))
from shadow_go2_policy import ShadowPolicy


class FakeSession:
    def __init__(self):
        self.inputs=[]
        self.action=np.ones((1,12),dtype=np.float32)
        self.callback=None
    def run(self, _, feed):
        self.inputs.append({k:v.copy() for k,v in feed.items()})
        if self.callback: self.callback()
        return [self.action.copy()]


class ShadowIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.contract=yaml.safe_load((TOOLS.parent/'contract/deploy.yaml').read_text())
        runtime=yaml.safe_load((TOOLS.parent.parent/'robots/go2/config/config.yaml').read_text())['FSM']['Parkour']
        self.session=FakeSession()
        self.core=ShadowPolicy(self.contract,runtime,self.session)
        self.low={'imu_state':{'quaternion':[1,0,0,0],'gyroscope':[0,0,0]},
                  'motor_state':[{'q':0.,'dq':0.} for _ in range(20)],'foot_force':[10]*4}
    def offer(self,kind,now,id=None,value=None,source=None):
        e={'kind':kind,'receipt_ns':now,'source_ns':now if source is None else source,
           'source_id':now if id is None else id}
        e['low' if kind=='low' else 'scan']=deepcopy(self.low) if kind=='low' and value is None else (
            np.zeros(132) if value is None else value)
        self.core.offer(e)
    def pair(self,now):
        self.offer('low',now); self.offer('scan',now)
    def test_missing_inputs_do_not_invoke_onnx(self):
        self.assertFalse(self.core.step(1)['allowed'])
        self.assertEqual(len(self.session.inputs),0)
    def test_bad_scan_blocks_and_recovery_reinitializes_last_action(self):
        self.pair(1_000_000)
        self.assertTrue(self.core.step(1_000_000)['allowed'])
        bad=np.zeros(132); bad[3]=np.nan
        self.offer('scan',2_000_000,value=bad)
        self.assertFalse(self.core.step(2_000_000)['allowed'])
        self.assertEqual(len(self.session.inputs),1)
        self.pair(3_000_000)
        self.assertTrue(self.core.step(3_000_000)['allowed'])
        self.assertTrue(np.all(self.session.inputs[-1]['prop'][0,37:49]==0))
        self.assertEqual(self.core.reset_count,2)
    def test_sensor_dropout_blocks_before_inference(self):
        self.pair(1_000_000)
        self.assertFalse(self.core.step(22_000_001)['allowed'])
        self.assertEqual(len(self.session.inputs),0)
    def test_old_source_with_new_receive_time_is_blocked(self):
        self.offer('scan',600_000_000,source=1_000_000)
        self.offer('low',600_000_000)
        self.assertFalse(self.core.step(600_000_000)['allowed'])
        self.assertEqual(len(self.session.inputs),0)
    def test_duplicate_source_cannot_be_used_as_fresh_map(self):
        self.pair(1_000_000)
        self.assertTrue(self.core.step(1_000_000)['allowed'])
        self.offer('scan',2_000_000,id=1_000_000,source=1_000_000)
        self.assertFalse(self.core.step(2_000_000)['allowed'])
        self.assertEqual(len(self.session.inputs),1)
    def test_clock_regression_is_fatal_until_runner_restart(self):
        self.pair(2_000_000)
        self.offer('low',3_000_000,id=1)
        self.pair(4_000_000)
        self.assertFalse(self.core.step(4_000_000)['allowed'])
        self.assertIsNotNone(self.core.fatal)
    def test_nonfinite_output_is_not_returned_as_target(self):
        self.pair(1_000_000)
        self.session.action[0,0]=np.inf
        result=self.core.step(1_000_000)
        self.assertFalse(result['allowed'])
        self.assertNotIn('q_target_sdk',result)
        self.assertIsNotNone(self.core.fatal)
    def test_fault_during_inference_discards_result(self):
        self.pair(1_000_000)
        self.session.callback=lambda:self.core.offer({'kind':'fault','reason':'malformed cloud'})
        result=self.core.step(1_000_000)
        self.assertFalse(result['allowed'])
        self.assertNotIn('q_target_sdk',result)
    def test_inputs_expiring_during_live_inference_discard_target(self):
        self.pair(1_000_000)
        with patch('shadow_go2_policy.time.monotonic_ns',side_effect=[1_000_000,22_000_001]):
            result=self.core.step(1_000_000,live=True)
        self.assertFalse(result['allowed'])
        self.assertNotIn('q_target_sdk',result)
    def test_sdk_targets_and_previous_raw_action(self):
        self.pair(1_000_000)
        result=self.core.step(1_000_000)
        self.pair(21_000_000)
        self.assertTrue(self.core.step(21_000_000)['allowed'])
        np.testing.assert_array_equal(self.session.inputs[1]['prop'][0,37:49],np.ones(12))
        mapping=self.contract['index_maps']['il_to_sdk']
        np.testing.assert_array_equal(np.asarray(result['q_target_sdk'])[mapping],result['q_target_il'])


if __name__=='__main__': unittest.main()
