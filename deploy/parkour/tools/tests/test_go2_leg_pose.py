import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from go2_leg_pose import LegPose, ROOT


def low():
    contract=yaml.safe_load((ROOT/'contract/deploy.yaml').read_text())
    return {'imu_state':{'quaternion':[1,0,0,0],'gyroscope':[0,0,0]},
            'motor_state':[{'q':q,'dq':0} for q in contract['default_joint_pos']['sdk']],
            'foot_force':[100,100,100,100]}


class LegPoseTest(unittest.TestCase):
    def test_stationary_initial_origin_and_contact_settling(self):
        adapter=LegPose()
        first=adapter.update(low(),100)
        self.assertFalse(first['pose_valid'])
        for tick in range(102,150,2):
            result=adapter.update(low(),tick)
        self.assertTrue(result['pose_valid'])
        self.assertEqual(result['reliable_feet'],4)
        np.testing.assert_allclose(list(result['position'].values()),0,atol=1e-12)
        self.assertEqual(result['source'],'leg')

    def test_sdk_to_il_mapping_and_normalization(self):
        adapter=LegPose(); row=low()
        for i,motor in enumerate(row['motor_state']): motor['q']=i
        row['foot_force']=[1,2,3,4]
        row['imu_state']['quaternion']=[1.01,0,0,0]
        with patch.object(adapter.estimator,'step',return_value=np.zeros(3)) as step:
            adapter.update(row,100)
        args=step.call_args.args
        np.testing.assert_array_equal(args[1],[3,0,9,6,4,1,10,7,5,2,11,8])
        np.testing.assert_array_equal(args[2],[1,0,0,0])
        np.testing.assert_array_equal(args[4],[2,1,4,3])

    def test_duplicate_does_not_step_and_clock_faults_require_restart(self):
        for next_tick in (99,201):
            adapter=LegPose(); adapter.update(low(),100)
            with patch.object(adapter.estimator,'step') as step:
                self.assertIsNone(adapter.update(low(),100))
                step.assert_not_called()
            with self.assertRaises(RuntimeError): adapter.update(low(),next_tick)

    def test_bad_lowstate_cannot_mutate_estimator(self):
        adapter=LegPose(); row=low(); row['motor_state'][0]['q']=float('nan')
        with self.assertRaises(ValueError): adapter.update(row,100)
        self.assertIsNone(adapter.last_tick)

    def test_no_support_is_invalid_after_hold_window(self):
        adapter=LegPose(); row=low()
        for tick in range(100,160,2): adapter.update(row,tick)
        row=copy.deepcopy(row); row['foot_force']=[0]*4
        for tick in range(160,500,2): result=adapter.update(row,tick)
        self.assertFalse(result['pose_valid'])
        self.assertEqual(result['branch'],4)


if __name__=='__main__': unittest.main()
