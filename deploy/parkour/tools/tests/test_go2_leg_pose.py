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
        adapter=LegPose(calibration_seconds=0)
        first=adapter.update(low(),100)
        self.assertFalse(first['pose_valid'])
        for tick in range(102,150,2):
            result=adapter.update(low(),tick)
        self.assertTrue(result['pose_valid'])
        self.assertEqual(result['reliable_feet'],4)
        np.testing.assert_allclose(list(result['position'].values()),0,atol=1e-12)
        self.assertEqual(result['source'],'leg')

    def test_sdk_to_il_mapping_and_normalization(self):
        adapter=LegPose(calibration_seconds=0); row=low()
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
            adapter=LegPose(calibration_seconds=0); adapter.update(low(),100)
            with patch.object(adapter.estimator,'step') as step:
                self.assertIsNone(adapter.update(low(),100))
                step.assert_not_called()
            with self.assertRaises(RuntimeError): adapter.update(low(),next_tick)

    def test_bad_lowstate_cannot_mutate_estimator(self):
        adapter=LegPose(calibration_seconds=0); row=low(); row['motor_state'][0]['q']=float('nan')
        with self.assertRaises(ValueError): adapter.update(row,100)
        self.assertIsNone(adapter.last_tick)

    def test_no_support_is_invalid_after_hold_window(self):
        adapter=LegPose(calibration_seconds=0); row=low()
        for tick in range(100,160,2): adapter.update(row,tick)
        row=copy.deepcopy(row); row['foot_force']=[0]*4
        for tick in range(160,500,2): result=adapter.update(row,tick)
        self.assertFalse(result['pose_valid'])
        self.assertEqual(result['branch'],4)




class GyroCalibrationTest(unittest.TestCase):
    def test_bias_removed_and_rotation_preserved(self):
        adapter = LegPose(calibration_seconds=.1)
        row = low()
        bias = np.array([-.008, -.007, .006])
        row['imu_state']['gyroscope'] = bias.tolist()
        for tick in range(0, 100, 2):
            self.assertFalse(adapter.update(row, tick)['pose_valid'])
        for tick in range(100, 200, 2): result = adapter.update(row, tick)
        self.assertTrue(result['pose_valid'])
        np.testing.assert_allclose(adapter.gyro_bias, bias, atol=1e-12)
        np.testing.assert_allclose(list(result['position'].values()), 0, atol=1e-12)
        row['imu_state']['gyroscope'] = (bias+[0, 0, .5]).tolist()
        with patch.object(adapter.estimator, 'step', return_value=np.zeros(3)) as step:
            adapter.update(row, 200)
        np.testing.assert_allclose(step.call_args.args[3], [0, 0, .5], atol=1e-12)
        np.testing.assert_allclose(adapter.gyro_bias, bias, atol=1e-12)

    def test_support_loss_and_joint_motion_restart(self):
        for change in ('contact', 'joint'):
            adapter = LegPose(calibration_seconds=.1)
            row = low()
            for tick in range(0, 80, 2): adapter.update(row, tick)
            if change == 'contact': row['foot_force'][0] = 0
            else: row['motor_state'][0]['q'] += .02
            adapter.update(row, 80)
            row = low()
            for tick in range(82, 180, 2):
                self.assertFalse(adapter.update(row, tick)['gyro_calibrated'])
            for tick in range(180, 220, 2): result = adapter.update(row, tick)
            self.assertTrue(result['gyro_calibrated'])

    def test_sustained_turn_not_learned(self):
        adapter = LegPose(calibration_seconds=1)
        row = low()
        row['imu_state']['gyroscope'] = [0, 0, .02]
        for tick in range(0, 3000, 20):
            yaw = tick*.001*.02
            row['imu_state']['quaternion'] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
            self.assertFalse(adapter.update(row, tick)['gyro_calibrated'])

    def test_quaternion_sign_invariance(self):
        adapter = LegPose(calibration_seconds=.1)
        row = low()
        for tick in range(0, 150, 2):
            row['imu_state']['quaternion'][0] = (-1)**(tick//2)
            result = adapter.update(row, tick)
        self.assertTrue(result['pose_valid'])

    def test_vibration_rejected(self):
        adapter = LegPose(calibration_seconds=.1)
        row = low()
        for tick in range(0, 300, 2):
            row['imu_state']['gyroscope'] = [.05*(-1)**(tick//2), 0, 0]
            self.assertFalse(adapter.update(row, tick)['gyro_calibrated'])


if __name__=='__main__': unittest.main()
