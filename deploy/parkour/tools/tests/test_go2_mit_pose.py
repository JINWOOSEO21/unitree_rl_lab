import copy
import gc
import weakref
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import go2_sensor_bridge as bridge
from go2_mit_pose import MitPose
from go2_sensor_bridge import low_row
from em_sidecar.mit_odometry import rotation_quaternion
from em_sidecar.kinematics import quat_to_mat
from test_go2_leg_pose import low


def sample():
    row = low()
    row['imu_state']['accelerometer'] = [0., 0., 9.81]
    return row


class MitTest(unittest.TestCase):
    def test_stationary_bias_and_initial_base(self):
        a = MitPose(calibration_seconds=.1, rate_hz=100.)
        row = sample()
        row['imu_state']['gyroscope'] = [.006,-.005,.003]
        initial = rotation_quaternion(np.array([.15,-.08,.6]))
        row['imu_state']['quaternion'] = initial.tolist()
        row['imu_state']['accelerometer'] = (quat_to_mat(initial).T @ [0.,0.,9.81]).tolist()
        for t in range(0,1500,2):
            result = a.update(row,t)
            if result is not None:
                final = result
        self.assertTrue(final['pose_valid'])
        np.testing.assert_allclose(a.gyro_bias,[.006,-.005,.003],atol=1e-12)
        np.testing.assert_allclose(list(final['position'].values()),0,atol=1e-10)
        np.testing.assert_allclose(list(final['orientation'].values()),[1,0,0,0],atol=1e-10)
        np.testing.assert_allclose(list(final['mapping_pose']['orientation'].values()),initial,atol=1e-10)
        self.assertGreaterEqual(np.linalg.eigvalsh(a.estimator.P).min(),-1e-12)

    def test_analytic_foot_velocity_matches_independent_fk_difference(self):
        a = MitPose(calibration_seconds=0)
        rng = np.random.default_rng(4)
        q = rng.normal(0,.4,12)
        dq = rng.normal(0,.3,12)
        feet, velocity, angular = a.estimator.foot_kinematics(q,dq)
        kin = a.estimator.kin
        p_plus,_ = kin.link_poses_base(q+1e-6*dq)
        p_minus,_ = kin.link_poses_base(q-1e-6*dq)
        numerical = (p_plus[a.estimator.feet]-p_minus[a.estimator.feet])/2e-6
        np.testing.assert_allclose(velocity,numerical,atol=1e-9)

    def test_translation_and_unequal_contact_heights(self):
        e = MitPose(calibration_seconds=0).estimator
        e.cfg.foot_radius = 0.
        origins = np.array([[.2,.1,-.3],[.2,-.1,-.2],[-.2,.1,-.4],[-.2,-.1,-.3]])
        e.foot_kinematics = lambda q,dq: (origins-q[:3],np.tile(-dq[:3],(4,1)),np.zeros((4,3)))
        for n in range(401):
            t=n*.01
            elapsed=max(0.,t-.2)
            speed=.15*min(elapsed,1.)
            distance=.075*min(elapsed,1.)**2+.15*max(0.,elapsed-1.)
            acc=.15 if 0 < elapsed < 1. else 0.
            q=np.zeros(12);q[0]=distance
            dq=np.zeros(12);dq[0]=speed
            e.step(t,q,dq,np.array([1.,0,0,0]),np.zeros(3),np.array([acc,0,9.81]),np.ones(4)*100)
        self.assertLess(abs(e.x[0]-distance),.015)
        self.assertLess(abs(e.x[3]-.15),.015)
        self.assertLess(abs(e.x[2]),.005)
        np.testing.assert_allclose(e.x[8::3],origins[:,2],atol=.005)

    def test_short_flight_uses_acceleration_then_invalidates_and_resets_map(self):
        a=MitPose(calibration_seconds=0)
        row=sample()
        for t in range(0,200,10):a.update(row,t)
        row['foot_force']=[0]*4
        row['imu_state']['accelerometer']=[0,0,0]  # specific force in free fall
        for t in range(200,250,10):result=a.update(row,t)
        self.assertTrue(result['pose_valid'])
        self.assertAlmostEqual(result['velocity']['z'],-9.81*.05,places=6)
        self.assertAlmostEqual(result['position']['z'],-.5*9.81*.05**2,places=6)
        resets=[]
        for t in range(250,500,10):
            result=a.update(row,t)
            if result['map_reset_required']:resets.append(t)
        self.assertFalse(result['pose_valid'])
        self.assertEqual(len(resets),1)

    def test_moving_pose_rebases_position_and_orientation_together(self):
        a=MitPose(calibration_seconds=0)
        initial=rotation_quaternion(np.array([.1,.2,1.]))
        position=np.array([1.,2.,3.])
        delta=rotation_quaternion(np.array([.2,0.,0.]))
        from em_sidecar.mit_odometry import multiply
        a.estimator.initial_quaternion=initial
        a.estimator.orientation=multiply(initial,delta)
        with patch.object(a.estimator,'step',return_value=position):
            result=a.update(sample(),0)
        np.testing.assert_allclose(list(result['position'].values()),quat_to_mat(initial).T @ position)
        np.testing.assert_allclose(list(result['orientation'].values()),delta,atol=1e-12)
        np.testing.assert_allclose(list(result['mapping_pose']['position'].values()),position)

    def test_yaw_integration_is_not_removed_by_gravity(self):
        a=MitPose(calibration_seconds=0)
        row=sample()
        for t in range(0,200,10): a.update(row,t)
        row['foot_force']=[0]*4
        row['imu_state']['gyroscope']=[0,0,.5]
        for t in range(200,1200,10): result=a.update(row,t)
        expected=rotation_quaternion(np.array([0,0,.5]))
        np.testing.assert_allclose(list(result['orientation'].values()),expected,atol=1e-9)
        self.assertFalse(result['pose_valid'])

    def test_gap_duplicate_regression_and_recovery(self):
        a=MitPose(calibration_seconds=0,rate_hz=100.)
        for t in range(0,200,10): old=a.update(sample(),t)
        self.assertIsNone(a.update(sample(),195))
        with self.assertRaises(RuntimeError):a.update(sample(),194)
        result=a.update(sample(),500)
        self.assertFalse(result['pose_valid'])
        self.assertAlmostEqual(result['pose_gap_s'],.31)
        np.testing.assert_allclose(list(result['position'].values()),list(old['position'].values()))
        self.assertIsNone(a.update(sample(),500))
        for t in range(510,610,10): result=a.update(sample(),t)
        self.assertTrue(result['pose_valid'])

    def test_acceleration_required_finite_and_not_corrected(self):
        for value in (None,[1,2],[0,float('nan'),9.81]):
            a=MitPose(calibration_seconds=0)
            row=sample()
            if value is None:del row['imu_state']['accelerometer']
            else:row['imu_state']['accelerometer']=value
            with self.assertRaises(ValueError):a.update(row,0)
            self.assertIsNone(a.last_tick)
        a=MitPose(calibration_seconds=0)
        row=sample(); row['imu_state']['accelerometer']=[.1,-.2,9.54]
        original=copy.deepcopy(row)
        result=a.update(row,0)
        self.assertEqual(row,original)
        self.assertFalse(result['acceleration_corrected'])

    def test_swing_leg_does_not_drag_base_and_landing_recovers(self):
        a=MitPose(calibration_seconds=0)
        row=sample()
        for t in range(0,200,10):a.update(row,t)
        row['foot_force'][0]=0
        for t in range(200,300,10):a.update(row,t)
        row['motor_state'][1]['q']+=.3
        row['motor_state'][1]['dq']=2.
        for t in range(300,500,10):result=a.update(row,t)
        self.assertEqual(result['reliable_feet'],3)
        self.assertLess(np.linalg.norm(list(result['position'].values())),.001)
        row['motor_state'][1]['dq']=0
        row['foot_force'][0]=100
        for t in range(500,800,10):result=a.update(row,t)
        self.assertEqual(result['reliable_feet'],4)
        self.assertLess(np.linalg.norm(list(result['position'].values())),.001)

class MitBridgeTest(unittest.TestCase):
    def test_mit_routing_acceleration_and_gravity_aligned_mapping(self):
        raw=sample()
        low_msg=NS(tick=100,imu_state=NS(**raw['imu_state']),
                   motor_state=[NS(**m) for m in raw['motor_state']],foot_force=raw['foot_force'])
        cloud=NS(header=NS(frame_id='utlidar_lidar',stamp=NS(sec=1,nanosec=0)))
        messages={'rt/lowstate':low_msg,'rt/utlidar/cloud':cloud}
        topics=[]
        class Sub:
            def __init__(self,topic,typ):self.topic=topic;topics.append(topic)
            def Init(self,callback,queue):callback(messages[self.topic])
            def Close(self):pass
        adapter=MitPose(calibration_seconds=0)
        for tick in range(0,100,10):pose=adapter.update(raw,tick)
        pose['mapping_pose']['position']['x']=.123
        events=[]
        with patch.object(bridge,'dependencies',return_value=(lambda *a:None,Sub,object,object,object)), \
             patch.object(bridge,'BaseMapper') as mapper, patch.object(bridge,'MitPose') as factory, \
             patch.object(bridge,'ScandotsOutput') as output, patch.object(bridge,'GyroBiasOutput'), \
             patch.object(bridge.time,'monotonic_ns',return_value=1_000_000_000):
            factory.return_value.update.return_value=pose
            mapper.return_value.update.return_value=(np.zeros(132),np.ones(132),np.zeros(132))
            code=bridge.run('test',0,.001,Path('.'),events.append,odom_source='mit',publish_scandots=True)
            self.assertEqual(code,0)
            factory.assert_called_once_with(contact_threshold=20.0,rate_hz=75.0)
            forwarded=factory.return_value.update.call_args.args[0]
            self.assertEqual(forwarded['imu_state']['accelerometer'],[0.,0.,9.81])
            self.assertIs(mapper.return_value.update.call_args.args[1],pose['mapping_pose'])
            np.testing.assert_allclose(output.return_value.publish.call_args.args[1],[.123,0,0])
        self.assertEqual(topics,['rt/lowstate','rt/utlidar/cloud'])
        self.assertTrue(any('mit_odometry' in e for e in events))
        self.assertTrue(any(e['kind']=='scan' and e['odometry']['source']=='mit' for e in events))
        self.assertNotIn('accelerometer',low_row(low_msg)['imu_state'])

class MitGcTest(unittest.TestCase):
    def test_new_cycles_are_collected_and_tracking_is_restored(self):
        from go2_mit_dds import initialized_heap_gc_scope
        class Cycle:
            pass
        before=gc.get_freeze_count()
        enabled=gc.isenabled()
        with initialized_heap_gc_scope():
            self.assertGreater(gc.get_freeze_count(),0)
            obj=Cycle(); obj.self=obj
            ref=weakref.ref(obj)
            del obj
            gc.collect()
            self.assertIsNone(ref())
            self.assertEqual(gc.isenabled(),enabled)
        self.assertEqual(gc.get_freeze_count(),before)

    def test_existing_caller_freeze_is_not_unfrozen(self):
        from go2_mit_dds import initialized_heap_gc_scope
        with patch('go2_mit_dds.gc.get_freeze_count',return_value=10), \
             patch('go2_mit_dds.gc.freeze') as freeze, patch('go2_mit_dds.gc.unfreeze') as unfreeze:
            with initialized_heap_gc_scope():pass
            freeze.assert_not_called()
            unfreeze.assert_not_called()

if __name__=='__main__':unittest.main()
