import base64
import struct
import unittest
import numpy as np
from lio.protocol import validate, InputClock
from lio.pose import base_pose, matrix_quat
from em_sidecar.kinematics import quat_to_mat
from em_sidecar.go2_cloud import RAW_TO_BASE_ROTATION, RAW_TO_BASE_TRANSLATION


def cloud():
    b=bytearray(32);struct.pack_into('<f',b,24,.0625)
    return dict(kind='cloud',stamp_ns=10**18,receipt_ns=10,frame_id='utlidar_lidar',width=1,height=1,
                point_step=32,row_step=32,is_bigendian=False,is_dense=True,
                fields=[dict(name=n,offset=o,datatype=d,count=1) for n,o,d in
                        [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('ring',20,4),('time',24,7)]],
                data_b64=base64.b64encode(b).decode())

class ProtocolTest(unittest.TestCase):
    def test_layout_and_time(self):
        e=cloud();self.assertEqual(validate(e),.0625);e['stamp_ns']=float(e['stamp_ns'])
        with self.assertRaises(ValueError):validate(e)
    def test_corrupt_fields_and_data_rejected(self):
        for change in ('time','length','nan','frame'):
            e=cloud()
            if change=='time':e['fields'][-1]['offset']=31
            if change=='length':e['row_step']=64
            if change=='nan':
                b=bytearray(base64.b64decode(e['data_b64']));struct.pack_into('<f',b,24,float('nan'));e['data_b64']=base64.b64encode(b).decode()
            if change=='frame':e['frame_id']='base_link'
            with self.assertRaises(ValueError):validate(e)
    def test_duplicate_regression_and_gap(self):
        c=InputClock();e=cloud();self.assertTrue(c.accept(e));self.assertFalse(c.accept(e));e['stamp_ns']-=1
        with self.assertRaises(ValueError):c.accept(e)
        e['stamp_ns']+=200_000_001;self.assertTrue(c.accept(e));self.assertEqual(len(c.gaps),1)
    def test_big_endian_and_row_padding(self):
        e=cloud();e.update(width=1,height=2,row_step=40,is_bigendian=True)
        b=bytearray(80);struct.pack_into('>f',b,24,.01);struct.pack_into('>f',b,64,.02)
        e['data_b64']=base64.b64encode(b).decode();self.assertAlmostEqual(validate(e),.02)

class PoseTest(unittest.TestCase):
    def test_quaternion_roundtrip(self):
        for q in ([1,0,0,0],[0,1,0,0],[.5,.5,.5,.5]):
            r=quat_to_mat(np.array(q));np.testing.assert_allclose(quat_to_mat(matrix_quat(r)),r,atol=1e-12)
    def test_transform_recovers_known_base(self):
        rwb=quat_to_mat(np.array([np.cos(.4),0,0,np.sin(.4)]));pwb=np.array([1,2,3.])
        rbi=RAW_TO_BASE_ROTATION;tbi=RAW_TO_BASE_TRANSLATION+rbi@np.array([-.007698,-.014655,.00667])
        event=dict(kind='lio_pose',stamp_ns=123,frame_id='camera_init',child_frame_id='aft_mapped',
                   position=(pwb+rwb@tbi).tolist(),orientation=matrix_quat(rwb@rbi).tolist())
        result=base_pose(event)
        np.testing.assert_allclose(list(result['position'].values()),pwb,atol=1e-12)
        np.testing.assert_allclose(quat_to_mat(np.array(list(result['orientation'].values()))),rwb,atol=1e-12)
    def test_wrong_frame_rejected(self):
        with self.assertRaises(ValueError):base_pose({'kind':'lio_pose','frame_id':'odom'})


from lio.sync import PoseBuffer
class BufferTest(unittest.TestCase):
    def row(self,t,x=0,g='a'):
        return dict(source_stamp_ns=t,generation=g,pose_valid=True,frame_id='odom',child_frame_id='base_link',position=dict(x=x,y=0,z=0),orientation=dict(w=1,x=0,y=0,z=0))
    def test_delayed_sensor_pairing(self):
        b=PoseBuffer();b.add(self.row(100_000_000),123);b.add(self.row(120_000_000,2),1000000000)
        self.assertAlmostEqual(b.at(110_000_000)['position']['x'],1)
        self.assertIsNone(b.at(121_000_000));self.assertIsNone(b.at(99_000_000))
    def test_reset_and_loss(self):
        b=PoseBuffer();b.add(self.row(100),1);b.add(self.row(200),2)
        self.assertTrue(b.add(self.row(300,g='b'),3));self.assertIsNone(b.at(150))
        b.add(dict(generation='b',pose_valid=False),4);self.assertIsNone(b.at(300))
    def test_clock_regression_and_long_gap(self):
        b=PoseBuffer();b.add(self.row(100),1);b.add(self.row(100_000_100),2)
        self.assertIsNone(b.at(50_000_100))
        with self.assertRaises(ValueError):b.add(self.row(1),3)

from lio.pose import gravity_disagreement
class GravityTest(unittest.TestCase):
    def test_yaw_independent_but_inversion_rejected(self):
        row=dict(orientation=dict(w=0,x=0,y=0,z=1))
        self.assertAlmostEqual(gravity_disagreement(row,[1,0,0,0]),0)
        row['orientation']=dict(w=0,x=1,y=0,z=0)
        self.assertAlmostEqual(gravity_disagreement(row,[1,0,0,0]),np.pi)

from types import SimpleNamespace as NS
from lio.sync import deskew_to_end
class DeskewTest(unittest.TestCase):
    def test_translation_to_scan_end_and_missing_coverage(self):
        b=PoseBuffer()
        for t,x in [(0,0.),(20_000_000,.02),(40_000_000,.04)]:
            b.add(dict(source_stamp_ns=t,generation='g',pose_valid=True,frame_id='odom',child_frame_id='base_link',
                       position=dict(x=x,y=0.,z=0.),orientation=dict(w=1.,x=0.,y=0.,z=0.)),t)
        raw=bytearray(64);struct.pack_into('<f',raw,56,.02)
        c=NS(header=NS(stamp=NS(sec=0,nanosec=0)),width=2,height=1,row_step=64,point_step=32,is_bigendian=False,data=raw,
             fields=[NS(name=n,offset=o,datatype=7,count=1) for n,o in [('x',0),('y',4),('z',8),('time',24)]])
        pts,row=deskew_to_end(c,b)
        np.testing.assert_allclose(pts[0],RAW_TO_BASE_TRANSLATION+[-.02,0,0],atol=1e-8)
        np.testing.assert_allclose(pts[1],RAW_TO_BASE_TRANSLATION,atol=1e-8)
        c.header.stamp.sec=1;self.assertIsNone(deskew_to_end(c,b))
    def test_map_cannot_publish_production_topic(self):
        from lio.map_shadow import run
        with self.assertRaises(ValueError):run('lo',0,1,None,None,True,'rt/parkour/scandots')

if __name__=='__main__':unittest.main()
