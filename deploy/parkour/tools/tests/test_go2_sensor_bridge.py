import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from go2_sensor_bridge import preceding_pose, stamp_id, low_row
import go2_sensor_bridge as bridge


class SensorBridgeTest(unittest.TestCase):
    def test_publish_rechecks_inputs_after_gpu_work(self):
        for fault in ('delay','invalid_low','invalid_scan'):
            with self.subTest(fault=fault):
                clock=[1_000_000_000]
                callbacks={}
                low=NS(tick=1,imu_state=NS(quaternion=[1,0,0,0],gyroscope=[0,0,0]),
                       motor_state=[NS(q=0.,dq=0.) for _ in range(20)],foot_force=[100]*4)
                cloud=NS(header=NS(frame_id='utlidar_lidar',stamp=NS(sec=1,nanosec=0)))
                odom=NS(header=NS(frame_id='odom',stamp=NS(sec=1,nanosec=0)),child_frame_id='base_link',
                        pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(w=1.,x=0.,y=0.,z=0.))))
                samples={'rt/lowstate':low,'rt/utlidar/cloud':cloud,'rt/utlidar/robot_odom':odom}
                class Sub:
                    def __init__(self,topic,typ):self.topic=topic
                    def Init(self,callback,queue):
                        callbacks[self.topic]=callback
                        callback(samples[self.topic])
                    def Close(self):pass
                def compute(*args):
                    if fault=='delay':clock[0]+=210_000_000
                    elif fault=='invalid_scan':raise ValueError('invalid policy scan')
                    else:
                        low.motor_state[0].q=float('nan')
                        callbacks['rt/lowstate'](low)
                    return np.zeros(132),np.ones(132),np.zeros(132)
                deps=(lambda *args:None,Sub,object,object,object)
                events=[]
                with patch.object(bridge,'dependencies',return_value=deps), patch.object(bridge,'BaseMapper') as mapper, patch.object(bridge,'ScandotsOutput') as output, patch.object(bridge.time,'monotonic_ns',side_effect=lambda:clock[0]):
                    mapper.return_value.update.side_effect=compute
                    bridge.run('test',0,.001,Path('.'),events.append,odom_source='robot',publish_scandots=True)
                    mapper.return_value.update.assert_called_once()
                    mapper.return_value.discard_map.assert_called_once()
                    output.return_value.publish.assert_not_called()
                self.assertFalse(any(e['kind']=='scan' for e in events))
                expected={'delay':'cloud_too_old_after_mapping',
                          'invalid_low':'inputs invalidated during map computation',
                          'invalid_scan':'invalid policy scan'}[fault]
                self.assertTrue(any(e.get('reason')==expected for e in events),events)

    def test_lowstate_gap_rebuilds_the_map_without_killing_the_bridge(self):
        """The map accumulated before a gap is offset by travel the pose never integrated."""
        callbacks={}
        def low_at(tick):
            return NS(tick=tick,imu_state=NS(quaternion=[1,0,0,0],gyroscope=[0,0,0]),
                      motor_state=[NS(q=0.,dq=0.) for _ in range(20)],foot_force=[100]*4)
        class Sub:
            def __init__(self,topic,typ):self.topic=topic
            def Init(self,callback,queue):
                callbacks[self.topic]=callback
                callback(low_at(100))
                callback(low_at(380))  # 280 ms hole: the startup spike seen on the Jetson
            def Close(self):pass
        deps=(lambda *args:None,Sub,object,object,object)
        events=[]
        with patch.object(bridge,'dependencies',return_value=deps), \
             patch.object(bridge,'BaseMapper') as mapper, \
             patch.object(bridge,'ScandotsOutput') as output, patch.object(bridge,'GyroBiasOutput'):
            code=bridge.run('test',0,.3,Path('.'),events.append,publish_scandots=True)
        self.assertEqual(code,0)  # degraded, not fatal
        mapper.return_value.discard_map.assert_called_once()
        output.return_value.invalidate.assert_called()
        self.assertFalse(any(e['kind']=='fatal' for e in events),events)
        self.assertTrue(any(e['kind']=='fault' and 'lowstate gap 280 ms' in e['reason']
                            for e in events),events)

    def test_map_kernels_are_warmed_before_any_subscriber_exists(self):
        """A cold first update() holds the GIL long enough to open the gap above."""
        order=[]
        class Sub:
            def __init__(self,topic,typ):order.append('subscribe')
            def Init(self,callback,queue):pass
            def Close(self):pass
        deps=(lambda *args:order.append('participant'),Sub,object,object,object)
        with patch.object(bridge,'dependencies',return_value=deps), \
             patch.object(bridge,'BaseMapper') as mapper:
            mapper.return_value.warmup.side_effect=lambda:order.append('warmup')
            bridge.run('test',0,.001,Path('.'),lambda e:None)
        self.assertEqual(order[0],'warmup',order)

    def test_warmup_leaves_no_synthetic_terrain_or_pose_behind(self):
        mapper=object.__new__(bridge.BaseMapper)
        mapper.backend=Mock()
        mapper.torch=Mock()
        mapper.last_position=None
        with patch.object(bridge.BaseMapper,'update') as update:
            mapper.warmup()
        update.assert_called_once()
        self.assertIsNone(update.call_args.args[0])  # synthetic points, never a real cloud
        mapper.backend.clear.assert_called_once_with([0])
        self.assertIsNone(mapper.last_position)

    def test_warmup_failure_is_reported_but_does_not_stop_startup(self):
        mapper=object.__new__(bridge.BaseMapper)
        mapper.backend=Mock()
        mapper.torch=Mock()
        mapper.last_position=None
        with patch.object(bridge.BaseMapper,'update',side_effect=ValueError('policy scan')):
            mapper.warmup()  # must not raise: warmup is an optimisation, not a precondition
        mapper.backend.clear.assert_called_once_with([0])

    def test_discard_clears_backend_without_losing_pose_continuity(self):
        mapper = object.__new__(bridge.BaseMapper)
        mapper.backend = Mock()
        position = np.array([1.,2.,3.])
        mapper.last_position = position
        mapper.discard_map()
        mapper.backend.clear.assert_called_once_with([0])
        self.assertIs(mapper.last_position, position)

    def test_raw_cloud_without_odometry_cannot_produce_scan(self):
        events = []
        cloud = NS(header=NS(frame_id='utlidar_lidar', stamp=NS(sec=1, nanosec=0)))
        class Sub:
            def __init__(self, topic, typ): self.topic = topic
            def Init(self, callback, queue):
                if self.topic == 'rt/utlidar/cloud': callback(cloud)
            def Close(self): pass
        deps = (lambda *args: None, Sub, object, object, object)
        with patch.object(bridge, 'dependencies', return_value=deps), patch.object(bridge, 'BaseMapper') as mapper:
            self.assertEqual(bridge.run('test', 0, .001, Path('.'), events.append, odom_source='robot'), 0)
            mapper.return_value.update.assert_not_called()
        self.assertIn({'kind':'fault', 'reason':'missing_preceding_odom'}, events)
        self.assertFalse(any(e['kind'] == 'scan' for e in events))

    def test_mapper_passes_transformed_base_points_to_backend_adapter(self):
        mapper = object.__new__(bridge.BaseMapper)
        mapper.last_position = None
        row = {'frame_id':'odom', 'child_frame_id':'base_link',
               'position':dict(x=0,y=0,z=.3), 'orientation':dict(w=1,x=0,y=0,z=0)}
        cloud = NS(header=NS(frame_id='utlidar_lidar'), width=1, height=1,
                   point_step=12, row_step=12, is_bigendian=False,
                   data=np.array([[1,0,0]], dtype='<f4').tobytes(),
                   fields=[NS(name=n,offset=i*4,datatype=7,count=1)
                           for i,n in enumerate(('x','y','z'))])
        # Stop at the GPU boundary; verify real decoding and transformation.
        with patch.object(bridge, 'backend_input_from_base_cloud', side_effect=RuntimeError('GPU boundary')) as adapt:
            with self.assertRaisesRegex(RuntimeError, 'GPU boundary'):
                mapper.update(cloud, row)
        points, position, rotation = adapt.call_args.args
        np.testing.assert_allclose(points, [[.808273915, -.838668173, .140854030]], atol=5e-8)
        np.testing.assert_allclose(position, [0,0,.3])
        np.testing.assert_array_equal(rotation, np.eye(3))

    def test_live_entry_creates_only_three_subscriptions(self):
        created=[]
        closed=[]
        initialized=[]
        class Sub:
            def __init__(self,topic,typ):
                self.topic=topic
                created.append(topic)
            def Init(self,callback,queue):
                self.callback=callback
                self.queue=queue
            def Close(self): closed.append(self.topic)
        deps=(lambda domain,interface:initialized.append((domain,interface)),Sub,object,object,object)
        with patch.object(bridge,'dependencies',return_value=deps), patch.object(bridge,'BaseMapper'):
            self.assertEqual(bridge.run('test-interface',42,.001,Path('.'),lambda e:None, odom_source='robot'),0)
        self.assertEqual(initialized,[(42,'test-interface')])
        self.assertEqual(created,['rt/lowstate','rt/utlidar/cloud','rt/utlidar/robot_odom'])
        self.assertEqual(closed,created)

    def test_leg_mode_subscribes_only_lowstate_and_raw_cloud(self):
        topics=[]
        class Sub:
            def __init__(self,topic,typ): topics.append(topic)
            def Init(self,callback,queue): pass
            def Close(self): pass
        deps=(lambda *args:None,Sub,object,object,object)
        with patch.object(bridge,'dependencies',return_value=deps), patch.object(bridge,'BaseMapper'), patch.object(bridge,'ScandotsOutput') as output:
            bridge.run('test',0,.001,Path('.'),lambda e:None)
            output.assert_not_called()
        self.assertEqual(topics,['rt/lowstate','rt/utlidar/cloud'])

    def test_terrain_publisher_is_explicit_opt_in_and_closed(self):
        class Sub:
            def __init__(self,*args):pass
            def Init(self,*args):pass
            def Close(self):pass
        deps=(lambda *args:None,Sub,object,object,object)
        with patch.object(bridge,'dependencies',return_value=deps), patch.object(bridge,'BaseMapper'), patch.object(bridge,'ScandotsOutput') as output, patch.object(bridge,'GyroBiasOutput') as bias_output:
            bridge.run('test',0,.001,Path('.'),lambda e:None,publish_scandots=True)
            output.assert_called_once_with('rt/parkour/scandots')
            output.return_value.publish.assert_not_called()
            output.return_value.close.assert_called_once()
            bias_output.assert_called_once_with()
            bias_output.return_value.close.assert_called_once_with(0)

    def test_invalid_leg_support_does_not_fall_back_to_older_pose(self):
        row = {'frame_id':'odom','child_frame_id':'base_link',
               'position':dict(x=0,y=0,z=0),'orientation':dict(w=1,x=0,y=0,z=0)}
        with self.assertRaisesRegex(ValueError,'no_reliable_support'):
            preceding_pose([(10,row),(15,dict(row,pose_valid=False))],20)

    def test_pose_pairing_is_causal_and_age_bounded(self):
        row = {'frame_id':'odom','child_frame_id':'base_link',
               'position':dict(x=0,y=0,z=.3),'orientation':dict(w=1,x=0,y=0,z=0)}
        self.assertIs(preceding_pose([(10,row),(30,row)],20,10),row)
        with self.assertRaisesRegex(ValueError,'missing'):
            preceding_pose([(30,row)],20)
        with self.assertRaisesRegex(ValueError,'old'):
            preceding_pose([(10,row)],21,10)
        bad = dict(row,child_frame_id='imu')
        with self.assertRaises(ValueError):
            preceding_pose([(10,bad)],20)

    def test_sensor_stamp_preserves_nanosecond_identity(self):
        msg=NS(header=NS(stamp=NS(sec=4,nanosec=123)))
        self.assertEqual(stamp_id(msg),4_000_000_123)
        msg.header.stamp.nanosec=1_000_000_000
        with self.assertRaises(ValueError): stamp_id(msg)

    def test_low_conversion_keeps_sdk_order(self):
        row=low_row(NS(imu_state=NS(quaternion=[1,0,0,0],gyroscope=[1,2,3]),
                       motor_state=[NS(q=i,dq=-i) for i in range(20)],foot_force=[4,3,2,1]))
        self.assertEqual(row['motor_state'][7],{'q':7,'dq':-7})
        self.assertEqual(row['foot_force'],[4,3,2,1])

    def test_shadow_constructs_no_writers_and_no_motor_commands_anywhere(self):
        forbidden={'ChannelPublisher','DataWriter','LowCmd_','SportClient','MotionSwitcherClient'}
        for name in ['go2_sensor_bridge.py','shadow_go2_policy.py']:
            tree=ast.parse((TOOLS/name).read_text())
            symbols={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
            symbols.update(n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute))
            self.assertFalse(symbols & forbidden)
        tree=ast.parse((TOOLS/'go2_scandots_output.py').read_text())
        symbols={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
        self.assertFalse(symbols & {'LowCmd_','SportClient','MotionSwitcherClient'})


if __name__=='__main__': unittest.main()
