"""Subscriber-only acquisition. IMU layout mirrors Unitree SDK2 Imu_.hpp."""
from dataclasses import dataclass
import queue
import time
import base64
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as ann
import cyclonedds.idl.types as ty
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from .protocol import stamp_ns

@dataclass
@ann.final
@ann.autoid('sequential')
class Imu_(idl.IdlStruct,typename='sensor_msgs.msg.dds_.Imu_'):
    header: Header_
    orientation: Quaternion_
    orientation_covariance: ty.array[ty.float64,9]
    angular_velocity: Vector3_
    angular_velocity_covariance: ty.array[ty.float64,9]
    linear_acceleration: Vector3_
    linear_acceleration_covariance: ty.array[ty.float64,9]


def xyz(v):return [v.x,v.y,v.z]


def events(interface, domain, duration):
    ChannelFactoryInitialize(domain,interface)
    pending=queue.Queue(maxsize=8192);errors=queue.Queue();subs=[];cloud_deadline=time.monotonic()+duration
    def callback(kind):
        def receive(m):
            try:
                if kind=='cloud' and time.monotonic()>=cloud_deadline:return
                e={'kind':kind,'receipt_ns':time.monotonic_ns()}
                if kind=='low':
                    e.update(tick=int(m.tick),low={'imu_state':{'quaternion':list(m.imu_state.quaternion),
                             'gyroscope':list(m.imu_state.gyroscope),'accelerometer':list(m.imu_state.accelerometer)},
                             'motor_state':[{'q':v.q,'dq':v.dq} for v in m.motor_state],
                             'foot_force':list(m.foot_force)})
                else:
                    e.update(stamp_ns=stamp_ns(m.header),frame_id=m.header.frame_id)
                    if kind=='imu':
                        e.update(angular_velocity=xyz(m.angular_velocity),linear_acceleration=xyz(m.linear_acceleration),
                                 orientation=[m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z])
                        for k in ('orientation_covariance','angular_velocity_covariance','linear_acceleration_covariance'):
                            e[k]=list(getattr(m,k))
                    else:
                        e.update(height=m.height,width=m.width,point_step=m.point_step,row_step=m.row_step,
                                 is_bigendian=m.is_bigendian,is_dense=m.is_dense,
                                 fields=[dict(name=v.name,offset=v.offset,datatype=v.datatype,count=v.count) for v in m.fields],
                                 data_b64=base64.b64encode(bytes(m.data)).decode())
                pending.put_nowait(e)
            except Exception as ex:
                if errors.empty():errors.put(str(ex) or 'input queue overflow')
        return receive
    try:
        for topic,typ,kind in [('rt/utlidar/cloud',PointCloud2_,'cloud'),('rt/utlidar/imu',Imu_,'imu'),('rt/lowstate',LowState_,'low')]:
            sub=ChannelSubscriber(topic,typ);sub.Init(callback(kind),0);subs.append(sub)
        deadline=cloud_deadline+.25  # Real trailing IMU for last rolling scan.
        while time.monotonic()<deadline:
            if not errors.empty():raise RuntimeError(errors.get())
            try:yield pending.get(timeout=.1)
            except queue.Empty:pass
    finally:
        for sub in subs:sub.Close()
