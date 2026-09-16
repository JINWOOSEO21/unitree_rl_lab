"""ROS1 side of a loopback-only, single-run sensor gateway. No DDS or motors."""
import base64
import json
import queue
import socket
import threading
import time
import rospy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from protocol import MAX_LINE, InputClock, validate


def main():
    rospy.init_node('go2_lio_gateway', disable_signals=True)
    cloud_pub=rospy.Publisher('/unilidar/cloud',PointCloud2,queue_size=2048)
    imu_pub=rospy.Publisher('/unilidar/imu',Imu,queue_size=16384)
    output=queue.Queue(maxsize=16384)
    failed=threading.Event(); stop=threading.Event(); last_output=[None]; cloud_ends=[]; last_imu=[None]
    def pose(m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        event=dict(kind='lio_pose',stamp_ns=m.header.stamp.to_nsec(),
                   frame_id=m.header.frame_id,child_frame_id=m.child_frame_id,
                   position=[p.x,p.y,p.z],orientation=[q.w,q.x,q.y,q.z])
        last_output[0]=event['stamp_ns']
        try:output.put_nowait(event)
        except queue.Full:failed.set()
    sub=rospy.Subscriber('/pointlio/odom',Odometry,pose,queue_size=16384,tcp_nodelay=True)
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind(('127.0.0.1',17654));server.listen(1);server.settimeout(120)
        print('gateway listening 127.0.0.1:17654',flush=True)
        connection,_=server.accept()
        with connection:
            connection.settimeout(10)
            deadline=time.monotonic()+30
            while not(cloud_pub.get_num_connections() and imu_pub.get_num_connections()):
                if time.monotonic()>deadline:raise RuntimeError('Point-LIO input subscribers not ready')
                time.sleep(.1)
            connection.sendall(b'{"kind":"ready","diagnostic_only":true}\n')
            def send():
                try:
                    while not stop.is_set():
                        if failed.is_set():raise RuntimeError('output queue overflow')
                        try:event=output.get(timeout=.1)
                        except queue.Empty:continue
                        connection.sendall((json.dumps(event,allow_nan=False)+'\n').encode())
                except Exception as e:
                    print('output error:',e,flush=True);failed.set()
                    connection.shutdown(socket.SHUT_RDWR)
            sender=threading.Thread(target=send,daemon=True);sender.start()
            clock=InputClock();counts={'cloud':0,'imu':0}
            try:
                with connection.makefile('rb') as stream:
                    while not rospy.is_shutdown():
                        if failed.is_set():raise RuntimeError('gateway output failed')
                        line=stream.readline(MAX_LINE+1)
                        if not line:break
                        if len(line)>MAX_LINE:raise ValueError('oversize message')
                        e=json.loads(line)
                        if e['kind']=='end':
                            # Flush only real sensor data. Never fabricate trailing IMU.
                            deadline=time.monotonic()+5
                            covered=[t for t in cloud_ends if last_imu[0] is not None and t<=last_imu[0]]
                            target=max(covered) if covered else None
                            while target is not None and (last_output[0] is None or last_output[0]<target-2_000_000) and time.monotonic()<deadline:
                                time.sleep(.02)
                            unprocessed=sum(last_output[0] is None or t>last_output[0]+2_000_000 for t in cloud_ends)
                            output.put(dict(kind='done',counts=counts,duplicates=clock.duplicates,gaps=clock.gaps,
                                            last_output_stamp_ns=last_output[0],last_imu_stamp_ns=last_imu[0],
                                            final_scan_end_ns=max(cloud_ends) if cloud_ends else None,
                                            scans_without_trailing_imu=len(cloud_ends)-len(covered),
                                            scans_beyond_final_output=unprocessed,
                                            complete_sensor_interval=bool(cloud_ends) and unprocessed==0))
                            deadline=time.monotonic()+3
                            while not output.empty() and time.monotonic()<deadline:time.sleep(.02)
                            time.sleep(.1);break
                        if not clock.accept(e):continue
                        if e['kind']=='cloud':
                            cloud_ends.append(e['stamp_ns']+round(validate(e)*1e9))
                            m=PointCloud2();m.height=e['height'];m.width=e['width']
                            m.fields=[PointField(**f) for f in e['fields']]
                            m.is_bigendian=e['is_bigendian'];m.point_step=e['point_step'];m.row_step=e['row_step']
                            m.is_dense=e['is_dense'];m.data=base64.b64decode(e['data_b64'],validate=True)
                        else:
                            last_imu[0]=e['stamp_ns']
                            m=Imu()
                            m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=e['angular_velocity']
                            m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=e['linear_acceleration']
                            m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z=e['orientation']
                            for key in ('orientation_covariance','angular_velocity_covariance','linear_acceleration_covariance'):
                                setattr(m,key,e.get(key,[0.]*9))
                        m.header.stamp=rospy.Time(e['stamp_ns']//10**9,e['stamp_ns']%10**9)
                        m.header.frame_id=e['frame_id']
                        (cloud_pub if e['kind']=='cloud' else imu_pub).publish(m)
                        counts[e['kind']]+=1
            except Exception as e:
                print('gateway fatal:',e,flush=True)
                output.put(dict(kind='fatal',reason=str(e)));time.sleep(.2)
                raise
            finally:
                stop.set();sender.join(timeout=2);sub.unregister()
    print('gateway complete',counts,flush=True)


if __name__=='__main__':main()
