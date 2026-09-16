"""Diagnostic LIO terrain path; production policy topic is forbidden."""
import json
import threading
import time
from collections import deque
from .sync import PoseBuffer, deskew_to_end


def run(interface,domain,duration,root,emit,publish_scandots,topic):
    if publish_scandots and topic != 'rt/parkour/scandots_lio_eval':
        raise ValueError('LIO remains diagnostic: use rt/parkour/scandots_lio_eval only')
    from tools.go2_sensor_bridge import dependencies,BaseMapper,low_row,stamp_id
    from tools.go2_scandots_output import ScandotsOutput
    from tools.policy_input_guard import GuardConfig,validate_lowstate
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    initialize,subscriber,low_type,cloud_type,_=dependencies()
    mapper=BaseMapper(root);poses=PoseBuffer();clouds=deque(maxlen=128);lock=threading.RLock()
    fatal=threading.Event();generation=[0];last_low=[0,None];last_cloud=[None];subs=[];output=None;dirty=[False];last_pose_reason=[None]
    def fault(reason,terminal=False):
        with lock:
            generation[0]+=1;dirty[0]=True
            if output:output.invalidate()
            if terminal:fatal.set()
            emit({'kind':'fatal' if terminal else 'fault','reason':reason})
    def low_callback(m):
        try:
            now=time.monotonic_ns();low=low_row(m);validate_lowstate(low,GuardConfig())
            with lock:
                if last_low[1] is not None and m.tick<last_low[1]:raise ValueError('LowState clock regressed')
                if m.tick!=last_low[1]:last_low[:]=[now,m.tick]
            emit({'kind':'low','receipt_ns':now,'source_id':int(m.tick),'low':low})
        except Exception as e:fault(str(e),True)
    def pose_callback(m):
        try:
            row=json.loads(m.data)
            with lock:
                reason=row.get('reason','LIO pose invalid') if not row.get('pose_valid') else None
                if reason is not None and reason!=last_pose_reason[0]:
                    emit({'kind':'fault','reason':reason})
                last_pose_reason[0]=reason
                if poses.add(row,time.monotonic_ns()):
                    generation[0]+=1;dirty[0]=True;clouds.clear()
                    if output:output.invalidate()
        except Exception as e:fault(str(e),True)
    def cloud_callback(m):
        try:
            stamp=stamp_id(m)
            if m.header.frame_id!='utlidar_lidar':raise ValueError('raw cloud frame mismatch')
            with lock:
                if last_cloud[0] is not None and stamp<last_cloud[0]:raise ValueError('cloud clock regressed')
                if stamp==last_cloud[0]:return
                if len(clouds)==clouds.maxlen:raise ValueError('LIO map cloud queue overflow')
                last_cloud[0]=stamp;clouds.append((time.monotonic_ns(),stamp,m))
        except Exception as e:fault(str(e),True)
    initialize(domain,interface)
    try:
        if publish_scandots:output=ScandotsOutput(topic)
        for name,typ,cb in [('rt/lowstate',low_type,low_callback),('rt/utlidar/cloud',cloud_type,cloud_callback),('rt/parkour/lio/pose',String_,pose_callback)]:
            sub=subscriber(name,typ);sub.Init(cb,0);subs.append(sub)
        began=time.monotonic();next_tick=began
        while not fatal.is_set() and (duration==0 or time.monotonic()-began<duration):
            time.sleep(max(0,next_tick-time.monotonic()));next_tick=max(next_tick+.1,time.monotonic())
            with lock:
                if dirty[0]:
                    mapper.discard_map();mapper.last_position=None;dirty[0]=False
                now=time.monotonic_ns()
                while clouds and now-clouds[0][0]>200_000_000:clouds.popleft()
                if now-last_low[0]>20_000_000 or now-poses.last_receipt>200_000_000:
                    dirty[0]=True
                    if output:output.invalidate()
                    continue
                selected=None
                for item in reversed(clouds):
                    aligned=deskew_to_end(item[2],poses)
                    if aligned is not None:selected=(item,aligned);break
                if selected is None:
                    if output:output.invalidate()
                    continue
                (receipt,stamp,cloud),(base_points,row)=selected;version=generation[0]
                while clouds and clouds[0][1]<=stamp:clouds.popleft()
            try:
                scan,valid,upper=mapper.update(cloud,row,base_points=base_points)
                with lock:
                    now=time.monotonic_ns()
                    if version!=generation[0] or now-receipt>200_000_000 or now-last_low[0]>20_000_000 or now-poses.last_receipt>200_000_000:
                        raise ValueError('LIO map inputs stale/invalidated')
                    if output:output.publish(scan,list(row['position'].values()),receipt,now)
                    emit({'kind':'scan','receipt_ns':now,'source_ns':receipt,'source_id':stamp,'scan':scan.tolist(),
                          'odometry':row,'valid_fraction':valid.tolist(),'upper_bound_fraction':upper.tolist(),
                          'diagnostic_only':True,'motion_compensated':True,'pose_reference':'scan_end'})
            except Exception as e:fault(str(e))
        return 1 if fatal.is_set() else 0
    finally:
        for sub in subs:sub.Close()
        if output:output.close()
