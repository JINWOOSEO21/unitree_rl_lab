"""Run independent Point-LIO capture/replay; no LowCmd or sport RPC.

python -m lio.host --replay captures/lio_feasibility_20260916 --output captures/lio_run
python -m lio.host --interface enp42s0 --duration 60 --output captures/lio_live
Requires a fresh go2-point-lio container for each run. Outputs are diagnostic.
"""
import argparse
import base64
import json
from pathlib import Path
import socket
import sys
import threading
import time
import uuid

from .protocol import MAX_LINE, InputClock
from .pose import base_pose, gravity_disagreement


def replay(directory):
    directory=Path(directory)
    if (directory/'input.jsonl').exists():
        with (directory/'input.jsonl').open() as f:
            for line in f:yield json.loads(line)
        return
    # Compatibility with the existing feasibility probe. Its float stamps have
    # already lost sub-microsecond precision; never describe them as exact.
    with (directory/'samples.jsonl').open() as f, (directory/'cloud.bin').open('rb') as binary:
        for line in f:
            r=json.loads(line);kind=r['kind']
            if kind not in ('low','imu','cloud'):continue
            e={'kind':kind,'receipt_ns':round(r['receipt']*1e9),'legacy_float_timestamps':True}
            if kind=='low':
                e.update(tick=r['tick'],low={'imu_state':{'quaternion':r['quat'],'gyroscope':r['gyro'],'accelerometer':r['acc']},
                    'motor_state':[{'q':q,'dq':0.0} for q in r['q']], 'foot_force':r['force']})
                e['legacy_missing_dq']=True
            else:
                e.update(stamp_ns=round(r['stamp']*1e9),frame_id=r['frame'])
                if kind=='imu':e.update(angular_velocity=r['gyro'],linear_acceleration=r['acc'],orientation=r['quat'])
                else:
                    binary.seek(r['offset']);data=binary.read(r['size'])
                    e.update({k:r[k] for k in ('width','height','point_step','row_step','is_bigendian','fields')})
                    e.update(is_dense=True,data_b64=base64.b64encode(data).decode())
            yield e


def run(args):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    generation=uuid.uuid4().hex
    meta={'generation':generation,'diagnostic_only':True,'extrinsic_verified':False,'acceleration_calibrated':False,
          'upstream_commit':'18ed5976d8fab2bd8a5148c26a40692bd3c0dc91',
          'source':str(args.replay) if args.replay else args.interface,'rate':args.rate}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    completion=[None]; latest_low=[None]; received=[];errors=[];done=threading.Event();clock=InputClock();counts={};publisher=None
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
    from go2_leg_pose import LegPose
    leg=LegPose()
    source=replay(args.replay) if args.replay else None
    with socket.create_connection(('127.0.0.1',args.port),timeout=35) as conn:
        stream=conn.makefile('rb');line=stream.readline(MAX_LINE)
        if json.loads(line).get('kind')!='ready':raise RuntimeError('gateway not ready')
        if source is None:
            from .dds_input import events
            source=events(args.interface,args.domain,args.duration)
        if args.publish_odom:
            # Initialize DDS once before lazy live generator starts. Same params.
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            ChannelFactoryInitialize(args.domain,args.interface)
            publisher=ChannelPublisher('rt/parkour/lio/pose',String_);publisher.Init()
        with (out/'input.jsonl').open('w') as raw, (out/'lio.jsonl').open('w') as lf, (out/'leg.jsonl').open('w') as legf:
            def receive():
                try:
                    while True:
                        line=stream.readline(MAX_LINE+1)
                        if not line:
                            if not done.is_set():raise RuntimeError('gateway closed before completion')
                            break
                        if len(line)>MAX_LINE:raise ValueError('oversize gateway output')
                        e=json.loads(line);e['receipt_ns']=time.monotonic_ns();e['generation']=generation
                        if e['kind']=='lio_pose':
                            e['base_pose']=base_pose(e)
                            ref=latest_low[0]
                            mismatch=gravity_disagreement(e['base_pose'],ref['low']['imu_state']['quaternion']) if ref else None
                            e['base_pose']['gravity_disagreement_rad']=mismatch
                            consistent=mismatch is not None and mismatch<=0.35
                            e['base_pose']['pose_valid']=consistent
                            e['base_pose']['frame_consistent']=consistent
                            e['base_pose']['tracking_verified']=False
                            if not consistent:e['base_pose']['reason']='nominal IMU/base gravity axes inconsistent or LowState missing'
                            if publisher:
                                packet=dict(e['base_pose'],generation=generation,receipt_ns=e['receipt_ns'])
                                if not publisher.Write(String_(data=json.dumps(packet,allow_nan=False))):
                                    raise RuntimeError('LIO diagnostic DDS write failed')
                        lf.write(json.dumps(e,allow_nan=False)+'\n');lf.flush()
                        received.append(e['kind'])
                        if e['kind']=='fatal':raise RuntimeError(e['reason'])
                        if e['kind']=='done':completion[0]=e;done.set();break
                except Exception as ex:errors.append(str(ex));done.set()
            worker=threading.Thread(target=receive,daemon=True);worker.start()
            first=None;start=time.monotonic();last_imu=None
            try:
                for e in source:
                    if errors:raise RuntimeError(errors[0])
                    if first is None:first=e['receipt_ns'];start=time.monotonic()
                    if args.replay:
                        wait=(e['receipt_ns']-first)*1e-9/args.rate-(time.monotonic()-start)
                        if wait>0:time.sleep(wait)
                    raw.write(json.dumps(e,allow_nan=False,separators=(',',':'))+'\n')
                    counts[e['kind']]=counts.get(e['kind'],0)+1
                    if e['kind']=='low':
                        latest_low[0]=e
                        pose=leg.update(e['low'],e['tick'])
                        if pose:
                            item={'receipt_ns':e['receipt_ns'],'tick':e['tick'],'pose':pose}
                            if last_imu:item['estimated_sensor_ns']=e['receipt_ns']+last_imu['stamp_ns']-last_imu['receipt_ns']
                            legf.write(json.dumps(item,allow_nan=False)+'\n')
                        continue
                    if not clock.accept(e):continue
                    if e['kind']=='imu':last_imu=e
                    conn.sendall((json.dumps(e,allow_nan=False,separators=(',',':'))+'\n').encode())
                conn.sendall(b'{"kind":"end"}\n')
                if not done.wait(15):raise RuntimeError('gateway completion timed out')
                if errors:raise RuntimeError(errors[0])
                if 'lio_pose' not in received:raise RuntimeError('Point-LIO produced no odometry')
            finally:
                if hasattr(source,'close'):source.close()
                if publisher:
                    publisher.Write(String_(data=json.dumps({'pose_valid':False,'generation':generation,'reason':'host stopped'})))
                    publisher.Close()
                conn.shutdown(socket.SHUT_RDWR);worker.join(timeout=2)
    summary={'counts':counts,'lio_poses':received.count('lio_pose'),'clock_gaps':clock.gaps,'duplicates':clock.duplicates,
             'diagnostic_only':True,'errors':errors,'completion':completion[0]}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True);mode.add_argument('--replay',type=Path);mode.add_argument('--interface')
    p.add_argument('--output',required=True,type=Path);p.add_argument('--duration',type=float,default=60)
    p.add_argument('--domain',type=int,default=0);p.add_argument('--rate',type=float,default=1)
    p.add_argument('--port',type=int,default=17654);p.add_argument('--publish-odom',action='store_true')
    a=p.parse_args()
    if not 0<a.duration<=300 or not 0<a.rate<=1 or not 0<=a.domain<=232:p.error('duration 0..300, replay rate 0..1, domain 0..232 required')
    if a.publish_odom and a.replay:p.error('replay must not publish live DDS odometry')
    existed=a.output.exists()
    try:
        run(a)
    except Exception as ex:
        if not existed and a.output.exists():
            (a.output/'failure.json').write_text(json.dumps({'error':str(ex),'diagnostic_only':True},indent=2)+'\n')
        raise


if __name__=='__main__':main()
