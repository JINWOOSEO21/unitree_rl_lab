"""Replay real sensor recordings through the live mapper on loopback/domain 179.

Test-only sensor writers: no LowCmd, no robot network, no service clients.
The fixed domain/interface prevent this replay reaching the physical controller.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=False)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_, sensor_msgs_msg_dds__PointField_Constants_PointCloud2_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_,HeightMap_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_,PointField_
    capture=ROOT/'captures/live_frame_check01'
    lows=[json.loads(l) for l in (capture/'lowstate.jsonl').read_text().splitlines()]
    rows=[json.loads(l) for l in (capture/'utlidar_cloud/cloud.jsonl').read_text().splitlines()]
    blob=(capture/'utlidar_cloud/cloud.bin').read_bytes()
    events=[]
    for row in lows:
        msg=unitree_go_msg_dds__LowState_()
        msg.tick=row['tick']
        msg.imu_state.quaternion=row['imu_state']['quaternion']
        msg.imu_state.gyroscope=row['imu_state']['gyroscope']
        msg.foot_force=row['foot_force']
        for i,motor in enumerate(row['motor_state']):
            msg.motor_state[i].q=motor['q'];msg.motor_state[i].dq=motor['dq']
        events.append((row['steady_ns'],'low',msg))
    for row in rows:
        msg=sensor_msgs_msg_dds__PointField_Constants_PointCloud2_()
        msg.header.frame_id=row['frame_id']
        msg.header.stamp.sec=row['stamp']['sec'];msg.header.stamp.nanosec=row['stamp']['nanosec']
        for field in ('height','width','point_step','row_step','is_bigendian','is_dense'):
            setattr(msg,field,row[field])
        msg.fields=[PointField_(**f) for f in row['fields']]
        msg.data=blob[row['binary_offset']:row['binary_offset']+row['binary_size']]
        events.append((row['steady_ns'],'cloud',msg))
    events.sort(key=lambda e:e[0])
    ChannelFactoryInitialize(179,'lo')
    pubs={'low':ChannelPublisher('rt/lowstate',LowState_),
          'cloud':ChannelPublisher('rt/utlidar/cloud',PointCloud2_)}
    for pub in pubs.values():pub.Init()
    received=[]
    def on_scan(msg):
        received.append({'time_ns':time.monotonic_ns(),'width':msg.width,'height':msg.height,
                         'frame_id':msg.frame_id,'stamp':msg.stamp,'data':list(msg.data)})
    reader=ChannelSubscriber('rt/parkour/test_scandots',HeightMap_);reader.Init(on_scan,0)
    child=None
    try:
        with (args.output_dir/'bridge.jsonl').open('w') as output, (args.output_dir/'stderr.log').open('w') as stderr:
            child=subprocess.Popen([sys.executable,str(ROOT/'tools/go2_sensor_bridge.py'),
                '--interface','lo','--domain','179','--odom','leg','--duration','13',
                '--publish-scandots','--scandots-topic','rt/parkour/test_scandots',
                '--summary-only'],stdout=output,stderr=stderr)
            time.sleep(3)  # mapper imports and DDS discovery; no robot is involved
            start=time.monotonic_ns()
            for source_ns,kind,msg in events:
                target=start+source_ns-events[0][0]
                time.sleep(max(0,(target-time.monotonic_ns())*1e-9))
                if not pubs[kind].Write(msg):raise RuntimeError(f'{kind} replay write failed')
            code=child.wait(timeout=20)
        valid=[r for r in received if len(r['data'])==132]
        summary={'interface':'lo','domain':179,'robot_commands':False,
                 'input_capture':str(capture),'received':len(received),'valid_scans':len(valid),
                 'invalidations':sum(not r['data'] for r in received),'bridge_exit_code':code,
                 'valid_values':bool(valid) and all(np.isfinite(r['data']).all() and np.max(np.abs(r['data']))<=1 for r in valid),
                 'notes':'Recorded sensors replayed at their PC receipt intervals; no hardware gait validation.'}
        (args.output_dir/'received.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in received))
        (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2))
        if code or not valid or not summary['valid_values'] or not summary['invalidations']:
            raise SystemExit('DDS mapper replay failed; see captured logs')
    finally:
        if child is not None and child.poll() is None:
            child.terminate();child.wait(timeout=5)
        reader.Close()
        for pub in pubs.values():pub.Close()


if __name__=='__main__':main()
