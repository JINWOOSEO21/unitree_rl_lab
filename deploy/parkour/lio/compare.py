"""Paired trajectory comparison. Residuals are NOT ground-truth errors."""
import argparse
import json
from pathlib import Path
import numpy as np


def position(row):return np.array([row['position'][k] for k in ('x','y','z')])
def yaw(row):
    w,x,y,z=[row['orientation'][k] for k in ('w','x','y','z')]
    return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))


def compare(directory):
    p=Path(directory)
    leg=[json.loads(l) for l in (p/'leg.jsonl').open()]
    leg=[r for r in leg if r['pose']['pose_valid'] and 'estimated_sensor_ns' in r]
    lio=[json.loads(l) for l in (p/'lio.jsonl').open()]
    lio=[r for r in lio if r['kind']=='lio_pose']
    frame_consistent=bool(lio) and all(r['base_pose'].get('frame_consistent',False) for r in lio)
    if not leg or not lio:raise ValueError('no valid leg/LIO pair; leg needs supported stationary initialization')
    lio=list({r['stamp_ns']:r for r in lio}.values());lio.sort(key=lambda r:r['stamp_ns'])
    origin=lio[0]['stamp_ns'];lt=np.array([(r['stamp_ns']-origin)*1e-9 for r in lio]);gt=np.array([(r['estimated_sensor_ns']-origin)*1e-9 for r in leg])
    use=(gt>=lt[0])&(gt<=lt[-1]);leg=[r for r,v in zip(leg,use) if v];gt=gt[use]
    if len(leg)<2:raise ValueError('insufficient common time interval')
    lp=np.array([position(r['base_pose']) for r in lio]);gp=np.array([position(r['pose']) for r in leg]);ly=np.unwrap([yaw(r['base_pose']) for r in lio]);gy=np.unwrap([yaw(r['pose']) for r in leg])
    interp=np.stack([np.interp(gt,lt,lp[:,i]) for i in range(3)],axis=1)
    iy=np.interp(gt,lt,ly);angle=gy[0]-iy[0];c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    aligned=(interp-interp[0])@rotation.T+gp[0];res=aligned-gp
    report={'scope':'cross-estimator residual, no ground truth; initial translation/yaw alignment only',
            'clock_alignment':'LowState receipt mapped by latest LiDAR IMU receipt/header offset; not hardware sync',
            'diagnostic_only':True,'frame_consistent':frame_consistent,
            'accuracy_comparison_valid':False,
            'warning':'Nominal-frame diagnostic only; no external ground truth and extrinsics remain unverified',
            'duration_s':float(gt[-1]-gt[0]),'paired_samples':len(gt),
            'leg_delta_m':(gp[-1]-gp[0]).tolist(),'lio_delta_aligned_m':(aligned[-1]-aligned[0]).tolist(),
            'residual_xyz_final_m':res[-1].tolist(),'residual_norm_p50_p95_max_m':np.percentile(np.linalg.norm(res,axis=1),[50,95,100]).tolist(),
            'yaw_residual_final_deg':float(np.rad2deg((iy[-1]-iy[0])-(gy[-1]-gy[0])))}
    (p/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez_compressed(p/'comparison.npz',time_s=gt-gt[0],leg=gp,lio_aligned=aligned,residual=res)
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(compare(parser.parse_args().directory),indent=2))
