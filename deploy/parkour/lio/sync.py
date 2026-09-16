"""Sensor-time pose pairing for delayed LIO. No arrival-time substitution."""
from collections import deque
import math
import numpy as np


class PoseBuffer:
    def __init__(self):
        self.rows=deque(maxlen=10000);self.generation=None;self.last_receipt=0
    def clear(self):self.rows.clear();self.last_receipt=0
    def add(self,row,receipt):
        generation=row.get('generation')
        if not isinstance(generation,str) or not generation:raise ValueError('LIO generation missing')
        reset=generation!=self.generation
        if reset:self.clear();self.generation=generation
        if not row.get('pose_valid'):
            self.clear();return True
        if row.get('frame_id')!='odom' or row.get('child_frame_id')!='base_link':raise ValueError('LIO base frame mismatch')
        stamp=row.get('source_stamp_ns')
        if type(stamp) is not int:raise ValueError('LIO source stamp missing')
        p=np.array([row['position'][k] for k in ('x','y','z')]);q=np.array([row['orientation'][k] for k in ('w','x','y','z')])
        if not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.05:raise ValueError('invalid LIO pose')
        if self.rows and stamp<self.rows[-1]['source_stamp_ns']:
            self.clear();raise ValueError('LIO source clock regressed')
        if self.rows and stamp==self.rows[-1]['source_stamp_ns']:return reset
        self.rows.append(row);self.last_receipt=receipt
        return reset
    def at(self,stamp):
        rows=list(self.rows)
        for i in range(len(rows)-1,-1,-1):
            a=rows[i];ta=a['source_stamp_ns']
            if ta==stamp:return dict(a)
            if ta<stamp:
                if i+1==len(rows):return None
                b=rows[i+1];tb=b['source_stamp_ns']
                if tb-ta>50_000_000:return None
                f=(stamp-ta)/(tb-ta)
                p=(1-f)*np.array([a['position'][k] for k in ('x','y','z')])+f*np.array([b['position'][k] for k in ('x','y','z')])
                qa=np.array([a['orientation'][k] for k in ('w','x','y','z')],dtype=float);qb=np.array([b['orientation'][k] for k in ('w','x','y','z')],dtype=float);qa/=np.linalg.norm(qa);qb/=np.linalg.norm(qb)
                dot=float(qa@qb)
                if dot<0:qb=-qb;dot=-dot
                if dot>.9995:q=(1-f)*qa+f*qb
                else:
                    theta=math.acos(np.clip(dot,-1,1));q=(math.sin((1-f)*theta)*qa+math.sin(f*theta)*qb)/math.sin(theta)
                q/=np.linalg.norm(q)
                return dict(a,source_stamp_ns=stamp,position=dict(zip(('x','y','z'),p.tolist())),orientation=dict(zip(('w','x','y','z'),q.tolist())))
        return None


def deskew_to_end(cloud,poses):
    """Transform each raw point at its own sensor time into scan-end base frame."""
    from em_sidecar.go2_cloud import RAW_TO_BASE_ROTATION,RAW_TO_BASE_TRANSLATION
    from em_sidecar.kinematics import quat_to_mat
    fields={f.name:f for f in cloud.fields};data=bytes(cloud.data)
    endian='>' if cloud.is_bigendian else '<'
    arrays=[]
    for name in ('x','y','z','time'):
        f=fields[name]
        if f.datatype!=7 or f.count!=1:raise ValueError('deskew requires float32 xyz/time')
        arrays.append(np.ndarray((cloud.height,cloud.width),dtype=endian+'f4',buffer=data,
                                offset=f.offset,strides=(cloud.row_step,cloud.point_step)).reshape(-1).astype(float))
    xyz=np.stack(arrays[:3],axis=1);relative=arrays[3]
    if not np.isfinite(relative).all() or (relative<0).any() or (relative>.2).any():raise ValueError('bad per-point time')
    start=cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
    stamps=start+np.rint(relative*1e9).astype(np.int64);end=int(stamps.max())
    row=poses.at(end)
    if row is None:return None
    records=list(poses.rows)
    times=np.array([r['source_stamp_ns'] for r in records],dtype=np.int64)
    if len(times)<2 or stamps.min()<times[0] or stamps.max()>times[-1]:return None
    hi=np.clip(np.searchsorted(times,stamps,side='right'),1,len(times)-1);lo=hi-1
    if ((times[hi]-times[lo])>50_000_000).any():return None
    f=((stamps-times[lo])/(times[hi]-times[lo]))[:,None]
    positions=np.array([[r['position'][k] for k in ('x','y','z')] for r in records])
    qs=np.array([[r['orientation'][k] for k in ('w','x','y','z')] for r in records]);qs/=np.linalg.norm(qs,axis=1)[:,None]
    qa=qs[lo];qb=qs[hi].copy();dot=(qa*qb).sum(1);qb[dot<0]*=-1;dot=np.abs(dot)
    theta=np.arccos(np.clip(dot,-1,1));sin=np.sin(theta);near=dot>.9995
    safe=np.where(near,1,sin)[:,None]
    q=(np.sin((1-f)*theta[:,None])*qa+np.sin(f*theta[:,None])*qb)/safe
    q[near]=((1-f)*qa+f*qb)[near];q/=np.linalg.norm(q,axis=1)[:,None]
    body=xyz@RAW_TO_BASE_ROTATION.T+RAW_TO_BASE_TRANSLATION
    v=q[:,1:];world=body+2*np.cross(v,np.cross(v,body)+q[:,:1]*body)+(1-f)*positions[lo]+f*positions[hi]
    refq=np.array([row['orientation'][k] for k in ('w','x','y','z')]);refp=np.array([row['position'][k] for k in ('x','y','z')])
    result=(world-refp)@quat_to_mat(refq)
    return result[np.isfinite(result).all(axis=1)],row
