"""Explicit nominal L1-IMU to Go2 base conversion; mounting not yet certified."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from em_sidecar.go2_cloud import RAW_TO_BASE_ROTATION, RAW_TO_BASE_TRANSLATION
from em_sidecar.kinematics import quat_to_mat


def matrix_quat(r):
    # Eigenvector method is stable at rotations near pi; output wxyz.
    k=np.array([[r[0,0]-r[1,1]-r[2,2],r[0,1]+r[1,0],r[0,2]+r[2,0],r[2,1]-r[1,2]],
                [r[0,1]+r[1,0],r[1,1]-r[0,0]-r[2,2],r[1,2]+r[2,1],r[0,2]-r[2,0]],
                [r[0,2]+r[2,0],r[1,2]+r[2,1],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
                [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],r.trace()]])/3
    q=np.linalg.eigh(k)[1][:,-1][[3,0,1,2]]
    return q if q[0]>=0 else -q


def base_pose(event):
    if event.get('kind') != 'lio_pose' or event.get('frame_id') != 'camera_init' or event.get('child_frame_id') != 'aft_mapped':
        raise ValueError('unexpected Point-LIO output frame')
    p=np.asarray(event['position'],float); q=np.asarray(event['orientation'],float)
    if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.05:
        raise ValueError('invalid Point-LIO pose')
    # T_B_I = T_B_L T_L_I; T_W_B = T_W_I inverse(T_B_I).
    rwi=quat_to_mat(q/np.linalg.norm(q))
    tbi=RAW_TO_BASE_TRANSLATION + RAW_TO_BASE_ROTATION @ np.array([-.007698,-.014655,.00667])
    rwb=rwi @ RAW_TO_BASE_ROTATION.T
    pwb=p-rwb@tbi
    return {'frame_id':'odom','child_frame_id':'base_link',
            'position':dict(zip(('x','y','z'),pwb.tolist())),
            'orientation':dict(zip(('w','x','y','z'),matrix_quat(rwb).tolist())),
            'source':'point_lio','source_stamp_ns':event['stamp_ns'],
            'pose_valid':True,'diagnostic_only':True,
            'extrinsic_status':'nominal_l1_imu_axes_unverified'}


def gravity_disagreement(row, low_quaternion):
    """Yaw-invariant tilt disagreement; does not certify yaw/extrinsics."""
    q=np.array([row['orientation'][k] for k in ('w','x','y','z')],float)
    ref=np.asarray(low_quaternion,float)
    if ref.shape!=(4,) or not np.isfinite(ref).all() or abs(np.linalg.norm(ref)-1)>.05:
        return float('inf')
    down=np.array([0.,0.,-1.])
    a=quat_to_mat(q/np.linalg.norm(q)).T@down
    b=quat_to_mat(ref/np.linalg.norm(ref)).T@down
    return float(np.arccos(np.clip(a@b,-1,1)))
