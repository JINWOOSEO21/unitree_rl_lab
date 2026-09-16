"""Validated LowState adapter for the existing leg estimator (no DDS).

Local position starts at zero; axes use the LowState IMU orientation. No sport
pose, IMU-site translation, or height guess is used. This is a diagnostic pose,
not a calibrated hardware odometry solution.
"""
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from em_sidecar.kinematics import Go2Kinematics
from em_sidecar.leg_odometry import LegOdometry, LegOdomCfg
from policy_input_guard import GuardConfig, validate_lowstate


class LegPose:
    def __init__(self, contract_dir=ROOT / 'contract', contact_threshold=20.0):
        if not np.isfinite(contact_threshold) or contact_threshold < 0:
            raise ValueError('leg contact threshold must be finite and non-negative')
        maps = yaml.safe_load((contract_dir / 'deploy.yaml').read_text())['index_maps']
        self.joints = np.asarray(maps['il_to_sdk'], dtype=int)
        self.feet = np.asarray(maps['il_foot_to_sdk'], dtype=int)
        if sorted(self.joints.tolist()) != list(range(12)) or sorted(self.feet.tolist()) != list(range(4)):
            raise ValueError('invalid joint/foot permutation')
        self.estimator = LegOdometry(Go2Kinematics(contract_dir / 'em_geometry.npz'),
                                    LegOdomCfg(contact_force_thr=contact_threshold))
        self.last_tick = None
        self.first_tick = None

    def update(self, low, tick):
        validate_lowstate(low, GuardConfig())
        tick = int(tick)
        if tick < 0:
            raise ValueError('negative LowState tick')
        if self.last_tick is not None:
            if tick < self.last_tick:
                raise RuntimeError('leg LowState clock regressed; restart bridge/map required')
            if tick == self.last_tick:
                return None  # Never refresh pose age with a duplicate source sample.
            if (tick-self.last_tick)*.001 > self.estimator.cfg.max_dt_s:
                raise RuntimeError('leg LowState gap too large; restart bridge/map required')
        if self.first_tick is None:
            self.first_tick = tick
        q = np.asarray([m['q'] for m in low['motor_state'][:12]])[self.joints]
        quat = np.asarray(low['imu_state']['quaternion'], dtype=float)
        quat = quat / np.linalg.norm(quat)
        gyro = np.asarray(low['imu_state']['gyroscope'], dtype=float)
        force = np.asarray(low['foot_force'], dtype=float)[self.feet]
        position = self.estimator.step((tick-self.first_tick)*.001, q, quat, gyro, force).copy()
        if not np.isfinite(position).all():
            raise RuntimeError('nonfinite leg odometry; restart bridge/map required')
        self.last_tick = tick
        return {'frame_id':'odom', 'child_frame_id':'base_link',
                'position':dict(zip(('x','y','z'), position.tolist())),
                'orientation':dict(zip(('w','x','y','z'), quat.tolist())),
                'source':'leg', 'source_tick':tick,
                'reliable_feet':self.estimator.last_n_both,
                'branch':self.estimator.last_branch,
                'pose_valid':self.estimator.last_branch in (1,2)}
