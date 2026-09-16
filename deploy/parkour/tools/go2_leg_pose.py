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
    def __init__(self, contract_dir=ROOT / 'contract', contact_threshold=20.0,
                 calibration_seconds=10.0):
        if not np.isfinite(contact_threshold) or contact_threshold < 0:
            raise ValueError('leg contact threshold must be finite and non-negative')
        if not np.isfinite(calibration_seconds) or calibration_seconds < 0:
            raise ValueError('calibration duration must be finite and non-negative')
        self.calibration_seconds = float(calibration_seconds)
        self.gyro_bias = np.zeros(3)
        self.calibrated = calibration_seconds == 0  # Explicit offline baseline only.
        self._calibration = None
        maps = yaml.safe_load((contract_dir / 'deploy.yaml').read_text())['index_maps']
        self.joints = np.asarray(maps['il_to_sdk'], dtype=int)
        self.feet = np.asarray(maps['il_foot_to_sdk'], dtype=int)
        if sorted(self.joints.tolist()) != list(range(12)) or sorted(self.feet.tolist()) != list(range(4)):
            raise ValueError('invalid joint/foot permutation')
        self.estimator = LegOdometry(Go2Kinematics(contract_dir / 'em_geometry.npz'),
                                    LegOdomCfg(contact_force_thr=contact_threshold))
        self.last_tick = None
        self.first_tick = None

    def _calibrate(self, t, q, quat, gyro, force):
        """One stationary startup window; freeze bias thereafter, including in turns.

        Contact alone cannot prove rest. Bound joint and quaternion excursion over
        the whole window as well as gyro magnitude/variance. This cannot detect
        coherent sliding or an erroneous frozen attitude signal.
        """
        if np.any(force <= self.estimator.cfg.contact_force_thr) or np.max(np.abs(gyro)) > 0.1:
            self._calibration = None
            return
        window = self._calibration
        if window is not None:
            angle = 2 * np.arccos(np.clip(abs(np.dot(quat, window['quat'])), 0, 1))
            qmin = np.minimum(window['qmin'], q)
            qmax = np.maximum(window['qmax'], q)
            if angle > 0.01 or np.max(qmax-qmin) > 0.005:
                window = None
        if window is None:
            self._calibration = dict(start=t, previous=t, quat=quat.copy(),
                                     qmin=q.copy(), qmax=q.copy(),
                                     total=np.zeros(3), square=np.zeros(3))
            return
        dt = t-window['previous']
        window.update(previous=t, qmin=qmin, qmax=qmax)
        window['total'] += gyro*dt
        window['square'] += gyro*gyro*dt
        elapsed = t-window['start']
        if elapsed >= self.calibration_seconds:
            bias = window['total']/elapsed
            std = np.sqrt(np.maximum(window['square']/elapsed-bias*bias, 0))
            if np.max(std) > 0.025:
                self._calibration = None
                return
            self.gyro_bias = bias
            self.calibrated = True
            self._calibration = None

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
        t = (tick-self.first_tick)*.001
        if not self.calibrated:
            self._calibrate(t, q, quat, gyro, force)
        # Do not integrate or seed the terrain map with uncalibrated motion.
        position = (self.estimator.step(t, q, quat, gyro-self.gyro_bias, force).copy()
                    if self.calibrated else np.zeros(3))
        if not np.isfinite(position).all():
            raise RuntimeError('nonfinite leg odometry; restart bridge/map required')
        self.last_tick = tick
        return {'frame_id':'odom', 'child_frame_id':'base_link',
                'position':dict(zip(('x','y','z'), position.tolist())),
                'orientation':dict(zip(('w','x','y','z'), quat.tolist())),
                'source':'leg', 'source_tick':tick,
                'reliable_feet':self.estimator.last_n_both,
                'branch':self.estimator.last_branch,
                'gyro_calibrated':self.calibrated,
                'gyro_bias_rad_s':self.gyro_bias.tolist(),
                'gyro_calibration_elapsed_s':(t-self._calibration['start']
                                              if self._calibration is not None else 0.0),
                'pose_valid':self.calibrated and self.estimator.last_branch in (1,2)}
