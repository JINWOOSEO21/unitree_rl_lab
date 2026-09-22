"""LowState adapter for MIT-style body-IMU/leg odometry; no DDS or ROS."""
import numpy as np
from .leg_pose import LegPose
from .input_guard import GuardConfig, validate_lowstate
from em_sidecar.mit_odometry import MitOdometry, MitConfig, multiply
from em_sidecar.kinematics import quat_to_mat


class MitPose(LegPose):
    """Reuse the proven leg adapter's stationary gyro calibration and gap budget.

    Pose is expressed in the initial robot base frame. mapping_pose retains a
    gravity-aligned world for elevation mapping (which assumes vertical Z).
    Only the Go2 body IMU is consumed. Acceleration is never bias/scale corrected.
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.estimator = MitOdometry(self.estimator.kin,
                                    MitConfig(contact_force_thr=self.estimator.cfg.contact_force_thr))
        self.seen_tick = None

    def update(self, low, tick):
        tick = int(tick)
        if tick < 0:
            raise ValueError('negative LowState tick')
        if self.seen_tick is not None:
            if tick < self.seen_tick:
                raise RuntimeError('MIT LowState clock regressed; restart required')
            if tick == self.seen_tick:
                return None
        validate_lowstate(low, GuardConfig())
        try:
            acc = np.asarray(low['imu_state']['accelerometer'], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('MIT requires Go2 body IMU accelerometer in m/s^2') from exc
        if acc.shape != (3,) or not np.isfinite(acc).all():
            raise ValueError('MIT accelerometer must be a finite 3-vector')
        self.seen_tick = tick
        gap = None if self.last_tick is None else (tick-self.last_tick)*.001
        if gap is not None and gap < self.min_interval_s:
            return None
        gap_s = self._note_gap(tick, gap) if gap is not None and gap > self.estimator.cfg.max_dt_s else None
        if self.first_tick is None:
            self.first_tick = tick
        q = np.asarray([m['q'] for m in low['motor_state'][:12]])[self.joints]
        dq = np.asarray([m['dq'] for m in low['motor_state'][:12]])[self.joints]
        quat = np.asarray(low['imu_state']['quaternion'], dtype=float)
        quat /= np.linalg.norm(quat)
        gyro = np.asarray(low['imu_state']['gyroscope'], dtype=float)
        force = np.asarray(low['foot_force'], dtype=float)[self.feet]
        t = (tick-self.first_tick)*.001
        if not self.calibrated:
            self._calibrate(t, q, quat, gyro, force)
        previous_branch = self.estimator.last_branch
        if self.calibrated:
            position = self.estimator.step(t, q, dq, quat, gyro-self.gyro_bias, acc, force)
            orientation = self.estimator.orientation
            initial = self.estimator.initial_quaternion
        else:
            position, orientation, initial = np.zeros(3), quat, quat
        inverse = initial*np.array([1.,-1.,-1.,-1.])
        relative_quat = multiply(inverse, orientation)
        relative_position = quat_to_mat(initial).T @ position
        def pose(p, r):
            return {'frame_id':'odom', 'child_frame_id':'base_link',
                    'position':dict(zip(('x','y','z'), p.tolist())),
                    'orientation':dict(zip(('w','x','y','z'), r.tolist()))}
        row = pose(relative_position, relative_quat)
        row.update(source='mit', source_tick=tick, reference_frame='initial_robot_base',
                   mapping_pose=dict(pose(position, orientation), reference_frame='gravity_aligned_odom'),
                   initial_base_orientation_in_mapping_wxyz=initial.tolist(),
                   velocity=dict(zip(('x','y','z'), (quat_to_mat(initial).T @ self.estimator.x[3:6]).tolist())),
                   pose_gap_s=gap_s, pose_gaps=self.gaps,
                   map_reset_required=(gap_s is not None or
                                       previous_branch in (1,2) and self.estimator.last_branch == 4),
                   reliable_feet=self.estimator.last_n_both, branch=self.estimator.last_branch,
                   gyro_calibrated=self.calibrated, gyro_bias_rad_s=self.gyro_bias.tolist(),
                   gyro_calibration_elapsed_s=(t-self._calibration['start'] if self._calibration is not None else 0.),
                   pose_valid=self.calibrated and gap_s is None and self.estimator.last_branch in (1,2),
                   acceleration_corrected=False, velocity_innovation_m_s=self.estimator.max_innovation)
        self.last_tick = tick
        return row
