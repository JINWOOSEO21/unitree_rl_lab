"""Validated LowState adapter for the existing leg estimator (no DDS).

Local position starts at zero; axes use the LowState IMU orientation. No sport
pose, IMU-site translation, or height guess is used. This is a diagnostic pose,
not a calibrated hardware odometry solution.
"""
from collections import deque
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
                 calibration_seconds=10.0, max_gaps=4, gap_window_s=30.0, rate_hz=0.0):
        if not np.isfinite(contact_threshold) or contact_threshold < 0:
            raise ValueError('leg contact threshold must be finite and non-negative')
        if not np.isfinite(calibration_seconds) or calibration_seconds < 0:
            raise ValueError('calibration duration must be finite and non-negative')
        if max_gaps < 0 or not np.isfinite(gap_window_s) or gap_window_s <= 0:
            raise ValueError('gap budget must be non-negative over a positive window')
        if not np.isfinite(rate_hz) or rate_hz < 0:
            raise ValueError('estimator rate must be finite and non-negative')
        self.max_gaps = int(max_gaps)
        self.gap_window_s = float(gap_window_s)
        # 0 keeps the historical behaviour: run the kinematics on every sample handed in.
        self.min_interval_s = 0.0 if rate_hz == 0 else 1.0/float(rate_hz)
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
        if self.min_interval_s > self.estimator.cfg.max_dt_s:
            raise ValueError('estimator rate slower than max_dt_s would gap on every sample')
        self.last_tick = None
        self.first_tick = None
        self.gaps = 0
        self._gap_ticks = deque()

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

    def _note_gap(self, tick, gap_s):
        """Survive one dropped stretch of LowState; refuse to limp through a stream of them.

        A single gap is recoverable and does happen: the map backend pays a one-time CUDA
        JIT on its first update (207 ms on an Orin NX against a 31 ms median) and can starve
        the DDS reader while it holds the GIL. Killing the bridge for that costs a whole run,
        so absorb it -- LegOdometry.step already skips the velocity update across a dt this
        large, and resume_after_gap clears the stale foot references it would otherwise trust.

        What the estimator cannot do is tell the caller the map is now wrong. Position is held
        across the gap instead of integrated, so the robot's real travel while we were blind
        is missing from it, and everything already accumulated in the map is offset by exactly
        that much. The caller must rebuild the map; that is what the returned gap reports.

        Repeated gaps are a different failure. Each one throws the map away, so a map that
        never survives long enough to be useful is worse than a clean stop -- escalate.
        """
        self._gap_ticks.append(tick)
        while self._gap_ticks and (tick-self._gap_ticks[0])*.001 > self.gap_window_s:
            self._gap_ticks.popleft()
        self.gaps += 1
        if len(self._gap_ticks) > self.max_gaps:
            raise RuntimeError(
                f'leg LowState gap {gap_s*1000:.0f} ms: {len(self._gap_ticks)} gaps within '
                f'{self.gap_window_s:.0f} s; restart bridge/map required')
        # The gyro is unmeasured across the gap, so any calibration window spanning it would
        # weight one sample over the whole hole. Start the stationary window over.
        self._calibration = None
        self.estimator.resume_after_gap()
        return gap_s

    def update(self, low, tick):
        """Fold one LowState sample in, or return None if it was a duplicate or decimated.

        The tick checks come before validate_lowstate so a decimated sample costs almost
        nothing: on the Jetson the kinematics below run 1.5 ms against a 2.0 ms budget at
        500 Hz, which is 76 % of a core held under the GIL, and everything else sharing the
        process starves behind it. Callers that need every sample validated at full rate
        must still do that themselves -- the bridge does.
        """
        tick = int(tick)
        if tick < 0:
            raise ValueError('negative LowState tick')
        gap = None
        if self.last_tick is not None:
            if tick < self.last_tick:
                raise RuntimeError('leg LowState clock regressed; restart bridge/map required')
            if tick == self.last_tick:
                return None  # Never refresh pose age with a duplicate source sample.
            gap = (tick-self.last_tick)*.001
            if gap < self.min_interval_s:
                return None  # Too soon to be worth the kinematics; see min_interval_s.
        validate_lowstate(low, GuardConfig())
        gap_s = (self._note_gap(tick, gap)
                 if gap is not None and gap > self.estimator.cfg.max_dt_s else None)
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
                # Not None only on the sample that closed a gap: the map accumulated before
                # it no longer lines up with this pose and must be rebuilt (see _note_gap).
                'pose_gap_s':gap_s, 'pose_gaps':self.gaps,
                'reliable_feet':self.estimator.last_n_both,
                'branch':self.estimator.last_branch,
                'gyro_calibrated':self.calibrated,
                'gyro_bias_rad_s':self.gyro_bias.tolist(),
                'gyro_calibration_elapsed_s':(t-self._calibration['start']
                                              if self._calibration is not None else 0.0),
                'pose_valid':self.calibrated and self.estimator.last_branch in (1,2)}
