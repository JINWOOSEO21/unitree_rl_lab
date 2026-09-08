"""학습(IsaacLab)이 EM 입력에 걸던 노이즈를 배포에서 재현한다.

왜 필요할 수 있는가
-------------------
학습 코드(parkour_isaaclab/envs/mdp/observations.py)는 라이다 측정과 odometry 에
노이즈를 건다. 그건 **실기에서 저절로 생길 오차의 모사**라, 배포에서는 그 오차가
진짜로 생기므로 다시 얹지 않는 것이 원칙이다. 그런데 sim2sim 에서는 MuJoCo
레이캐스트와 sportmodestate 가 **완전히 정확**해서 그 항이 비어 버린다.

실측 결과 배포 지도가 학습보다 2.6 배 정확하다(평지 셀 기준):
    학습  편향 -3.48 cm  sigma 10.46 cm   EM 갱신간 떨림 1.91 cm
    배포  편향 -0.47 cm  sigma  4.43 cm                0.92 cm
정책은 "흔들리는 지도" 를 전제로 학습됐으므로, 너무 깨끗한 입력이 오히려 분포 밖일
수 있다. 이 모듈은 그 가설을 실험으로 가릴 수 있게 학습과 **같은 모델**을 재현한다.

파라미터는 학습 기본값 그대로:
    라이다   range_std 0.02 m,  ray_dir_std 0.2 deg
    odometry 위치  Delta_meas = Delta_true*(1+b+bias) + n
                   b ~ N(0, 0.02) (tick 마다), bias ~ U(-0.03, 0.03) (에피소드 상수),
                   n ~ N(0, 0.005^2)  — 전부 body frame 축별
             yaw   Delta_meas = Delta_true + bias*dt + n
                   |bias| ~ U(0.01, 0.05) deg/s 랜덤 부호, n ~ N(0, (0.003 deg)^2)
             roll/pitch  백색잡음 0.5 deg (드리프트 없음 — 중력으로 관측되므로)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """XYZ 외재 오일러 → 회전행렬. quat_to_mat 과 같은 규약을 되돌리기 위한 것."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


@dataclass
class TrainNoiseCfg:
    range_std: float = 0.02
    ray_dir_std_deg: float = 0.2
    odom_scale_var: float = 0.02
    odom_pos_walk_std: float = 0.005
    odom_scale_bias_max: float = 0.03
    odom_yaw_bias_range_dps: tuple = (0.01, 0.05)
    odom_yaw_walk_std_deg: float = 0.003
    odom_rp_std_deg: float = 0.5
    # 실험용: 위치 scale bias 를 무작위(U(±max)) 대신 고정값으로 (예: −0.06 = 6 % 짧게).
    # 추정기의 계통 편향을 흉내 내 "정책이 그 편향을 견디는가" 를 sport(GT) 지도에서 잰다.
    odom_scale_bias_fixed: float | None = None
    seed: int | None = None


class TrainNoise:
    """학습과 같은 노이즈를 tick 입력에 건다. tick 주기(dt)는 호출 쪽이 준다."""

    def __init__(self, cfg: TrainNoiseCfg | None = None):
        self.cfg = cfg or TrainNoiseCfg()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.reset()

    def reset(self) -> None:
        c = self.cfg
        # 에피소드 상수 (실기 캘리브레이션 오차에 해당)
        self._pos_scale_bias = self.rng.uniform(-c.odom_scale_bias_max,
                                                c.odom_scale_bias_max, size=3)
        if c.odom_scale_bias_fixed is not None:
            self._pos_scale_bias = np.full(3, float(c.odom_scale_bias_fixed))
        lo, hi = c.odom_yaw_bias_range_dps
        mag = self.rng.uniform(np.deg2rad(lo), np.deg2rad(hi))
        self._yaw_bias_rps = mag * (1.0 if self.rng.random() < 0.5 else -1.0)
        # 누적 오차 상태
        self._pos_err = np.zeros(3)
        self._yaw_err = 0.0
        self._prev_true_pos: np.ndarray | None = None
        self._prev_true_yaw: float | None = None

    # -- 라이다 -------------------------------------------------------------
    def perturb_points(self, pts_sensor: np.ndarray) -> np.ndarray:
        """센서 프레임 점군에 거리 잡음 + 빔 지향 잡음."""
        c = self.cfg
        if pts_sensor.shape[0] == 0:
            return pts_sensor
        d = np.linalg.norm(pts_sensor, axis=1, keepdims=True)
        d = np.maximum(d, 1e-6)
        u = pts_sensor / d
        # 거리 잡음: 시선 방향으로
        d_n = d + self.rng.normal(0.0, c.range_std, size=d.shape)
        # 지향 잡음: 시선에 수직인 작은 랜덤 성분 (소각 근사)
        s = np.deg2rad(c.ray_dir_std_deg)
        perp = self.rng.normal(0.0, s, size=pts_sensor.shape)
        perp -= (perp * u).sum(axis=1, keepdims=True) * u
        u_n = u + perp
        u_n /= np.maximum(np.linalg.norm(u_n, axis=1, keepdims=True), 1e-9)
        return (u_n * d_n).astype(pts_sensor.dtype)

    # -- odometry -----------------------------------------------------------
    def perturb_pose(self, base_pos: np.ndarray, yaw: float, roll: float,
                     pitch: float, dt: float):
        """참 pose → 관측된 pose. 위치는 변화량 기반이라 오차가 누적된다."""
        c = self.cfg
        if self._prev_true_pos is None:
            self._prev_true_pos = base_pos.copy()
            self._prev_true_yaw = yaw
            return base_pos.copy(), yaw, roll, pitch

        d_true = base_pos - self._prev_true_pos
        self._prev_true_pos = base_pos.copy()
        # body frame 축별로 scale 오차 (보폭/슬립 오차는 로봇 기준)
        Rz = _rot_z(yaw)
        d_body = Rz.T @ d_true
        b = self.rng.normal(0.0, np.sqrt(c.odom_scale_var), size=3)
        d_body_meas = d_body * (1.0 + b + self._pos_scale_bias)
        d_meas = Rz @ d_body_meas + self.rng.normal(0.0, c.odom_pos_walk_std, size=3)
        self._pos_err += d_meas - d_true

        # yaw: 참 증분은 그대로 반영되고 **오차만** 쌓인다 (gyro bias drift 모델).
        #   Delta_meas = Delta_true + bias*dt + n   →   err += bias*dt + n
        self._prev_true_yaw = yaw
        self._yaw_err += (self._yaw_bias_rps * dt
                          + self.rng.normal(0.0, np.deg2rad(c.odom_yaw_walk_std_deg)))
        rp_n = self.rng.normal(0.0, np.deg2rad(c.odom_rp_std_deg), size=2)
        return (base_pos + self._pos_err, yaw + self._yaw_err,
                roll + rp_n[0], pitch + rp_n[1])
