"""EM 사이드카 — L1 점군을 받아 정책이 먹을 scandots 132 를 만들어 발행한다.

역할
----
학습(IsaacLab)에서 이 일은 관측항 `elevation_map_scan` 이 했다. 배포에서는 그
자리가 비어 있다. 사이드카가 그 자리를 채운다:

    rt/utlidar/cloud (L1 점군, 센서 프레임)
    rt/lowstate      (관절각 12, IMU 자세)          →  [사이드카]  →  rt/parkour/scandots
    rt/sportmodestate(odometry 위치)                                 (HeightMap_, 132 float)

파이프라인은 학습 코드(parkour_isaaclab/envs/mdp/observations.py 의
elevation_map_scan._update)와 같은 순서다:

    점군 → self-filter → elevation_mapping_cupy 갱신 → scandots 격자 샘플
         → clip(base_z − h − 0.3, ±1)

학습과 **일부러 다르게 한 것**
-----------------------------
학습 코드에는 range/빔지향 백색잡음과 odometry drift 모델이 들어 있다. 그건 전부
domain randomization, 즉 "실기에서 저절로 생길 오차를 시뮬레이터에서 흉내 낸 것"
이다. 배포에서는 그 오차가 진짜로 생기므로 **다시 얹지 않는다**:

  - 거리/빔 잡음 → 실기 L1 이 자체적으로 가진다. (MuJoCo 레이캐스트는 정확하므로
    sim2sim 에서는 이 항이 비어 있는 셈이다. 알려진 sim2sim 격차로 남긴다.)
  - odometry drift → 실기 sportmodestate 가 자체적으로 가진다. MuJoCo 에서는
    GT 라서 역시 비어 있다.

self-filter 는 반대다. 실기 L1 도 자기 몸을 찍으므로 배포에서 **반드시** 필요하다.

pose 출처
---------
  위치   : SidecarCfg.odom_source 로 고른다.
             "sport" — rt/sportmodestate.position. MuJoCo 에서는 브리지가 주는 GT,
                       실기에서는 sport 서비스 추정기. 저수준 제어 중 계속 온다는
                       보장이 없다.
             "leg"   — 다리 운동학 + IMU 로 직접 적분 (leg_odometry.py). lowstate
                       만 있으면 된다. 실기 기본 후보.
  자세   : rt/lowstate.imu_state.quaternion
sportmodestate 에도 imu_state 필드가 있지만 unitree_mujoco 브리지는 position/velocity
만 채운다. 자세는 lowstate 쪽이 실기·시뮬레이터 양쪽에서 항상 차 있다.

발행 메시지
-----------
unitree_go::HeightMap_ 를 132 float 전송용으로 쓴다. width=12, height=11,
resolution=0.15 는 격자 모양 그대로지만, 이 격자는 **로봇 yaw 를 따라 도는**
지역 격자라 origin+resolution 만으로는 표현되지 않는다. 그래서

    data   : 정책 obs[53:185] 에 그대로 들어갈 132 벡터 (x 안쪽 12, y 바깥쪽 11)
    origin : 그 시점의 base (x, y) — 진단용
    stamp  : 점군 프레임 시각

로 약속한다. 소비자(State_Parkour)는 data 를 그대로 쓴다. 커스텀 IDL 을 새로
만들면 idlc 코드 생성이 C++/파이썬 양쪽에 붙어 배포가 무거워지므로 피했다.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .kinematics import Go2Kinematics, quat_to_mat, yaw_from_quat
from .selffilter import capsule_self_hits

# GO2_LIDAR_CFG (parkour_tasks/default_cfg.py) 의 마운트 — C++ 시뮬레이터
# simulate/src/l1_lidar.h 의 mount_pos/mount_quat 과 같은 값이어야 한다.
MOUNT_POS = np.array([0.28, 0.0, 0.10])
MOUNT_QUAT = np.array([0.0, 1.0, 0.0, 0.0])  # (w,x,y,z) — x축 180°

# odometry 가 보고하는 점이 base 원점에서 얼마나 떨어져 있는가 (base 프레임).
#
# unitree_mujoco 는 rt/sportmodestate.position 을 base 바디가 아니라 **imu site**
# 에 붙여 낸다:
#     <framepos name="frame_pos" objtype="site" objname="imu"/>
#     <site name="imu" pos="-0.02557 0 0.04232"/>
# 그래서 받은 위치에서 이 오프셋을 빼야 base 원점이 된다. 이걸 안 빼면 센서
# 마운트(0.28, 0, 0.10)를 엉뚱한 점에 얹게 되고, 로봇이 회전할 때마다 지도가
# 최대 |offset| = 5 cm 씩 어긋난다 (같은 tick 안에서는 상쇄되지만 tick 사이에는
# 안 된다 — 지도는 누적되기 때문이다).
#
# 실기의 SportModeState.position 은 유니트리 자체 추정기가 내는 **몸체 프레임**
# 위치다. 그 경우 이 값은 0 이어야 한다. 실기로 옮길 때 SidecarCfg.odom_offset_in_base
# 를 0 으로 두고 확인할 것 [실측 필요].
IMU_SITE_IN_BASE = np.array([-0.02557, 0.0, 0.04232])

# EM tick 주기. L1 한 프레임(0.1 s)마다 한 번 — 학습의 update_interval=5 x step_dt=0.02.
EM_TICK_S = 0.1

TOPIC_CLOUD = "rt/utlidar/cloud"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_SPORT = "rt/sportmodestate"
TOPIC_SCANDOTS = "rt/parkour/scandots"


@dataclass
class SidecarCfg:
    contract_dir: Path
    emcupy_root: Path
    device: str = "cuda:0"
    em_resolution: float = 0.1
    em_map_length: float = 3.2
    domain_id: int = 0
    interface: str = "lo"
    publish_topic: str = TOPIC_SCANDOTS
    # 진단 토픽 <publish_topic>_valid 에 valid_frac 을 함께 낸다. 셀이 직접
    # 관측된 것인지 upper_bound 대체값인지 구분해야 오차의 출처를 가릴 수 있다
    # (실기에서도 "지금 앞을 실제로 보고 있는가"를 아는 유일한 방법이다).
    publish_diag: bool = True
    # 위치가 한 tick 에 이만큼 뛰면 순간이동으로 보고 지도를 비운다 [m].
    teleport_jump_m: float = 1.0
    # sportmodestate.position 이 가리키는 점의 base 프레임 오프셋 (위 상수 설명 참조).
    odom_offset_in_base: np.ndarray = field(
        default_factory=lambda: IMU_SITE_IN_BASE.copy()
    )
    # base 위치 출처 — "sport" | "leg" (모듈 docstring 'pose 출처' 참조).
    # leg 는 base **원점**을 직접 추정하므로 odom_offset_in_base 를 적용하지 않고,
    # sportmodestate 가 오면 기록용 GT 로만 쓴다.
    odom_source: str = "sport"
    leg_odom: "LegOdomCfg | None" = None  # None 이면 leg_odometry.LegOdomCfg() 기본값
    # 그림자 모드: 지도는 sport(GT) 로 만들되 leg 추정기를 옆에서 돌려 기록만 남긴다.
    # 추정기가 틀리면 지도가 뒤틀려 로봇이 이상하게 걷고 그게 다시 추정을 망치는
    # 되먹임이 있어, 추정기 자체의 성능은 정상 보행 위에서 따로 재야 한다.
    leg_shadow: bool = False
    # leg 의 시작 위치를 sportmodestate 첫 값에 맞춘다 — sim2sim 에서 GT 와 같은 프레임에
    # 놓아 드리프트를 바로 비교하기 위해서다. 그 토픽이 없으면(실기 저수준 제어) 0 에서
    # 시작하며, 지도는 상대 좌표라 문제없다.
    leg_seed_from_sport: bool = True
    # tick 마다 (시각, base pose, scan, valid) 를 모아 npz 로 남긴다. 정지 상태
    # 게이트만으로는 **주행 중** 지도가 어긋나는지 알 수 없다 — 정책이 헛것을 보고
    # 반응하는지 판정하려면 자세와 함께 기록해야 한다.
    record_path: Path | None = None
    # record_path 와 함께: tick 마다 EM 전체 지도(34×34 레이어)도 남긴다 (영상·진단용).
    record_map: bool = False
    # 학습과 같은 EM 입력 노이즈를 얹는다 (실험용). MuJoCo 는 라이다·odometry 가
    # GT 라 학습의 노이즈 항이 비어 있는데, 정책은 흔들리는 지도를 전제로 학습됐다.
    # 실기에서는 그 오차가 진짜로 생기므로 켜면 안 된다 — sim2sim 실험용이다.
    train_noise: bool = False
    train_noise_seed: int | None = None
    train_noise_cfg: "TrainNoiseCfg | None" = None  # None 이면 학습값 그대로
    verbose: bool = True


@dataclass
class _Latest:
    """리더 스레드가 쓰고 처리 스레드가 읽는 최신값 (락 보호)."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    q_sdk: np.ndarray | None = None
    quat: np.ndarray | None = None
    pos: np.ndarray | None = None        # EM 이 쓸 위치 (sport 원시값 또는 leg 추정)
    pos_sport: np.ndarray | None = None  # sportmodestate 원시값 (leg 모드에서 GT 기록용)
    pos_est: np.ndarray | None = None    # leg 추정 (leg / shadow 모드, 기록용)
    sport_stamp: float = float("nan")    # sportmodestate.stamp (브리지: sim 시각) — 정렬용
    n_low: int = 0
    n_sport: int = 0


class EmSidecar:
    def __init__(self, cfg: SidecarCfg):
        self.cfg = cfg
        os.environ.setdefault("EMCUPY_ROOT", str(cfg.emcupy_root))

        self.kin = Go2Kinematics(cfg.contract_dir / "em_geometry.npz")
        self.scan_xy = self.kin.scan_offsets_xy  # (132,2) base 프레임 (yaw 정렬)
        self.num_points = self.scan_xy.shape[0]
        self.il_to_sdk = _load_sdk_to_il(cfg.contract_dir / "deploy.yaml")

        self.R_mount = quat_to_mat(MOUNT_QUAT)
        self.latest = _Latest()
        self.h_obs = np.zeros(self.num_points, dtype=np.float32)
        self.valid_frac = np.zeros(self.num_points, dtype=np.float32)
        self.ub_frac = np.zeros(self.num_points, dtype=np.float32)
        self._prev_pos: np.ndarray | None = None
        self.n_ticks = 0
        self.last_stats: dict[str, float] = {}
        self._rec: list[tuple] = []
        self._rec_map: list[tuple] = []  # record_map: tick 별 EM 전체 지도
        # leg odometry 스텝별 진단 (record_path 가 있을 때만):
        # t, dt, n_both, branch, vel(3), |acc|, foot_force_il(4), pos(3)
        self._odom_dbg: list[tuple] = []
        self._raw_dbg: list[tuple] = []
        self._est_for_record: np.ndarray | None = None
        self._last_stamp: float | None = None
        self._noise = None
        if cfg.train_noise:
            from .train_noise import TrainNoise, TrainNoiseCfg
            nc = cfg.train_noise_cfg or TrainNoiseCfg()
            nc.seed = cfg.train_noise_seed
            self._noise = TrainNoise(nc)
            if cfg.verbose:
                print(f"[em] EM 입력 노이즈 켜짐 (실험용): scale σ {np.sqrt(nc.odom_scale_var):.3f} "
                      f"walk {nc.odom_pos_walk_std*100:.1f}cm bias "
                      f"{'고정 %+.3f' % nc.odom_scale_bias_fixed if nc.odom_scale_bias_fixed is not None else '±%.3f' % nc.odom_scale_bias_max}",
                      flush=True)

        if cfg.odom_source not in ("sport", "leg"):
            raise ValueError(f"odom_source 는 'sport' 또는 'leg': {cfg.odom_source!r}")
        self._use_leg = cfg.odom_source == "leg"
        self._odom = None
        if self._use_leg or cfg.leg_shadow:
            from .leg_odometry import LegOdomCfg, LegOdometry
            self._odom = LegOdometry(self.kin, cfg.leg_odom or LegOdomCfg())
            self.il_foot_to_sdk = _load_index(cfg.contract_dir / "deploy.yaml", "il_foot_to_sdk")
            self._odom_seeded = not cfg.leg_seed_from_sport
            self._odom_clock: str | None = None  # "tick" | "wall", 첫 lowstate 에서 정한다
            if cfg.verbose:
                c = self._odom.cfg
                print(f"[em] leg odometry {'사용' if self._use_leg else '그림자(기록만)'} "
                      f"(접촉 임계 {c.contact_force_thr} N, 발 반지름 {c.foot_radius*100:.2f} cm, "
                      f"{'힘가중' if c.weight_by_force else '균등'})", flush=True)

        self._backend = None  # 지연 초기화 (cupy 컨텍스트를 첫 tick 에서 만든다)
        self._torch = None
        self._pub = None
        self._pub_diag = None

    # -- 백엔드 --------------------------------------------------------------
    def _ensure_backend(self):
        if self._backend is not None:
            return
        import importlib.util
        import sys

        import torch

        self._torch = torch
        # vendored 사본을 경로로 적재한다 (패키지 import 경로에 의존하지 않게).
        src = Path(__file__).resolve().parent.parent / "vendored" / "elevation_map_backend.py"
        spec = importlib.util.spec_from_file_location("em_backend_vendored", src)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = mod
        spec.loader.exec_module(mod)
        self._backend = mod.BatchedElevationMapBackend(
            num_envs=1,
            device=self.cfg.device,
            resolution=self.cfg.em_resolution,
            map_length=self.cfg.em_map_length,
        )
        if self.cfg.verbose:
            print(f"[em] backend 준비 — cell_n={self._backend.cell_n} device={self.cfg.device}",
                  flush=True)

    # -- 콜백 ----------------------------------------------------------------
    def on_lowstate(self, msg) -> None:
        # DDS 리더 스레드에서 예외가 나면 스레드만 조용히 죽고 증상은 "위치가 안 바뀐다"
        # 로만 보인다 (예전에 지역 import 하나로 배치 세 번을 날렸다). 반드시 찍는다.
        try:
            self._on_lowstate(msg)
        except Exception:
            import traceback
            print("[em] on_lowstate 예외 — 리더 스레드가 죽는다:", flush=True)
            traceback.print_exc()
            raise

    def _on_lowstate(self, msg) -> None:
        q = np.array([msg.motor_state[i].q for i in range(12)], dtype=np.float64)
        quat = np.array(list(msg.imu_state.quaternion), dtype=np.float64)
        pos = None
        if self._odom is not None:
            # 시각: lowstate.tick [ms] (브리지는 sim 시각, 실기는 MCU 시각). 0 이면 벽시계.
            if self._odom_clock is None:
                self._odom_clock = "tick" if int(msg.tick) > 0 else "wall"
            t = int(msg.tick) * 1e-3 if self._odom_clock == "tick" else time.monotonic()
            gyro = np.array(list(msg.imu_state.gyroscope), dtype=np.float64)
            acc = np.array(list(msg.imu_state.accelerometer), dtype=np.float64)
            ff_il = np.array(list(msg.foot_force), dtype=np.float64)[self.il_foot_to_sdk]
            if self._odom_seeded:
                pos = self._odom.step(t, q[self.il_to_sdk], quat, gyro, ff_il, acc).copy()
                if self.cfg.record_path is not None:
                    o = self._odom
                    self._odom_dbg.append((t, o.last_dt, o.last_n_both, o.last_branch,
                                           *o.vel, float(np.linalg.norm(acc)), *ff_il, *pos))
                    # 원시 입력 스트림 — 오프라인 재생용 (tick, q_il 12, quat 4, gyro 3,
                    # ff_il 4, sport 원시 위치 3, sport stamp). 라이브 반복 없이 추정기를
                    # 결정론적으로 다시 돌려 볼 수 있다. sport 위치는 다른 스레드로 오므로
                    # stamp 로 다시 정렬해야 한다.
                    ps = self.latest.pos_sport
                    self._raw_dbg.append((t, *q[self.il_to_sdk], *quat, *gyro, *ff_il,
                                          *(np.full(3, np.nan) if ps is None else ps),
                                          self.latest.sport_stamp))
        with self.latest.lock:
            self.latest.q_sdk = q
            self.latest.quat = quat
            if pos is not None:
                self.latest.pos_est = pos
                if self._use_leg:
                    self.latest.pos = pos
            self.latest.n_low += 1

    def on_sport(self, msg) -> None:
        pos = np.array(list(msg.position), dtype=np.float64)
        with self.latest.lock:
            self.latest.pos_sport = pos
            self.latest.sport_stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
            self.latest.n_sport += 1
            if not self._use_leg:
                self.latest.pos = pos
            if self._odom is None:
                return
            elif not self._odom_seeded and self.latest.quat is not None:
                # leg 시작점을 GT 프레임에 맞춘다 (imu site → base 원점 보정 포함)
                seed = pos - quat_to_mat(self.latest.quat) @ self.cfg.odom_offset_in_base
                self._odom.reset(seed)
                self._odom_seeded = True
                if self.cfg.verbose:
                    print(f"[em] leg odometry 시작점을 sportmodestate 에 맞춤: {seed.round(3)}",
                          flush=True)

    def on_cloud(self, msg) -> None:
        with self.latest.lock:
            q_sdk, quat, pos = self.latest.q_sdk, self.latest.quat, self.latest.pos
            pos_sport, pos_est = self.latest.pos_sport, self.latest.pos_est
        if q_sdk is None or quat is None or pos is None:
            return  # 아직 상태가 안 왔다 — 이 프레임은 버린다
        n = int(msg.width)
        if n <= 0:
            return
        pts = np.frombuffer(bytes(msg.data), dtype=np.float32)[: n * 3].reshape(n, 3)
        self.tick(pts.astype(np.float64), q_sdk, quat, pos, float(msg.header.stamp.sec)
                  + float(msg.header.stamp.nanosec) * 1e-9,
                  gt_pos=pos_sport if self._odom is not None else None,
                  est_pos=pos_est)

    # -- 본 처리 -------------------------------------------------------------
    def tick(
        self,
        points_sensor: np.ndarray,
        q_sdk: np.ndarray,
        base_quat: np.ndarray,
        base_pos: np.ndarray,
        stamp: float = 0.0,
        gt_pos: np.ndarray | None = None,
        est_pos: np.ndarray | None = None,
    ) -> np.ndarray:
        """점군 한 프레임 → scandots 132. 반환값은 h_obs (발행과 별개로 쓸 수 있다).

        gt_pos : leg/그림자 모드에서 sportmodestate 원시 위치 (기록용 GT). 지도에는 안 쓴다.
        est_pos: leg 추정 base 원점 (기록용). leg 모드에서는 base_pos 와 같다.
        """
        self._ensure_backend()
        torch = self._torch
        dev = self.cfg.device

        q_il = q_sdk[self.il_to_sdk]
        R_base = quat_to_mat(base_quat)
        # sportmodestate 가 준 위치는 base 원점이 아니다 (IMU_SITE_IN_BASE 주석 참조) —
        # 되돌린다. leg odometry 는 base 원점을 직접 추정하므로 그대로 쓴다.
        if not self._use_leg:
            base_pos = base_pos - R_base @ self.cfg.odom_offset_in_base
        gt_base = None if gt_pos is None else gt_pos - R_base @ self.cfg.odom_offset_in_base
        self._est_for_record = est_pos

        # 학습과 같은 EM 입력 노이즈 (실험용, train_noise.py 참조).
        # MuJoCo 는 라이다·odometry 가 GT 라 학습의 노이즈 항이 비어 있다. 정책은
        # 흔들리는 지도를 전제로 학습됐으므로 너무 깨끗한 입력이 분포 밖일 수 있다.
        if self._noise is not None:
            # 주의: 여기서 `from .kinematics import yaw_from_quat` 를 하면 그 이름이
            # **함수 지역 변수**가 되어, 노이즈가 꺼졌을 때 아래(291행)의 원래 사용처가
            # UnboundLocalError 로 죽는다. 모듈 상단 import 를 그대로 쓴다.
            from .train_noise import rpy_to_mat as _rpy_to_mat
            yaw = yaw_from_quat(base_quat)
            roll = np.arctan2(2.0 * (base_quat[0] * base_quat[1]
                                     + base_quat[2] * base_quat[3]),
                              1.0 - 2.0 * (base_quat[1] ** 2 + base_quat[2] ** 2))
            sp = np.clip(2.0 * (base_quat[0] * base_quat[2]
                                - base_quat[3] * base_quat[1]), -1.0, 1.0)
            pitch = np.arcsin(sp)
            dt = 1.0 / 10.0 if self._last_stamp is None else max(
                1e-3, stamp - self._last_stamp)
            self._last_stamp = stamp
            base_pos, yaw, roll, pitch = self._noise.perturb_pose(
                base_pos, yaw, roll, pitch, dt)
            R_base = _rpy_to_mat(roll, pitch, yaw)
            points_sensor = self._noise.perturb_points(points_sensor)
        R_s = R_base @ self.R_mount
        t_s = base_pos + R_base @ MOUNT_POS

        # 순간이동(리셋) 감지 — 지도에 옛 지형이 남으면 안 된다.
        if self._prev_pos is not None:
            if np.linalg.norm(base_pos - self._prev_pos) > self.cfg.teleport_jump_m:
                self._backend.clear([0])
                if self.cfg.verbose:
                    print("[em] 위치 점프 감지 — 지도 초기화", flush=True)
        self._prev_pos = base_pos.copy()

        # --- self-filter (센서 프레임에서 판정) ---
        a_w, b_w, rad = self.kin.capsule_endpoints(q_il, base_pos, R_base)
        a_s = (a_w - t_s) @ R_s
        b_s = (b_w - t_s) @ R_s
        self_hit = capsule_self_hits(points_sensor, a_s, b_s, rad)
        pts_keep = points_sensor[~self_hit]

        # --- EM 갱신 ---
        p_t = torch.as_tensor(pts_keep, dtype=torch.float32, device=dev)
        self._backend.update(
            [p_t],
            torch.as_tensor(R_s, dtype=torch.float32, device=dev).unsqueeze(0),
            torch.as_tensor(t_s, dtype=torch.float32, device=dev).unsqueeze(0),
            torch.as_tensor(base_pos, dtype=torch.float32, device=dev).unsqueeze(0),
            torch.as_tensor(R_base, dtype=torch.float32, device=dev).unsqueeze(0),
        )

        # --- scandots 샘플 (yaw 만 따라 도는 지역 격자) ---
        yaw = yaw_from_quat(base_quat)
        cy, sy = np.cos(yaw), np.sin(yaw)
        ox, oy = self.scan_xy[:, 0], self.scan_xy[:, 1]
        px = base_pos[0] + cy * ox - sy * oy
        py = base_pos[1] + sy * ox + cy * oy
        pxy = torch.as_tensor(np.stack([px, py], axis=-1), dtype=torch.float32,
                              device=dev).unsqueeze(0)
        bz = torch.as_tensor([base_pos[2]], dtype=torch.float32, device=dev)
        h, vf, uf = self._backend.sample(pxy, bz)

        self.h_obs = h[0].cpu().numpy().astype(np.float32)
        self.valid_frac = vf[0].cpu().numpy().astype(np.float32)
        self.ub_frac = uf[0].cpu().numpy().astype(np.float32)
        self.n_ticks += 1
        self.last_stats = {
            "n_in": float(points_sensor.shape[0]),
            "n_self": float(int(self_hit.sum())),
            "n_used": float(pts_keep.shape[0]),
            "valid_cells": float(int((self.valid_frac > 1e-6).sum())),
            "ub_cells": float(int((self.ub_frac > 1e-6).sum())),
            "stamp": stamp,
        }
        if self.cfg.record_path is not None:
            # base_pos 는 이미 odom 보정을 거친 **base 원점**이다. 나중에 지형과
            # 대조할 때 이 값을 그대로 써야 한다 (raw sportmodestate 를 쓰면 4.2 cm
            # 틀린다 — 예전에 그 실수로 "지도 표류" 라는 잘못된 결론을 냈다).
            est = self._est_for_record
            self._rec.append((float(stamp), base_pos.copy(), base_quat.copy(),
                              self.h_obs.copy(), self.valid_frac.copy(),
                              np.full(3, np.nan) if gt_base is None else gt_base.copy(),
                              np.full(3, np.nan) if est is None else np.asarray(est, float).copy()))
            if self.cfg.record_map:
                # EM 전체 지도 (영상·진단용). 레이어 규약은 vendored 백엔드 참조:
                # [0] center z 상대 높이, [2] 직접 관측 여부, [5] 상한값, [6] 상한 여부.
                # 인덱스 [ix, iy], ix = floor((x − cx)/res + n/2).
                L = self._backend.layers()[0].detach().cpu().numpy()
                c = self._backend.centers_t()[0].detach().cpu().numpy()
                self._rec_map.append((L[0].astype(np.float16), (L[2] > 0.5),
                                      L[5].astype(np.float16), (L[6] > 0.5), c.astype(np.float32)))
        if self._pub is not None:
            self._publish(stamp, base_pos)
        return self.h_obs

    def save_record(self) -> None:
        """기록해 둔 tick 들을 npz 로 떨군다 (record_path 가 있을 때만)."""
        if self.cfg.record_path is None or not self._rec:
            return
        # 리더 스레드가 아직 tick 을 넣고 있을 수 있다 — 스냅샷을 떠서 키마다 길이가
        # 같게 한다 (한 번 304/305 로 어긋나 분석 도구가 죽었다).
        rec = list(self._rec)
        np.savez_compressed(
            self.cfg.record_path,
            stamp=np.array([r[0] for r in rec], dtype=np.float64),
            base_pos=np.stack([r[1] for r in rec]).astype(np.float32),
            base_quat=np.stack([r[2] for r in rec]).astype(np.float32),
            scan=np.stack([r[3] for r in rec]).astype(np.float32),
            valid=np.stack([r[4] for r in rec]).astype(np.float32),
            # leg odometry 모드에서만 채워진다 (sportmodestate 의 base 원점 위치). 그 외 NaN.
            gt_pos=np.stack([r[5] for r in rec]).astype(np.float32),
            # leg 추정 (leg / 그림자 모드). 그 외 NaN.
            est_pos=np.stack([r[6] for r in rec]).astype(np.float32),
            odom_source=np.array(self.cfg.odom_source + ("+shadow" if self.cfg.leg_shadow else "")),
            odom_dbg=np.array(list(self._odom_dbg), dtype=np.float64).reshape(-1, 15),
            lowstate_raw=np.array(list(self._raw_dbg), dtype=np.float64).reshape(-1, 28),
            **self._map_arrays(len(rec)),
        )
        if self.cfg.verbose:
            print(f"[em] tick 기록 {len(rec)}개 → {self.cfg.record_path}", flush=True)

    def _map_arrays(self, n: int) -> dict:
        """record_map 이 켜져 있으면 tick 별 EM 지도 배열들 (길이 n 으로 맞춤)."""
        if not self._rec_map:
            return {}
        m = list(self._rec_map)[:n]
        return {
            "em_elev": np.stack([r[0] for r in m]),        # (T,n,n) center z 상대 [m]
            "em_valid": np.stack([r[1] for r in m]),       # (T,n,n) bool
            "em_ub": np.stack([r[2] for r in m]),          # (T,n,n) 상한 [m]
            "em_is_ub": np.stack([r[3] for r in m]),       # (T,n,n) bool
            "em_center": np.stack([r[4] for r in m]),      # (T,3) odom frame
            "em_res": np.array(self.cfg.em_resolution),
        }

    # -- DDS -----------------------------------------------------------------
    def _publish(self, stamp: float, base_pos: np.ndarray) -> None:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

        def make(values: np.ndarray) -> HeightMap_:
            return HeightMap_(
                stamp=float(stamp),
                frame_id="base_yaw",
                resolution=float(self.kin.scan_offsets_xy[1, 0]
                                 - self.kin.scan_offsets_xy[0, 0]),
                width=12,
                height=11,
                origin=[float(base_pos[0]), float(base_pos[1])],
                data=values.astype(np.float32).tolist(),
            )

        self._pub.Write(make(self.h_obs))
        if self._pub_diag is not None:
            self._pub_diag.Write(make(self.valid_frac))

    def start_dds(self) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LowState_, SportModeState_

        from .dds_compat import init_dds

        init_dds(self.cfg.domain_id, self.cfg.interface)

        self._pub = ChannelPublisher(self.cfg.publish_topic, HeightMap_)
        self._pub.Init()
        if self.cfg.publish_diag:
            self._pub_diag = ChannelPublisher(self.cfg.publish_topic + "_valid", HeightMap_)
            self._pub_diag.Init()

        self._sub_low = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub_low.Init(self.on_lowstate, 10)
        self._sub_sport = ChannelSubscriber(TOPIC_SPORT, SportModeState_)
        self._sub_sport.Init(self.on_sport, 10)
        # 점군 처리는 무거우므로 큐 길이를 1 로 둔다 — 밀리면 최신 프레임만 본다.
        self._sub_cloud = ChannelSubscriber(TOPIC_CLOUD, PointCloud2_)
        self._sub_cloud.Init(self.on_cloud, 1)

        if self.cfg.verbose:
            print(f"[em] DDS 구독 시작 (domain={self.cfg.domain_id} iface={self.cfg.interface})",
                  flush=True)
            print(f"[em] 발행 토픽: {self.cfg.publish_topic}", flush=True)

    def run(self, duration: float | None = None, report_period: float = 2.0) -> None:
        try:
            self._run(duration, report_period)
        finally:
            # SIGINT 로 끝나는 게 보통이라 기록은 반드시 여기서 떨군다.
            self.save_record()

    def _run(self, duration: float | None, report_period: float) -> None:
        self.start_dds()
        t0 = time.time()
        last = t0
        while duration is None or time.time() - t0 < duration:
            time.sleep(0.1)
            if self.cfg.verbose and time.time() - last >= report_period:
                last = time.time()
                s = self.last_stats
                if not s:
                    with self.latest.lock:
                        nl, ns = self.latest.n_low, self.latest.n_sport
                    print(f"[em] 대기 중 — lowstate {nl}건 sportmodestate {ns}건, 점군 0건",
                          flush=True)
                    continue
                obs = self.h_obs[self.valid_frac > 1e-6]
                print(
                    f"[em] tick {self.n_ticks:5d}  점 {s['n_in']:.0f}"
                    f"(자기몸 {s['n_self']:.0f} 제외 → {s['n_used']:.0f})  "
                    f"관측셀 {s['valid_cells']:.0f}/{self.num_points} "
                    f"상한셀 {s['ub_cells']:.0f}  "
                    f"h[관측] "
                    + (f"평균 {obs.mean():+.3f} 범위 [{obs.min():+.3f}, {obs.max():+.3f}]"
                       if obs.size else "-"),
                    flush=True,
                )


def _load_index(deploy_yaml: Path, name: str) -> np.ndarray:
    """계약 파일의 index_maps.<name> 을 읽는다 (하드코딩 금지)."""
    import yaml

    with open(deploy_yaml) as f:
        c = yaml.safe_load(f)
    return np.asarray(c["index_maps"][name], dtype=np.int64)


def _load_sdk_to_il(deploy_yaml: Path) -> np.ndarray:
    """SDK 모터 순서 → IsaacLab 관절 순서 색인. il_to_sdk[i] = IL i 번 관절의 SDK 인덱스."""
    return _load_index(deploy_yaml, "il_to_sdk")
