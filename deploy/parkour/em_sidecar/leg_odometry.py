"""다리 운동학 + IMU 로 base 위치를 적분한다 — 순수 numpy.

왜 있는가
---------
정책이 쓰는 양 중 배포에서 출처가 비는 것은 **base 위치(xy, z)** 하나뿐이다.
  · 선속도  → 정책 안의 estimator MLP (prop → 9) 가 낸다. 외부 추정 불필요.
  · 자세/yaw → rt/lowstate.imu_state.quaternion. sport 서비스와 무관하게 항상 온다.
  · 위치    → MuJoCo 에서는 브리지가 GT 를 rt/sportmodestate 로 흘려 줬다. 실기에서는
             그 토픽이 sport 서비스 것이라 저수준 제어(lowcmd) 중에 계속 나온다는
             보장이 없다. 그래서 여기서 직접 만든다.

위치를 쓰는 곳은 EM 사이드카뿐이고, 지도는 3.2 m 국소 창을 tick 마다 시프트하는
구조라 **절대 정확도가 아니라 몇 초 / 3 m 안의 국소 정합성**만 있으면 된다.
학습의 odometry 노이즈 모델도 절대 위치가 아니라 증분에 걸려 있다:

    Δ_meas = Δ_true · (1 + b + bias) + n     b~N(0,0.02)  bias~U(±0.03)  n: 0.005 m walk

이 추정기의 합격선은 그 범위 안에 드는 것이다 (tests/test_leg_odometry.py).

방법
----
접촉 중인 발은 지면에 정지해 있다고 본다. 발 i 의 base 프레임 위치 p_i(q) 를
기존 Go2Kinematics(IsaacLab 실측에서 역산한 기구학) 로 풀고, 시간 차분으로
ṗ_i 를 얻는다. 발이 월드에서 정지해 있으면

    0 = v_base_w + R (ṗ_i + ω × p_i)      →   v_base_w = −R (ṗ_i + ω × p_i)

접촉 발 전부의 평균을 base 속도로 쓰고 적분한다. Jacobian 은 필요 없다 — 시간
차분이 곧 J·q̇ 이다.

접촉 판정 — 착지 충격을 걸러야 한다
----------------------------------
발이 닿는 순간은 정지해 있지 않다. 충격으로 다리가 눌리며 발이 base 쪽으로 빠르게
움직이고, 그걸 "정지 발" 로 믿으면 base 가 −2~−4 m/s 로 떨어진다고 계산된다
(라이브에서 실제로 그렇게 나왔고, z 가 초당 −0.1 m 씩 흘렀다). 그래서 접촉이
contact_settle_s 이상 이어진 발만 쓰고, 믿는 발들은 **균등** 평균한다 (LegOdomCfg
weight_by_force 주석 참조).

믿을 발이 없을 때
----------------
직전 속도를 max_hold_s 동안 유지하고, 넘기면 0 으로 둔다. 트롯 보행에서 믿을 발이
비는 구간은 접촉 전환의 수십 ms 뿐이고, 그보다 길면 엎드려 있거나(Passive)
넘어진 것이다. 가속도계는 **쓰지 않는다**: 시뮬레이터에서는 접촉 채터로 |f| 가
튀어 정지 상태가 "비행" 으로 오판되고, 그때 중력을 적분하면 z 가 흐른다 (엎드린
10 초 동안 1 m). 실기 가속도계도 편향이 있어 적분하지 않는 편이 안전하다. 진짜
비행(도약) 동안 중력을 무시하는 대가는 0.1 s 도약에서 z 5 cm 이고 착지 후 다리
운동학이 다시 잡는다.

발 구름 보정 (빠뜨리면 −9% 로 짧게 잰다)
--------------------------------------
발 링크 원점은 구 중심이고, 지면에 정지한 것은 중심이 아니라 **접촉점**이다.
종아리가 기립 중 회전하면 구가 굴러 중심이 r·Δθ 만큼 앞으로 이동한다. 무미끄러짐
구름이면 중심 속도는 ω_foot × (r ẑ) 이므로

    v_base_w = −R (ṗ_i + ω × p_i) + r · (ω_foot,w × ẑ)

ω_foot,w 는 FK 가 주는 발 링크 회전의 시간 차분이다. IsaacLab 20 s 트레이스로 재면
보정 없이 −8.4/−9.1 % 이던 scale 오차가 r = 2.34 cm 에서 −0.05/−1.2 % 가 된다.
그 2.34 cm 는 정적 기립 높이 실측(dump_stand_height.py)에서 나온 IsaacLab 발
충돌 반지름과 같다 — MJCF 의 2.2 cm 가 아니다. ẑ 는 월드 수직을 쓴다(경사면
법선을 모르므로). 29° 램프에서 보정항의 12 % ≈ 속도의 1 % 오차로, 감수한다.

yaw 는 추정하지 않는다. 자세는 호출자가 주는 쿼터니언을 그대로 믿는다 (실기 IMU
의 yaw 드리프트는 학습의 gyro bias 모델 0.01~0.05 °/s 가 상정한 것이다).

입력 규약
---------
  q_il         관절각 12, **IsaacLab 순서** (사이드카가 SDK→IL 매핑을 먼저 한다)
  quat         (w, x, y, z) base 자세
  gyro_b       각속도 [rad/s], base 프레임
  acc_b        가속도계 [m/s²] — 받기만 하고 쓰지 않는다 (위 설명). 진단 기록용.
  foot_force_il 발 4개 접촉력 크기 [N], IsaacLab 발 순서 FL, FR, RL, RR
               (사이드카가 SDK 발 순서 FR, FL, RR, RL 에서 매핑한다)
  t            시각 [s]. lowstate.tick(ms) 이든 벽시계든 단조 증가하면 된다.

출력은 base **원점**의 월드(odom) 위치다 — imu site 가 아니다. 사이드카의
odom_offset_in_base 는 이 소스에서 0 이어야 한다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import Go2Kinematics, quat_to_mat

FOOT_LINKS_IL = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
GRAVITY_W = np.array([0.0, 0.0, -9.81])
Z_W = np.array([0.0, 0.0, 1.0])


@dataclass
class LegOdomCfg:
    # 발 접촉 임계 [N]. 학습 contact 판정은 2 N 이지만, 막 닿거나 막 떼는 발은
    # 월드에서 움직이고 있어 속도 평균을 오염시킨다 — 하중이 실린 발만 쓴다.
    # 실기 foot_force 는 원시 정수라 재보정 필요.
    contact_force_thr: float = 20.0
    # 발 구름 보정 반지름 [m]. IsaacLab 실측 2.34 cm (위 설명).
    foot_radius: float = 0.0234
    # 접촉력 가중 평균을 **쓰지 않는다**. IsaacLab 트레이스(50 Hz, 매끈한 힘)에서는
    # 조금 낫지만(창 p95 5.1 → 2.8 cm), MuJoCo lowstate(500 Hz) 에서는 힘이 0/10/74/59 N
    # 으로 튕기고 큰 힘은 정확히 발이 튀는 순간이라 가중이 움직이는 순간을 골라 뽑는다
    # (같은 보행에서 힘가중 전체 드리프트 xy 154 / z 86 cm → 균등 26 / 0.4 cm). 실기
    # 발 센서도 잡음이 있으므로 균등이 안전하다.
    weight_by_force: bool = False
    # (시도했다가 뺀 것) 직전 속도와 어긋나는 발을 exp(−|Δv|²/σ²) 로 낮추는 강건 평균:
    # 2 ms 단위 발 속도가 접촉 채터로 ±0.3 m/s 흔들려 판별력이 없고, σ≤0.5 에서
    # scale 이 −17~−34 % 로 무너졌다. 시간 평균이 먼저다.
    # 접촉이 이만큼 이어진 발만 정지 발로 믿는다 [s] (착지 충격 제외).
    contact_settle_s: float = 0.02
    # 접촉력 저역통과 시간상수 [s]. 시뮬레이터에서는 발이 표면에서 8 ms 주기로 튕겨
    # 힘이 (74, 59, 0, 10, 71, …) 로 찍히고 실기 센서도 잡음이 있다. 1~2 표본짜리 딥은
    # 평균 안에 묻히고, 진짜 liftoff 는 힘이 계속 줄어드니 몇 ms 안에 임계 아래로
    # 내려온다. 0 이면 끔.
    force_lp_tau_s: float = 0.006
    # 시간 디바운스 (저역통과 이전 방식): 힘이 임계 아래로 떨어져도 이 시간 안에 다시
    # 올라오면 접촉 유지. **막 뗀 발을 그만큼 오래 붙잡아** 스윙 가속(위·앞)이 base
    # 속도로 뒤집혀 들어간다 — MuJoCo 재생에서 z −7 cm/s, 진행 −2~−11 % 편향. 0 권장.
    contact_release_s: float = 0.0
    # 믿을 발이 없을 때 직전 속도를 유지하는 최대 시간 [s]. 넘기면 0.
    max_hold_s: float = 0.15
    # 표본 간격이 이보다 크면(드롭) 그 구간의 속도 갱신을 건너뛴다 [s].
    # 브리지 lowstate 는 리더 스레드가 EM tick 과 GIL 을 다투어 44~54 ms 구멍이
    # 생긴다. 차분식은 발이 계속 접촉해 있는 한 간격과 무관하게 정확하므로 넓게 둔다.
    max_dt_s: float = 0.1


class LegOdometry:
    def __init__(self, kin: Go2Kinematics, cfg: LegOdomCfg | None = None):
        self.kin = kin
        self.cfg = cfg or LegOdomCfg()
        self.foot_idx = np.array([kin.link_names.index(n) for n in FOOT_LINKS_IL])
        self.reset()

    def reset(self, pos: np.ndarray | None = None) -> None:
        self.pos = np.zeros(3) if pos is None else np.asarray(pos, dtype=np.float64).copy()
        self.vel = np.zeros(3)
        self._prev_t: float | None = None
        self._prev_feet_b: np.ndarray | None = None
        self._prev_Rfoot_w: np.ndarray | None = None
        self._prev_reliable: np.ndarray | None = None
        self._contact_since = np.full(4, np.nan)  # 발별 접촉 시작 시각
        self._last_above = np.full(4, -np.inf)     # 발별 마지막으로 임계를 넘은 시각
        self._force_lp: np.ndarray | None = None   # 저역통과된 접촉력
        self._t_last_contact: float | None = None
        self.n_steps = 0
        self.n_flight = 0
        # 마지막 step 의 진단 — 라이브 디버그 기록용
        self.last_dt = 0.0
        self.last_n_both = 0
        self.last_branch = 0  # 0 skip, 1 leg, 2 hold, 4 zero

    def feet_base(self, q_il: np.ndarray) -> np.ndarray:
        """발 4개의 base 프레임 위치 (4,3), IsaacLab 발 순서."""
        P, _ = self.kin.link_poses_base(q_il)
        return P[self.foot_idx]

    def step(
        self,
        t: float,
        q_il: np.ndarray,
        quat: np.ndarray,
        gyro_b: np.ndarray,
        foot_force_il: np.ndarray,
        acc_b: np.ndarray | None = None,
    ) -> np.ndarray:
        """표본 하나를 넣고 갱신된 base 원점 위치(월드)를 돌려준다."""
        P, RL = self.kin.link_poses_base(q_il)
        feet_b = P[self.foot_idx]
        R = quat_to_mat(quat)
        Rfoot_w = np.einsum("ij,kjl->kil", R, RL[self.foot_idx])  # (4,3,3) 발 링크 월드 회전
        force = np.asarray(foot_force_il, dtype=np.float64)
        # 접촉력 저역통과 (채터 딥 제거). 표본 간격은 호출자의 시각으로 잰다.
        if self.cfg.force_lp_tau_s > 0.0:
            if self._force_lp is None or self._prev_t is None:
                self._force_lp = force.copy()
            else:
                a = min(1.0, max(0.0, (t - self._prev_t) / self.cfg.force_lp_tau_s))
                self._force_lp = self._force_lp + a * (force - self._force_lp)
            f_use = self._force_lp
        else:
            f_use = force
        above = f_use > self.cfg.contact_force_thr
        self._last_above = np.where(above, t, self._last_above)
        # (선택) 시간 디바운스: 잠깐 임계 아래로 떨어진 것은 무시한다
        contact = (t - self._last_above) <= self.cfg.contact_release_s
        # 접촉 시작 시각 갱신 → 안착한(settle) 발만 믿는다
        self._contact_since = np.where(contact, np.where(np.isnan(self._contact_since), t,
                                                         self._contact_since), np.nan)
        reliable = contact & ((t - self._contact_since) >= self.cfg.contact_settle_s)

        if self._prev_t is None:
            self._remember(t, feet_b, Rfoot_w, reliable)
            return self.pos

        dt = t - self._prev_t
        self.last_dt = dt
        if dt <= 0.0 or dt > self.cfg.max_dt_s:
            # 시각이 안 흘렀거나 드롭 — 이 구간은 속도를 못 믿는다. 기준만 갱신.
            self._remember(t, feet_b, Rfoot_w, reliable)
            self.last_branch, self.last_n_both = 0, 0
            return self.pos

        # 차분이 유효하려면 발이 **두 표본 모두** 안착 접촉이어야 한다.
        both = reliable & self._prev_reliable
        self.last_n_both = int(both.sum())
        if both.any():
            self.last_branch = 1
            idx = np.where(both)[0]
            pdot_b = (feet_b[idx] - self._prev_feet_b[idx]) / dt
            w = np.asarray(gyro_b, dtype=np.float64)
            # 정지 접촉점: v_base_w = −R (ṗ_i + ω × p_i) + r (ω_foot,w × ẑ)
            v = -(R @ (pdot_b + np.cross(w, feet_b[idx])).T).T
            if self.cfg.foot_radius > 0.0:
                for j, i in enumerate(idx):
                    w_foot = _rotvec(Rfoot_w[i] @ self._prev_Rfoot_w[i].T) / dt
                    v[j] += self.cfg.foot_radius * np.cross(w_foot, Z_W)
            wt = force[idx] if self.cfg.weight_by_force else np.ones(len(idx))
            # 가중치는 **실제 힘**이다. 막 떼는 발은 힘이 0 으로 빠지며 저절로 가중치를
            # 잃는다 (해제 디바운스 동안 그 발의 스윙 속도가 base 속도로 뒤집혀 들어가는
            # 것을 막는다 — 하한을 두었을 때 z 가 −10 cm/s 로 흘렀다). 튕김으로 모든 발이
            # 순간 0 이면 이 표본은 못 믿으므로 직전 속도를 유지한다.
            if wt.sum() > 1e-6:
                self.vel = (v * wt[:, None]).sum(axis=0) / wt.sum()
                self._t_last_contact = t
            else:
                self.last_branch = 2
        else:
            self.n_flight += 1
            in_window = (self._t_last_contact is not None
                         and t - self._t_last_contact <= self.cfg.max_hold_s)
            if in_window:
                self.last_branch = 2            # 접촉 전환 — 직전 속도 유지
            else:
                self.vel = np.zeros(3)          # 엎드림/넘어짐 — 정지로 본다
                self.last_branch = 4

        self.pos = self.pos + self.vel * dt
        self.n_steps += 1
        self._remember(t, feet_b, Rfoot_w, reliable)
        return self.pos

    def _remember(self, t, feet_b, Rfoot_w, reliable) -> None:
        self._prev_t = t
        self._prev_feet_b = feet_b
        self._prev_Rfoot_w = Rfoot_w
        self._prev_reliable = reliable


def _rotvec(Rrel: np.ndarray) -> np.ndarray:
    """회전행렬 → 회전벡터 (축 × 각). 소각 근사가 아닌 일반식."""
    c = np.clip((np.trace(Rrel) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2.0 * np.sin(th)) * np.array(
        [Rrel[2, 1] - Rrel[1, 2], Rrel[0, 2] - Rrel[2, 0], Rrel[1, 0] - Rrel[0, 1]])


# ---------------------------------------------------------------------------
# 평가 — 추정 궤적을 정답 궤적과 **학습 노이즈 모델의 눈으로** 비교한다.
# 오프라인 테스트(IsaacLab 트레이스)와 라이브 점검(MuJoCo GT)이 같은 함수를 쓴다.
# ---------------------------------------------------------------------------
def increment_error_stats(
    t: np.ndarray,
    est: np.ndarray,
    gt: np.ndarray,
    tick_s: float = 0.1,
    window_m: float = 3.2,
) -> dict[str, float]:
    """증분 오차 통계.

    tick_s  : 학습 EM tick(0.1 s) 간격의 증분 Δ 를 비교한다.
    window_m: 지도 창 길이만큼 걸은 구간의 누적 오차 — 국소 정합성의 직접 지표.
    반환값의 단위는 전부 m 또는 비율.
    """
    t = np.asarray(t, dtype=np.float64)
    est = np.asarray(est, dtype=np.float64)
    gt = np.asarray(gt, dtype=np.float64)
    n = min(len(t), len(est), len(gt))
    t, est, gt = t[:n], est[:n], gt[:n]

    # tick 간격으로 재표본
    idx = [0]
    for i in range(1, n):
        if t[i] - t[idx[-1]] >= tick_s - 1e-9:
            idx.append(i)
    idx = np.array(idx)
    if len(idx) < 3:
        return {"n_ticks": float(len(idx))}
    d_est = np.diff(est[idx], axis=0)
    d_gt = np.diff(gt[idx], axis=0)
    err = d_est - d_gt
    err_xy = np.linalg.norm(err[:, :2], axis=1)
    mag_xy = np.linalg.norm(d_gt[:, :2], axis=1)
    moving = mag_xy > 0.01  # tick 당 1 cm 이상 움직인 구간에서만 비율을 잰다
    # 진행 방향 성분의 scale 오차 (학습의 b + bias 에 해당)
    scale = np.full(len(err), np.nan)
    if moving.any():
        u = d_gt[moving, :2] / mag_xy[moving, None]
        scale[moving] = np.einsum("ij,ij->i", d_est[moving, :2], u) / mag_xy[moving] - 1.0

    # 지도 창 길이만큼 걸은 구간의 누적 오차
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))])
    win_err = []
    j = 0
    for i in range(n):
        while j < n and s[j] - s[i] < window_m:
            j += 1
        if j >= n:
            break
        e = (est[j] - est[i]) - (gt[j] - gt[i])
        win_err.append(np.linalg.norm(e[:2]))
    win_err = np.array(win_err) if win_err else np.array([np.nan])

    total = est[-1] - gt[-1] - (est[0] - gt[0])
    return {
        "n_ticks": float(len(idx)),
        "tick_err_xy_mean": float(err_xy.mean()),
        "tick_err_xy_p95": float(np.percentile(err_xy, 95)),
        "tick_err_z_mean": float(np.abs(err[:, 2]).mean()),
        "scale_mean": float(np.nanmean(scale)),
        "scale_std": float(np.nanstd(scale)),
        "window_err_xy_mean": float(np.nanmean(win_err)),
        "window_err_xy_p95": float(np.nanpercentile(win_err, 95)),
        "total_drift_xy": float(np.linalg.norm(total[:2])),
        "total_drift_z": float(abs(total[2])),
        "path_len": float(s[-1]),
        "duration": float(t[-1] - t[0]),
    }


def format_stats(st: dict[str, float]) -> str:
    if "tick_err_xy_mean" not in st:
        return f"표본 부족 (tick {st.get('n_ticks', 0):.0f}개)"
    return (
        f"구간 {st['duration']:.1f}s / 경로 {st['path_len']:.2f}m / tick {st['n_ticks']:.0f}개\n"
        f"  0.1s 증분 오차 xy  평균 {st['tick_err_xy_mean']*100:.2f}cm  p95 {st['tick_err_xy_p95']*100:.2f}cm"
        f"   z 평균 {st['tick_err_z_mean']*100:.2f}cm\n"
        f"  진행방향 scale 오차  평균 {st['scale_mean']*100:+.2f}%  표준편차 {st['scale_std']*100:.2f}%"
        f"   (학습 가정: bias ±3%, b σ2%)\n"
        f"  3.2m 창 누적 오차 xy  평균 {st['window_err_xy_mean']*100:.1f}cm  p95 {st['window_err_xy_p95']*100:.1f}cm\n"
        f"  전체 드리프트  xy {st['total_drift_xy']*100:.1f}cm   z {st['total_drift_z']*100:.1f}cm"
    )
