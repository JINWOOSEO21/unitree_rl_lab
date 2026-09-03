"""사이드카 파이프라인 게이트 — 알고 있는 지형에 대해 scandots 가 맞는지 본다.

DDS 도 MuJoCo 도 쓰지 않는다. 지형을 수식 z = f(x, y) 로 두고 점군을 직접
레이캐스트해 사이드카의 tick() 에 먹인 뒤, 나온 132 값을 정답과 대조한다.

    정답 = clip(base_z − f(px, py) − 0.3, ±1)

이렇게 하는 이유: EM 커널 자체는 학습과 **같은 코드**(vendored)라 다시 검증할
필요가 없다. 여기서 실제로 위험한 것은 그 주변 배관이다 —

    · 점군이 어느 프레임에 있는가 (센서 vs 월드)
    · 마운트 회전을 제대로 걸었는가
    · scandots 격자를 yaw 로 제대로 돌렸는가 (x 안쪽 12 / y 바깥쪽 11 순서 포함)
    · base_z 를 어디서 가져왔는가

이 중 하나만 틀려도 값이 조용히 어긋난다. 정답이 해석적으로 알려진 지형에
대고 재면 그게 전부 한 번에 드러난다.

지형은 앞쪽에 계단 하나를 둔다 — 평지만 쓰면 격자를 90° 돌려도 답이 같아서
방향 오류를 못 잡는다.

사용법:
    python -m em_sidecar.tests.test_pipeline_offline [--device cuda:0]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG.parent))

from em_sidecar.kinematics import quat_to_mat  # noqa: E402
from em_sidecar.sidecar import (  # noqa: E402
    IMU_SITE_IN_BASE,
    MOUNT_POS,
    MOUNT_QUAT,
    EmSidecar,
    SidecarCfg,
)

CONTRACT = PKG.parent / "contract"
DEFAULT_EMCUPY = Path.home() / "workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy"

# 로봇 기립 자세 (IsaacLab 관절 순서) — deploy.yaml 의 default_joint_pos.
Q_STAND_IL = np.array([0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5])

STEP_X = 1.2  # 계단이 시작되는 x [m]
STEP_H = 0.15  # 계단 높이 [m]


def terrain(x, y):
    """z = f(x, y). x >= STEP_X 에서 STEP_H 만큼 올라간 계단."""
    return np.where(np.asarray(x) >= STEP_X, STEP_H, 0.0)


def raycast(origin: np.ndarray, dirs: np.ndarray, t_max: float = 10.0) -> np.ndarray:
    """수식 지형에 대한 레이캐스트. 계단이 축정렬이라 구간별로 정확히 풀 수 있다.

    각 평면 z = h 와의 교점을 구하고, 그 교점이 실제로 그 평면의 영역
    (아래는 x < STEP_X, 위는 x >= STEP_X)에 있는지 확인해 가장 가까운 것을 쓴다.
    계단 옆면(수직면 x = STEP_X)도 함께 푼다.
    """
    o = origin
    best = np.full(dirs.shape[0], np.inf)

    for h, lo, hi in ((0.0, -np.inf, STEP_X), (STEP_H, STEP_X, np.inf)):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (h - o[2]) / dirs[:, 2]
        x = o[0] + t * dirs[:, 0]
        good = (t > 1e-3) & (t < t_max) & (x >= lo) & (x < hi)
        best = np.where(good & (t < best), t, best)

    # 계단의 수직 옆면: x = STEP_X, 0 <= z <= STEP_H
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (STEP_X - o[0]) / dirs[:, 0]
    z = o[2] + t * dirs[:, 2]
    good = (t > 1e-3) & (t < t_max) & (z >= 0.0) & (z <= STEP_H)
    best = np.where(good & (t < best), t, best)

    hit = np.isfinite(best)
    return (dirs[hit] * best[hit, None]), hit


def make_cloud(rng, base_pos, base_quat, n_rays=6000):
    """센서 프레임 점군을 만든다 — L1 처럼 아래쪽 반구를 넓게 훑는다."""
    R_base = quat_to_mat(base_quat)
    R_s = R_base @ quat_to_mat(MOUNT_QUAT)
    t_s = base_pos + R_base @ MOUNT_POS

    # 월드에서 방향을 뽑는다 (아래쪽으로 치우친 구면 균등).
    v = rng.normal(size=(n_rays, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    v[:, 2] = -np.abs(v[:, 2]) * 0.9 - 0.05  # 대부분 아래를 본다
    v /= np.linalg.norm(v, axis=1, keepdims=True)

    pts_w_rel, hit = raycast(t_s, v)
    # 센서 프레임으로: p_s = R_sᵀ (p_w − t_s), 여기서 p_w − t_s 가 곧 pts_w_rel
    return pts_w_rel @ R_s


def evaluate(sc, yaw_deg: float, ticks: int, rng_seed: int = 0):
    """yaw 를 주고 그 방향으로 전진시킨 뒤, 마지막 tick 의 오차를 구간별로 나눈다."""
    q_sdk = np.zeros(12)
    q_sdk[sc.il_to_sdk] = Q_STAND_IL
    yaw = np.deg2rad(yaw_deg)
    bq = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
    rng = np.random.default_rng(rng_seed)
    bz = 0.33
    bp = np.array([0.0, 0.0, bz])
    R_base = quat_to_mat(bq)
    for k in range(ticks):
        d = 0.6 * k / max(ticks - 1, 1)
        bp = np.array([d * np.cos(yaw), d * np.sin(yaw), bz])
        # tick() 은 실기/시뮬레이터와 같은 것을 받는다 — base 원점이 아니라
        # odometry 가 보고하는 점(= base + R·IMU_SITE_IN_BASE)이다.
        sc.tick(make_cloud(rng, bp, bq), q_sdk, bq, bp + R_base @ IMU_SITE_IN_BASE)

    cy, sy = np.cos(yaw), np.sin(yaw)
    ox, oy = sc.scan_xy[:, 0], sc.scan_xy[:, 1]
    px = bp[0] + cy * ox - sy * oy
    py = bp[1] + sy * ox + cy * oy
    want = np.clip(bz - terrain(px, py) - 0.3, -1.0, 1.0)
    seen = sc.valid_frac > 1e-6
    err = np.abs(sc.h_obs - want)
    d_edge = px - STEP_X
    # 모서리에서 떨어진 셀 — 여기서 틀리면 배관이 틀린 것이다.
    far = seen & (np.abs(d_edge) > 0.20)
    near = seen & (np.abs(d_edge) <= 0.20)
    on_step = seen & (d_edge > 0.20)
    return dict(seen=int(seen.sum()), err=err, far=far, near=near, on_step=on_step,
                far_max=float(err[far].max()) if far.any() else 0.0,
                near_max=float(err[near].max()) if near.any() else 0.0,
                mean=float(err[seen].mean()) if seen.any() else np.inf,
                h_step=float(sc.h_obs[on_step].mean()) if on_step.any() else np.nan,
                want_step=float(bz - STEP_H - 0.3))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--emcupy-root", default=str(DEFAULT_EMCUPY))
    ap.add_argument("--ticks", type=int, default=25)
    ap.add_argument("--tol-far-cm", type=float, default=0.5,
                    help="모서리에서 20cm 이상 떨어진 셀의 허용 최대오차")
    ap.add_argument("--tol-mean-cm", type=float, default=2.0)
    a = ap.parse_args()

    print(f"지형: x >= {STEP_X} m 에서 {STEP_H*100:.0f} cm 계단")
    print(f"로봇: base_z 0.33 m, 각 yaw 방향으로 0 → 0.6 m 전진\n")
    print(f"{'yaw':>5} {'관측셀':>7} {'평균':>9} {'모서리밖 최대':>14} {'모서리±20cm 최대':>18}")

    fails: list[str] = []
    results = {}
    for yaw_deg in (0.0, 45.0, 90.0, 180.0):
        sc = EmSidecar(SidecarCfg(contract_dir=CONTRACT, emcupy_root=Path(a.emcupy_root),
                                  device=a.device, verbose=False))
        r = evaluate(sc, yaw_deg, a.ticks)
        results[yaw_deg] = r
        print(f"{yaw_deg:5.0f} {r['seen']:5d}/132 {r['mean']*100:8.4f}cm "
              f"{r['far_max']*100:12.4f}cm {r['near_max']*100:16.4f}cm")
        if r["far_max"] * 100 > a.tol_far_cm:
            fails.append(f"yaw {yaw_deg:.0f}: 모서리 밖 오차 {r['far_max']*100:.3f} cm")
        if r["mean"] * 100 > a.tol_mean_cm:
            fails.append(f"yaw {yaw_deg:.0f}: 평균오차 {r['mean']*100:.3f} cm")

    # 방향이 실제로 맞는지 — 이게 없으면 평지만 보고 통과할 수 있다.
    print()
    r0 = results[0.0]
    n_step0 = int(r0["on_step"].sum())
    print(f"yaw   0°: 계단 위 셀 {n_step0}개, h 평균 {r0['h_step']:+.4f} "
          f"(정답 {r0['want_step']:+.4f})")
    if n_step0 < 10:
        fails.append("yaw 0 에서 계단을 못 봤다 — 격자 x 방향 의심")
    elif abs(r0["h_step"] - r0["want_step"]) > 0.005:
        fails.append(f"yaw 0 계단 높이 {r0['h_step']:+.4f} != {r0['want_step']:+.4f}")

    n_step90 = int(results[90.0]["on_step"].sum())
    print(f"yaw  90°: 계단 위 셀 {n_step90}개 (격자가 yaw 를 따라 돌면 0 이어야 한다)")
    if n_step90 != 0:
        fails.append("yaw 90 에서 계단이 격자에 들어왔다 — yaw 회전 누락 의심")

    print()
    for f in fails:
        print(f"  FAIL: {f}")
    ok = not fails
    print(f"[RESULT] {'OK' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
