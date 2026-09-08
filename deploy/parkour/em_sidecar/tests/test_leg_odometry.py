"""leg odometry 오프라인 게이트 — IsaacLab 트레이스의 정답 위치와 대조한다.

DDS 도 MuJoCo 도 GPU 도 안 쓴다. contract/long_trace.npz (IsaacLab PLAY, 20 s,
50 Hz) 에서 lowstate 가 줄 것과 같은 양만 꺼내 추정기에 흘리고, 나온 위치를
root_pos_w 와 비교한다:

    입력  joint_pos(IL 순서), root_quat_w, root_ang_vel_b, foot_contact_forces
    정답  root_pos_w

즉 **학습 세계의 정답**으로 직접 판정한다. 합격선은 학습의 odometry 노이즈 모델이
상정한 크기다 — 그 안에 들면 정책 입장에서는 학습 때 본 흔들림과 같다:

    진행방향 scale 오차   |평균| ≤ 3%          (bias ~ U(±0.03))
    0.1 s 증분 오차 xy    평균 ≤ 5%·|Δ| + 0.5 cm  (b σ 2% + walk 0.005 m)
    3.2 m 창 누적 오차 xy p95 ≤ 20 cm          (창 안 국소 정합성)

사용법:
    python -m em_sidecar.tests.test_leg_odometry [--trace PATH] [--contact-thr N] [--foot-radius M]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG.parent))

from em_sidecar.kinematics import Go2Kinematics  # noqa: E402
from em_sidecar.leg_odometry import (  # noqa: E402
    LegOdomCfg,
    LegOdometry,
    format_stats,
    increment_error_stats,
)

CONTRACT = PKG.parent / "contract"
POLICY_DT = 0.02  # IsaacLab 정책 주기 (sim 0.005 s × decimation 4)


def episodes(ep_len: np.ndarray) -> list[tuple[int, int]]:
    """episode_length 가 줄어드는 곳에서 자른다 → [(start, end), ...]."""
    cuts = np.where(np.diff(ep_len) < 0)[0] + 1
    bounds = [0, *cuts.tolist(), len(ep_len)]
    return [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]


def run_episode(kin, g, s, e, cfg) -> dict[str, float]:
    q = g["joint_pos"][s:e, 0].astype(np.float64)
    quat = g["root_quat_w"][s:e, 0].astype(np.float64)
    gyro = g["root_ang_vel_b"][s:e, 0].astype(np.float64)
    ff = g["foot_contact_forces"][s:e, 0].astype(np.float64)  # (N,4,3) IL 발 순서
    gt = g["root_pos_w"][s:e, 0].astype(np.float64)
    n = e - s
    t = np.arange(n) * POLICY_DT
    force = np.linalg.norm(ff, axis=2)  # lowstate.foot_force 에 해당

    odo = LegOdometry(kin, cfg)
    odo.reset(gt[0])
    est = np.zeros_like(gt)
    for k in range(n):
        est[k] = odo.step(t[k], q[k], quat[k], gyro[k], force[k])
    st = increment_error_stats(t, est, gt)
    st["flight_frac"] = odo.n_flight / max(1, odo.n_steps)
    return st


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", default=str(CONTRACT / "long_trace.npz"))
    ap.add_argument("--contract-dir", default=str(CONTRACT))
    d = LegOdomCfg()
    ap.add_argument("--contact-thr", type=float, default=d.contact_force_thr, help="[N]")
    ap.add_argument("--foot-radius", type=float, default=d.foot_radius,
                    help="[m] 발 구름 보정 반지름. 0 이면 보정 없음")
    ap.add_argument("--weighted", action="store_true", help="접촉력 가중 평균 (기본은 균등)")
    ap.add_argument("--max-scale", type=float, default=0.03)
    ap.add_argument("--max-window-p95", type=float, default=0.20)
    a = ap.parse_args()

    kin = Go2Kinematics(Path(a.contract_dir) / "em_geometry.npz")
    g = np.load(a.trace)
    ep = g["episode_length"][:, 0]
    cfg = LegOdomCfg(contact_force_thr=a.contact_thr, foot_radius=a.foot_radius,
                     weight_by_force=a.weighted, max_dt_s=POLICY_DT * 1.5)
    print(f"설정: 접촉 임계 {cfg.contact_force_thr} N, 발 반지름 {cfg.foot_radius*100:.2f} cm, "
          f"{'힘가중' if cfg.weight_by_force else '균등'} 평균, 안착 {cfg.contact_settle_s*1e3:.0f} ms")

    ok = True
    for i, (s, e) in enumerate(episodes(ep)):
        if e - s < 50:
            continue
        st = run_episode(kin, g, s, e, cfg)
        print(f"\n[에피소드 {i}] 프레임 {s}..{e}  도약 비율 {st['flight_frac']*100:.1f}%")
        print(format_stats(st))
        step = st["path_len"] / st["n_ticks"]
        crit = [
            ("scale |평균| ≤ 3%", abs(st["scale_mean"]) <= a.max_scale),
            ("0.1s 증분 xy 평균 ≤ 5%·|Δ|+0.5cm",
             st["tick_err_xy_mean"] <= 0.05 * step + 0.005),
            ("3.2m 창 p95 ≤ 20cm", st["window_err_xy_p95"] <= a.max_window_p95),
        ]
        for name, passed in crit:
            print(f"    {'OK ' if passed else 'NG '} {name}")
            ok &= passed

    print(f"\n[RESULT] {'OK' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
