"""leg odometry 라이브 점검 — 사이드카 기록(--odom leg --record)의 추정 위치를 GT 와 대조.

MuJoCo 에서는 브리지가 rt/sportmodestate 로 GT 를 계속 흘리므로, 사이드카가 leg 모드로
달리면서 그 GT 를 기록해 둔다 (base_pos = leg 추정, gt_pos = sport GT, 둘 다 base 원점).
같은 지표를 오프라인 게이트(em_sidecar/tests/test_leg_odometry.py)와 공유한다 — 학습의
odometry 노이즈 모델이 상정한 크기 안에 드는가.

    python tools/odom_check.py em_ticks_a1.npz [em_ticks_a2.npz ...]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from em_sidecar.leg_odometry import format_stats, increment_error_stats  # noqa: E402


def check(path: str) -> dict[str, float] | None:
    r = np.load(path)
    if "gt_pos" not in r.files:
        print(f"{path}: gt_pos 없음 — leg 모드 기록이 아니다")
        return None
    n = min(len(r["stamp"]), len(r["base_pos"]), len(r["gt_pos"]))  # 저장 중 경쟁 대비
    gt = r["gt_pos"][:n].astype(np.float64)
    # est_pos 가 있으면(leg/그림자) 그것, 없으면 지도가 쓴 base_pos
    est_all = r["est_pos"][:n].astype(np.float64) if "est_pos" in r.files else r["base_pos"][:n].astype(np.float64)
    ok = np.isfinite(gt).all(axis=1) & np.isfinite(est_all).all(axis=1)
    if ok.sum() < 3:
        print(f"{path}: GT/추정 표본 부족 ({ok.sum()})")
        return None
    t = r["stamp"][:n][ok].astype(np.float64)
    est = est_all[ok]
    st = increment_error_stats(t, est, gt[ok])
    print(f"\n{Path(path).name}  (odom_source={str(r['odom_source']) if 'odom_source' in r.files else '?'})")
    print(format_stats(st))
    return st


def main() -> int:
    rows = [s for s in (check(p) for p in sys.argv[1:]) if s and "scale_mean" in s]
    if len(rows) > 1:
        print("\n=== 전체 ===")
        for k, lab, mul in [("scale_mean", "scale 평균 %", 100), ("window_err_xy_p95", "3.2m 창 p95 cm", 100),
                            ("tick_err_xy_mean", "0.1s 증분 평균 cm", 100), ("total_drift_xy", "전체 xy cm", 100)]:
            v = np.array([r[k] for r in rows]) * mul
            print(f"  {lab:<18} 중앙값 {np.median(v):+7.2f}   범위 [{v.min():+.2f}, {v.max():+.2f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
