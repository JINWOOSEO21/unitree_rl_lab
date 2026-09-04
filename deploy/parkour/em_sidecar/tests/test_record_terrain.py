"""주행 기록 게이트: **달리는 동안** scandots 가 지형과 맞는가.

test_live_terrain 은 서 있는 로봇을 짧게 재기 때문에 "움직이면 지도가 어긋난다" 를
못 잡는다. 정책은 걷다가 갑자기 자세를 무너뜨리는데, 그때 헛것을 본 것인지
(지도 문제) 아니면 제대로 보고도 못 한 것인지 (동역학 문제) 가리려면 주행 내내
자세와 함께 기록한 scandots 를 지형 정답과 대조해야 한다.

    python -m em_sidecar.run --record /tmp/em_ticks.npz &
    ...정책 주행...
    python -m em_sidecar.tests.test_record_terrain /tmp/em_ticks.npz

정답 = clip(base_z − terrain(px, py) − 0.3, ±1),
셀 중심 (px, py) 는 base 위치에서 yaw 만 돌린 오프셋 (사이드카와 같은 규약).
직접 관측(valid≈1) 셀만 채점한다 — 상한 대체 셀은 학습에도 있는 아티팩트다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent))

from em_sidecar.kinematics import yaw_from_quat  # noqa: E402
from em_sidecar.tests.test_live_terrain import META, Terrain  # noqa: E402

CONTRACT = HERE.parent.parent / "contract"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--tol-cm", type=float, default=3.0,
                    help="직접 관측 셀의 허용 평균 절대오차 [cm]")
    a = ap.parse_args()

    d = np.load(a.record)
    stamp, pos, quat = d["stamp"], d["base_pos"], d["base_quat"]
    scan, valid = d["scan"], d["valid"]
    off = np.load(CONTRACT / "em_geometry.npz")["scan_offsets_xy"]
    terr = Terrain(META)

    t = stamp - stamp[0]
    n = len(t)
    print(f"tick {n}개, {t[-1]:.1f}초, {n/max(t[-1],1e-9):.1f} Hz")
    print(f"이동 거리 {np.linalg.norm(pos[-1,:2]-pos[0,:2]):.2f} m  "
          f"x {pos[0,0]:+.2f} → {pos[-1,0]:+.2f}\n")

    rows = []
    for k in range(n):
        yaw = yaw_from_quat(quat[k])
        cy, sy = np.cos(yaw), np.sin(yaw)
        px = pos[k, 0] + cy * off[:, 0] - sy * off[:, 1]
        py = pos[k, 1] + sy * off[:, 0] + cy * off[:, 1]
        h = terr.height(px, py)
        want = np.clip(pos[k, 2] - h - 0.3, -1.0, 1.0)
        direct = (valid[k] > 0.999) & np.isfinite(want)
        if direct.sum() < 10:
            rows.append((t[k], pos[k, 0], np.nan, np.nan, int(direct.sum())))
            continue
        err = scan[k][direct] - want[direct]
        rows.append((t[k], pos[k, 0], float(np.abs(err).mean()),
                     float(np.abs(err).max()), int(direct.sum())))

    arr = np.array([(r[0], r[1], r[2], r[3], r[4]) for r in rows], dtype=np.float64)
    ok_rows = arr[np.isfinite(arr[:, 2])]
    print(f"{'t[s]':>6} {'base x':>8} {'평균오차[cm]':>12} {'최대[cm]':>9} {'직접셀':>7}")
    step = max(1, n // 30)
    for r in arr[::step]:
        if np.isnan(r[2]):
            print(f"{r[0]:6.1f} {r[1]:8.2f} {'-':>12} {'-':>9} {int(r[4]):7d}")
        else:
            print(f"{r[0]:6.1f} {r[1]:8.2f} {r[2]*100:12.2f} {r[3]*100:9.2f} {int(r[4]):7d}")

    if len(ok_rows) == 0:
        print("\n[RESULT] FAILED — 직접 관측 셀이 거의 없다")
        return 1
    mean_cm = ok_rows[:, 2].mean() * 100
    p95_cm = np.percentile(ok_rows[:, 2], 95) * 100
    worst = ok_rows[np.argmax(ok_rows[:, 2])]
    print(f"\n직접 관측 셀 평균 절대오차: 전체 평균 {mean_cm:.2f} cm, "
          f"p95 {p95_cm:.2f} cm")
    print(f"최악 tick: t={worst[0]:.1f}s  x={worst[1]:+.2f}  {worst[2]*100:.2f} cm")
    ok = mean_cm < a.tol_cm
    print(f"[RESULT] {'OK' if ok else 'FAILED'} — 기준 평균 {a.tol_cm} cm")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
