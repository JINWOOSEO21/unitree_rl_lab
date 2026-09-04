"""라이브 관측(State_Parkour 녹화) vs 학습 관측(골든 트레이스) — 채널별 대조.

왜 필요한가
-----------
골든 게이트 4종은 **녹화된 입력**으로 조립·추론을 검증한다. 그래서 폐루프에서만
생기는 문제(어떤 채널이 학습 분포를 벗어나는 것)는 못 잡는다. 실제로 정책이
평지에서 1.2~1.5 m 걷다가 옆으로 넘어지는데, 지형도 토크 포화도 원인이 아니었다.

여기서는 정책이 **실제로 본 53채널**을 학습 때 값과 나란히 놓고, 분포가 어긋난
채널을 찾는다. 어긋난 채널이 곧 이식이 덜 된 곳이다.

주의: 골든 트레이스는 IsaacLab 의 **한 궤적**(100스텝, 2초)이라 분포 표본으로는
작다. 그래서 "학습 범위를 벗어났는가" 를 1차 신호로 보고, 스케일·부호·듀티비처럼
**궤적이 달라도 변하면 안 되는 성질**을 함께 본다.

    python tools/compare_live_obs.py --live /tmp/parkour_obs.bin
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TRACE = HERE.parent / "contract" / "golden_trace.npz"

# prop 53 의 의미 (학습 observations.py:70-86)
CHANNELS = [
    (0, 3, "gyro*0.25", "scale"),
    (3, 5, "roll,pitch", "angle"),
    (5, 6, "상수 0", "const"),
    (6, 8, "delta_yaw, delta_next", "heading"),
    (8, 10, "상수 0,0", "const"),
    (10, 11, "cmd_vx", "cmd"),
    (11, 13, "상수 1, 0", "const"),
    (13, 25, "joint_pos - default", "joint"),
    (25, 37, "joint_vel*0.05", "joint"),
    (37, 49, "직전 raw action", "action"),
    (49, 53, "contact - 0.5", "contact"),
]


def load_live(path: Path):
    raw = path.read_bytes()
    if raw[:4] != b"PKOB":
        raise SystemExit(f"magic 불일치: {raw[:4]!r}")
    (rec,) = struct.unpack_from("<i", raw, 4)
    body = np.frombuffer(raw, dtype=np.float32, offset=8)
    n = body.size // rec
    body = body[: n * rec].reshape(n, rec)
    return dict(t=body[:, 0], prop=body[:, 1:54], scan=body[:, 54:186],
                action=body[:, 186:198])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", required=True)
    ap.add_argument("--trace", default=str(TRACE))
    ap.add_argument("--skip", type=float, default=0.3,
                    help="시작 후 무시할 시간 [s] (기립 직후 과도구간)")
    ap.add_argument("--drop-last", type=float, default=0.5,
                    help="끝에서 버릴 시간 [s] (넘어진 뒤 구간)")
    a = ap.parse_args()

    L = load_live(Path(a.live))
    d = np.load(a.trace)
    G = d["prop"][:, 0, :]
    Gs = d["scan"][:, 0, :]

    t = L["t"]
    keep = (t >= t[0] + a.skip) & (t <= t[-1] - a.drop_last)
    P = L["prop"][keep]
    S = L["scan"][keep]
    print(f"라이브 {L['prop'].shape[0]} 스텝 중 {P.shape[0]} 사용 "
          f"({t[keep][0]:.1f}~{t[keep][-1]:.1f} s, {1/np.median(np.diff(t)):.1f} Hz)")
    print(f"학습   {G.shape[0]} 스텝\n")

    print(f"{'채널':22s} {'라이브 평균':>11s} {'학습 평균':>10s} "
          f"{'라이브 |max|':>12s} {'학습 |max|':>11s}  비고")
    flags = []
    for lo, hi, name, kind in CHANNELS:
        pl, gl = P[:, lo:hi], G[:, lo:hi]
        note = ""
        if kind == "const":
            if not np.allclose(pl, gl[0], atol=1e-6):
                note = "상수가 다르다!"
                flags.append(f"{name}: 상수 불일치 (라이브 {np.unique(pl)[:3]})")
        elif kind == "contact":
            # 접촉 듀티비 — 궤적이 달라도 보행이면 비슷해야 한다
            dl = (pl > 0).mean()
            dg = (gl > 0).mean()
            note = f"접촉률 라이브 {dl*100:.0f}% vs 학습 {dg*100:.0f}%"
            if abs(dl - dg) > 0.25:
                flags.append(f"{name}: 접촉률 {dl*100:.0f}% vs {dg*100:.0f}%")
        else:
            r = np.abs(pl).max() / max(np.abs(gl).max(), 1e-9)
            if r > 3.0 or r < 0.33:
                note = f"진폭비 {r:.2f}x"
                flags.append(f"{name}: 진폭이 학습의 {r:.2f}배")
        print(f"{name:22s} {pl.mean():11.4f} {gl.mean():10.4f} "
              f"{np.abs(pl).max():12.4f} {np.abs(gl).max():11.4f}  {note}")

    print(f"\n{'scan 132':22s} {S.mean():11.4f} {Gs.mean():10.4f} "
          f"{np.abs(S).max():12.4f} {np.abs(Gs).max():11.4f}")

    # 시간에 따른 변화 — 발산하는 채널이 있는가
    print("\n구간별 평균 (초반 / 중반 / 후반)")
    n = P.shape[0]
    seg = [P[: n // 3], P[n // 3: 2 * n // 3], P[2 * n // 3:]]
    for lo, hi, name, kind in CHANNELS:
        if kind == "const":
            continue
        vals = [f"{np.abs(s[:, lo:hi]).mean():7.4f}" for s in seg]
        drift = abs(float(vals[2]) - float(vals[0])) / max(float(vals[0]), 1e-6)
        mark = "  ← 발산" if drift > 1.0 else ""
        print(f"  {name:22s} " + "  ".join(vals) + mark)

    print()
    if flags:
        for f in flags:
            print(f"  의심: {f}")
    else:
        print("  53채널 모두 학습 분포와 같은 규모다 — 관측 조립은 문제가 아니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
