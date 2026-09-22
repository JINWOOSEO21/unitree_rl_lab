"""사이드카 기록(npz) 한 개 → 추정 odometry 와 GT 의 차이 그림 (xy 와 z 를 나눠서).

    python eval/plot_odom_error.py em_ticks.npz --out videos/ramp15/mit_odom_error.png [--title "..."]

기록의 base_pos 는 지도에 쓴 위치(leg/mit 추정), gt_pos 는 sportmodestate(MuJoCo GT) 의 base 원점이다.
둘 다 같은 시작점에 seed 되어 있으므로 차이는 순수 드리프트다. 세 패널:
  위   : 위쪽에서 본 xy 궤적 (GT vs 추정)
  가운데: x, y 오차와 xy 거리 오차 [cm] 대 시간
  아래 : z 오차 [cm] 대 시간
가운데·아래에는 GT x 로 판정한 램프 구간(오르막 / 꼭대기 / 내리막)을 띠로 깔아 지형과 오차의 상관을 본다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

BLUE, ORANGE, AQUA, GRAY = "#2a78d6", "#eb6834", "#1baf7a", "#52514e"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default=None)
    ap.add_argument(
        "--ramp-x",
        default="1.5,5.8,8.4,12.75",
        help="오르막 시작, 꼭대기 시작, 내리막 시작, 내리막 끝 x [m] (ramp15 기본값)",
    )
    a = ap.parse_args()

    r = np.load(a.record)
    t = r["stamp"].astype(float)
    est = r["base_pos"].astype(float)
    gt = r["gt_pos"].astype(float)
    src = str(r["odom_source"]) if "odom_source" in r.files else "sport"
    ok = np.isfinite(gt).all(1)
    t, est, gt = t[ok], est[ok], gt[ok]
    # 걷기 시작(GT x 가 20 cm 전진한 때 − 0.5 s)을 0 초로. 기립 때도 base 가 몇 cm 움직이므로 거리 5 cm 로는 안 된다.
    go = max(0, int(np.argmax(gt[:, 0] > gt[0, 0] + 0.2)) - 5)
    t = t - t[go]
    d = est - gt
    dxy = np.hypot(d[:, 0], d[:, 1])
    walk = t >= 0

    # 램프 구간 → 시간 띠 (GT x 가 각 경계를 처음 넘는 시각)
    bx = [float(v) for v in a.ramp_x.split(",")]
    bt = [t[np.argmax(gt[:, 0] >= x)] if (gt[:, 0] >= x).any() else t[-1] for x in bx]
    bands = [
        (bt[0], bt[1], "up", "#e6eef9"),
        (bt[1], bt[2], "top", "#eeeeeb"),
        (bt[2], bt[3], "down", "#e6eef9"),
    ]

    fig, ax = plt.subplots(
        3, 1, figsize=(10, 10.5), gridspec_kw={"height_ratios": [1.3, 1, 1]}
    )
    fig.patch.set_facecolor("#fcfcfb")
    for x in ax:
        x.set_facecolor("#fcfcfb")
        x.grid(True, color="#e6e5e1", linewidth=0.8)
        for s in ("top", "right"):
            x.spines[s].set_visible(False)

    # --- 위: xy 궤적 ---
    ax[0].plot(gt[walk, 0], gt[walk, 1], color=BLUE, linewidth=2, label="GT (MuJoCo)")
    ax[0].plot(
        est[walk, 0],
        est[walk, 1],
        color=ORANGE,
        linewidth=2,
        label=f"estimated ({src})",
    )
    ax[0].plot(
        gt[go, 0],
        gt[go, 1],
        "o",
        color=GRAY,
        markersize=8,
        markerfacecolor="#fcfcfb",
        label="start",
    )
    ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_xlabel("x [m]  (spawn = 0, ramp 1.5–12.8 m)")
    ax[0].set_ylabel("y [m]")
    ax[0].legend(frameon=False, loc="upper left")
    ax[0].set_title(
        a.title or f"{Path(a.record).name}: {src} odometry vs GT",
        loc="left",
        fontsize=12,
    )

    for x in ax[1:]:
        for t0, t1, name, col in bands:
            x.axvspan(t0, t1, color=col, zorder=0)
            x.text(
                (t0 + t1) / 2,
                0.02,
                name,
                transform=x.get_xaxis_transform(),
                ha="center",
                va="bottom",
                fontsize=8,
                color=GRAY,
            )

    # --- 가운데: xy 오차 ---
    ax[1].plot(t[walk], d[walk, 0] * 100, color=BLUE, linewidth=1.6, label="Δx")
    ax[1].plot(t[walk], d[walk, 1] * 100, color=AQUA, linewidth=1.6, label="Δy")
    ax[1].plot(t[walk], dxy[walk] * 100, color=ORANGE, linewidth=2, label="|Δxy|")
    ax[1].axhline(0, color=GRAY, linewidth=0.8)
    ax[1].set_ylabel("xy error  est − GT [cm]")
    ax[1].legend(frameon=False, loc="upper left", ncol=3)
    ax[1].text(
        0.99,
        0.95,
        f"|Δxy|  median {np.median(dxy[walk]) * 100:.1f} cm   max {dxy[walk].max() * 100:.1f} cm   "
        f"final {dxy[walk][-1] * 100:.1f} cm  ({dxy[walk][-1] / max(np.hypot(*(gt[-1, :2] - gt[go, :2])), 1e-6) * 100:.1f} % of path)",
        transform=ax[1].transAxes,
        ha="right",
        va="top",
        fontsize=9,
        color=GRAY,
    )

    # --- 아래: z 오차 ---
    ax[2].plot(
        t[walk], d[walk, 2] * 100, color=ORANGE, linewidth=2, label="Δz  est − GT"
    )
    ax[2].axhline(0, color=GRAY, linewidth=0.8)
    ax[2].set_ylabel("z error [cm]")
    ax[2].set_xlabel("time since walking started [s]")
    ax[2].text(
        0.99,
        0.04,
        f"Δz  min {d[walk, 2].min() * 100:+.1f}   max {d[walk, 2].max() * 100:+.1f}   final {d[walk, 2][-1] * 100:+.1f} cm",
        transform=ax[2].transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        color=GRAY,
    )
    ax[2].legend(frameon=False, loc="lower left")

    fig.tight_layout()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=130)
    print(
        f"저장: {a.out}   |Δxy| median {np.median(dxy[walk]) * 100:.1f} / max {dxy[walk].max() * 100:.1f} cm, "
        f"Δz min {d[walk, 2].min() * 100:+.1f} / max {d[walk, 2].max() * 100:+.1f} cm"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
