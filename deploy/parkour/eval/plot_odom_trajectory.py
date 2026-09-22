"""기록(npz) 한 개 → 추정 odometry 궤적을 GT 와 함께 그린다 (xy 와 z 를 나눠서).

    python eval/plot_odom_trajectory.py rec.npz --out traj.png [--title ...] [--ramp-x 1.5,3.6,6.2,8.35]

패널
  1. 위에서 본 xy 궤적 — GT(파랑) vs 추정(주황)
  2. z 대 시간 — GT vs 추정
  3. 오차 — |Δxy| 와 Δz 대 시간 (참고)
시간 0 = 걷기 시작(GT x 20 cm 전진 − 0.5 s). 전도한 실행은 전도 시각까지만 그린다.
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
        default=None,
        help="오르막 시작, 꼭대기 시작, 내리막 시작, 내리막 끝 x [m]",
    )
    a = ap.parse_args()

    r = np.load(a.record)
    src = str(r["odom_source"]) if "odom_source" in r.files else "sport"
    t, est, gt = (
        r["stamp"].astype(float),
        r["base_pos"].astype(float),
        r["gt_pos"].astype(float),
    )
    ok = np.isfinite(gt).all(1)
    t, est, gt = t[ok], est[ok], gt[ok]
    go = max(0, int(np.argmax(gt[:, 0] > gt[0, 0] + 0.2)) - 5)
    raw = r["lowstate_raw"]
    q = raw[:, 13:17]
    tilt = np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1))
    fell = (tilt > 1.0) & (raw[:, 0] > t[go])
    t_end = raw[np.argmax(fell), 0] if fell.any() else t[-1]
    keep = (t >= t[go]) & (t <= t_end)
    t, est, gt = t[keep] - t[go], est[keep], gt[keep]
    d = est - gt
    dxy = np.hypot(d[:, 0], d[:, 1])

    fig, ax = plt.subplots(
        3, 1, figsize=(10, 11), gridspec_kw={"height_ratios": [1.2, 1, 1]}
    )
    fig.patch.set_facecolor("#fcfcfb")
    for x in ax:
        x.set_facecolor("#fcfcfb")
        x.grid(True, color="#e6e5e1", linewidth=0.8)
        for s in ("top", "right"):
            x.spines[s].set_visible(False)

    if a.ramp_x:
        bx = [float(v) for v in a.ramp_x.split(",")]
        bt = [
            t[np.argmax(gt[:, 0] >= x)] if (gt[:, 0] >= x).any() else t[-1] for x in bx
        ]
        for x in ax[1:]:
            x.axvspan(bt[0], bt[1], color="#e6eef9", zorder=0)
            x.axvspan(bt[1], bt[2], color="#eeeeeb", zorder=0)
            x.axvspan(bt[2], bt[3], color="#e6eef9", zorder=0)
            for name, (t0, t1) in zip(("up", "top", "down"), zip(bt[:-1], bt[1:])):
                if t1 > t0:
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
        for x in bx:
            ax[0].axvline(x, color="#d5d4cf", linewidth=0.8, zorder=0)

    # 1. xy 궤적
    ax[0].plot(gt[:, 0], gt[:, 1], color=BLUE, linewidth=2.2, label="GT")
    ax[0].plot(
        est[:, 0], est[:, 1], color=ORANGE, linewidth=2, label=f"estimated ({src})"
    )
    ax[0].plot(
        gt[0, 0],
        gt[0, 1],
        "o",
        color=GRAY,
        markersize=8,
        markerfacecolor="#fcfcfb",
        label="start",
    )
    if fell.any():
        ax[0].plot(gt[-1, 0], gt[-1, 1], "x", color=BLUE, markersize=9)
        ax[0].plot(est[-1, 0], est[-1, 1], "x", color=ORANGE, markersize=9)
    ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_xlabel("x [m]  (spawn = 0)")
    ax[0].set_ylabel("y [m]")
    ax[0].legend(frameon=False, loc="upper left")
    ax[0].set_title(
        a.title or f"{Path(a.record).name}: {src} vs GT", loc="left", fontsize=12
    )

    # 2. z 대 시간
    ax[1].plot(t, gt[:, 2], color=BLUE, linewidth=2.2, label="GT z")
    ax[1].plot(t, est[:, 2], color=ORANGE, linewidth=2, label=f"estimated z ({src})")
    if fell.any():
        ax[1].plot(t[-1], gt[-1, 2], "x", color=BLUE, markersize=9)
        ax[1].plot(t[-1], est[-1, 2], "x", color=ORANGE, markersize=9)
    ax[1].set_ylabel("base z [m]")
    ax[1].legend(frameon=False, loc="upper left")

    # 3. 오차
    ax[2].plot(t, dxy * 100, color=AQUA, linewidth=2, label="|Δxy|")
    ax[2].plot(t, d[:, 2] * 100, color=ORANGE, linewidth=2, label="Δz")
    ax[2].axhline(0, color=GRAY, linewidth=0.8)
    ax[2].set_ylabel("error  est − GT [cm]")
    ax[2].set_xlabel(
        "time since walking started [s]" + ("   (× = fell)" if fell.any() else "")
    )
    ax[2].legend(frameon=False, loc="upper left", ncol=2)
    ax[2].text(
        0.99,
        0.96,
        f"|Δxy| median {np.median(dxy) * 100:.1f} / max {dxy.max() * 100:.1f} cm    "
        f"Δz min {d[:, 2].min() * 100:+.1f} / max {d[:, 2].max() * 100:+.1f} cm",
        transform=ax[2].transAxes,
        ha="right",
        va="top",
        fontsize=9,
        color=GRAY,
    )

    fig.tight_layout()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=130)
    print(
        f"저장: {a.out}  ({'fell' if fell.any() else 'ok'}, {t[-1]:.1f} s, x {gt[0, 0]:.2f}→{gt[-1, 0]:.2f} m)  "
        f"|Δxy| p50 {np.median(dxy) * 100:.1f} max {dxy.max() * 100:.1f} cm, Δz min {d[:, 2].min() * 100:+.1f} cm"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
