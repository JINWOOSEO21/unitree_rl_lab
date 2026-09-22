"""여러 기록(npz)의 odometry 오차를 한 그림에 겹친다 — 지형 하나에 대해 leg vs mit, 시도별.

    python eval/plot_odom_compare.py --out fig.png --title "ramp15" \
        --leg a1.npz a2.npz a3.npz --mit b1.npz b2.npz b3.npz [--ramp-x 1.5,5.8,8.4,12.75]

패널: (1) |Δxy| [cm] 대 시간, (2) Δz [cm] 대 시간. 시간 0 = 걷기 시작(GT x 20 cm 전진 − 0.5 s).
같은 출처는 같은 색, 시도는 선 굵기/투명도만 다르다. 전도한 시도는 전도 시각까지만 그린다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"leg": "#eb6834", "mit": "#2a78d6"}
GRAY = "#52514e"


def load(path: str):
    r = np.load(path)
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
    d = est[keep] - gt[keep]
    return (
        t[keep] - t[go],
        np.hypot(d[:, 0], d[:, 1]) * 100,
        d[:, 2] * 100,
        gt[keep],
        fell.any(),
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--leg", nargs="*", default=[])
    ap.add_argument("--mit", nargs="*", default=[])
    ap.add_argument(
        "--ramp-x",
        default=None,
        help="구간 경계 x (오르막 시작, 꼭대기 시작, 내리막 시작, 끝)",
    )
    a = ap.parse_args()

    fig, ax = plt.subplots(2, 1, figsize=(10, 7.5), sharex=True)
    fig.patch.set_facecolor("#fcfcfb")
    for x in ax:
        x.set_facecolor("#fcfcfb")
        x.grid(True, color="#e6e5e1", linewidth=0.8)
        for s in ("top", "right"):
            x.spines[s].set_visible(False)
        x.axhline(0, color=GRAY, linewidth=0.8)

    summary = []
    band_done = False
    # 구간 띠는 완주한 시도 중 가장 빠른 것(대개 mit) 기준으로 그린다 — mit 을 먼저 처리한다.
    for src, files in (("mit", a.mit), ("leg", a.leg)):
        for k, f in enumerate(files):
            t, dxy, dz, gt, fell = load(f)
            lw = 2.0 if k == 0 else 1.4
            lab = f"{src}" if k == 0 else None
            ax[0].plot(
                t, dxy, color=COLORS[src], linewidth=lw, alpha=0.9 - 0.2 * k, label=lab
            )
            ax[1].plot(
                t, dz, color=COLORS[src], linewidth=lw, alpha=0.9 - 0.2 * k, label=lab
            )
            if fell:
                ax[0].plot(t[-1], dxy[-1], "x", color=COLORS[src], markersize=8)
                ax[1].plot(t[-1], dz[-1], "x", color=COLORS[src], markersize=8)
            summary.append(
                (
                    src,
                    Path(f).name,
                    t[-1],
                    np.median(dxy),
                    dxy.max(),
                    dz.min(),
                    dz.max(),
                    fell,
                    gt[-1, 0] - gt[0, 0],
                )
            )
            if a.ramp_x and not band_done and not fell:
                bx = [float(v) for v in a.ramp_x.split(",")]
                bt = [
                    t[np.argmax(gt[:, 0] >= x)] if (gt[:, 0] >= x).any() else t[-1]
                    for x in bx
                ]
                for x in ax:
                    x.axvspan(bt[0], bt[1], color="#e6eef9", zorder=0)
                    x.axvspan(bt[1], bt[2], color="#eeeeeb", zorder=0)
                    x.axvspan(bt[2], bt[3], color="#e6eef9", zorder=0)
                for name, (t0, t1) in zip(("up", "top", "down"), zip(bt[:-1], bt[1:])):
                    ax[1].text(
                        (t0 + t1) / 2,
                        0.02,
                        name,
                        transform=ax[1].get_xaxis_transform(),
                        ha="center",
                        va="bottom",
                        fontsize=8,
                        color=GRAY,
                    )
                band_done = True
    ax[0].set_ylabel("|Δxy|  est − GT [cm]")
    ax[1].set_ylabel("Δz  est − GT [cm]")
    ax[1].set_xlabel("time since walking started [s]   (× = fell)")
    ax[0].legend(frameon=False, loc="upper left")
    ax[0].set_title(a.title, loc="left", fontsize=12)
    fig.tight_layout()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=130)
    print(f"저장: {a.out}")
    for s in summary:
        print(
            f"  {s[0]:3s} {s[1]:26s} t {s[2]:5.1f} s  dist {s[8]:5.2f} m  |Δxy| p50 {s[3]:6.1f} max {s[4]:6.1f} cm  "
            f"Δz [{s[5]:+6.1f}, {s[6]:+6.1f}] cm  {'fell' if s[7] else 'ok'}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
