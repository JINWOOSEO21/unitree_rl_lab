"""Summarize recorded stationary Go2 odometry and IMU without robot access."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_go2_recording import read_rows


def orientation_summary(quaternions):
    q = np.asarray(quaternions, dtype=float)
    norms = np.linalg.norm(q, axis=1)
    if not np.isfinite(q).all() or np.any(norms < 1e-6):
        raise ValueError("Invalid quaternion")
    q = q / norms[:, None]
    w, x, y, z = q.T
    rpy = np.column_stack([
        np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
        np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    ])
    rpy = np.rad2deg(np.unwrap(rpy, axis=0))
    angle = np.rad2deg(2*np.arccos(np.clip(np.abs(q @ q[0]), 0, 1)))
    return rpy, {"initial_rpy_deg": rpy[0].tolist(),
                 "rpy_peak_to_peak_deg": np.ptp(rpy, axis=0).tolist(),
                 "rpy_end_minus_start_deg": (rpy[-1]-rpy[0]).tolist(),
                 "max_rotation_from_start_deg": float(angle.max())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    sport, st = read_rows(args.recording / "sportmodestate.jsonl")
    low, lt = read_rows(args.recording / "lowstate.jsonl")
    pos = np.asarray([r["position"] for r in sport])
    vel = np.asarray([r["velocity"] for r in sport])
    height = np.asarray([r["body_height"] for r in sport])
    ts = (st-st[0])*1e-9
    tl = (lt-lt[0])*1e-9
    if not np.isfinite(pos).all() or not np.isfinite(vel).all():
        raise ValueError("Nonfinite sport state")
    rpy, imu = orientation_summary([r["imu_state"]["quaternion"] for r in low])
    displacement = pos-pos[0]
    integrated = np.sum((vel[1:]+vel[:-1])*.5*np.diff(ts)[:, None], axis=0)
    report = {
        "scope": "Robot stationary per user; measurements are estimator outputs, not surveyed physical motion",
        "sport_count": len(sport), "lowstate_count": len(low), "sport_duration_s": float(ts[-1]),
        "position": {"initial_m": pos[0].tolist(), "median_m": np.median(pos, axis=0).tolist(),
            "peak_to_peak_mm": (np.ptp(pos, axis=0)*1000).tolist(),
            "std_mm": (pos.std(axis=0)*1000).tolist(),
            "end_minus_start_mm": (displacement[-1]*1000).tolist(),
            "max_distance_from_start_mm": float(np.linalg.norm(displacement, axis=1).max()*1000)},
        "velocity": {"mean_m_s": vel.mean(axis=0).tolist(),
            "std_m_s": vel.std(axis=0).tolist(),
            "min_m_s": vel.min(axis=0).tolist(), "max_m_s": vel.max(axis=0).tolist(),
            "integral_of_reported_components_m": integrated.tolist(),
            "integral_caveat": "Componentwise integral by PC callback time; coordinate frame and delay unverified. Not used as trajectory."},
        "body_height": {"median_m": float(np.median(height)),
                        "peak_to_peak_mm": float(np.ptp(height)*1000)},
        "imu_orientation": imu,
        "sport_error_codes": sorted({r["error_code"] for r in sport}),
        "limitations": ["Stationary consistency does not establish absolute pose accuracy or moving drift.",
                        "Sport position, body_height and IMU body origin need not share a reference point.",
                        "Recorded PC callback timestamps do not measure acquisition-time synchronization."],
    }
    (args.out/"odometry_summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), constrained_layout=True)
    for i, (axis, color) in enumerate(zip("xyz", ["#d84b43", "#199563", "#397cc4"])):
        axes[0].plot(ts, displacement[:,i]*1000, label=axis, color=color, lw=.8)
        axes[2].plot(ts, vel[:,i], label=axis, color=color, lw=.6)
    for i, name in enumerate(["roll", "pitch", "yaw"]):
        axes[1].plot(tl, rpy[:,i]-rpy[0,i], label=name, lw=.8)
    axes[0].set_ylabel("position change (mm)")
    axes[0].set_title("Sport odometry position relative to first sample")
    axes[1].set_ylabel("orientation change (deg)")
    axes[1].set_title("LowState IMU orientation relative to first sample")
    axes[2].set_ylabel("reported velocity (m/s)")
    axes[2].set_title("Sport velocity: compare against nearly constant position")
    for ax in axes:
        ax.set_xlabel("elapsed PC callback time (s)");ax.legend(loc="upper right");ax.grid(alpha=.25)
    fig.suptitle("Go2: stationary standing, recorded 15 s | estimator consistency, not absolute accuracy")
    fig.savefig(args.out/"odometry_stationary.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
