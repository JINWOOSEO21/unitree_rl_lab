"""Replay a recorded Go2 cloud through EmSidecar without DDS or publishers.

The replay uses the latest state received before each cloud according to the
recording's local monotonic timestamps.  It is a diagnostic candidate run: the
mount pose is still the production sidecar's assumed Rx180 rotation and
[0.28, 0, 0.10] m translation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

PARKOUR_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PARKOUR_ROOT))

from em_sidecar.kinematics import quat_to_mat
from em_sidecar.pointcloud import decode_xyz
from em_sidecar.sidecar import EmSidecar, IMU_SITE_IN_BASE, SidecarCfg


MAX_QUAT_NORM_ERROR = 0.05


def _read_rows(path: Path) -> tuple[list[dict], np.ndarray]:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"no messages in {path}")
    times = np.asarray([row["steady_ns"] for row in rows], dtype=np.int64)
    if np.any(np.diff(times) < 0):
        raise ValueError(f"non-monotonic steady_ns in {path}")
    return rows, times


def _preceding(times: np.ndarray, at_ns: int) -> tuple[int, float]:
    index = int(np.searchsorted(times, at_ns, side="right")) - 1
    age_ms = float("nan") if index < 0 else (int(at_ns) - int(times[index])) * 1e-6
    return index, age_ms


def _eligible_preceding(
    times: np.ndarray, at_ns: int, max_age_ms: float
) -> tuple[int, float, str | None]:
    """Select a causal state and explain why it cannot be used, if so."""
    index, age_ms = _preceding(times, at_ns)
    if index < 0:
        return index, age_ms, "missing_preceding_state"
    if age_ms > max_age_ms:
        return index, age_ms, "state_too_old"
    return index, age_ms, None


def _stats(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def _valid_vector(value: np.ndarray, shape: tuple[int, ...]) -> bool:
    return value.shape == shape and bool(np.isfinite(value).all())


def replay(
    recording: Path,
    odom_offset: np.ndarray,
    emcupy_root: Path,
    device: str,
    max_state_age_ms: float,
) -> tuple[dict[str, np.ndarray], dict]:
    low, low_times = _read_rows(recording / "lowstate.jsonl")
    sport, sport_times = _read_rows(recording / "sportmodestate.jsonl")
    clouds, _ = _read_rows(recording / "cloud.jsonl")

    sidecar = EmSidecar(SidecarCfg(
        contract_dir=PARKOUR_ROOT / "contract",
        emcupy_root=emcupy_root,
        device=device,
        odom_offset_in_base=odom_offset.copy(),
        odom_source="sport",
        publish_diag=False,
        verbose=False,
    ))

    count = len(clouds)
    scan = np.full((count, sidecar.num_points), np.nan, dtype=np.float32)
    valid = np.full_like(scan, np.nan)
    upper_bound = np.full_like(scan, np.nan)
    base = np.full((count, 3), np.nan, dtype=np.float64)
    quat = np.full((count, 4), np.nan, dtype=np.float64)
    low_index = np.full(count, -1, dtype=np.int64)
    sport_index = np.full(count, -1, dtype=np.int64)
    low_age_ms = np.full(count, np.nan, dtype=np.float64)
    sport_age_ms = np.full(count, np.nan, dtype=np.float64)
    finite_points = np.zeros(count, dtype=np.int64)
    accepted = np.zeros(count, dtype=bool)
    reject_reason = np.full(count, "", dtype="U32")
    cloud_steady_ns = np.asarray([row["steady_ns"] for row in clouds], dtype=np.int64)
    sensor_stamp_s = np.asarray([
        row["stamp"]["sec"] + row["stamp"]["nanosec"] * 1e-9 for row in clouds
    ], dtype=np.float64)

    expected_offset = 0
    with (recording / "cloud.bin").open("rb") as binary:
        for cloud_index, row in enumerate(clouds):
            if int(row["binary_offset"]) != expected_offset:
                raise ValueError("cloud payload offsets have a gap or overlap")
            payload = binary.read(int(row["binary_size"]))
            if len(payload) != int(row["binary_size"]):
                raise ValueError("truncated cloud payload")
            expected_offset += len(payload)

            li, lage, low_reject = _eligible_preceding(
                low_times, int(row["steady_ns"]), max_state_age_ms
            )
            si, sage, sport_reject = _eligible_preceding(
                sport_times, int(row["steady_ns"]), max_state_age_ms
            )
            low_index[cloud_index], sport_index[cloud_index] = li, si
            low_age_ms[cloud_index], sport_age_ms[cloud_index] = lage, sage
            if low_reject == "missing_preceding_state" or sport_reject == "missing_preceding_state":
                reject_reason[cloud_index] = "missing_preceding_state"
                continue
            if low_reject == "state_too_old" or sport_reject == "state_too_old":
                reject_reason[cloud_index] = "state_too_old"
                continue

            q = np.asarray(low[li]["imu_state"]["quaternion"], dtype=np.float64)
            q_norm = float(np.linalg.norm(q))
            if (q.shape != (4,) or not np.isfinite(q).all() or not np.isfinite(q_norm)
                    or q_norm < 1e-6 or abs(q_norm - 1.0) > MAX_QUAT_NORM_ERROR):
                reject_reason[cloud_index] = "invalid_quaternion"
                continue
            q /= q_norm

            msg = SimpleNamespace(
                **{**row, "data": payload,
                   "fields": [SimpleNamespace(**field) for field in row["fields"]]}
            )
            points = decode_xyz(msg)
            finite_points[cloud_index] = len(points)
            if not len(points):
                reject_reason[cloud_index] = "empty_cloud"
                continue

            q_sdk = np.asarray(
                [motor["q"] for motor in low[li]["motor_state"][:12]], dtype=np.float64
            )
            raw_pos = np.asarray(sport[si]["position"], dtype=np.float64)
            if not _valid_vector(q_sdk, (12,)) or not _valid_vector(raw_pos, (3,)):
                reject_reason[cloud_index] = "invalid_state"
                continue
            corrected_base = raw_pos - quat_to_mat(q) @ odom_offset
            result = sidecar.tick(
                points_sensor=points,
                q_sdk=q_sdk,
                base_quat=q,
                base_pos=raw_pos,
                stamp=float(sensor_stamp_s[cloud_index]),
            )
            scan[cloud_index] = result
            valid[cloud_index] = sidecar.valid_frac
            upper_bound[cloud_index] = sidecar.ub_frac
            base[cloud_index] = corrected_base
            quat[cloud_index] = q
            accepted[cloud_index] = True

        if binary.read(1):
            raise ValueError("unindexed trailing bytes in cloud.bin")

    keep = accepted
    if not np.any(keep):
        raise ValueError("all cloud frames were rejected")
    for name, array in (("scan", scan[keep]), ("valid", valid[keep]),
                        ("upper_bound", upper_bound[keep]), ("base", base[keep]),
                        ("quat", quat[keep])):
        if not np.isfinite(array).all():
            raise ValueError(f"accepted {name} contains non-finite values")
    if np.any(np.abs(scan[keep]) > 1.000001):
        raise ValueError("accepted scan exceeds the policy [-1, 1] bounds")

    arrays = {
        "cloud_steady_ns": cloud_steady_ns,
        "sensor_stamp_s": sensor_stamp_s,
        "low_index": low_index,
        "sport_index": sport_index,
        "low_age_ms": low_age_ms,
        "sport_age_ms": sport_age_ms,
        "finite_points": finite_points,
        "accepted": accepted,
        "reject_reason": reject_reason,
        "base": base,
        "base_quat": quat,
        "scan": scan,
        "valid": valid,
        "upper_bound": upper_bound,
    }
    reasons, reason_counts = np.unique(reject_reason[~keep], return_counts=True)
    duration_s = (cloud_steady_ns[-1] - cloud_steady_ns[0]) * 1e-9
    metrics = {
        "scope": "Offline actual recording replay; no DDS initialization or publishing",
        "candidate_only": True,
        "mount_assumption": {"rotation": "Rx180", "translation_m": [0.28, 0.0, 0.10]},
        "odom_offset_in_base_m": odom_offset.tolist(),
        "pairing": "latest preceding local receipt timestamp",
        "timing_caveat": (
            "cloud steady_ns was recorded after its binary payload write; pair ages are local "
            "callback ordering diagnostics, not sensor acquisition or transport latency"
        ),
        "max_state_age_ms": max_state_age_ms,
        "cloud": {
            "received": count,
            "accepted": int(keep.sum()),
            "rejected": int((~keep).sum()),
            "reject_reasons": {str(k): int(v) for k, v in zip(reasons, reason_counts)},
            "input_callback_hz": (count - 1) / duration_s,
            "finite_points": _stats(finite_points[keep]),
        },
        "pair_age_ms": {
            "lowstate": _stats(low_age_ms[keep]),
            "sportmodestate": _stats(sport_age_ms[keep]),
        },
        "scan": {
            "shape": list(scan[keep].shape),
            "value": _stats(scan[keep]),
            "all_finite": bool(np.isfinite(scan[keep]).all()),
            "within_minus1_plus1": bool(np.all(np.abs(scan[keep]) <= 1.000001)),
        },
        "coverage": {
            "valid_fraction": _stats(valid[keep]),
            "upper_bound_fraction": _stats(upper_bound[keep]),
            "cells_with_any_direct_observation": _stats((valid[keep] > 1e-6).sum(axis=1)),
        },
        "limitations": [
            "The recording input rate is measured, not resampled to the 10 Hz training update rate.",
            "A flat-floor recording does not certify mount extrinsics, odometry origin, or control readiness.",
            "Candidate comparison changes only odometry offset; it preserves the assumed mount pose.",
        ],
    }
    return arrays, metrics


def _plot_final_frame(arrays: dict[str, np.ndarray], output: Path, candidate: str) -> None:
    """Plot the final policy scan and its observation source classification."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap

    accepted_indices = np.flatnonzero(arrays["accepted"])
    index = int(accepted_indices[-1])
    scan = arrays["scan"][index].reshape(11, 12)
    valid = arrays["valid"][index].reshape(11, 12)
    upper = arrays["upper_bound"][index].reshape(11, 12)
    source = np.zeros_like(valid, dtype=np.int8)
    source[upper > 1e-6] = 1
    source[valid > 1e-6] = 2
    uncovered = int((source == 0).sum())

    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    image = axes[0].imshow(scan, origin="lower", vmin=-1.0, vmax=1.0, cmap="coolwarm")
    axes[0].set_title("Policy scan observation (not ground truth)")
    fig.colorbar(image, ax=axes[0], label="clipped observation")

    image = axes[1].imshow(valid, origin="lower", vmin=0.0, vmax=1.0, cmap="viridis")
    axes[1].set_title(f"Direct observation fraction\n{int((valid > 1e-6).sum())}/132 cells")
    fig.colorbar(image, ax=axes[1], label="fraction")

    cmap = ListedColormap(["#d73027", "#fee08b", "#1a9850"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    axes[2].imshow(source, origin="lower", cmap=cmap, norm=norm)
    axes[2].set_title(f"Cell source: {uncovered} uncovered")
    for y, x in np.argwhere(source == 0):
        axes[2].text(x, y, "×", ha="center", va="center", color="white", fontsize=7)
    axes[2].text(
        0.5, -0.16, "red: uncovered   yellow: upper-bound   green: direct",
        transform=axes[2].transAxes, ha="center", va="top", fontsize=9,
    )
    for axis in axes:
        axis.set_xlabel("x-grid index (12)")
        axis.set_ylabel("y-grid index (11)")
    fig.suptitle(f"Offline Go2 replay: {candidate} odometry offset, final accepted frame")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--odom-offset", choices=("zero", "sim"), required=True)
    parser.add_argument("--emcupy-root", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--max-state-age-ms", type=float, default=20.0)
    parser.add_argument("--out-npz", type=Path, required=True)
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--out-plot", type=Path)
    args = parser.parse_args()
    if args.max_state_age_ms <= 0:
        parser.error("--max-state-age-ms must be positive")
    offset = np.zeros(3, dtype=np.float64) if args.odom_offset == "zero" else IMU_SITE_IN_BASE
    arrays, metrics = replay(
        args.recording, offset, args.emcupy_root, args.device, args.max_state_age_ms
    )
    args.out_npz.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out_npz, **arrays)
    args.out_json.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
    if args.out_plot is not None:
        _plot_final_frame(arrays, args.out_plot, args.odom_offset)
    print(json.dumps(metrics, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
