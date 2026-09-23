"""Replay recorded ``cloud_base`` directly into the policy elevation-map scan.

This is an offline diagnostic.  Point coordinates are already expressed in
``base_link``; the replay therefore applies only the recorded ``robot_odom``
pose.  No LiDAR mount rotation, LowState IMU pose, self filter, DDS publisher,
or policy action path is involved.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

PARKOUR_ROOT = Path(__file__).resolve().parents[1]

from em_sidecar.kinematics import Go2Kinematics, quat_to_mat, yaw_from_quat
from em_sidecar.pointcloud import decode_xyz


SENSOR_ORIGIN_IN_BASE = np.array([0.282160014, 0.0, 0.0], dtype=np.float64)
EXPECTED_FRAME = "base_link"
POLICY_SCAN_SIZE = 132


def _read_rows(path: Path) -> tuple[list[dict], np.ndarray]:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"no messages in {path}")
    times = np.asarray([row["steady_ns"] for row in rows], dtype=np.int64)
    if np.any(np.diff(times) < 0):
        raise ValueError(f"non-monotonic steady_ns in {path}")
    return rows, times


def _preceding(times: np.ndarray, at_ns: int, max_age_ms: float) -> tuple[int, float, str | None]:
    index = int(np.searchsorted(times, at_ns, side="right")) - 1
    if index < 0:
        return index, float("nan"), "missing_preceding_odom"
    age_ms = (int(at_ns) - int(times[index])) * 1e-6
    if age_ms > max_age_ms:
        return index, age_ms, "odom_too_old"
    return index, age_ms, None


def build_schedule(
    cloud_times: np.ndarray,
    rate: str,
    max_cloud_age_ms: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build causal processing ticks and select each cloud at most once."""
    times = np.asarray(cloud_times, dtype=np.int64)
    if times.ndim != 1 or not len(times) or np.any(np.diff(times) < 0):
        raise ValueError("cloud times must be a non-empty monotonic vector")
    if rate == "native":
        ticks = times.copy()
    elif rate == "fixed10":
        period_ns = 100_000_000
        count = int((int(times[-1]) - int(times[0])) // period_ns) + 1
        ticks = int(times[0]) + np.arange(count, dtype=np.int64) * period_ns
    else:
        raise ValueError("rate must be 'fixed10' or 'native'")

    indices = np.full(len(ticks), -1, dtype=np.int64)
    ages_ms = np.full(len(ticks), np.nan, dtype=np.float64)
    reasons = np.full(len(ticks), "", dtype="U32")
    last_index = -1
    for i, tick in enumerate(ticks):
        index = int(np.searchsorted(times, tick, side="right")) - 1
        if index < 0:
            reasons[i] = "missing_preceding_cloud"
            continue
        indices[i] = index
        ages_ms[i] = (int(tick) - int(times[index])) * 1e-6
        if ages_ms[i] > max_cloud_age_ms:
            reasons[i] = "cloud_too_old"
            continue
        if index == last_index:
            reasons[i] = "no_new_cloud"
            continue
        last_index = index
    return ticks, indices, ages_ms, reasons


def odom_pose(row: dict) -> tuple[np.ndarray, np.ndarray]:
    """Return base position and normalized wxyz quaternion from robot_odom."""
    if row.get("frame_id") != "odom" or row.get("child_frame_id") != EXPECTED_FRAME:
        raise ValueError("robot_odom must describe odom -> base_link")

    p = row["position"]
    o = row["orientation"]
    position = np.asarray([p["x"], p["y"], p["z"]], dtype=np.float64)
    quat = np.asarray([o["w"], o["x"], o["y"], o["z"]], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(position).all() or not np.isfinite(quat).all() or norm < 1e-6:
        raise ValueError("robot_odom contains an invalid pose")

    return position, quat / norm


def backend_input_from_base_cloud(
    points_base: np.ndarray,
    base_position: np.ndarray,
    base_rotation: np.ndarray,
    sensor_origin_in_base: np.ndarray = SENSOR_ORIGIN_IN_BASE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Adapt base-frame points to a backend API that also needs the ray origin.

    The backend computes ``p_odom = R_sensor @ p_input + t_sensor`` and treats
    ``t_sensor`` as the ray origin.  Translating the input by the physical
    observed sensor origin in the published base_link makes both statements
    true without applying a LiDAR
    extrinsic to an already base-frame cloud.
    """
    points = np.asarray(points_base, dtype=np.float64)
    position = np.asarray(base_position, dtype=np.float64)
    rotation = np.asarray(base_rotation, dtype=np.float64)
    origin = np.asarray(sensor_origin_in_base, dtype=np.float64)

    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("points_base must have shape (N, 3)")

    if position.shape != (3,) or rotation.shape != (3, 3) or origin.shape != (3,):
        raise ValueError("invalid pose or sensor-origin shape")

    points_from_sensor_origin = points - origin
    sensor_position_odom = position + rotation @ origin
    return points_from_sensor_origin, rotation, sensor_position_odom


def validate_policy_scan(scan: np.ndarray) -> np.ndarray:
    scan = np.asarray(scan, dtype=np.float32)
    if scan.shape != (POLICY_SCAN_SIZE,):
        raise ValueError(f"policy scan must have shape ({POLICY_SCAN_SIZE},)")
    if not np.isfinite(scan).all() or np.any(np.abs(scan) > 1.000001):
        raise ValueError("policy scan must be finite and normalized to [-1, 1]")
    return scan


def load_backend(emcupy_root: Path, device: str):
    import os

    os.environ["EMCUPY_ROOT"] = str(emcupy_root)
    source = PARKOUR_ROOT / "vendored" / "elevation_map_backend.py"
    spec = importlib.util.spec_from_file_location("cloud_base_em_backend", source)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load elevation-map backend: {source}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.BatchedElevationMapBackend(
        num_envs=1, device=device, resolution=0.1, map_length=3.2
    )


def _stats(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def replay(
    recording: Path,
    emcupy_root: Path,
    device: str = "cuda:0",
    max_odom_age_ms: float = 20.0,
    rate: str = "fixed10",
    max_cloud_age_ms: float = 200.0,
) -> tuple[dict[str, np.ndarray], dict]:
    import torch

    cloud_dir = recording / "utlidar_cloud_base"
    clouds, _ = _read_rows(cloud_dir / "cloud.jsonl")
    odom, odom_times = _read_rows(recording / "robot_odom.jsonl")
    backend = load_backend(emcupy_root, device)
    scan_xy = Go2Kinematics(PARKOUR_ROOT / "contract" / "em_geometry.npz").scan_offsets_xy
    if scan_xy.shape != (POLICY_SCAN_SIZE, 2):
        raise ValueError(f"unexpected scan geometry {scan_xy.shape}")

    input_cloud_times = np.asarray([row["steady_ns"] for row in clouds], dtype=np.int64)
    tick_steady_ns, source_cloud_index, tick_source_age_ms, schedule_rejection = build_schedule(
        input_cloud_times, rate, max_cloud_age_ms
    )
    count = len(tick_steady_ns)
    rejection = schedule_rejection.copy()
    accepted = np.zeros(count, dtype=bool)
    odom_index = np.full(count, -1, dtype=np.int64)
    odom_age_ms = np.full(count, np.nan, dtype=np.float64)
    finite_points = np.zeros(count, dtype=np.int64)
    base_position = np.full((count, 3), np.nan, dtype=np.float64)
    base_quat = np.full((count, 4), np.nan, dtype=np.float64)
    scans = np.full((count, POLICY_SCAN_SIZE), np.nan, dtype=np.float32)
    valid = np.full_like(scans, np.nan)
    upper = np.full_like(scans, np.nan)

    expected_offset = 0
    for row in clouds:
        if int(row["binary_offset"]) != expected_offset:
            raise ValueError("cloud payload offsets have a gap or overlap")
        expected_offset += int(row["binary_size"])
    if expected_offset != (cloud_dir / "cloud.bin").stat().st_size:
        raise ValueError("cloud binary size does not match metadata")

    with (cloud_dir / "cloud.bin").open("rb") as binary:
        for frame, cloud_index in enumerate(source_cloud_index):
            if rejection[frame]:
                continue
            row = clouds[int(cloud_index)]
            if row.get("frame_id") != EXPECTED_FRAME:
                raise ValueError(f"cloud_base frame is {row.get('frame_id')!r}, expected base_link")
            binary.seek(int(row["binary_offset"]))
            payload = binary.read(int(row["binary_size"]))
            if len(payload) != int(row["binary_size"]):
                raise ValueError("truncated cloud payload")

            # Pose is paired to the selected cloud callback, never to the later
            # fixed-rate processing tick.
            oi, age, reason = _preceding(odom_times, int(row["steady_ns"]), max_odom_age_ms)
            odom_index[frame], odom_age_ms[frame] = oi, age
            if reason is not None:
                rejection[frame] = reason
                continue
            try:
                position, quat = odom_pose(odom[oi])
            except ValueError:
                rejection[frame] = "invalid_odom_pose"
                continue
            msg = SimpleNamespace(
                **{
                    **row,
                    "data": payload,
                    "fields": [SimpleNamespace(**field) for field in row["fields"]],
                }
            )
            points_base = decode_xyz(msg)
            finite_points[frame] = len(points_base)
            if not len(points_base):
                rejection[frame] = "empty_cloud"
                continue

            rotation = quat_to_mat(quat)
            points_input, sensor_rotation, sensor_position = backend_input_from_base_cloud(
                points_base, position, rotation
            )
            backend.update(
                [torch.as_tensor(points_input, dtype=torch.float32, device=device)],
                torch.as_tensor(sensor_rotation, dtype=torch.float32, device=device).unsqueeze(0),
                torch.as_tensor(sensor_position, dtype=torch.float32, device=device).unsqueeze(0),
                torch.as_tensor(position, dtype=torch.float32, device=device).unsqueeze(0),
                torch.as_tensor(rotation, dtype=torch.float32, device=device).unsqueeze(0),
            )

            yaw = yaw_from_quat(quat)
            cy, sy = np.cos(yaw), np.sin(yaw)
            px = position[0] + cy * scan_xy[:, 0] - sy * scan_xy[:, 1]
            py = position[1] + sy * scan_xy[:, 0] + cy * scan_xy[:, 1]
            query = torch.as_tensor(
                np.stack([px, py], axis=-1), dtype=torch.float32, device=device
            ).unsqueeze(0)
            height = torch.as_tensor([position[2]], dtype=torch.float32, device=device)
            h, vf, uf = backend.sample(query, height)
            scans[frame] = validate_policy_scan(h[0].detach().cpu().numpy())
            valid[frame] = vf[0].detach().cpu().numpy()
            upper[frame] = uf[0].detach().cpu().numpy()
            base_position[frame], base_quat[frame] = position, quat
            accepted[frame] = True

    if not np.any(accepted):
        raise ValueError("all cloud_base frames were rejected")
    layers = backend.layers()[0].detach().cpu().numpy()
    center = backend.centers_t()[0].detach().cpu().numpy()
    keep = accepted
    reasons, reason_counts = np.unique(rejection[~keep], return_counts=True)
    input_duration_s = (input_cloud_times[-1] - input_cloud_times[0]) * 1e-9
    arrays = {
        "input_cloud_steady_ns": input_cloud_times,
        "tick_steady_ns": tick_steady_ns,
        "source_cloud_index": source_cloud_index,
        "source_cloud_steady_ns": np.where(
            source_cloud_index >= 0,
            input_cloud_times[np.maximum(source_cloud_index, 0)],
            -1,
        ),
        "tick_source_age_ms": tick_source_age_ms,
        "accepted": accepted,
        "reject_reason": rejection,
        "odom_index": odom_index,
        "odom_age_ms": odom_age_ms,
        "finite_points": finite_points,
        "base_position": base_position,
        "base_quat_wxyz": base_quat,
        "scan": scans,
        "valid_fraction": valid,
        "upper_bound_fraction": upper,
        "final_map_layers": layers.astype(np.float32),
        "final_map_center": center.astype(np.float64),
        "scan_offsets_xy": scan_xy.astype(np.float64),
    }
    final_index = int(np.flatnonzero(keep)[-1])
    final_direct = valid[final_index] > 1e-6
    final_upper = upper[final_index] > 1e-6
    summary = {
        "scope": "offline cloud_base -> elevation map -> 132-value policy scan replay",
        "schedule": {
            "mode": rate,
            "period_ms": 100.0 if rate == "fixed10" else None,
            "scheduled_ticks": count,
            "selected_updates": int(np.count_nonzero(schedule_rejection == "")),
            "accepted_updates": int(keep.sum()),
            "tick_source_age_ms": _stats(tick_source_age_ms[schedule_rejection == ""]),
            "causal_selection": "latest cloud receipt <= tick; each cloud used at most once",
            "pose_pairing": "latest robot_odom receipt <= selected cloud receipt",
        },
        "input_cloud_frame": EXPECTED_FRAME,
        "pose_source": "robot_odom odom -> base_link only",
        "point_transform": "p_odom = R_odom_base @ p_base + t_odom_base",
        "lidar_mount_transform_applied": False,
        "self_filter_applied": False,
        "sensor_origin_in_base_m": SENSOR_ORIGIN_IN_BASE.tolist(),
        "sensor_origin_role": "ray origin only; algebraically canceled from point coordinates",
        "cloud": {
            "input_frames": len(clouds),
            "input_hz": float((len(clouds) - 1) / input_duration_s),
            "processing_rejections": int((~keep).sum()),
            "reject_reasons": {str(k): int(v) for k, v in zip(reasons, reason_counts)},
            "finite_points": _stats(finite_points[keep]),
        },
        "odom_age_ms": _stats(odom_age_ms[keep]),
        "scan": {
            "shape": list(scans[keep].shape),
            "value": _stats(scans[keep]),
            "all_finite": bool(np.isfinite(scans[keep]).all()),
            "within_minus1_plus1": bool(np.all(np.abs(scans[keep]) <= 1.000001)),
            "final_direct_cells": int(final_direct.sum()),
            "final_upper_bound_only_cells": int((final_upper & ~final_direct).sum()),
            "final_unknown_cells": int((~final_direct & ~final_upper).sum()),
            "unknown_value_semantics": (
                "0 is the policy fallback for an unobserved cell; it is not a measured flat floor"
            ),
        },
        "live_readiness": False,
        "blocking_findings": [
            "The previously observed approximately 35 mm floor/body-height mismatch is unresolved.",
            "No self filter was applied because its live base-frame geometry path is not yet validated.",
            "Offline callback timing does not certify live scheduling latency or jitter.",
            "Unknown scan cells use fallback zero and must not be interpreted as observed terrain.",
        ],
    }
    return arrays, summary


def plot_replay(arrays: dict[str, np.ndarray], output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap

    last = int(np.flatnonzero(arrays["accepted"])[-1])
    scan = arrays["scan"][last].reshape(11, 12)
    vf = arrays["valid_fraction"][last].reshape(11, 12)
    uf = arrays["upper_bound_fraction"][last].reshape(11, 12)
    source = np.zeros((11, 12), dtype=np.int8)
    source[uf > 1e-6] = 1
    source[vf > 1e-6] = 2

    layers = arrays["final_map_layers"]
    center = arrays["final_map_center"]
    elevation = layers[0] + center[2]
    elevation = np.where(layers[2] > 0.5, elevation, np.nan)
    n = elevation.shape[0]
    res = 0.1
    extent = [center[1] - n * res / 2, center[1] + n * res / 2,
              center[0] - n * res / 2, center[0] + n * res / 2]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), constrained_layout=True)
    im = axes[0].imshow(elevation, origin="lower", extent=extent, cmap="terrain")
    axes[0].set_title("Final elevation map\n(directly observed cells)")
    axes[0].set_xlabel("odom y [m]")
    axes[0].set_ylabel("odom x [m]")
    fig.colorbar(im, ax=axes[0], label="z [m]")

    # Stored scan order is [y, x]; display [x, y] to match the map axes.
    im = axes[1].imshow(scan.T, origin="lower", vmin=-1, vmax=1, cmap="coolwarm")
    axes[1].set_title("Final 132-value policy scan")
    axes[1].set_xlabel("y-grid index (11)")
    axes[1].set_ylabel("x-grid index (12)")
    fig.colorbar(im, ax=axes[1], label="normalized height observation")

    cmap = ListedColormap(["#d73027", "#fee08b", "#1a9850"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    axes[2].imshow(source.T, origin="lower", cmap=cmap, norm=norm)
    axes[2].set_title("Scan source\nred unknown / yellow upper / green direct")
    axes[2].set_xlabel("y-grid index (11)")
    axes[2].set_ylabel("x-grid index (12)")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--emcupy-root", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--max-odom-age-ms", type=float, default=20.0)
    parser.add_argument("--rate", choices=("fixed10", "native"), default="fixed10")
    parser.add_argument("--max-cloud-age-ms", type=float, default=200.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.max_odom_age_ms <= 0 or args.max_cloud_age_ms <= 0:
        parser.error("age limits must be positive")

    arrays, summary = replay(
        args.recording, args.emcupy_root, args.device, args.max_odom_age_ms,
        args.rate, args.max_cloud_age_ms,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output_dir / "replay.npz", **arrays)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    plot_replay(arrays, args.output_dir / "scan_and_map.png")
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
