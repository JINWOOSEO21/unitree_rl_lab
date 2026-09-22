"""Measure which raw Go2 LiDAR returns appear in ``cloud_base`` recordings.

This is an offline comparison.  Points are paired only when their DDS source
timestamp matches and their (intensity bits, ring, time bits) tuple is unique
in both messages.  Geometry is reported as an observed selection pattern; it
does not identify the publisher's undocumented filter implementation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from em_sidecar.go2_cloud import RAW_TO_BASE_ROTATION, RAW_TO_BASE_TRANSLATION


KEY_DTYPE = np.dtype([("intensity_bits", "<u4"), ("ring", "<u2"), ("time_bits", "<u4")])


def _rows(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _stamp(row: dict) -> tuple[int, int]:
    return int(row["stamp"]["sec"]), int(row["stamp"]["nanosec"])


def _payloads(directory: Path) -> dict[tuple[int, int], tuple[dict, bytes]]:
    rows = _rows(directory / "cloud.jsonl")
    binary = (directory / "cloud.bin").read_bytes()
    result = {}
    for row in rows:
        begin = int(row["binary_offset"])
        end = begin + int(row["binary_size"])
        if end > len(binary):
            raise ValueError(f"Truncated payload in {directory}")
        stamp = _stamp(row)
        if stamp in result:
            raise ValueError(f"Duplicate source timestamp {stamp} in {directory}")
        result[stamp] = row, binary[begin:end]
    return result


def _decode(row: dict, payload: bytes) -> tuple[np.ndarray, np.ndarray]:
    if row["is_bigendian"]:
        raise ValueError("Big-endian PointCloud2 is not supported")
    if int(row["point_step"]) != 32 or len(payload) % 32:
        raise ValueError("Expected recorded 32-byte Go2 PointCloud2 points")
    raw = np.frombuffer(payload, dtype=np.uint8).reshape(-1, 32)
    xyz = raw[:, :12].copy().view("<f4").reshape(-1, 3).astype(np.float64)
    keys = np.empty(len(raw), dtype=KEY_DTYPE)
    keys["intensity_bits"] = raw[:, 16:20].copy().view("<u4").ravel()
    keys["ring"] = raw[:, 20:22].copy().view("<u2").ravel()
    keys["time_bits"] = raw[:, 24:28].copy().view("<u4").ravel()
    return xyz, keys


def _unique_lookup(keys: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    unique, first, count = np.unique(keys, return_index=True, return_counts=True)
    mask = count == 1
    return unique[mask], first[mask], int(np.sum(count[count > 1]))


def _quantiles(values: np.ndarray) -> dict[str, float | None]:
    if not len(values):
        return {name: None for name in ("min", "p05", "median", "p95", "max")}
    q = np.quantile(values, [0, .05, .5, .95, 1])
    return {name: float(value) for name, value in zip(("min", "p05", "median", "p95", "max"), q)}


def _axis_stats(points: np.ndarray) -> dict:
    return {
        "x_m": _quantiles(points[:, 0]),
        "y_m": _quantiles(points[:, 1]),
        "z_m": _quantiles(points[:, 2]),
        "range_3d_m": _quantiles(np.linalg.norm(points, axis=1)),
    }


def _binned_fraction(values: np.ndarray, retained: np.ndarray, edges: list[float]) -> list[dict]:
    bins = np.digitize(values, edges[1:-1], right=False)
    result = []
    for index in range(len(edges) - 1):
        selected = bins == index
        count = int(selected.sum())
        result.append({
            "lower": _finite_edge(edges[index]),
            "upper": _finite_edge(edges[index + 1]),
            "raw_points": count,
            "retained_points": int(retained[selected].sum()),
            "retained_fraction": float(retained[selected].mean()) if count else None,
        })
    return result


def _finite_edge(value: float) -> float | str:
    if np.isneginf(value):
        return "-inf"
    if np.isposinf(value):
        return "inf"
    return value


def analyze_capture(capture: Path) -> tuple[dict, np.ndarray, np.ndarray]:
    raw_messages = _payloads(capture / "utlidar_cloud")
    base_messages = _payloads(capture / "utlidar_cloud_base")
    stamps = sorted(set(raw_messages) & set(base_messages))
    if not stamps:
        raise ValueError(f"No exactly timestamp-matched cloud/cloud_base frames in {capture}")

    all_points, all_retained = [], []
    frame_fractions, residuals = [], []
    raw_duplicate_points = base_duplicate_points = unmatched_base_points = 0
    raw_total = base_total = matched_total = 0
    for stamp in stamps:
        raw_xyz, raw_keys = _decode(*raw_messages[stamp])
        base_xyz, base_keys = _decode(*base_messages[stamp])
        raw_unique, raw_indices, raw_dupes = _unique_lookup(raw_keys)
        base_unique, base_indices, base_dupes = _unique_lookup(base_keys)
        common, raw_pos, base_pos = np.intersect1d(
            raw_unique, base_unique, assume_unique=True, return_indices=True)
        matched_raw_indices = raw_indices[raw_pos]
        matched_base_indices = base_indices[base_pos]
        retained = np.zeros(len(raw_xyz), dtype=bool)
        retained[matched_raw_indices] = True
        transformed = raw_xyz @ RAW_TO_BASE_ROTATION.T + RAW_TO_BASE_TRANSLATION
        finite = np.isfinite(transformed).all(axis=1)
        if not finite.all():
            retained = retained[finite]
            transformed = transformed[finite]
        delta = (raw_xyz[matched_raw_indices] @ RAW_TO_BASE_ROTATION.T
                 + RAW_TO_BASE_TRANSLATION - base_xyz[matched_base_indices])
        residuals.append(np.linalg.norm(delta, axis=1))
        all_points.append(transformed)
        all_retained.append(retained)
        raw_total += len(raw_xyz)
        base_total += len(base_xyz)
        matched_total += len(common)
        raw_duplicate_points += raw_dupes
        base_duplicate_points += base_dupes
        unmatched_base_points += len(base_xyz) - len(common)
        frame_fractions.append(len(common) / len(raw_xyz))

    points = np.concatenate(all_points)
    retained = np.concatenate(all_retained)
    residual = np.concatenate(residuals)
    report = {
        "capture": str(capture.resolve()),
        "matched_source_stamp_frames": len(stamps),
        "unpaired_raw_frames": len(raw_messages) - len(stamps),
        "unpaired_base_frames": len(base_messages) - len(stamps),
        "raw_points": raw_total,
        "cloud_base_points": base_total,
        "uniquely_matched_points": matched_total,
        "retained_fraction_total": matched_total / raw_total,
        "removed_fraction_total": 1.0 - matched_total / raw_total,
        "retained_fraction_per_frame": _quantiles(np.asarray(frame_fractions)),
        "attribute_key_ambiguity": {
            "raw_points_in_duplicate_keys": raw_duplicate_points,
            "base_points_in_duplicate_keys": base_duplicate_points,
            "base_points_without_unique_raw_match": unmatched_base_points,
        },
        "coordinate_validation_m": {
            "rms": float(np.sqrt(np.mean(residual ** 2))),
            "max": float(np.max(residual)),
        },
        "retained_geometry_base_axes": _axis_stats(points[retained]),
        "removed_geometry_base_axes": _axis_stats(points[~retained]),
        "retention_by_range_3d_m": _binned_fraction(
            np.linalg.norm(points, axis=1), retained, [0, .25, .5, 1, 2, 3, 5, 10, float("inf")]),
        "retention_by_z_m": _binned_fraction(
            points[:, 2], retained, [float("-inf"), -.5, -.25, 0, .25, .5, 1, 2, float("inf")]),
        "retention_by_x_m": _binned_fraction(
            points[:, 0], retained, [float("-inf"), -2, -1, -.5, 0, .5, 1, 2, 5, float("inf")]),
    }
    return report, points, retained


def _plot(datasets: list[tuple[str, np.ndarray, np.ndarray]], output: Path) -> None:
    rng = np.random.default_rng(0)
    figure, axes = plt.subplots(len(datasets), 2, figsize=(12, 5 * len(datasets)), squeeze=False)
    for row, (label, points, retained) in enumerate(datasets):
        for keep, color, name in ((False, "#d95f02", "removed"), (True, "#1b9e77", "retained")):
            indices = np.flatnonzero(retained == keep)
            if len(indices) > 40000:
                indices = rng.choice(indices, 40000, replace=False)
            axes[row, 0].scatter(points[indices, 0], points[indices, 1], s=.5, alpha=.25,
                                 c=color, rasterized=True, label=name)
            axes[row, 1].scatter(points[indices, 0], points[indices, 2], s=.5, alpha=.25,
                                 c=color, rasterized=True, label=name)
        axes[row, 0].set(title=f"{label}: near-field top view", xlabel="base x (front) [m]", ylabel="base y (left) [m]",
                         xlim=(-2.5, 2.5), ylim=(-2.5, 2.5))
        axes[row, 1].set(title=f"{label}: near-field side view", xlabel="base x (front) [m]", ylabel="base z (up) [m]",
                         xlim=(-2.5, 2.5), ylim=(-1.5, 1.5))
        for axis in axes[row]:
            axis.set_aspect("equal", adjustable="box")
            axis.grid(alpha=.2)
            axis.legend(markerscale=8)
    figure.suptitle("Observed cloud_base membership by exact stamp and unique point attributes")
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    reports, datasets = [], []
    for capture in args.captures:
        report, points, retained = analyze_capture(capture)
        reports.append(report)
        datasets.append((capture.name, points, retained))
    result = {
        "method": (
            "Exact source stamp pairing; membership from (float32 intensity bits, uint16 ring, "
            "float32 per-point time bits) keys that are unique in both messages. Raw XYZ is "
            "mapped with the independently measured fixed raw-to-base transform."
        ),
        "verified_scope": (
            "Reports observed cloud_base membership in these recordings and validates matched "
            "coordinates. It does not establish the publisher's internal filter algorithm."
        ),
        "captures": reports,
    }
    report_path = args.output_dir / "summary.json"
    report_path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    _plot(datasets, args.output_dir / "membership.png")
    print(json.dumps({"report": str(report_path), "captures": reports}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
