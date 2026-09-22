"""Offline integrity, timing and geometry diagnostics for go2_record output.

No DDS imports or robot connections. Geometry results compare hypotheses;
they do not calibrate the sensor or certify control readiness.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from em_sidecar.kinematics import quat_to_mat
from em_sidecar.pointcloud import decode_xyz


def read_rows(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"No messages: {path}")
    times = np.array([r["steady_ns"] for r in rows], dtype=np.int64)
    if np.any(np.diff(times) < 0):
        raise ValueError(f"Non-monotonic callback timestamps: {path}")
    return rows, times


def stats(values):
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError("Nonfinite state/diagnostic value")
    return {name: fn(a, axis=0).tolist() for name, fn in
            (("min", np.min), ("median", np.median), ("max", np.max), ("std", np.std))}


def timing(times):
    span = (times[-1] - times[0]) * 1e-9
    gaps = np.diff(times) * 1e-6
    return {"count": len(times), "span_s": span,
            "callback_hz": (len(times) - 1) / span if span else None,
            "gap_ms": stats(gaps) if len(gaps) else None}


def preceding(times, at):
    """Causal pairing by local receipt, not sensor time synchronization."""
    idx = int(np.searchsorted(times, at, side="right")) - 1
    return idx, None if idx < 0 else (int(at) - int(times[idx])) * 1e-6


def horizontal_surface(points):
    """Find a dominant near-horizontal surface without labeling it ground."""
    radius = np.linalg.norm(points[:, :2], axis=1)
    p = points[(radius > 0.5) & (radius < 2.5) & (np.abs(points[:, 2]) < 2)]
    if len(p) < 30:
        return {"available": False}
    edges = np.arange(-2, 2.021, 0.02)
    counts, _ = np.histogram(p[:, 2], edges)
    k = int(counts.argmax())
    center = (edges[k] + edges[k + 1]) / 2
    selected = p[np.abs(p[:, 2] - center) < 0.04]
    if len(selected) < 30:
        return {"available": False}
    design = np.column_stack((selected[:, :2], np.ones(len(selected))))
    coeff, _, rank, _ = np.linalg.lstsq(design, selected[:, 2], rcond=None)
    residual = selected[:, 2] - design @ coeff
    return {"available": bool(rank == 3), "points": len(selected),
            "candidate_fraction": len(selected) / len(p),
            "z_equals_ax_by_c": coeff.tolist(),
            "tilt_deg": float(np.degrees(np.arctan(np.linalg.norm(coeff[:2])))),
            "residual_rms_m": float(np.sqrt(np.mean(residual**2)))}


def analyze(directory):
    low, lt = read_rows(directory / "lowstate.jsonl")
    sport, st = read_rows(directory / "sportmodestate.jsonl")
    clouds, ct = read_rows(directory / "cloud.jsonl")
    manifest = json.loads((directory / "manifest.json").read_text())
    for key, count in (("lowstate_count", len(low)), ("sportmodestate_count", len(sport)),
                       ("cloud_count", len(clouds))):
        if manifest[key] != count:
            raise ValueError(f"Manifest count mismatch: {key}")
    report = {
        "scope": "Offline recording diagnostics; no motor output, no health certification",
        "pairing": "Latest preceding local callback; not acquisition-time synchronization",
        "timestamp_semantics": manifest.get("timestamp_semantics",
            "Original recorder: low/sport before serialization; cloud after binary write. "
            "Cloud pairing ages include file-write overhead; not exact callback-entry timing."),
        "timing": {"lowstate": timing(lt), "sport": timing(st), "cloud": timing(ct)},
        "sport_error_codes": sorted({r["error_code"] for r in sport}),
        "sport_modes": sorted({r["mode"] for r in sport}),
        "sport_position_m": stats([r["position"] for r in sport]),
        "sport_velocity": stats([r["velocity"] for r in sport]),
        "body_height_m": stats([r["body_height"] for r in sport]),
        "joint_q_sdk_rad": stats([[m["q"] for m in r["motor_state"][:12]] for r in low]),
        "joint_dq_sdk_rad_s": stats([[m["dq"] for m in r["motor_state"][:12]] for r in low]),
        "gyro_rad_s": stats([r["imu_state"]["gyroscope"] for r in low]),
        "acceleration": stats([r["imu_state"]["accelerometer"] for r in low]),
        "quaternion_norm": stats([np.linalg.norm(r["imu_state"]["quaternion"]) for r in low]),
        "foot_force_raw_sdk": stats([r["foot_force"] for r in low]),
        "low_tick_delta": stats(np.diff([r["tick"] for r in low])),
    }
    ages_low, ages_sport, counts, paired = [], [], [], []
    expected_offset = 0
    with (directory / "cloud.bin").open("rb") as binary:
        for row in clouds:
            if row["binary_offset"] != expected_offset:
                raise ValueError("Cloud payload offsets have a gap or overlap")
            payload = binary.read(row["binary_size"])
            if len(payload) != row["binary_size"]:
                raise ValueError("Truncated cloud payload")
            expected_offset += len(payload)
            msg = SimpleNamespace(**{**row, "data": payload,
                "fields": [SimpleNamespace(**f) for f in row["fields"]]})
            points = decode_xyz(msg)
            counts.append([row["width"] * row["height"], len(points)])
            i, la = preceding(lt, row["steady_ns"])
            j, sa = preceding(st, row["steady_ns"])
            if i < 0 or j < 0:
                continue
            ages_low.append(la)
            ages_sport.append(sa)
            # Subsample each frame for bounded geometric analysis.
            paired.append((points[::max(1, len(points) // 1000)], low[i], sport[j]))
        if binary.read(1):
            raise ValueError("Unindexed trailing bytes in cloud.bin")
    if manifest["cloud_bytes"] != expected_offset or manifest["cloud_limit_hit"]:
        raise ValueError("Manifest byte mismatch or capture truncated by byte limit")
    if not paired:
        raise ValueError("No cloud has preceding lowstate and sport observations")
    report["cloud"] = {"frames_decoded": len(clouds), "bytes": expected_offset,
        "paired_frames": len(paired), "point_counts_total_finite": stats(counts),
        "low_receipt_age_ms": stats(ages_low), "sport_receipt_age_ms": stats(ages_sport),
        "frame_ids": sorted({r["frame_id"] for r in clouds})}
    report["geometry_hypotheses"] = {}
    for name, mount_r in (("sim_Rx180", np.diag([1, -1, -1])), ("identity", np.eye(3))):
        for offset_name, offset in (("zero_odom_offset", np.zeros(3)),
                                   ("sim_imu_offset", np.array([-0.02557, 0, 0.04232]))):
            world = []
            for points, lrow, srow in paired:
                q = np.asarray(lrow["imu_state"]["quaternion"], dtype=float)
                norm = np.linalg.norm(q)
                if not np.isfinite(norm) or norm < 1e-6:
                    raise ValueError("Invalid IMU quaternion")
                rotation = quat_to_mat(q / norm)
                pos = np.asarray(srow["position"]) - rotation @ offset
                world.append(points @ (rotation @ mount_r).T + pos
                             + rotation @ np.array([0.28, 0, 0.10]))
            report["geometry_hypotheses"][name + "_" + offset_name] = horizontal_surface(np.vstack(world))
    report["geometry_limit"] = (
        "Uses assumed mount translation [0.28,0,0.10]. Dominant surface can be furniture; "
        "flat ground alone does not identify full extrinsics or sport origin. Do not auto-apply a candidate.")
    report["sha256"] = {}
    for filename in ("lowstate.jsonl", "sportmodestate.jsonl", "cloud.jsonl", "cloud.bin"):
        with (directory / filename).open("rb") as stream:
            report["sha256"][filename] = hashlib.file_digest(stream, "sha256").hexdigest()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.recording)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: report[k] for k in ("timing", "cloud", "geometry_hypotheses")}, indent=2))


if __name__ == "__main__":
    main()
