"""Add explicit publisher-cloud_base and transformed-raw modes to the viewer.

The existing standalone viewer is intentionally used as the presentation
template so its camera controls, axes, floor plane, and URDF/FK body overlay
remain unchanged.  This script only replaces its anonymous point arrays with
named sources and embeds the recorded publisher ``cloud_base`` alongside the
already embedded transformed raw cloud.
"""
from __future__ import annotations

import argparse
import base64
import json
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAPTURE = ROOT / "captures/frame_inspection_20260915/dds_capture"
DEFAULT_VIEWER = Path.home() / "바탕화면/2026-09-17/lidar_observed_base.html"
sys.path.insert(0, str(ROOT))

from em_sidecar.go2_cloud import RAW_TO_BASE_ROTATION, RAW_TO_BASE_TRANSLATION
from em_sidecar.pointcloud import decode_xyz


ARRAYS_RE = re.compile(
    r"const xyz=new Float32Array\(bytes\('(?P<xyz>[^']+)'\)\.buffer\), "
    r"times=new Float32Array\(bytes\('(?P<times>[^']+)'\)\.buffer\), "
    r"flags=bytes\('(?P<flags>[^']+)'\);"
)


def _pack(array: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")


def _message(row: dict, payload: bytes) -> SimpleNamespace:
    return SimpleNamespace(
        **{**row, "data": payload,
           "fields": [SimpleNamespace(**field) for field in row["fields"]]}
    )


def _read_clouds(directory: Path) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    rows = [json.loads(line) for line in (directory / "cloud.jsonl").read_text().splitlines()]
    first_ns = rows[0]["steady_ns"]
    point_sets: list[np.ndarray] = []
    time_sets: list[np.ndarray] = []
    with (directory / "cloud.bin").open("rb") as stream:
        for row in rows:
            stream.seek(row["binary_offset"])
            payload = stream.read(row["binary_size"])
            if len(payload) != row["binary_size"]:
                raise ValueError(f"short point-cloud payload at {row['binary_offset']}")
            points = decode_xyz(_message(row, payload)).astype("<f4", copy=False)
            point_sets.append(points)
            time_sets.append(np.full(len(points), (row["steady_ns"] - first_ns) / 1e9,
                                     dtype="<f4"))
    return np.concatenate(point_sets), np.concatenate(time_sets), rows


def _capsule_flags(points: np.ndarray, overlay: dict) -> np.ndarray:
    flags = np.zeros(len(points), dtype=np.uint8)
    for start, end, radius in zip(
            overlay["capsule_a"], overlay["capsule_b"], overlay["capsule_radius"]):
        start = np.asarray(start, dtype=np.float64)
        segment = np.asarray(end, dtype=np.float64) - start
        denominator = float(segment @ segment)
        offset = points - start
        if denominator == 0.0:
            distance2 = np.sum(offset ** 2, axis=1)
        else:
            alpha = np.clip(offset @ segment / denominator, 0.0, 1.0)
            distance2 = np.sum((offset - alpha[:, None] * segment) ** 2, axis=1)
        flags[distance2 <= float(radius) ** 2] = 1
    return flags


def build(viewer: Path, capture: Path, output: Path) -> dict:
    source = viewer.read_text()
    if "LIDAR_SOURCE_MODES_V1" in source:
        raw_count = int(re.search(r"const rawXyz=.*?", source).group(0) is not None)
        return {
            "output": str(viewer),
            "already_built": True,
            "raw_mode_present": bool(raw_count),
            "publisher_mode_present": "publisherBaseXyz" in source,
        }
    match = ARRAYS_RE.search(source)
    if not match:
        raise ValueError("could not find original xyz/times/flags arrays")

    embedded_xyz = np.frombuffer(base64.b64decode(match["xyz"]), dtype="<f4").reshape(-1, 3)
    embedded_times = np.frombuffer(base64.b64decode(match["times"]), dtype="<f4")
    raw_xyz, _, raw_rows = _read_clouds(capture / "utlidar_cloud")
    transformed_raw = (
        raw_xyz.astype(np.float64) @ RAW_TO_BASE_ROTATION.T + RAW_TO_BASE_TRANSLATION
    ).astype("<f4")
    if embedded_xyz.shape != transformed_raw.shape:
        raise ValueError(f"embedded/raw shape mismatch: {embedded_xyz.shape} != {transformed_raw.shape}")
    max_embedded_error = float(np.max(np.linalg.norm(
        embedded_xyz.astype(np.float64) - transformed_raw.astype(np.float64), axis=1)))
    if max_embedded_error > 2e-6:
        raise ValueError(f"existing viewer does not use measured raw transform: {max_embedded_error}")

    base_xyz, base_times, base_rows = _read_clouds(capture / "utlidar_cloud_base")
    if any(row["frame_id"] != "base_link" for row in base_rows):
        raise ValueError("publisher cloud_base recording is not in base_link")
    # The two topics carry identical source stamps but arrive a few hundred
    # microseconds apart. Use the raw-viewer's per-frame time for cloud_base so
    # switching sources compares the exact same acquisition window.
    cursor = 0
    stamp_times: dict[tuple[int, int], float] = {}
    for row in raw_rows:
        stamp = (row["stamp"]["sec"], row["stamp"]["nanosec"])
        stamp_times[stamp] = float(embedded_times[cursor])
        cursor += row["width"] * row["height"]
    base_times = np.concatenate([
        np.full(row["width"] * row["height"],
                stamp_times[(row["stamp"]["sec"], row["stamp"]["nanosec"])], dtype="<f4")
        for row in base_rows
    ])
    overlay = json.loads(re.search(r"const OVERLAY=(.*?);", source).group(1))
    base_flags = _capsule_flags(base_xyz.astype(np.float64), overlay)

    replacement = (
        "/* LIDAR_SOURCE_MODES_V1 */\n"
        f"const rawXyz=new Float32Array(bytes('{match['xyz']}').buffer), "
        f"rawTimes=new Float32Array(bytes('{match['times']}').buffer), "
        f"rawFlags=bytes('{match['flags']}');\n"
        f"const publisherBaseXyz=new Float32Array(bytes('{_pack(base_xyz.astype('<f4'))}').buffer), "
        f"publisherBaseTimes=new Float32Array(bytes('{_pack(base_times)}').buffer), "
        f"publisherBaseFlags=bytes('{_pack(base_flags)}');\n"
        "const SOURCES={\n"
        " publisher:{xyz:publisherBaseXyz,times:publisherBaseTimes,flags:publisherBaseFlags,"
        "label:'발행 cloud_base · base_link · publisher 선택 적용'},\n"
        " raw:{xyz:rawXyz,times:rawTimes,flags:rawFlags,"
        "label:'원시 cloud · 측정 고정변환 적용 · 모든 finite 점'}\n"
        "};\n"
        "const sourceKey=location.hash==='#raw-transformed'?'raw':'publisher', "
        "activeSource=SOURCES[sourceKey], xyz=activeSource.xyz, times=activeSource.times, "
        "flags=activeSource.flags, TOTAL=activeSource.xyz.length/3;\n"
    )
    source = source[:match.start()] + replacement + source[match.end():]
    old_total = "const TOTAL=513956, DURATION="
    if source.count(old_total) != 1:
        raise ValueError("unexpected TOTAL declaration in viewer template")
    source = source.replace(
        old_total,
        "const DURATION=",
        1,
    )
    source = source.replace(
        "표시 좌표는 로봇이 발행한 base_link입니다. 각 시각의 몸체 좌표로 점군을 모았으며 world/odom 지도는 아닙니다. 원본 점군에 관측 변환을 적용했고 모든 점을 보존했습니다.<br>",
        "표시 좌표는 base_link이며 world/odom 지도는 아닙니다. "
        "<label>점군 출처 <select id=\"cloudSource\">"
        "<option value=\"publisher\">발행된 cloud_base</option>"
        "<option value=\"raw\">cloud → 측정 고정변환 → base_link</option>"
        "</select></label> · <span id=\"sourceInfo\"></span><br>",
        1,
    )
    source = source.replace(
        "const U=n=>gl.getUniformLocation(prog,n), el=id=>document.getElementById(id);",
        "const U=n=>gl.getUniformLocation(prog,n), el=id=>document.getElementById(id);\n"
        "el('cloudSource').value=sourceKey;el('sourceInfo').textContent=activeSource.label;\n"
        "el('cloudSource').onchange=e=>{location.hash=e.target.value==='raw'?'raw-transformed':'publisher-cloud-base';location.reload()};",
        1,
    )
    source = source.replace(
        "el('count').dataset.total=String(TOTAL);",
        "el('count').dataset.total=String(TOTAL);el('count').dataset.source=sourceKey;",
        1,
    )
    output.write_text(source)
    return {
        "output": str(output),
        "raw_transformed_points": len(embedded_xyz),
        "publisher_cloud_base_points": len(base_xyz),
        "publisher_retained_fraction_of_raw": len(base_xyz) / len(raw_xyz),
        "raw_frames": len(raw_rows),
        "publisher_frames": len(base_rows),
        "existing_raw_transform_max_error_m": max_embedded_error,
        "publisher_self_filter_candidates": int(base_flags.sum()),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", type=Path, default=DEFAULT_VIEWER)
    parser.add_argument("--capture", type=Path, default=DEFAULT_CAPTURE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.viewer
    print(json.dumps(build(args.viewer, args.capture, output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
