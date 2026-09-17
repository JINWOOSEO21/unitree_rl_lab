"""Staged GPU smoke test for the bridge mapping path (plan stage B-3).

No DDS, no robot, no network. Each stage prints PASS/FAIL so the first broken layer is obvious.
Usage: python gpu_smoke.py <deploy/parkour dir> <elevation_mapping_cupy root> [updates]
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

parkour, emcupy = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
n_updates = int(sys.argv[3]) if len(sys.argv) > 3 else 50
sys.path[:0] = [str(parkour / "tools"), str(parkour)]


def stage(name, fn):
    t0 = time.perf_counter()
    try:
        detail = fn()
        print(f"PASS {name} ({time.perf_counter() - t0:.2f}s) {detail or ''}", flush=True)
    except Exception as exc:  # report and stop: later stages depend on this one
        print(f"FAIL {name}: {type(exc).__name__}: {exc}", flush=True)
        raise SystemExit(1)


def s_versions():
    import numpy, scipy, torch, cupy
    return (f"python {sys.version.split()[0]} numpy {numpy.__version__} scipy {scipy.__version__} "
            f"torch {torch.__version__} cupy {cupy.__version__}")


def s_torch():
    import torch
    assert torch.cuda.is_available(), "torch.cuda.is_available() is False"
    x = torch.arange(1024, dtype=torch.float32, device="cuda:0")
    assert float((x * 2).sum().cpu()) == 1024 * 1023
    return f"{torch.cuda.get_device_name(0)} cuda {torch.version.cuda}"


def s_cupy_kernel():
    import cupy as cp
    k = cp.ElementwiseKernel("float32 x", "float32 y", "y = x * 2 + 1", "smoke_axpb")  # needs NVRTC
    y = k(cp.arange(8, dtype=cp.float32))
    assert y.get().tolist() == [1, 3, 5, 7, 9, 11, 13, 15]
    return f"runtime {cp.cuda.runtime.runtimeGetVersion()} nvrtc ok"


def s_interop():
    import cupy as cp
    import torch
    t = torch.zeros(16, dtype=torch.float32, device="cuda:0")
    c = cp.asarray(t)  # the backend exchanges memory this way (__cuda_array_interface__)
    c += 3
    assert float(t.sum().cpu()) == 48.0, "torch tensor did not see the cupy write (not zero-copy)"
    return "cp.asarray(torch tensor) shares memory"


def s_mapper():
    import numpy as np
    import torch
    from go2_sensor_bridge import BaseMapper

    mapper = BaseMapper(emcupy)
    row = {"frame_id": "odom", "child_frame_id": "base_link",
           "position": {"x": 0.0, "y": 0.0, "z": 0.3},
           "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    rng = np.random.default_rng(0)
    xy = rng.uniform(-1.5, 1.5, size=(6000, 2))
    z = np.where((xy[:, 0] > 0.6) & (xy[:, 0] < 1.0), -0.15, -0.30)  # flat ground with a 15 cm step
    pts = np.column_stack([xy, z]).astype(np.float32)

    lat = []
    for i in range(n_updates):
        row["position"]["x"] = 0.002 * i
        t0 = time.perf_counter()
        scan, valid, _upper = mapper.update(None, row, base_points=pts)
        torch.cuda.synchronize()
        lat.append((time.perf_counter() - t0) * 1e3)
    lat_s = sorted(lat[5:])  # drop kernel-compile warmup
    mem = torch.cuda.max_memory_allocated() / 2**20
    return (f"scan{scan.shape} valid {int(valid.sum())}/{valid.size} range [{scan.min():.3f},{scan.max():.3f}] "
            f"update+sample ms p50 {lat_s[len(lat_s) // 2]:.1f} p95 {lat_s[int(len(lat_s) * .95)]:.1f} "
            f"max {lat_s[-1]:.1f} first {lat[0]:.0f} | torch peak {mem:.0f} MiB (bridge deadline: 200 ms)")


def s_dds_import():
    import cyclonedds
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_  # noqa: F401
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: F401  (imported, NOT called)
    # Another CycloneDDS on the box (ROS, /usr/local) must not be the one that got loaded.
    import os
    with open("/proc/self/maps") as f:
        loaded = sorted({line.split()[-1] for line in f if "libddsc" in line})
    home = os.environ.get("CYCLONEDDS_HOME")
    if home and loaded and not all(os.path.realpath(p).startswith(os.path.realpath(home)) for p in loaded):
        raise RuntimeError(f"libddsc loaded from {loaded}, expected under CYCLONEDDS_HOME={home}")
    return (f"cyclonedds {getattr(cyclonedds, '__version__', '?')} + unitree_sdk2py import only, "
            f"no participant created | libddsc: {loaded or 'not mapped yet'}")


for name, fn in [("versions", s_versions), ("torch-cuda", s_torch), ("cupy-nvrtc-kernel", s_cupy_kernel),
                 ("torch-cupy-zero-copy", s_interop), ("dds-import", s_dds_import), ("elevation-mapper", s_mapper)]:
    stage(name, fn)
print("ALL PASS")
