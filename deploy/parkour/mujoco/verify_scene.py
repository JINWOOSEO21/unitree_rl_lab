"""MuJoCo 씬이 IsaacLab 지형과 같은지, 로봇이 제대로 서는지 검증한다 (Phase 1 게이트).

검사 2개
--------
1. **지형 일치**: 같은 (x, y) 에서 위→아래로 레이를 쏴 지면 높이를 재고,
   MuJoCo hfield 와 IsaacLab 원본 삼각망(terrain.obj)을 비교한다.
   이 검사는 hfield 데이터뿐 아니라 XML 의 size/pos 계산, row/col 순서,
   [0,1] 정규화까지 **한꺼번에** 검증한다. 하나라도 틀리면 값이 어긋난다.

   앞서 export 쪽에서 한 height_scanner 대조는 로봇이 스폰 시 평지에 서 있어서
   평지만 확인한 셈이라 약했다. 여기서는 경사·계단을 포함해 전 영역을 본다.

2. **로봇 정착**: 스폰 지점에 기본 자세로 놓고 PD 로 자세를 유지시키며 1초 굴려,
   지형을 뚫고 내려가지 않고 안정된 높이에 서는지 본다.

실행
----
    python deploy/parkour/mujoco/verify_scene.py
"""

from __future__ import annotations

import os
import sys

import numpy as np
import mujoco
import trimesh
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from build_scene import TERRAIN_DIR, load_scene  # noqa: E402

CONTRACT = os.path.join(os.path.dirname(_HERE), "contract", "deploy.yaml")


def mj_ground_height(model, data, xy, z_from=5.0):
    """MuJoCo 씬에서 (x, y) 의 **지형** 높이. 못 맞히면 NaN.

    `mj_ray` 를 쓰면 안 된다 — 씬의 모든 geom 을 보므로 로봇 몸통을 먼저 맞힌다.
    (처음에 그렇게 짰다가 "발밑 지면 0.352m" 라는 말이 안 되는 값이 나왔다.
     그게 로봇 등판 높이였다.) `mj_rayHfield` 는 지정한 geom 하나만 본다.
    """
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
    if gid < 0:
        raise RuntimeError("geom 'terrain' 을 찾지 못했다")
    out = np.full(len(xy), np.nan)
    vec = np.array([0.0, 0.0, -1.0])
    for k, (x, y) in enumerate(xy):
        pnt = np.array([float(x), float(y), z_from])
        dist = mujoco.mj_rayHfield(model, data, gid, pnt, vec)
        if dist >= 0:
            out[k] = z_from - dist
    return out


def mesh_ground_height(mesh, xy, z_from=5.0):
    n = len(xy)
    org = np.column_stack([xy[:, 0], xy[:, 1], np.full(n, z_from)])
    dirs = np.tile(np.array([0.0, 0.0, -1.0]), (n, 1))
    loc, idx, _ = mesh.ray.intersects_location(org, dirs, multiple_hits=False)
    out = np.full(n, -np.inf)
    np.maximum.at(out, idx, loc[:, 2])
    out[~np.isfinite(out)] = np.nan
    return out


def check_terrain(model, data, meta, n_samples=4000, seed=0):
    mesh = trimesh.load(os.path.join(TERRAIN_DIR, "terrain.obj"), process=False)
    rng = np.random.default_rng(seed)
    # 지형 타일 영역만 본다 (테두리는 평지라 정보가 없다).
    half_x = float(meta["num_rows"]) * float(meta["tile_size"][0]) / 2.0
    half_y = float(meta["num_cols"]) * float(meta["tile_size"][1]) / 2.0
    xy = np.column_stack([
        rng.uniform(-half_x + 0.1, half_x - 0.1, n_samples),
        rng.uniform(-half_y + 0.1, half_y - 0.1, n_samples),
    ])

    z_mj = mj_ground_height(model, data, xy)
    z_gt = mesh_ground_height(mesh, xy)
    ok = np.isfinite(z_mj) & np.isfinite(z_gt)
    d = np.abs(z_mj[ok] - z_gt[ok])

    print(f"\n[1] 지형 일치 — 무작위 {n_samples}점 (MuJoCo hfield vs IsaacLab 삼각망)")
    print(f"  적중 {ok.sum()}/{n_samples}")
    print(f"  max|diff|  = {d.max() * 1000:7.2f} mm")
    print(f"  mean|diff| = {d.mean() * 1000:7.2f} mm")
    print(f"  p99        = {np.percentile(d, 99) * 1000:7.2f} mm")
    print(f"  1cm 초과 비율 = {(d > 0.01).mean() * 100:.2f} %")
    # 격자 간격 50mm 라 단차 옆면(수직면)에서는 셀 안에서 최대 단차만큼 어긋날 수
    # 있다. 그건 이산화의 본질적 한계이므로 '대부분의 점' 기준으로 판정한다.
    verdict = (d.mean() < 0.005) and ((d > 0.01).mean() < 0.05)
    print(f"  => {'PASS' if verdict else 'FAIL'} (평균 5mm 미만, 1cm 초과 5% 미만)")
    return verdict


def check_robot_settles(model, data, meta, seconds=1.0):
    contract = yaml.safe_load(open(CONTRACT))
    q_def_mj = np.array(contract["default_joint_pos"]["mjcf"], dtype=np.float64)
    kp = np.array(contract["actuator"]["stiffness"], dtype=np.float64)
    kd = np.array(contract["actuator"]["damping"], dtype=np.float64)
    il_to_mj = np.array(contract["index_maps"]["il_to_mj"])
    # 게인은 IsaacLab 순서라 MJCF 순서로 옮긴다.
    kp_mj = np.empty(12); kp_mj[il_to_mj] = kp
    kd_mj = np.empty(12); kd_mj[il_to_mj] = kd

    spawn = np.asarray(meta["robot_spawn_pos_w"], dtype=np.float64)
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = spawn
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[7:] = q_def_mj
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    z0 = float(data.qpos[2])
    n = int(seconds / model.opt.timestep)
    for _ in range(n):
        q, dq = data.qpos[7:], data.qvel[6:]
        data.ctrl[:] = kp_mj * (q_def_mj - q) - kd_mj * dq
        mujoco.mj_step(model, data)

    z1 = float(data.qpos[2])
    ground = mj_ground_height(model, data, np.array([[data.qpos[0], data.qpos[1]]]))[0]
    clearance = z1 - ground
    joint_err = float(np.abs(data.qpos[7:] - q_def_mj).max())

    print(f"\n[2] 로봇 정착 — 스폰 {spawn}, PD 로 기본 자세 유지, {seconds}s")
    print(f"  base z: 시작 {z0:.4f} -> 끝 {z1:.4f} m")
    print(f"  발밑 지면 높이 {ground:.4f} m,  지상고 {clearance:.4f} m")
    print(f"  관절 최대 편차 {joint_err:.4f} rad")
    # Go2 기본 자세의 몸통 높이는 대략 0.30~0.35m 다. 지형을 뚫었으면 음수가 된다.
    verdict = (0.20 < clearance < 0.45) and joint_err < 0.35
    print(f"  => {'PASS' if verdict else 'FAIL'} (지상고 0.20~0.45m, 관절 편차 0.35rad 미만)")
    return verdict


def main():
    model, data, meta = load_scene()
    mujoco.mj_forward(model, data)
    print(f"씬 로드 OK: nq={model.nq} nv={model.nv} nu={model.nu} timestep={model.opt.timestep}")

    r1 = check_terrain(model, data, meta)
    r2 = check_robot_settles(model, data, meta)
    print("\n" + ("전체 PASS" if (r1 and r2) else "실패한 검사가 있다"))
    sys.exit(0 if (r1 and r2) else 1)


if __name__ == "__main__":
    main()
