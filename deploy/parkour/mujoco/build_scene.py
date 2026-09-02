"""IsaacLab 파쿠르 지형 + Go2 로 이루어진 MuJoCo 씬을 만든다.

입력 (Isaaclab_Parkour 의 deploy/tools/export_terrain.py 산출물)
    assets/terrain/terrain_meta.npz : hfield 격자 + 목표점/스폰 등 메타
    assets/terrain/terrain.obj      : 원본 삼각망 (검증용)
    assets/unitree_go2/go2.xml      : mujoco_menagerie e4049d0

출력
    scene.xml  : go2 + hfield 지형. hfield 데이터는 XML 에 못 담으므로
                 nrow/ncol/size 만 선언해 두고 실제 높이는 `load_scene()` 이
                 model.hfield_data 에 부어 넣는다.

왜 hfield 데이터를 XML 에 안 넣나
---------------------------------
MuJoCo 는 hfield 를 PNG 로만 파일 로드할 수 있는데, PNG 는 8/16bit 정수라
높이가 양자화된다. 지형 높이 범위가 1.29m 이므로 8bit 면 5mm 단위로 뭉개진다 —
정책이 보는 scandot 이 cm 단위라 무시할 수 없다. 파이썬에서 float32 를 직접
부으면 양자화가 없다.

MuJoCo hfield 규약 (틀리기 쉬운 곳)
-----------------------------------
- `size = (radius_x, radius_y, elevation_z, base_z)`. 격자는 geom 중심 기준
  x ∈ [-radius_x, +radius_x], y ∈ [-radius_y, +radius_y] 를 덮는다.
- 데이터는 [0,1] 로 정규화된 nrow×ncol 배열이고, 실제 높이 = data * elevation_z.
  따라서 원본 z 를 그대로 넣을 수 없고 (z - zmin)/(zmax - zmin) 로 정규화한 뒤
  geom 을 z=zmin 에 놓아야 월드 좌표가 맞는다.
- 저장 순서는 row-major, row 가 y, col 이 x. 우리 H[j, i] (j=y, i=x) 의
  `ravel()` 이 그대로 맞는다.
- 격자 간격은 2*radius/(n-1) 이다. 그래서 radius_x = (ncol-1)*res/2 로 잡아야
  간격이 정확히 res 가 된다. (n*res/2 로 잡는 실수를 하기 쉽다.)
"""

from __future__ import annotations

import argparse
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
TERRAIN_DIR = os.path.join(_HERE, "assets", "terrain")
GO2_XML = os.path.join(_HERE, "assets", "unitree_go2", "go2.xml")
SCENE_XML = os.path.join(_HERE, "scene.xml")

# IsaacLab 지형 물성 (parkour_tasks/default_cfg.py TerrainImporterCfg.physics_material).
# 로봇 쪽 마찰은 startup DR 로 0.6~2.0 을 뽑지만 지형 자체는 고정 1.0 이다.
TERRAIN_FRICTION = 1.0


def load_meta(terrain_dir: str = TERRAIN_DIR) -> dict:
    z = np.load(os.path.join(terrain_dir, "terrain_meta.npz"), allow_pickle=True)
    return {k: z[k] for k in z.files}


def build_xml(meta: dict, out_path: str = SCENE_XML, timestep: float = 0.005) -> str:
    H = meta["hfield"].astype(np.float64)
    ny, nx = H.shape
    res = float(meta["res"])
    x0, y0 = float(meta["x0"]), float(meta["y0"])
    zmin, zmax = float(H.min()), float(H.max())
    elev = max(zmax - zmin, 1e-6)

    radius_x = (nx - 1) * res / 2.0
    radius_y = (ny - 1) * res / 2.0
    cx = x0 + radius_x
    cy = y0 + radius_y
    base_z = 1.0  # 격자 아래로 두께를 줘서 로봇이 옆에서 파고들지 않게 한다

    out_dir = os.path.dirname(os.path.abspath(out_path))
    go2_rel = os.path.relpath(GO2_XML, out_dir)
    mesh_rel = os.path.relpath(os.path.join(os.path.dirname(GO2_XML), "assets"), out_dir)
    xml = f"""<mujoco model="go2 parkour">
  <!-- 이 파일은 build_scene.py 가 생성한다. 직접 고치지 말 것. -->
  <include file="{go2_rel}"/>

  <!-- go2.xml 의 meshdir="assets" 는 **최상위 파일** 기준으로 해석되므로
       include 만 하면 mesh 경로가 깨진다. include 뒤에 compiler 를 다시 선언해
       덮어쓴다 (뒤에 오는 값이 이긴다 — 실측 확인). -->
  <compiler angle="radian" meshdir="{mesh_rel}" autolimits="true"/>

  <!-- IsaacLab 과 같은 물리 스텝. 정책은 decimation=4 로 50Hz. -->
  <option timestep="{timestep}"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <!-- 높이 데이터는 load_scene() 이 model.hfield_data 로 부어 넣는다 (PNG 양자화 회피). -->
    <hfield name="parkour" nrow="{ny}" ncol="{nx}" size="{radius_x:.6f} {radius_y:.6f} {elev:.6f} {base_z:.6f}"/>
  </asset>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true"/>
    <geom name="terrain" type="hfield" hfield="parkour"
          pos="{cx:.6f} {cy:.6f} {zmin:.6f}"
          friction="{TERRAIN_FRICTION} 0.005 0.0001" condim="3" contype="1" conaffinity="1"/>
  </worldbody>
</mujoco>
"""
    with open(out_path, "w") as f:
        f.write(xml)
    return out_path


def load_scene(terrain_dir: str = TERRAIN_DIR, scene_xml: str = SCENE_XML, rebuild: bool = True):
    """씬을 컴파일하고 hfield 높이를 채워 (model, data, meta) 를 돌려준다."""
    import mujoco

    meta = load_meta(terrain_dir)
    if rebuild or not os.path.exists(scene_xml):
        build_xml(meta, scene_xml)

    model = mujoco.MjModel.from_xml_path(scene_xml)
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "parkour")
    if hid < 0:
        raise RuntimeError("hfield 'parkour' 를 찾지 못했다")

    H = meta["hfield"].astype(np.float64)
    zmin, zmax = float(H.min()), float(H.max())
    elev = max(zmax - zmin, 1e-6)
    norm = ((H - zmin) / elev).astype(np.float32)

    adr = model.hfield_adr[hid]
    n = model.hfield_nrow[hid] * model.hfield_ncol[hid]
    assert n == norm.size, f"hfield 크기 불일치: 모델 {n} vs 데이터 {norm.size}"
    model.hfield_data[adr : adr + n] = norm.ravel()

    data = mujoco.MjData(model)
    return model, data, meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--terrain-dir", default=TERRAIN_DIR)
    ap.add_argument("--out", default=SCENE_XML)
    args = ap.parse_args()

    meta = load_meta(args.terrain_dir)
    H = meta["hfield"]
    path = build_xml(meta, args.out)
    print(f"scene.xml 생성: {path}")
    print(f"  hfield {H.shape} (ny,nx), res={float(meta['res'])}m, z ∈ [{H.min():.3f}, {H.max():.3f}]")
    print(f"  지형 원점(row,col) =\n{meta['terrain_origins']}")
    print(f"  로봇 스폰(world) = {meta['robot_spawn_pos_w']}")
    print(f"  지형 이름 = {meta['terrain_names'].ravel().tolist()}")

    model, data, _ = load_scene(args.terrain_dir, args.out)
    print(f"\n컴파일 OK: nq={model.nq} nv={model.nv} nu={model.nu} ngeom={model.ngeom}")


if __name__ == "__main__":
    main()
