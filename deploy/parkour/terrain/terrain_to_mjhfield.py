"""IsaacLab 파쿠르 지형(terrain_meta.npz) → MuJoCo 바이너리 hfield + 씬 XML.

왜 바이너리인가
---------------
Phase 1 의 scene.xml 은 높이를 **파이썬이 런타임에** `model.hfield_data` 로 부어
넣었다. 그때 "MuJoCo 는 hfield 를 PNG 로만 파일 로드할 수 있다" 고 적었는데
**그건 틀렸다.** libmujoco 에 `hfield missing header`, `image/vnd.mujoco.hfield`
문자열이 있고, MuJoCo 는 자체 바이너리 포맷을 읽는다:

    int32 nrow, int32 ncol, float32 data[nrow*ncol]     (row-major, [0,1] 정규화)

C++ 시뮬레이터(unitree_mujoco)는 XML 만 로드하고 데이터를 부어 넣을 자리가 없으므로
이 경로가 필요하다. PNG 였다면 8bit 양자화로 5mm 씩 뭉개졌을 텐데, 이 포맷은
float32 라 양자화가 없다.

MuJoCo hfield 규약 (틀리기 쉬운 곳)
-----------------------------------
- `size = (radius_x, radius_y, elevation_z, base_z)`. 격자는 geom 중심 기준
  x ∈ [-radius_x, +radius_x] 를 덮는다.
- 데이터는 [0,1] 정규화값이고 실제 높이 = data * elevation_z. 원본 z 를 그대로
  넣을 수 없다 — (z - zmin)/(zmax - zmin) 로 정규화하고 geom 을 z=zmin 에 둔다.
- row 가 y, col 이 x, row-major.

좌표계
------
IsaacLab 월드에서 로봇 스폰은 (-11, -2) 다. unitree_mujoco 는 go2.xml 이 정한
자리(0, 0)에 로봇을 놓으므로, **지형을 +(11, 2) 만큼 옮겨** 월드 원점이 스폰
지점이 되게 한다. 그러면 로봇을 건드리지 않고도 IsaacLab 과 같은 출발선에 선다.
z 는 옮기지 않는다 — IsaacLab 월드 z 를 그대로 쓴다.

사용:
    python terrain_to_mjhfield.py --out-dir <unitree_mujoco>/unitree_robots/go2
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
META = HERE / "assets" / "terrain" / "terrain_meta.npz"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--meta", default=str(META))
    ap.add_argument("--out-dir", required=True, help="unitree_robots/go2")
    ap.add_argument("--hfield-name", default="parkour_terrain.hfield")
    ap.add_argument("--scene-name", default="scene_parkour.xml")
    a = ap.parse_args()

    m = np.load(a.meta)
    H = m["hfield"].astype(np.float64)          # (ny, nx) 월드 z [m]
    ny, nx = H.shape
    res = float(m["res"])
    x0, y0 = float(m["x0"]), float(m["y0"])
    spawn = m["robot_spawn_pos_w"].astype(np.float64)

    zmin, zmax = float(H.min()), float(H.max())
    elev = max(zmax - zmin, 1e-6)
    norm = (H - zmin) / elev                     # [0, 1]

    out_dir = Path(a.out_dir)
    # MuJoCo 는 hfield file 을 meshdir 기준으로 찾는다. go2.xml 이 meshdir="assets"
    # 이므로 거기에 둔다 (씬 XML 에서는 파일명만 적는다).
    asset_dir = out_dir / "assets"
    asset_dir.mkdir(parents=True, exist_ok=True)
    hf_path = asset_dir / a.hfield_name
    with open(hf_path, "wb") as f:
        f.write(struct.pack("<ii", ny, nx))      # nrow=y, ncol=x
        f.write(np.ascontiguousarray(norm, dtype=np.float32).tobytes())

    radius_x = (nx - 1) * res / 2.0
    radius_y = (ny - 1) * res / 2.0
    # 격자 중심(IsaacLab 월드) → 스폰이 원점이 되도록 이동
    cx = x0 + (nx - 1) * res / 2.0 - spawn[0]
    cy = y0 + (ny - 1) * res / 2.0 - spawn[1]

    print(f"hfield : {hf_path}")
    print(f"  격자 {ny} x {nx} (res {res} m), z [{zmin:.4f}, {zmax:.4f}] → elev {elev:.4f}")
    print(f"  IsaacLab 스폰 {spawn.tolist()} 를 월드 원점으로 옮김")
    print(f"  geom pos = ({cx:.4f}, {cy:.4f}, {zmin:.4f})  size = "
          f"({radius_x:.4f}, {radius_y:.4f}, {elev:.4f}, 1.0)")

    scene = f'''<mujoco model="go2 parkour scene">
  <!-- IsaacLab 파쿠르 지형(trained_v1.3 PLAY 와 같은 난이도 0.7, row 1) 위의 Go2.
       terrain_to_mjhfield.py 가 생성. 원본은 Isaaclab_Parkour 의
       deploy/tools/export_terrain.py 가 IsaacLab 삼각망에서 레이캐스팅으로 구운 것
       (평균 오차 3.29 mm). 높이는 float32 바이너리라 양자화가 없다.

       좌표: IsaacLab 스폰 {spawn.tolist()} 이 월드 원점에 오도록 지형을 옮겼다.
       로봇(go2.xml)은 원점에 그대로 두면 IsaacLab 과 같은 출발선에 선다. -->
  <include file="go2.xml"/>

  <statistic center="0 0 0.1" extent="2.0"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <hfield name="parkour" file="{a.hfield_name}" size="{radius_x:.6f} {radius_y:.6f} {elev:.6f} 1.0"/>
  </asset>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true"/>
    <geom name="terrain" type="hfield" hfield="parkour"
          pos="{cx:.6f} {cy:.6f} {zmin:.6f}"
          material="groundplane" friction="0.6" condim="3"/>
  </worldbody>
</mujoco>
'''
    scene_path = out_dir / a.scene_name
    scene_path.write_text(scene)
    print(f"scene  : {scene_path}")

    # 재현용: 몇 개 지점의 기대 높이를 남긴다 (검증 스크립트가 쓴다)
    probe = []
    rng = np.random.default_rng(0)
    for _ in range(200):
        i = int(rng.integers(0, nx))
        j = int(rng.integers(0, ny))
        wx = x0 + i * res - spawn[0]
        wy = y0 + j * res - spawn[1]
        probe.append((wx, wy, float(H[j, i])))
    np.savez(out_dir / "parkour_terrain_probe.npz", probe=np.array(probe),
             spawn_world=spawn, res=res)
    print(f"probe  : {out_dir / 'parkour_terrain_probe.npz'} (200점 기대 높이)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
