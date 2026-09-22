"""Terrain height interpolation for MuJoCo recording visualization."""
from pathlib import Path

import numpy as np

META = Path(__file__).resolve().parents[1] / "terrain/assets/terrain/terrain_meta.npz"


class Terrain:
    """terrain_meta.npz 의 높이 격자를 MuJoCo 월드 좌표에서 쌍선형 보간한다.

    MuJoCo 씬은 IsaacLab 스폰이 원점에 오도록 지형을 옮겼으므로
    (terrain_to_mjhfield.py), 월드 (x, y) → 격자 좌표는 spawn 만큼 되더한다.
    """

    def __init__(self, path: Path):
        m = np.load(path)
        self.H = m["hfield"].astype(np.float64)      # (ny, nx)
        self.res = float(m["res"])
        self.x0, self.y0 = float(m["x0"]), float(m["y0"])
        self.spawn = m["robot_spawn_pos_w"].astype(np.float64)
        self.ny, self.nx = self.H.shape

    def height(self, wx, wy):
        """월드 (x, y) 배열 → 지형 z. 격자 밖은 NaN."""
        gx = (np.asarray(wx) + self.spawn[0] - self.x0) / self.res
        gy = (np.asarray(wy) + self.spawn[1] - self.y0) / self.res
        i0 = np.floor(gx).astype(int)
        j0 = np.floor(gy).astype(int)
        fx = gx - i0
        fy = gy - j0
        ok = (i0 >= 0) & (i0 < self.nx - 1) & (j0 >= 0) & (j0 < self.ny - 1)
        i0c = np.clip(i0, 0, self.nx - 2)
        j0c = np.clip(j0, 0, self.ny - 2)
        h = ((1 - fx) * (1 - fy) * self.H[j0c, i0c]
             + fx * (1 - fy) * self.H[j0c, i0c + 1]
             + (1 - fx) * fy * self.H[j0c + 1, i0c]
             + fx * fy * self.H[j0c + 1, i0c + 1])
        return np.where(ok, h, np.nan)
