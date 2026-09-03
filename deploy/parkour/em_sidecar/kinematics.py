"""Go2 순기구학 + self-filter 캡슐 — 순수 numpy.

학습(IsaacLab)에서는 시뮬레이터가 모든 링크 pose 를 그냥 알려줬다. 배포에서는
lowstate 의 관절각 12개와 IMU 자세밖에 없으므로 링크 pose 를 직접 풀어야 한다.
self-filter 캡슐이 다리에 붙어 있어서 이게 없으면 로봇 자기 몸이 지형으로
찍힌다.

기구학 상수는 contract/em_geometry.npz 에 있고, 그 값은 손으로 옮겨 적은 것이
아니라 IsaacLab 실측 pose 에서 역산한 것이다 (deploy/tools/build_em_contract.py).
npz 에는 역산에 쓰인 자세 8개와 그때의 링크 위치도 함께 들어 있어서, 이 모듈이
같은 결과를 내는지 IsaacLab 없이 여기서 다시 확인할 수 있다 — test_kinematics.py.

mujoco 를 쓰지 않는 이유: 젯슨에 물리엔진을 얹지 않기 위해서다. 이 파일은
numpy 만 있으면 돈다.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """(w, x, y, z) → (..., 3, 3). IsaacLab/Unitree IMU 공통 규약."""
    q = np.asarray(q, dtype=np.float64)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack(
        [
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
        ],
        axis=-1,
    ).reshape(*q.shape[:-1], 3, 3)


def yaw_from_quat(q: np.ndarray) -> float:
    """(w, x, y, z) → yaw [rad]. ZYX 오일러의 z 성분."""
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _rot_axis_angle(a: np.ndarray, th: np.ndarray) -> np.ndarray:
    """(3,) 축 + (N,) 각 → (N, 3, 3) Rodrigues."""
    th = np.atleast_1d(np.asarray(th, dtype=np.float64))
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    s = np.sin(th)[:, None, None]
    c = (1.0 - np.cos(th))[:, None, None]
    return np.eye(3)[None] + s * K[None] + c * (K @ K)[None]


@dataclass
class Capsule:
    """월드 좌표 선분 a-b + 반경. self-filter 프리미티브."""

    ia: int  # 링크 인덱스
    off_a: np.ndarray  # 링크 프레임 오프셋
    ib: int
    off_b: np.ndarray
    radius: float


class Go2Kinematics:
    """관절각 → 링크 pose → self-filter 캡슐 끝점."""

    def __init__(self, contract_npz: str | Path):
        d = np.load(str(contract_npz))
        self.link_names = [str(s) for s in d["fk_names"]]
        self.parent = d["fk_parent"].astype(np.int64)  # -1 == base
        self.joint = d["fk_joint"].astype(np.int64)  # -1 == 고정 링크
        self.p0 = d["fk_p0"].astype(np.float64)
        self.R0 = d["fk_R0"].astype(np.float64)
        self.axis = d["fk_axis"].astype(np.float64)
        self.joint_names = [str(s) for s in d["joint_names"]]
        self.scan_offsets_xy = d["scan_offsets_xy"].astype(np.float64)

        # 회귀 기준값 (IsaacLab 실측) — test_kinematics.py 가 쓴다.
        self.q_test = d["q_test"].astype(np.float64)
        self.pos_ref = d["cap_pos_in_base_ref"].astype(np.float64)

        idx = {n: i for i, n in enumerate(self.link_names)}
        idx["base"] = -1  # base 는 사슬의 루트라 링크 배열에 없다
        self.capsules = [
            Capsule(idx[str(na)], np.asarray(oa, dtype=np.float64),
                    idx[str(nb)], np.asarray(ob, dtype=np.float64), float(r))
            for na, oa, nb, ob, r in zip(
                d["capsule_a"], d["capsule_off_a"], d["capsule_b"],
                d["capsule_off_b"], d["capsule_radius"],
            )
        ]
        self._n = len(self.link_names)

    def link_poses_base(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """관절각(IsaacLab 순서 12) → base 프레임 링크 (위치 (L,3), 회전 (L,3,3))."""
        q = np.asarray(q, dtype=np.float64)
        P = np.zeros((self._n, 3))
        R = np.zeros((self._n, 3, 3))
        for i in range(self._n):
            p_par = np.zeros(3) if self.parent[i] < 0 else P[self.parent[i]]
            R_par = np.eye(3) if self.parent[i] < 0 else R[self.parent[i]]
            P[i] = p_par + R_par @ self.p0[i]
            Ri = self.R0[i]
            if self.joint[i] >= 0:
                Ri = Ri @ _rot_axis_angle(self.axis[i], q[self.joint[i]])[0]
            R[i] = R_par @ Ri
        return P, R

    def capsule_endpoints(
        self, q: np.ndarray, base_pos: np.ndarray, R_base: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """캡슐 끝점을 월드(odom) 좌표로. 반환 (a (C,3), b (C,3), radius (C,))."""
        P, R = self.link_poses_base(q)

        def resolve(i: int, off: np.ndarray) -> np.ndarray:
            # i < 0 이면 base 자신 (base 프레임 원점 = 0, 회전 = I)
            p_l = off if i < 0 else P[i] + R[i] @ off
            return base_pos + R_base @ p_l

        a = np.stack([resolve(c.ia, c.off_a) for c in self.capsules])
        b = np.stack([resolve(c.ib, c.off_b) for c in self.capsules])
        r = np.array([c.radius for c in self.capsules])
        return a, b, r
