"""self-hit 필터 — 학습(parkour_isaaclab/sensors/l1_scan_ray_caster.py)의 numpy 판.

L1 은 로봇 몸통 위에 달려 있어서 다리·머리·몸통을 그대로 찍는다. 이걸 안 걸면
EM 이 로봇 자신을 지형으로 알아듣고, scandots 가 "눈앞에 벽" 이라고 말한다.

판정은 hit 점과 몸의 **거리**가 아니라 **레이 경로의 관통**으로 한다. 거리로 하면
몸을 스치고 지나가 뒤쪽 바닥을 찍은 점(shadow/mixed point)을 못 잡는다.
마운트 자체가 몸통 캡슐 안에 있으므로 t0(=0.12 m) 이후 구간만 검사한다.

수식은 학습 코드와 같은 선분-선분 최근접거리(Ericson, Real-Time Collision
Detection)다. 여기서는 센서 프레임에서 계산하므로 레이 원점이 항상 0 이고,
그만큼 식이 짧아진다.
"""
from __future__ import annotations

import numpy as np


def capsule_self_hits(
    points_sensor: np.ndarray,
    caps_a_sensor: np.ndarray,
    caps_b_sensor: np.ndarray,
    radius: np.ndarray,
    t0: float = 0.12,
) -> np.ndarray:
    """센서 프레임 점군 중 로봇 자신을 맞힌 점을 True 로 표시한다.

    Args:
        points_sensor: (M, 3) 센서 프레임 점군. 원점이 곧 센서라 방향/거리가 이 안에 있다.
        caps_a_sensor, caps_b_sensor: (C, 3) 센서 프레임 캡슐 끝점.
        radius: (C,) 캡슐 반경.
        t0: 검사 시작 거리 [m]. 마운트 캡슐 최대 탈출거리(0.09)와 L1 최소
            측정거리(0.05)를 모두 넘는 값이어야 한다.

    Returns:
        (M,) bool — True 면 자기 몸(또는 몸을 관통한 경로).
    """
    m = points_sensor.shape[0]
    if m == 0:
        return np.zeros(0, dtype=bool)

    t_hit = np.linalg.norm(points_sensor, axis=1)  # (M,)
    safe = np.maximum(t_hit, 1e-12)
    dirs = points_sensor / safe[:, None]  # (M,3)

    seg_len = np.maximum(t_hit - t0, 0.0)  # (M,)
    p1 = dirs * t0  # (M,3)   레이 구간 시작점
    d1 = dirs * seg_len[:, None]  # (M,3)   레이 구간 벡터

    p2 = caps_a_sensor  # (C,3)
    d2 = caps_b_sensor - caps_a_sensor  # (C,3)

    r = p1[:, None, :] - p2[None, :, :]  # (M,C,3)
    a = (d1 * d1).sum(-1)[:, None]  # (M,1)
    e = (d2 * d2).sum(-1)[None, :]  # (1,C)
    f = (d2[None] * r).sum(-1)  # (M,C)
    c = (d1[:, None] * r).sum(-1)  # (M,C)
    b = d1 @ d2.T  # (M,C)

    denom = np.maximum(a * e - b * b, 1e-9)
    s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
    t = np.clip((b * s + f) / np.maximum(e, 1e-9), 0.0, 1.0)
    s = np.clip((b * t - c) / np.maximum(a, 1e-9), 0.0, 1.0)

    closest1 = p1[:, None, :] + d1[:, None, :] * s[..., None]
    closest2 = p2[None, :, :] + d2[None, :, :] * t[..., None]
    dist = np.linalg.norm(closest1 - closest2, axis=-1)  # (M,C)
    return (dist < radius[None, :]).any(axis=1)
