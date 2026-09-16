"""Observed hardware Go2 LiDAR -> published base_link transform.

Source: captures/frame_inspection_20260915/verified_findings.json, independently
checked on captures/live_frame_check01. This reproduces coordinates of matching
cloud_base points, not the publisher's point selection or motion deskewing.
It does not establish alignment between published base_link and the policy URDF.
Do not use these hardware extrinsics for the MuJoCo LiDAR.
"""
from __future__ import annotations

import numpy as np

from .pointcloud import decode_xyz

RAW_FRAME = "utlidar_lidar"
RAW_TO_BASE_ROTATION = np.array([
    [0.5261139015413693, -0.8101358160799055, 0.25861964757044975],
    [-0.8386681732699831, -0.5446427224718022, 1.5787507760715023e-06],
    [0.14085402993491933, -0.2168968980023663, -0.9659792326380745],
], dtype=np.float64)
RAW_TO_BASE_TRANSLATION = np.array([
    0.2821600136070268, -2.1560153107280655e-08, -3.586840124913948e-08,
], dtype=np.float64)
RAW_TO_BASE_ROTATION.setflags(write=False)
RAW_TO_BASE_TRANSLATION.setflags(write=False)


def raw_cloud_to_base(msg) -> np.ndarray:
    """Decode finite raw XYZ (metres) and return base_link XYZ, without odometry.

    Require the raw frame so a cloud already in base_link cannot be transformed
    twice. Preserve the input message and its source timestamp unchanged.
    """
    if msg.header.frame_id != RAW_FRAME:
        raise ValueError(f"raw cloud must be {RAW_FRAME}, got {msg.header.frame_id!r}")
    return decode_xyz(msg) @ RAW_TO_BASE_ROTATION.T + RAW_TO_BASE_TRANSLATION
