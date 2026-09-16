"""Unit tests for metadata-aware PointCloud2 XYZ decoding."""
from __future__ import annotations

import struct
import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG.parent))

from em_sidecar.pointcloud import decode_xyz  # noqa: E402


@dataclass
class Field:
    name: str
    offset: int
    datatype: int = 7  # sensor_msgs/PointField.FLOAT32
    count: int = 1


def cloud(*, width: int, height: int, point_step: int, row_step: int,
          data: bytes, fields: list[Field], big: bool = False):
    return SimpleNamespace(width=width, height=height, point_step=point_step,
                           row_step=row_step, data=data, fields=fields,
                           is_bigendian=big)


XYZ = [Field("x", 0), Field("y", 4), Field("z", 8)]


class DecodeXyzTest(unittest.TestCase):
    def test_simulator_packed_xyz12(self):
        values = [(1.0, 2.0, 3.0), (-4.0, 5.5, 6.0)]
        msg = cloud(width=2, height=1, point_step=12, row_step=24,
                    data=b"".join(struct.pack("<fff", *p) for p in values), fields=XYZ)
        np.testing.assert_array_equal(decode_xyz(msg), np.asarray(values, dtype=np.float64))

    def test_real_lidar_stride32(self):
        fields = XYZ + [Field("intensity", 16), Field("ring", 20, 4), Field("time", 24)]
        raw = bytearray(64)
        struct.pack_into("<fff", raw, 0, 1.0, 2.0, 3.0)
        struct.pack_into("<fff", raw, 32, 4.0, 5.0, 6.0)
        struct.pack_into("<f", raw, 16, 99.0)
        struct.pack_into("<H", raw, 20, 7)
        struct.pack_into("<f", raw, 24, 0.01)
        struct.pack_into("<f", raw, 48, 88.0)
        struct.pack_into("<H", raw, 52, 8)
        struct.pack_into("<f", raw, 56, 0.02)
        msg = cloud(width=2, height=1, point_step=32, row_step=64,
                    data=bytes(raw), fields=fields)
        np.testing.assert_array_equal(
            decode_xyz(msg), np.asarray([[1, 2, 3], [4, 5, 6]], dtype=np.float64))

    def test_organized_cloud_with_row_padding(self):
        raw = bytearray(64)  # two 24-byte rows plus 8 bytes padding per row
        for offset, point in zip((0, 12, 32, 44),
                                 ((1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12))):
            struct.pack_into("<fff", raw, offset, *point)
        msg = cloud(width=2, height=2, point_step=12, row_step=32,
                    data=bytes(raw), fields=XYZ)
        np.testing.assert_array_equal(
            decode_xyz(msg), np.arange(1, 13, dtype=np.float64).reshape(4, 3))

    def test_big_endian_float64(self):
        fields = [Field("x", 0, 8), Field("y", 8, 8), Field("z", 16, 8)]
        msg = cloud(width=1, height=1, point_step=24, row_step=24,
                    data=struct.pack(">ddd", 1.25, -2.5, 3.75), fields=fields, big=True)
        np.testing.assert_array_equal(
            decode_xyz(msg), np.asarray([[1.25, -2.5, 3.75]], dtype=np.float64))

    def test_nonfinite_points_are_removed(self):
        values = [(1.0, 2.0, 3.0), (float("nan"), 0.0, 1.0),
                  (2.0, float("inf"), 3.0)]
        msg = cloud(width=3, height=1, point_step=12, row_step=36,
                    data=b"".join(struct.pack("<fff", *p) for p in values), fields=XYZ)
        np.testing.assert_array_equal(decode_xyz(msg), np.asarray([values[0]], dtype=np.float64))

    def test_empty_cloud_is_safe(self):
        msg = cloud(width=0, height=0, point_step=12, row_step=0, data=b"", fields=XYZ)
        self.assertEqual(decode_xyz(msg).shape, (0, 3))

    def test_rejects_malformed_metadata(self):
        cases = [
            cloud(width=2, height=1, point_step=12, row_step=24,
                  data=b"\0" * 12, fields=XYZ),
            cloud(width=2, height=1, point_step=12, row_step=20,
                  data=b"\0" * 20, fields=XYZ),
            cloud(width=1, height=1, point_step=12, row_step=12,
                  data=b"\0" * 12, fields=XYZ[:2]),
            cloud(width=1, height=1, point_step=12, row_step=12,
                  data=b"\0" * 12, fields=[Field("x", 8, 8), *XYZ[1:]]),
            cloud(width=1, height=1, point_step=12, row_step=12,
                  data=b"\0" * 12, fields=[Field("x", 0, count=2), *XYZ[1:]]),
            cloud(width=1, height=1, point_step=12, row_step=12,
                  data=b"\0" * 12, fields=[*XYZ, Field("x", 0)]),
        ]
        for msg in cases:
            with self.subTest(msg=msg), self.assertRaises(ValueError):
                decode_xyz(msg)


if __name__ == "__main__":
    unittest.main()
